/*
 * battery_charge.c
 *
 * Closed loop control for the buck charger PWM.
 */

#include "battery_charge.h"
#include "main.h"
#include "battery_balance.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define FAN_PULSES_PER_REV 2 
// #define FAN_DEBUG 1

// Per-cell overvoltage guard (applied in charger_update)
#define CELL_OVERVOLTAGE_LIMIT 4.1f

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

typedef struct {
    ChargerControllerCfg cfg;
    uint8_t enabled;
    ChargerState state;
    float target_voltage;
    float target_current;
    float duty;
    float duty_min_counts;
    float duty_max_counts;
    float pwm_counts_max;
    PI_Controller current_pi;
    PI_Controller voltage_pi;
    uint32_t last_tick;
    uint32_t termination_timer;
    uint8_t fast_mode;  // 1 = fast loop (50ms), 0 = slow loop (300ms)
    float filtered_current;  // EMA filtered current for smooth control
} ChargerController;

static ChargerController charger = {0};

static void charger_apply_pwm(float duty_counts);
static void charger_log_state_change(ChargerState prev_state, ChargerState new_state, const char *reason);
static void charger_disable_internal(const char *reason);
static void charger_storage_update(const VoltageValues *meas);

void pi_controller_init(PI_Controller *controller, float kp, float ki, float integral_limit, float output_min, float output_max) {
    controller->kp = kp;
    controller->ki = ki;
    controller->integral_limit = integral_limit;
    controller->integral = 0.0f;
    controller->output_min = output_min;
    controller->output_max = output_max;
}

float pi_controller_update(PI_Controller *controller, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // Integral term with anti-windup
    controller->integral += error * dt;
    if (controller->integral > controller->integral_limit) {
        controller->integral = controller->integral_limit;
    } else if (controller->integral < -controller->integral_limit) {
        controller->integral = -controller->integral_limit;
    }

    // PI controller output
    float output = (controller->kp * error) + (controller->ki * controller->integral);

    // Clamp output
    if (output > controller->output_max) {
        output = controller->output_max;
    } else if (output < controller->output_min) {
        output = controller->output_min;
    }

    return output;
}

void charger_fault_clear_all(void) {
    charger_faults.overvoltage = false;
    charger_faults.overcurrent = false;
    charger_faults.timeout = false;
    charger_faults.cell_imbalance = false;
    charger_faults.battery_not_present = false;
    charger_faults.temp_high = false;
    charger_faults.temp_low = false;
    charger_faults.fan_error = false;
    charger_faults.unknown = false;
    // log disabled
}

void charger_controller_init(ChargerControllerCfg cfg) {
    charger.cfg = cfg;
    if (charger.cfg.duty_min < 0.0f) {
        charger.cfg.duty_min = 0.0f;
    }
    if (charger.cfg.duty_max > 100.0f) {
        charger.cfg.duty_max = 100.0f;
    }
    if (charger.cfg.duty_max < charger.cfg.duty_min) {
        charger.cfg.duty_max = charger.cfg.duty_min;
    }
    if (charger.cfg.update_period_ms == 0) {
        charger.cfg.update_period_ms = 10;
    }

    float pwm_counts_max = (float)__HAL_TIM_GET_AUTORELOAD(&htim2);
    if (pwm_counts_max <= 0.0f) {
        pwm_counts_max = 1.0f;
    }
    charger.pwm_counts_max = pwm_counts_max;
    charger.duty_min_counts = (charger.cfg.duty_min / 100.0f) * pwm_counts_max;
    charger.duty_max_counts = (charger.cfg.duty_max / 100.0f) * pwm_counts_max;
    if (charger.duty_min_counts > charger.duty_max_counts) {
        charger.duty_min_counts = charger.duty_max_counts;
    }

    pi_controller_init(&charger.current_pi,
                       cfg.current_kp,
                       cfg.current_ki,
                       cfg.integral_limit,
                       charger.duty_min_counts,
                       charger.duty_max_counts);
    pi_controller_init(&charger.voltage_pi,
                       cfg.voltage_kp,
                       cfg.voltage_ki,
                       cfg.integral_limit,
                       charger.duty_min_counts,
                       charger.duty_max_counts);

    charger.enabled = 0;
    charger.state = CHARGER_STATE_IDLE;
    charger.target_voltage = 0.0f;
    charger.target_current = 0.0f;
    charger.duty = 0.0f;
    charger.last_tick = HAL_GetTick();
    charger.termination_timer = 0;
    charger_apply_pwm(0.0f);
    charger_fault_clear_all();
}

void charger_set_targets(float target_voltage, float target_current) {
    charger.target_voltage = target_voltage;
    charger.target_current = target_current;
    charger.current_pi.integral = 0.0f;
    charger.voltage_pi.integral = 0.0f;
}

void charger_enable(void) {
    if (charger.target_current <= 0.0f || charger.target_voltage <= 0.0f) {
        return;
    }
    charger.enabled = 1;
    charger.state = CHARGER_STATE_CC;
    charger.duty = 250.0f;  // Start from 250 counts for faster initial ramp
        charger.fast_mode = 1;  // Start in fast mode for quick ramp-up
        charger.filtered_current = 0.0f;  // Reset filtered current
    charger.current_pi.integral = 0.0f;
    charger.voltage_pi.integral = 0.0f;
    charger.last_tick = HAL_GetTick();
    charger.termination_timer = 0;
    charger_apply_pwm(charger.duty);
}

void charger_disable_with_reason(const char *reason) {
    charger_disable_internal(reason);
}

static void charger_disable_internal(const char *reason) {
    ChargerState prev_state = charger.state;
    uint8_t was_enabled = charger.enabled;
    charger.enabled = 0;
    charger.state = CHARGER_STATE_IDLE;
    charger.duty = 0.0f;
    charger.current_pi.integral = 0.0f;
    charger.voltage_pi.integral = 0.0f;
    charger_apply_pwm(0.0f);
    if (was_enabled) {
        if (reason && reason[0] != '\0') {
            // log disabled
        } else {
            // log disabled
        }
        charger_log_state_change(prev_state, charger.state, reason);
    }
}

void charger_update(const VoltageValues *meas) {
    if (!charger.enabled || charger.state == CHARGER_STATE_IDLE) {
        return;
    }
    if (meas == NULL) {
        return;
    }
    if (charger.target_voltage <= 0.0f || charger.target_current <= 0.0f) {
        charger_disable_with_reason("invalid targets");
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = charger.cfg.update_period_ms;
    if (charger.last_tick != 0) {
        elapsed_ms = now - charger.last_tick;
    }
    charger.last_tick = now;

    ChargerState prev_state = charger.state;
    const char *state_change_reason = NULL;

    // State transition: CC -> CV
    if (charger.state == CHARGER_STATE_CC &&
        meas->battery_voltage >= (charger.target_voltage - charger.cfg.voltage_hysteresis)) {
        charger.state = CHARGER_STATE_CV;
        state_change_reason = "voltage reached CV threshold";
    }

    // State transition: CV -> Complete
    if (charger.state == CHARGER_STATE_CV && charger.cfg.termination_current > 0.0f) {
        if (meas->current <= charger.cfg.termination_current) {
            charger.termination_timer += elapsed_ms;
            if (charger.termination_timer >= charger.cfg.termination_hold_ms) {
                charger.state = CHARGER_STATE_COMPLETE;
                state_change_reason = "termination current sustained";
            }
        } else {
            charger.termination_timer = 0;
        }
    }

    // --- FAST RAMP TO OPERATING POINT, THEN 1-COUNT STEPS ---
    const float DUTY_STEP_SLOW = 1.0f;    // 1 count per update near operating point
    const float DUTY_STEP_FAST = 40.0f;   // fast ramp-up before reaching target
    const float CURRENT_WINDOW = 0.5f;   // "operating point" window around target current
    const float dt_s = (float)elapsed_ms / 1000.0f;
    
    // Exponential moving average filter for current (reduce measurement noise)
    const float CURRENT_FILTER_ALPHA = 0.3f;  // 0.3 = heavy filtering
    charger.filtered_current = CURRENT_FILTER_ALPHA * meas->current + 
                               (1.0f - CURRENT_FILTER_ALPHA) * charger.filtered_current;
    float current = charger.filtered_current;  // Use filtered value for control

    // --- CELL OVERVOLTAGE PROTECTION ---
    // Check if any cell exceeds safe charging voltage and reduce current
    uint8_t cell_overvoltage_detected = 0;
    for (uint8_t i = 0; i < 6; i++) {
        if (meas->cell[i] > CELL_OVERVOLTAGE_LIMIT) {
            cell_overvoltage_detected = 1;
            charger_faults.overvoltage = true;
            #ifdef DEBUG_BALANCE
            // log disabled
            #endif
            break;
        }
    }

    switch (charger.state) {
        case CHARGER_STATE_CC:
            if (charger.cfg.control_mode == CHARGER_CTRL_PI) {
                charger.duty = pi_controller_update(&charger.current_pi,
                                                    charger.target_current,
                                                    current,
                                                    dt_s);
            } else {
                // Drive based on how far buck voltage is above the battery.
                float current_error = charger.target_current - current;
                float current_error_abs = fabsf(current_error);
                uint8_t at_operating_point = (current_error_abs <= CURRENT_WINDOW);

                // One-way: exit fast mode after initial convergence
                if (charger.fast_mode && at_operating_point) {
                    charger.fast_mode = 0;
                }

                // Fast ramp only while below target and before operating point
                if (current < charger.target_current - CURRENT_WINDOW) {
                    charger.duty += at_operating_point ? DUTY_STEP_SLOW : DUTY_STEP_FAST;
                }

                // Once at/above target, only adjust by 1 count per update
                if (current > charger.target_current + CURRENT_WINDOW) {
                    charger.duty -= DUTY_STEP_SLOW;
                    // Reduce duty if any cell overvoltage detected
                    if (cell_overvoltage_detected) {
                        charger.duty -= DUTY_STEP_SLOW;
                    }
                }
            }
            break;

        case CHARGER_STATE_CV:
            if (charger.cfg.control_mode == CHARGER_CTRL_PI) {
                charger.duty = pi_controller_update(&charger.voltage_pi,
                                                    charger.target_voltage,
                                                    meas->battery_voltage,
                                                    dt_s);
                // Safety: back off if current exceeds target
                if (meas->current > charger.target_current + 0.4f) {
                    charger.duty -= DUTY_STEP_SLOW;
                }
            } else {
                // In CV mode, adjust PWM to meet target_voltage
                if (meas->battery_voltage < charger.target_voltage) {
                    charger.duty += DUTY_STEP_SLOW;
                } else if (meas->battery_voltage > charger.target_voltage+0.2f) {
                    charger.duty -= DUTY_STEP_SLOW;
                }
                // Additionally, ensure we don't exceed the target current as a safety measure
                if (meas->current > charger.target_current+0.4f) {
                    // Reduce duty if any cell overvoltage detected
                    if (cell_overvoltage_detected) {
                        charger.duty -= DUTY_STEP_SLOW;  // Softer reduction to avoid oscillation
                    }
                    charger.duty -= DUTY_STEP_SLOW;
                }
            }
            break;

        case CHARGER_STATE_COMPLETE:
            charger_disable_with_reason("charge complete");
            return; // Exit 

        case CHARGER_STATE_FAULT:
            charger_disable_with_reason("charger fault");
            return; // Exit 

        case CHARGER_STATE_STORAGE:
            charger_storage_update(meas);
            break;
        case CHARGER_STATE_IDLE:
        default:
            // Should not happen if charger.enabled is true, but as a safeguard:
            charger_disable_with_reason("unexpected idle state");
            return;
    }

    // Clamp duty cycle to the min/max range
    if (charger.duty > charger.duty_max_counts) {
        charger.duty = charger.duty_max_counts;
    } else if (charger.duty < charger.duty_min_counts) {
        charger.duty = charger.duty_min_counts;
    }

    charger_apply_pwm(charger.duty);
    charger_log_state_change(prev_state, charger.state, state_change_reason);

}

ChargerState charger_get_state(void) {
    return charger.state;
}

void charger_enter_storage_mode(void) {
    if (charger.target_voltage <= 0.0f || charger.target_current <= 0.0f) {
        return;
    }
    if (!charger.enabled) {
        charger_enable();
    }
    ChargerState prev = charger.state;
    charger.state = CHARGER_STATE_STORAGE;
    charger_log_state_change(prev, charger.state, "enter storage mode");
}

float charger_get_pwm_duty(void) {
    if (charger.pwm_counts_max <= 0.0f) {
        return 0.0f;
    }
    return (charger.duty / charger.pwm_counts_max) * 100.0f;
}

float charger_get_pwm_counts_raw(void) {
    return charger.duty;
}

float charger_get_pwm_counts_max(void) {
    return charger.pwm_counts_max;
}

uint16_t charger_get_update_period_ms(void) {
    // Return 100ms in fast mode, configured period (300ms) in slow mode
    return charger.fast_mode ? 100 : charger.cfg.update_period_ms;
}

static void charger_apply_pwm(float duty_counts) {
    if (duty_counts < 0.0f) {
        duty_counts = 0.0f;
    }
    if (charger.pwm_counts_max <= 0.0f) {
        charger.pwm_counts_max = (float)__HAL_TIM_GET_AUTORELOAD(&htim2);
        if (charger.pwm_counts_max <= 0.0f) {
            charger.pwm_counts_max = 1.0f;
        }
    }
    float max_counts = charger.pwm_counts_max;
    if (duty_counts > max_counts) {
        duty_counts = max_counts;
    }
    __disable_irq();
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)duty_counts);
    __enable_irq();

}

// Logging disabled (JSON telemetry supersedes UART prints)
static void charger_log_state_change(ChargerState prev_state, ChargerState new_state, const char *reason) {
    if (prev_state == new_state) {
        return;
    }
    (void)reason;
}

#define STORAGE_BALANCE_KP           150.0f
#define STORAGE_BALANCE_DEADBAND_V    0.030f   // ignore deltas below 6 mV
#define STORAGE_BALANCE_MAX_DUTY      95U
#define STORAGE_CELL_TARGET_V         3.70f

static uint8_t storage_compute_duty(float delta_v) {
    if (delta_v <= 0.0f) {
        return 0U;
    }
    float duty = delta_v * STORAGE_BALANCE_KP;
    if (duty > (float)STORAGE_BALANCE_MAX_DUTY) {
        duty = (float)STORAGE_BALANCE_MAX_DUTY;
    }
    if (duty < 0.0f) {
        duty = 0.0f;
    }
    uint8_t duty_u8 = (uint8_t)duty;
    if (duty_u8 == 0U && duty > 0.0f) {
        duty_u8 = 1U;
    }
    return duty_u8;
}

static void charger_storage_update(const VoltageValues *meas) {
    if (meas == NULL) {
        balance_disable_all_cells();
        return;
    }

    const uint8_t cell_count = 6U;
    float lowest = meas->cell[0];
    float highest = meas->cell[0];
    for (uint8_t i = 1; i < cell_count; ++i) {
        if (meas->cell[i] < lowest) {
            lowest = meas->cell[i];
        }
        if (meas->cell[i] > highest) {
            highest = meas->cell[i];
        }
    }

    // When every cell meets the storage target, stop charging/balancing.
    // if (lowest >= STORAGE_CELL_TARGET_V) {
    //     balance_disable_all_cells();
    //     charger_disable_with_reason("storage target reached");
    //     return;
    // }

    // If cells are already even, make sure all bleed paths are off.
    if ((highest - lowest) < STORAGE_BALANCE_DEADBAND_V) {
        balance_disable_all_cells();
        return;
    }

    for (uint8_t i = 0; i < cell_count; ++i) {
        float delta = meas->cell[i] - lowest;
        if (delta <= STORAGE_BALANCE_DEADBAND_V) {
            disable_cell_balance(i);
            continue;
        }
        uint8_t duty = storage_compute_duty(delta - STORAGE_BALANCE_DEADBAND_V);
        if (duty == 0U) {
            disable_cell_balance(i);
        } else {
            enable_cell_balance(i, duty);
        }
    }
}


void CalculateFanRPM(int measurement_time_ms)
{
    (void)measurement_time_ms;  // Unused - we measure actual elapsed time
    static uint32_t last_fan_int_count = 0;
    static uint32_t last_measurement_tick = 0;
    
    uint32_t now = HAL_GetTick();
    uint32_t pulses = fan_int_count - last_fan_int_count;
    last_fan_int_count = fan_int_count;
    
    // Calculate actual elapsed time since last measurement
    uint32_t elapsed_ms = (last_measurement_tick == 0) ? 100 : (now - last_measurement_tick);
    last_measurement_tick = now;
    
    // Avoid division by zero
    if (elapsed_ms == 0) {
        elapsed_ms = 1;
    }
    
    // RPM = (pulses / pulses_per_rev) * (60000 ms/min / elapsed_ms)
    fan_rpm = (pulses * 60000) / (FAN_PULSES_PER_REV * elapsed_ms);
    charger_faults.fan_error = (fan_rpm < 1000U);

#ifdef FAN_DEBUG
    // log disabled
#endif
}
