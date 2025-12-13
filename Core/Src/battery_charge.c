/*
 * battery_charge.c
 *
 * Closed loop control for the buck charger PWM.
 */

#include "battery_charge.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define FAN_PULSES_PER_REV 2 

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
} ChargerController;

static ChargerController charger = {0};
static uint32_t pwm_log_last_tick = 0;
static float pwm_log_last_duty = -1.0f;

static void charger_apply_pwm(float duty_counts);
static void charger_log(const char *fmt, ...);
static const char *charger_state_to_string(ChargerState state);
static void charger_log_state_change(ChargerState prev_state, ChargerState new_state, const char *reason);
static void charger_disable_internal(const char *reason);

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
    charger_log("[CHARGER] Fault flags cleared\r\n");
}

bool charger_fault_any(void) {
    return charger_faults.overvoltage ||
           charger_faults.overcurrent ||
           charger_faults.timeout ||
           charger_faults.cell_imbalance ||
           charger_faults.battery_not_present ||
           charger_faults.temp_high ||
           charger_faults.temp_low ||
           charger_faults.fan_error ||
           charger_faults.unknown;
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

    int voltage_centi = (int)lroundf(charger.target_voltage * 100.0f);
    int current_centi = (int)lroundf(charger.target_current * 100.0f);
    charger_log("[CHARGER] Targets set: %d.%02d V, %d.%02d A\r\n",
                voltage_centi / 100, abs(voltage_centi % 100),
                current_centi / 100, abs(current_centi % 100));
}

void charger_enable(void) {
    if (charger.target_current <= 0.0f || charger.target_voltage <= 0.0f) {
        return;
    }
    charger.enabled = 1;
    charger.state = CHARGER_STATE_CC;
    charger.duty = charger.duty_min_counts;
    charger.current_pi.integral = 0.0f;
    charger.voltage_pi.integral = 0.0f;
    charger.last_tick = HAL_GetTick();
    charger.termination_timer = 0;
    charger_apply_pwm(charger.duty);

    int voltage_centi = (int)lroundf(charger.target_voltage * 100.0f);
    int current_centi = (int)lroundf(charger.target_current * 100.0f);
    charger_log("[CHARGER] Enabled: start duty %.0f (targets %d.%02d V, %d.%02d A)\r\n",
                charger.duty,
                voltage_centi / 100, abs(voltage_centi % 100),
                current_centi / 100, abs(current_centi % 100));
}

void charger_disable(void) {
    charger_disable_internal(NULL);
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
            charger_log("[CHARGER] Disabled (%s)\r\n", reason);
        } else {
            charger_log("[CHARGER] Disabled\r\n");
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

    // --- VERY SIMPLE HYSTERESIS CONTROL ---
    const float DUTY_STEP = 1.0f;        // Standard step for fine adjustments
    const float RAMP_UP_STEP = 40.0f;    // Large step for initial fast ramp-up
    const float VOLTAGE_SLOW_GAP = 0.25f; // Slow ramp when buck almost equals battery
    const float VOLTAGE_MED_GAP  = 1.0f;  // Medium ramp threshold

    switch (charger.state) {
        case CHARGER_STATE_CC:
            // Drive based on how far buck voltage is above the battery.
            {
                float voltage_gap = meas->buck_voltage - meas->battery_voltage;
                float step = RAMP_UP_STEP;
                if (voltage_gap <= VOLTAGE_SLOW_GAP) {
                    step = DUTY_STEP; // almost equal -> crawl
                } else if (voltage_gap <= VOLTAGE_MED_GAP) {
                    step = 5.0f * DUTY_STEP; // moderate gap -> medium ramp
                }
                if (charger.duty >= 320.0f && step > DUTY_STEP) {
                    step = DUTY_STEP; // beyond threshold only fine adjustments
                }
                charger.duty += step;
            }
            break;

        case CHARGER_STATE_CV:
            // In CV mode, adjust PWM to meet target_voltage
            if (meas->battery_voltage < charger.target_voltage) {
                charger.duty += DUTY_STEP;
            } else {
                charger.duty -= DUTY_STEP;
            }
            // Additionally, ensure we don't exceed the target current as a safety measure
            if (meas->current > charger.target_current) {
                charger.duty -= DUTY_STEP;
            }
            break;

        case CHARGER_STATE_COMPLETE:
            charger_disable_with_reason("charge complete");
            return; // Exit

        case CHARGER_STATE_FAULT:
            charger_disable_with_reason("charger fault");
            return; // Exit

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

    // --- PWM Value Logging ---
    uint32_t now_tick = HAL_GetTick();
    // Log if the value has changed or if 1 second has passed
    if (fabsf(charger.duty - pwm_log_last_duty) >= 1.0f || (now_tick - pwm_log_last_tick) >= 1000) {
        if (charger.pwm_counts_max > 0.0f) {
            uint32_t duty_counts_int = (uint32_t)lroundf(charger.duty);
            
            // Calculate percentage using integer math to avoid float printing issues
            uint32_t percent_times_100 = (uint32_t)((charger.duty / charger.pwm_counts_max) * 10000.0f);
            uint32_t percent_int = percent_times_100 / 100;
            uint32_t percent_frac = percent_times_100 % 100;

            charger_log("[CHARGER] PWM Duty: %lu/%lu (%lu.%02lu%%)\r\n",
                        (unsigned long)duty_counts_int,
                        (unsigned long)lroundf(charger.pwm_counts_max),
                        (unsigned long)percent_int,
                        (unsigned long)percent_frac);

            pwm_log_last_tick = now_tick;
            pwm_log_last_duty = charger.duty;
        }
    }
}

ChargerState charger_get_state(void) {
    return charger.state;
}

float charger_get_pwm_duty(void) {
    if (charger.pwm_counts_max <= 0.0f) {
        return 0.0f;
    }
    return (charger.duty / charger.pwm_counts_max) * 100.0f;
}

uint16_t charger_get_update_period_ms(void) {
    return charger.cfg.update_period_ms;
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

static void charger_log(const char *fmt, ...) {
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (len <= 0) {
        return;
    }
    if (len > (int)sizeof(buffer)) {
        len = sizeof(buffer);
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

static const char *charger_state_to_string(ChargerState state) {
    switch (state) {
        case CHARGER_STATE_IDLE:
            return "IDLE";
        case CHARGER_STATE_CC:
            return "CC";
        case CHARGER_STATE_CV:
            return "CV";
        case CHARGER_STATE_COMPLETE:
            return "COMPLETE";
        case CHARGER_STATE_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

static void charger_log_state_change(ChargerState prev_state, ChargerState new_state, const char *reason) {
    if (prev_state == new_state) {
        return;
    }
    const char *from = charger_state_to_string(prev_state);
    const char *to = charger_state_to_string(new_state);
    if (reason && reason[0] != '\0') {
        charger_log("[CHARGER] State %s -> %s (%s)\r\n", from, to, reason);
    } else {
        charger_log("[CHARGER] State %s -> %s\r\n", from, to);
    }
}


void CalculateFanRPM(void)
{
    static uint32_t last_fan_int_count = 0;
    uint32_t pulses = fan_int_count - last_fan_int_count;
    last_fan_int_count = fan_int_count;

    // If called every 1 second:
    fan_rpm = (pulses / FAN_PULSES_PER_REV) * 60; // measurement_time = 1s
    charger_faults.fan_error = (fan_rpm < 1000U);
}
