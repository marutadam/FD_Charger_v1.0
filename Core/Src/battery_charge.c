/*
 * battery_charge.c
 *
 * Closed loop control for the buck charger PWM.
 */

#include "battery_charge.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


#define FAN_PULSES_PER_REV 2 

extern TIM_HandleTypeDef htim2;

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

static void charger_apply_pwm(float duty_counts);

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
}

void charger_disable(void) {
    charger.enabled = 0;
    charger.state = CHARGER_STATE_IDLE;
    charger.duty = 0.0f;
    charger.current_pi.integral = 0.0f;
    charger.voltage_pi.integral = 0.0f;
    charger_apply_pwm(0.0f);
}



void charger_update(const VoltageValues *meas) {
    if (!charger.enabled || charger.state == CHARGER_STATE_IDLE) {
        return;
    }
    if (meas == NULL) {
        return;
    }
    if (charger.target_voltage <= 0.0f || charger.target_current <= 0.0f) {
        charger_disable();
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t prev_tick = charger.last_tick;
    charger.last_tick = now;

    uint32_t elapsed_ms = charger.cfg.update_period_ms;
    if (prev_tick != 0) {
        elapsed_ms = now - prev_tick;
    }

    float dt = elapsed_ms / 1000.0f;
    if (dt <= 0.0f) {
        dt = charger.cfg.update_period_ms / 1000.0f;
    }
    if (dt > 0.5f) {
        dt = 0.5f;
    }

    bool overvoltage_fault = false;
    float max_cell_voltage = 0.0f;
    for (size_t i = 0; i < (sizeof(meas->cell) / sizeof(meas->cell[0])); ++i) {
        if (meas->cell[i] > max_cell_voltage) {
            max_cell_voltage = meas->cell[i];
        }
    }

    if (charger.cfg.cell_overvoltage_limit > 0.0f &&
        max_cell_voltage >= charger.cfg.cell_overvoltage_limit) {
        overvoltage_fault = true;
    }

    float current_derate = 1.0f;
    if (max_cell_voltage > 4.10f) {
        current_derate = (4.20f - max_cell_voltage) / (4.20f - 4.10f);
        if (current_derate < 0.0f) {
            current_derate = 0.0f;
        } else if (current_derate > 1.0f) {
            current_derate = 1.0f;
        }
    }
    charger_faults.overvoltage = overvoltage_fault;

    if (overvoltage_fault) {
        charger.enabled = 0;
        charger.state = CHARGER_STATE_FAULT;
        charger.duty = 0.0f;
        charger_apply_pwm(0.0f);
        return;
    }

    if (charger.state == CHARGER_STATE_CC &&
        charger.target_voltage > 0.0f &&
        meas->battery_voltage >= (charger.target_voltage - charger.cfg.voltage_hysteresis)) {
        charger.state = CHARGER_STATE_CV;
        charger.voltage_pi.integral = 0.0f;
    }

    if (charger.state == CHARGER_STATE_CV &&
        charger.cfg.termination_current > 0.0f &&
        charger.cfg.termination_hold_ms > 0U) {
        if (meas->current <= charger.cfg.termination_current) {
            if (charger.termination_timer < charger.cfg.termination_hold_ms) {
                charger.termination_timer += elapsed_ms;
            }
            if (charger.termination_timer >= charger.cfg.termination_hold_ms) {
                charger.state = CHARGER_STATE_COMPLETE;
                charger.enabled = 0;
                charger.duty = 0.0f;
                charger_apply_pwm(0.0f);
                return;
            }
        } else {
            charger.termination_timer = 0;
        }
    } else if (charger.state != CHARGER_STATE_CV) {
        charger.termination_timer = 0;
    }

    float effective_current_target = charger.target_current * current_derate;

    float duty_cmd = charger.duty;
    if (charger.state == CHARGER_STATE_CC) {
        duty_cmd = pi_controller_update(&charger.current_pi, effective_current_target, meas->current, dt);
    } else if (charger.state == CHARGER_STATE_CV) {
        duty_cmd = pi_controller_update(&charger.voltage_pi, charger.target_voltage, meas->battery_voltage, dt);
    } else if (charger.state == CHARGER_STATE_COMPLETE || charger.state == CHARGER_STATE_FAULT) {
        charger.enabled = 0;
        charger.duty = 0.0f;
        charger_apply_pwm(0.0f);
        return;
    }

    charger.duty = duty_cmd;

    charger_apply_pwm(charger.duty);
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


void CalculateFanRPM(void)
{
    static uint32_t last_fan_int_count = 0;
    uint32_t pulses = fan_int_count - last_fan_int_count;
    last_fan_int_count = fan_int_count;

    // If called every 1 second:
    fan_rpm = (pulses / FAN_PULSES_PER_REV) * 60; // measurement_time = 1s
    charger_faults.fan_error = (fan_rpm < 1000U);
    

}
