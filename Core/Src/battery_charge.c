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

extern TIM_HandleTypeDef htim2;

typedef struct {
    ChargerControllerCfg cfg;
    uint8_t enabled;
    ChargerState state;
    float target_voltage;
    float target_current;
    float duty;
    float current_integrator;
    float voltage_integrator;
    uint32_t last_tick;
    uint32_t termination_timer;
} ChargerController;

static ChargerController charger = {0};

static void charger_apply_pwm(float duty_percent);
static void charger_reset_integrators(void);

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
    charger.enabled = 0;
    charger.state = CHARGER_STATE_IDLE;
    charger.target_voltage = 0.0f;
    charger.target_current = 0.0f;
    charger.duty = 0.0f;
    charger_reset_integrators();
    charger.last_tick = HAL_GetTick();
    charger.termination_timer = 0;
    charger_apply_pwm(0.0f);
    charger_fault_clear_all();
}

void charger_set_targets(float target_voltage, float target_current) {
    charger.target_voltage = target_voltage;
    charger.target_current = target_current;
    charger_reset_integrators();
}

void charger_enable(void) {
    if (charger.target_current <= 0.0f || charger.target_voltage <= 0.0f) {
        return;
    }
    charger.enabled = 1;
    charger.state = CHARGER_STATE_CC;
    charger.duty = charger.cfg.duty_min;
    charger_reset_integrators();
    charger.last_tick = HAL_GetTick();
    charger.termination_timer = 0;
    charger_apply_pwm(charger.duty);
}

void charger_disable(void) {
    charger.enabled = 0;
    charger.state = CHARGER_STATE_IDLE;
    charger.duty = 0.0f;
    charger_reset_integrators();
    charger_apply_pwm(0.0f);
}

void charger_update(const ChargerMeasurements *meas) {
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
    if (charger.cfg.cell_overvoltage_limit > 0.0f &&
        meas->max_cell_voltage >= charger.cfg.cell_overvoltage_limit) {
        overvoltage_fault = true;
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
        meas->pack_voltage >= (charger.target_voltage - charger.cfg.voltage_hysteresis)) {
        charger.state = CHARGER_STATE_CV;
        charger.voltage_integrator = 0.0f;
    }

    if (charger.state == CHARGER_STATE_CV &&
        charger.cfg.termination_current > 0.0f &&
        charger.cfg.termination_hold_ms > 0U) {
        if (meas->charge_current <= charger.cfg.termination_current) {
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

    float duty_cmd = charger.duty;
    if (charger.state == CHARGER_STATE_CC) {
        float err = charger.target_current - meas->charge_current;
        charger.current_integrator += err * dt;
        if (charger.cfg.integral_limit > 0.0f) {
            if (charger.current_integrator > charger.cfg.integral_limit) {
                charger.current_integrator = charger.cfg.integral_limit;
            } else if (charger.current_integrator < -charger.cfg.integral_limit) {
                charger.current_integrator = -charger.cfg.integral_limit;
            }
        }
        duty_cmd = (charger.cfg.current_kp * err) +
                   (charger.cfg.current_ki * charger.current_integrator);
    } else if (charger.state == CHARGER_STATE_CV) {
        float err = charger.target_voltage - meas->pack_voltage;
        charger.voltage_integrator += err * dt;
        if (charger.cfg.integral_limit > 0.0f) {
            if (charger.voltage_integrator > charger.cfg.integral_limit) {
                charger.voltage_integrator = charger.cfg.integral_limit;
            } else if (charger.voltage_integrator < -charger.cfg.integral_limit) {
                charger.voltage_integrator = -charger.cfg.integral_limit;
            }
        }
        duty_cmd = (charger.cfg.voltage_kp * err) +
                   (charger.cfg.voltage_ki * charger.voltage_integrator);
    } else if (charger.state == CHARGER_STATE_COMPLETE || charger.state == CHARGER_STATE_FAULT) {
        charger.enabled = 0;
        charger.duty = 0.0f;
        charger_apply_pwm(0.0f);
        return;
    }

    charger.duty = duty_cmd;
    if (charger.duty > charger.cfg.duty_max) {
        charger.duty = charger.cfg.duty_max;
    }
    if (charger.duty < 0.0f) {
        charger.duty = 0.0f;
        if (charger.state == CHARGER_STATE_CC) {
            charger.current_integrator = 0.0f;
        } else if (charger.state == CHARGER_STATE_CV) {
            charger.voltage_integrator = 0.0f;
        }
    } else if (charger.state == CHARGER_STATE_CC &&
               charger.duty > 0.0f &&
               charger.duty < charger.cfg.duty_min) {
        charger.duty = charger.cfg.duty_min;
    }

    charger_apply_pwm(charger.duty);
}

ChargerState charger_get_state(void) {
    return charger.state;
}

float charger_get_pwm_duty(void) {
    return charger.duty;
}

uint16_t charger_get_update_period_ms(void) {
    return charger.cfg.update_period_ms;
}

static void charger_apply_pwm(float duty_percent) {
    if (duty_percent < 0.0f) {
        duty_percent = 0.0f;
    }
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pulse = (uint32_t)((duty_percent * (arr + 1U)) / 100.0f);
    if (pulse > arr) {
        pulse = arr;
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

static void charger_reset_integrators(void) {
    charger.current_integrator = 0.0f;
    charger.voltage_integrator = 0.0f;
}

#define FAN_PULSES_PER_REV 2 // Check your fan's datasheet!

void CalculateFanRPM(void)
{
    static uint32_t last_fan_int_count = 0;
    uint32_t pulses = fan_int_count - last_fan_int_count;
    last_fan_int_count = fan_int_count;

    // If called every 1 second:
    fan_rpm = (pulses / FAN_PULSES_PER_REV) * 60; // measurement_time = 1s
    charger_faults.fan_error = (fan_rpm < 1000U);
    
    // Print to UART
    char msg[64];
    int len = snprintf(msg, sizeof(msg),
                       "[FAN] Speed: %lu RPM\r\nFault fan:%d any:%d\r\n",
                       fan_rpm,
                       charger_faults.fan_error ? 1 : 0,
                       charger_fault_any() ? 1 : 0);
    if (len < 0) {
        return;
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
