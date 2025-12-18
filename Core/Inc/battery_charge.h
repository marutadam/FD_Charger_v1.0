/*
 * battery_charge.h
 *
 * Control interface for the buck-converter charger.
 */

#ifndef BATTERY_CHARGE_H
#define BATTERY_CHARGE_H

#include "stm32f4xx_hal.h"
#include "adc_mux.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// PI Controller structure
typedef struct {
    float kp;
    float ki;
    float integral_limit;
    float integral;
    float output_min;
    float output_max;
} PI_Controller;

typedef enum {
    CHARGER_STATE_IDLE = 0,
    CHARGER_STATE_CC = 1,
    CHARGER_STATE_CV = 2,
    CHARGER_STATE_COMPLETE = 3,
    CHARGER_STATE_FAULT = 4,
    CHARGER_STATE_STORAGE = 5,

} ChargerState;

typedef struct {
    bool overvoltage;
    bool overcurrent;
    bool timeout;
    bool cell_imbalance;
    bool battery_not_present;
    bool temp_high;
    bool temp_low;
    bool fan_error;
    bool unknown;
} ChargerFaultStatus;

typedef struct {
    float current_kp;            // proportional gain for current loop
    float current_ki;            // integral gain for current loop
    float voltage_kp;            // proportional gain for voltage loop
    float voltage_ki;            // integral gain for voltage loop
    float integral_limit;        // clamp for PI integrators (amp*sec or volt*sec)
    float duty_min;              // minimum duty cycle in percent when enabled
    float duty_max;              // maximum duty cycle in percent
    float voltage_hysteresis;    // hysteresis before transitioning CC->CV (volts)
    float termination_current;   // charge current threshold to finish (amps)
    uint16_t termination_hold_ms;// time below termination current before finish
    float cell_overvoltage_limit;// fault if any cell exceeds this voltage
    uint16_t update_period_ms;   // nominal control update period
} ChargerControllerCfg;

void charger_controller_init(ChargerControllerCfg cfg);
void charger_set_targets(float target_voltage, float target_current);
void charger_enable(void);
void charger_disable(void);
void charger_disable_with_reason(const char *reason);
void charger_update(const VoltageValues *meas);
void charger_fault_clear_all(void);
bool charger_fault_any(void);

// Force charger into storage mode (uses current targets)
void charger_enter_storage_mode(void);

ChargerState charger_get_state(void);
float charger_get_pwm_duty(void);
uint16_t charger_get_update_period_ms(void);
void CalculateFanRPM(int measurement_time_ms);


// Function prototypes
void pi_controller_init(PI_Controller *controller, float kp, float ki, float integral_limit, float output_min, float output_max);
float pi_controller_update(PI_Controller *controller, float setpoint, float measurement, float dt);

// Fan RPM measurement variables defined in main.c
extern volatile uint32_t fan_int_count;
extern volatile uint32_t fan_rpm;
extern volatile ChargerFaultStatus charger_faults;
extern volatile float end_voltage;
extern volatile float end_voltage_storage;
extern volatile float set_current;

#ifdef __cplusplus
}
#endif

#endif // BATTERY_CHARGE_H
