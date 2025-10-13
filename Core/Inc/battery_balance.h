/*
 * battery_balance.h
 *
 * Simple battery cell balancing interface
 */

#ifndef BATTERY_BALANCE_H
#define BATTERY_BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * Balance a single cell if its voltage exceeds a threshold.
 */
void balance_cell(uint8_t cell_index, float *cell_voltages, float threshold);

/**
 * Balance all cells by checking each cell's voltage against threshold.
 */
void balance_all_cells(float *cell_voltages, uint8_t num_cells, float threshold);

/** Configuration for the P regulator with hysteresis and min on-time */
typedef struct {
    float Kp;              // proportional gain (duty % per volt)
    float enable_thresh;   // volts above target to start balancing
    float disable_thresh;  // volts above target to stop balancing (hysteresis)
    uint32_t min_on_ms;    // minimum on time in milliseconds
    uint8_t duty_max;      // maximum duty percent (0-100)
} BalanceControllerCfg;

void balance_controller_init(uint8_t cell_index, BalanceControllerCfg cfg);
void balance_controller_update(uint8_t cell_index, float cell_voltage, float target_voltage);

void enable_cell_balance(uint8_t cell_index, uint8_t duty_percent);
void disable_cell_balance(uint8_t cell_index);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_BALANCE_H
