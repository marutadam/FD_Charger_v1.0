/*
 * battery_balance.h
 *
 * Simple battery cell balancing interface
 */

#ifndef BATTERY_BALANCE_H
#define BATTERY_BALANCE_H

// #include "stm32f4xx_hal_tim.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "param_types.h"
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4; 


/**
 * Balance a single cell if it sits above the pack's lowest voltage plus a deadband.
 * The deadband parameter defines how far above the lowest cell we allow the others.
 */
void balance_cell(uint8_t cell_index, float *cell_voltages, float deadband);

/**
 * Balance every cell by first finding the lowest voltage cell and then
 * bleeding any higher cells down toward (lowest + deadband).
 */
 void balance_all_cells(float *cell_voltages, uint8_t num_cells, float deadband);

void balance_controller_configure(float kp,
                                  float enable_thresh,
                                  float disable_thresh,
                                  uint16_t min_on_ms,
                                  uint8_t duty_max);

void enable_cell_balance(uint8_t cell_index, uint8_t duty_percent);
void disable_cell_balance(uint8_t cell_index);
void balance_disable_all_cells(void);
uint8_t balance_get_last_duty(uint8_t cell_index);
void StartStorageMode(void);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_BALANCE_H
