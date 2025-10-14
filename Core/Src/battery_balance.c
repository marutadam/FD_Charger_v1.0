/*
 * battery_balance.c
 *
 * Functions for battery cell balancing.
 */
#include "battery_balance.h"
#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// Example: Balance a single cell if voltage exceeds threshold
void balance_cell(uint8_t cell_index, float *cell_voltages, float threshold) {
    if (cell_voltages[cell_index] > threshold) {
        // Add hardware-specific balancing logic here
        // For example, enable balancing circuit for cell_index
    }
}

// Example: Balance all cells
void balance_all_cells(float *cell_voltages, uint8_t num_cells, float threshold) {
    for (uint8_t i = 0; i < num_cells; ++i) {
        balance_cell(i, cell_voltages, threshold);
    }
}

// Map cell index to timer handle and channel
static TIM_HandleTypeDef* cell_timer(uint8_t cell_index, uint32_t *channel) {
    switch(cell_index) {
        case 0: *channel = TIM_CHANNEL_1; return &htim3; // CELL1 -> TIM3_CH1
        case 1: *channel = TIM_CHANNEL_2; return &htim3; // CELL2 -> TIM3_CH2
        case 2: *channel = TIM_CHANNEL_3; return &htim3; // CELL3 -> TIM3_CH3
        case 3: *channel = TIM_CHANNEL_4; return &htim3; // CELL4 -> TIM3_CH4
        case 4: *channel = TIM_CHANNEL_1; return &htim4; // CELL5 -> TIM4_CH1
        case 5: *channel = TIM_CHANNEL_2; return &htim4; // CELL6 -> TIM4_CH2
        default: *channel = 0; return NULL;
    }
}

void enable_cell_balance(uint8_t cell_index, uint8_t duty_percent) {
    uint32_t channel;
    TIM_HandleTypeDef *htim = cell_timer(cell_index, &channel);
    
    if (htim == NULL) return;

    // Calculate compare (pulse) value from duty percent and timer ARR
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t pulse = (duty_percent * (arr + 1)) / 100;

    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void disable_cell_balance(uint8_t cell_index) {
    uint32_t channel;
    TIM_HandleTypeDef *htim = cell_timer(cell_index, &channel);
    if (htim == NULL) return;

    // Don't stop the timer; set compare to 0 to effectively disable output
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
}

// --- Controller state -------------------------------------------------
#define MAX_CELLS 6
typedef struct {
    BalanceControllerCfg cfg;
    uint8_t enabled; // 0 = off, 1 = on
    uint32_t last_on_ts; // HAL_GetTick() timestamp when turned on
} CellController;

static CellController controllers[MAX_CELLS];

void balance_controller_init(uint8_t cell_index, BalanceControllerCfg cfg) {
    if (cell_index >= MAX_CELLS) return;
    controllers[cell_index].cfg = cfg;
    controllers[cell_index].enabled = 0;
    controllers[cell_index].last_on_ts = 0;
}

void balance_controller_update(uint8_t cell_index, float cell_voltage, float target_voltage) {
    if (cell_index >= MAX_CELLS) return;
    CellController *c = &controllers[cell_index];
    float err = cell_voltage - target_voltage;

    if (!c->enabled) {
        // Check enable threshold
        if (err >= c->cfg.enable_thresh) {
            // compute duty
            float duty_f = c->cfg.Kp * err;
            if (duty_f < 0) duty_f = 0;
            if (duty_f > c->cfg.duty_max) duty_f = c->cfg.duty_max;
            enable_cell_balance(cell_index, (uint8_t)duty_f);
            c->enabled = 1;
            c->last_on_ts = HAL_GetTick();
        }
    } else {
        // currently balancing: maintain until disable threshold or minimum on time passed
        uint32_t now = HAL_GetTick();
        uint32_t on_elapsed = now - c->last_on_ts;

        if (err <= c->cfg.disable_thresh && on_elapsed >= c->cfg.min_on_ms) {
            disable_cell_balance(cell_index);
            c->enabled = 0;
        } else {
            // update duty while on
            float duty_f = c->cfg.Kp * err;
            if (duty_f < 0) duty_f = 0;
            if (duty_f > c->cfg.duty_max) duty_f = c->cfg.duty_max;
            enable_cell_balance(cell_index, (uint8_t)duty_f);
        }
    }
}
