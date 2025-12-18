/*
 * battery_balance.c
 *
 * Functions for battery cell balancing.
 *
 * Balancing strategy overview:
 *   1. The pack's lowest cell voltage is sampled each cycle.
 *   2. A target voltage is derived as (lowest + deadband).
 *   3. Each cell controller looks at its delta above that target.
 *   4. When the delta exceeds enable_thresh the bleed MOSFET is PWM'd with
 *      a duty proportional to the delta (clamped at duty_max).
 *   5. Balancing stops once the delta falls below disable_thresh for at least
 *      min_on_ms, giving each channel a small hysteresis window.
 */
#include "battery_balance.h"
#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
CellPwmConfig battery_cell[6]={
    {&htim3, TIM_CHANNEL_3}, // CELL1 -> TIM3_CH1
    {&htim3, TIM_CHANNEL_4}, // CELL2 -> TIM3_CH2
    {&htim3, TIM_CHANNEL_1}, // CELL3 -> TIM3_CH3
    {&htim3, TIM_CHANNEL_2}, // CELL4 -> TIM3_CH4
    {&htim4, TIM_CHANNEL_1}, // CELL5 -> TIM4_CH1
    {&htim4, TIM_CHANNEL_2}  // CELL6 -> TIM4_CH2
}; // Configuration for PWM timers/channels for each cell
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

static void remember_duty(uint8_t cell_index, uint8_t duty_percent);

void enable_cell_balance(uint8_t cell_index, uint8_t duty_percent) {
    uint32_t channel;
    TIM_HandleTypeDef *htim = cell_timer(cell_index, &channel);

    if (htim == NULL) return;

    // Calculate compare (pulse) value from duty percent and timer ARR
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t pulse = (duty_percent * (arr + 1)) / 100;

    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
    remember_duty(cell_index, duty_percent);
}

void disable_cell_balance(uint8_t cell_index) {
 enable_cell_balance(cell_index, 0);
}

static float lowest_cell_voltage(const float *cell_voltages, uint8_t num_cells) {
    if (cell_voltages == NULL || num_cells == 0) {
        return 0.0f;
    }
    float lowest = cell_voltages[0];
    for (uint8_t i = 1; i < num_cells; ++i) {
        if (cell_voltages[i] < lowest) {
            lowest = cell_voltages[i];
        }
    }
    return lowest;
}

// --- Controller state -------------------------------------------------
#define MAX_CELLS 6
static uint8_t cell_last_duty[MAX_CELLS];
typedef struct {
    uint8_t enabled; // 0 = off, 1 = on
    uint32_t last_on_ts; // HAL_GetTick() timestamp when turned on
} CellController;

static CellController controllers[MAX_CELLS];
typedef struct {
    float kp;
    float enable_thresh;
    float disable_thresh;
    uint16_t min_on_ms;
    uint8_t duty_max;
} BalanceConfig;

static BalanceConfig balance_cfg = {
    .kp = 900.0f,
    .enable_thresh = 0.03f,
    .disable_thresh = 0.01f,
    .min_on_ms = 250U,
    .duty_max = 60U
};

static void balance_reset_state(void) {
    for (uint8_t cell = 0; cell < MAX_CELLS; ++cell) {
        controllers[cell].enabled = 0;
        controllers[cell].last_on_ts = 0;
        disable_cell_balance(cell);
        cell_last_duty[cell] = 0;
    }
}

void balance_controller_configure(float kp,
                                  float enable_thresh,
                                  float disable_thresh,
                                  uint16_t min_on_ms,
                                  uint8_t duty_max)
{
    balance_cfg.kp = (kp < 0.0f) ? 0.0f : kp;
    balance_cfg.enable_thresh = enable_thresh;
    balance_cfg.disable_thresh = disable_thresh;
    balance_cfg.min_on_ms = min_on_ms;
    balance_cfg.duty_max = duty_max;

    if (balance_cfg.duty_max == 0U) {
        balance_cfg.duty_max = 60U;
    }
    if (balance_cfg.enable_thresh < balance_cfg.disable_thresh) {
        balance_cfg.enable_thresh = balance_cfg.disable_thresh;
    }

    balance_reset_state();
}

static uint8_t compute_simple_duty(float delta_v) {
    if (delta_v <= 0.0f) {
        return 0;
    }
    float duty = balance_cfg.duty_max;
    if (balance_cfg.kp > 0.0f) {
        duty = delta_v * balance_cfg.kp;
    }
    if (duty > balance_cfg.duty_max) {
        duty = balance_cfg.duty_max;
    }
    if (duty < 0.0f) {
        duty = 0.0f;
    }
    uint8_t duty_u8 = (uint8_t)duty;
    if (duty_u8 == 0 && duty > 0.0f) {
        duty_u8 = 1;
    }
    return duty_u8;
}

void balance_controller_update(uint8_t cell_index, float cell_voltage, float target_voltage) {
    if (cell_index >= MAX_CELLS) return;
    CellController *c = &controllers[cell_index];
    float delta_v = cell_voltage - target_voltage;
    uint32_t now = HAL_GetTick();

    if (!c->enabled) {
        if (delta_v < balance_cfg.enable_thresh) {
            return;
        }
        c->enabled = 1;
        c->last_on_ts = now;
    } else {
        uint32_t on_elapsed = now - c->last_on_ts;
        if (delta_v <= balance_cfg.disable_thresh && on_elapsed >= balance_cfg.min_on_ms) {
            disable_cell_balance(cell_index);
            c->enabled = 0;
            return;
        }
    }

    uint8_t duty = compute_simple_duty(delta_v);
    if (duty == 0) {
        // No meaningful duty requested; make sure the channel is off.
        disable_cell_balance(cell_index);
        c->enabled = 0;
        return;
    }

    enable_cell_balance(cell_index, duty);
}

void balance_disable_all_cells(void) {
    balance_reset_state();
}

static void remember_duty(uint8_t cell_index, uint8_t duty_percent) {
    if (cell_index < MAX_CELLS) {
        cell_last_duty[cell_index] = duty_percent;
    }
}

uint8_t balance_get_last_duty(uint8_t cell_index) {
    if (cell_index >= MAX_CELLS) {
        return 0;
    }
    return cell_last_duty[cell_index];
}

void balance_cell(uint8_t cell_index, float *cell_voltages, float deadband) {
    if (cell_index >= MAX_CELLS || cell_voltages == NULL) {
        return;
    }
    float reference = lowest_cell_voltage(cell_voltages, MAX_CELLS) + deadband;
    balance_controller_update(cell_index, cell_voltages[cell_index], reference);
}

void balance_all_cells(float *cell_voltages, uint8_t num_cells, float deadband) {
    if (cell_voltages == NULL || num_cells == 0) {
        return;
    }
    if (num_cells > MAX_CELLS) {
        num_cells = MAX_CELLS;
    }
    float reference = lowest_cell_voltage(cell_voltages, num_cells) + deadband;
    for (uint8_t i = 0; i < num_cells; ++i) {
        balance_controller_update(i, cell_voltages[i], reference);
    }
}

void StartStorageMode() {
    // Prepare hardware state for storage charging/balancing.
    balance_disable_all_cells();
    charger_enter_storage_mode();
}
