#ifndef PARAM_TYPES_H
#define PARAM_TYPES_H
#include <stm32f4xx_hal.h>
typedef struct {
    uint8_t can_id;          // 1 byte
    uint16_t Kp;             // 2 bytes
    float enable_thresh;   // 4 bytes
    float disable_thresh;  // 4 bytes
    uint8_t duty_max;      // 1 byte
    uint16_t min_on_ms;    // 2 bytes
    float storage_volt;    // 4 bytes
} FlashParams;



typedef struct {
    float cell[6];         // CELL1–CELL6 voltages
    int16_t cell_raw[6];   // raw ADC codes for each cell
    float battery_voltage; // Battery voltage
    float shunt_voltage;   // Shunt voltage
    float buck_voltage;    // BUCK converter voltage
    float current;         // Battery current
} VoltageValues;

typedef struct {
    TIM_HandleTypeDef *htim;  // wskaźnik na timer, np. &htim3 lub &htim4
    uint32_t channel;         // TIM_CHANNEL_1 .. TIM_CHANNEL_4
} CellPwmConfig;

#endif // PARAM_TYPES_H
