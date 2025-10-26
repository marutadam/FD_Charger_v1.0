#pragma once

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "param_types.h"
#include <stdio.h>
#include <string.h>
#include "main.h" // for huart1
typedef enum {
    CELL_1 = 4,
    CELL_2 = 5,
    CELL_3 = 2,
    CELL_4 = 1,
    CELL_5 = 0,
    CELL_6 = 3
} MuxChannelName;

typedef enum {
    ADS1115_PGA_6V144 = 0,  // ±6.144 V
    ADS1115_PGA_4V096 = 1,  // ±4.096 V
    ADS1115_PGA_2V048 = 2,  // ±2.048 V (default)
    ADS1115_PGA_1V024 = 3,  // ±1.024 V
    ADS1115_PGA_0V512 = 4,  // ±0.512 V
    ADS1115_PGA_0V256 = 5   // ±0.256 V (ranges 5-7 identical)
} ADS1115_PGA;

void mux_set_channel(uint8_t channel);
int16_t ads1115_read_voltage(I2C_HandleTypeDef *hi2c, uint8_t channel, ADS1115_PGA pga);
VoltageValues ads1115_read_all_voltages(I2C_HandleTypeDef *hi2c);
void print_all_voltages_uart(const VoltageValues *values);
