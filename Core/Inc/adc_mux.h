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

void mux_set_channel(uint8_t channel);
int16_t ads1115_read_voltage(I2C_HandleTypeDef *hi2c, uint8_t channel);
VoltageValues ads1115_read_all_voltages(I2C_HandleTypeDef *hi2c);
void print_all_voltages_uart(const VoltageValues *values);
