#pragma once

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "param_types.h"
#include <stdio.h>
#include <string.h>
#include "main.h" // for huart1
typedef enum {
    ADS1115_PGA_6V144 = 0,  // ±6.144 V
    ADS1115_PGA_4V096 = 1,  // ±4.096 V
    ADS1115_PGA_2V048 = 2,  // ±2.048 V (default)
    ADS1115_PGA_1V024 = 3,  // ±1.024 V
    ADS1115_PGA_0V512 = 4,  // ±0.512 V
    ADS1115_PGA_0V256 = 5   // ±0.256 V (ranges 5-7 identical)
} ADS1115_PGA;

// ADS1115 channel mapping (address, channel) to voltage measurement
typedef struct {
    uint8_t i2c_addr;
    uint8_t channel;
    const char *name;
    
} ADS1115_ChannelConfig;

typedef enum {
    // ADS1115_ADDR (0x48)
    SHUNT_PLUS_VOLTAGE = 0,     // ADS1115_ADDR (0x48), AIN0
    BUCK_VOLTAGE = 1,           // ADS1115_ADDR (0x48), AIN1
    SHUNT_MINUS_VOLTAGE = 2,    // ADS1115_ADDR (0x48), AIN2
    VIN_36V = 3,                // ADS1115_ADDR (0x48), AIN3
    
    // ADS1115_ADDR_2 (0x49)
    CELL_1_VOLTAGE = 4,         // ADS1115_ADDR_2 (0x49), AIN0
    CELL_2_VOLTAGE = 5,         // ADS1115_ADDR_2 (0x49), AIN1
    CELL_3_VOLTAGE = 6,         // ADS1115_ADDR_2 (0x49), AIN2
    CELL_4_VOLTAGE = 7,         // ADS1115_ADDR_2 (0x49), AIN3
    
    // ADS1115_ADDR_3 (0x4A)
    CELL_5_VOLTAGE = 8,         // ADS1115_ADDR_3 (0x4A), AIN0
    CELL_6_VOLTAGE = 9          // ADS1115_ADDR_3 (0x4A), AIN1
} ADS1115_VoltageChannel;

// Helper function to get ADS1115 address and channel from voltage channel enum
static inline ADS1115_ChannelConfig get_ads1115_config(ADS1115_VoltageChannel ch) {
    ADS1115_ChannelConfig cfg = {0};
    
    switch (ch) {
        case SHUNT_PLUS_VOLTAGE:
            cfg.i2c_addr = 0x48; cfg.channel = 0; cfg.name = "SHUNT_PLUS";
            break;
        case BUCK_VOLTAGE:
            cfg.i2c_addr = 0x48; cfg.channel = 1; cfg.name = "BUCK";
            break;
        case SHUNT_MINUS_VOLTAGE:
            cfg.i2c_addr = 0x48; cfg.channel = 2; cfg.name = "SHUNT_MINUS";
            break;
        case VIN_36V:
            cfg.i2c_addr = 0x48; cfg.channel = 3; cfg.name = "VIN_36V";
            break;
        case CELL_1_VOLTAGE:
            cfg.i2c_addr = 0x49; cfg.channel = 0; cfg.name = "CELL_1";
            break;
        case CELL_2_VOLTAGE:
            cfg.i2c_addr = 0x49; cfg.channel = 1; cfg.name = "CELL_2";
            break;
        case CELL_3_VOLTAGE:
            cfg.i2c_addr = 0x49; cfg.channel = 2; cfg.name = "CELL_3";
            break;
        case CELL_4_VOLTAGE:
            cfg.i2c_addr = 0x49; cfg.channel = 3; cfg.name = "CELL_4";
            break;
        case CELL_5_VOLTAGE:
            cfg.i2c_addr = 0x4A; cfg.channel = 0; cfg.name = "CELL_5";
            break;
        case CELL_6_VOLTAGE:
            cfg.i2c_addr = 0x4A; cfg.channel = 1; cfg.name = "CELL_6";
            break;
        default:
            cfg.i2c_addr = 0xFF; cfg.channel = 0xFF; cfg.name = "UNKNOWN";
    }
    return cfg;
}
int16_t ads1115_read_voltage(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t channel, ADS1115_PGA pga);
VoltageValues ads1115_read_all_voltages(I2C_HandleTypeDef *hi2c);
