#include "adc_mux.h"
#include "cmsis_os2.h"
#include <math.h> // Add this include

#define ADS1115_ADDR 0x48 // Default I2C address
#define PARALLEL(r1, r2) (1.0f / (((1.0f) / (r1)) + ((1.0f) / (r2))))
#define DIVIDER_SCALE(rt, rb) (((rt) + (rb)) / (rb))

// Voltage divider scaling factors (top resistor, bottom resistor) per schematic.
// The effective bottom resistor is 10k for each channel; the 2.2k resistor on the
// common output is handled separately in hardware and is not treated as parallel here.
static const float cell_bottom_resistance = 10000.0f;
static const float cell_divider_scale[6] = {
    DIVIDER_SCALE(4300.0f, cell_bottom_resistance),
    DIVIDER_SCALE(18000.0f, cell_bottom_resistance),
    DIVIDER_SCALE(33000.0f, cell_bottom_resistance),
    DIVIDER_SCALE(47000.0f, cell_bottom_resistance),
    DIVIDER_SCALE(68000.0f, cell_bottom_resistance),
    DIVIDER_SCALE(82000.0f, cell_bottom_resistance)
};
// Optional per-cell calibration gains to compensate for hardware tolerances.
// Values of 1.0f leave the reading unchanged.
static const float cell_calibration_gain[6] = {
    0.995f, 0.997f, 0.996f, 0.978f, 1.032f, 0.984f
};

// AIN1 (buck) and AIN0/AIN2 (current sense high & battery) all use 91k/10k dividers
static const float buck_divider_scale = DIVIDER_SCALE(91000.0f, 10000.0f);
static const float battery_divider_scale = DIVIDER_SCALE(91000.0f, 10000.0f);
static const float current_divider_scale = DIVIDER_SCALE(91000.0f, 10000.0f);

static const float pga_full_scale_table[] = {
    6.144f,
    4.096f,
    2.048f,
    1.024f,
    0.512f,
    0.256f
};

static inline float ads1115_raw_to_voltage(int16_t raw, ADS1115_PGA pga)
{
    uint8_t idx = (uint8_t)pga;
    if (idx >= sizeof(pga_full_scale_table)/sizeof(pga_full_scale_table[0])) {
        idx = sizeof(pga_full_scale_table)/sizeof(pga_full_scale_table[0]) - 1;
    }
    float full_scale = pga_full_scale_table[idx];
    return (float)raw * full_scale / 32768.0f;
}

void mux_set_channel(uint8_t channel)
{
    // Only use lower 3 bits
    channel &= 0x07;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // A
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET); // B
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET); // C
}

int16_t ads1115_read_voltage(I2C_HandleTypeDef *hi2c, uint8_t channel, ADS1115_PGA pga)
{
    // Set config for single-shot, single-ended channel (AINx vs GND)
    uint16_t config = 0x8183;              // OS=1, PGA bits cleared, MODE=single-shot, 128 SPS
    config |= ((uint16_t)pga & 0x07) << 9; // apply PGA selection
    config &= ~0x7000;                     // clear MUX bits
    config |= 0x4000 | ((channel & 0x03) << 12); // select AINx single-ended (0b100 + channel)
    uint8_t config_bytes[3] = {0x01, config >> 8, config & 0xFF};
    HAL_I2C_Master_Transmit(hi2c, ADS1115_ADDR << 1, config_bytes, 3, HAL_MAX_DELAY);

    // Wait for conversion (minimum 8ms for 128SPS)
    HAL_Delay(10);

    // Read conversion result
    uint8_t reg = 0x00;
    uint8_t data[2];
    HAL_I2C_Master_Transmit(hi2c, ADS1115_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, ADS1115_ADDR << 1, data, 2, HAL_MAX_DELAY);

    int16_t raw = (data[0] << 8) | data[1];
    return raw;
}

static void readCells(I2C_HandleTypeDef *hi2c, float *cells, int16_t *rawCells)
{
    float node_voltage[6] = {0};
    const char *debug_header = "[ADC][DBG] ";
    const ADS1115_PGA cell_pga = ADS1115_PGA_4V096;
    for (uint8_t ch = 0; ch < 6; ch++) {
        switch (ch) {
            case 0: mux_set_channel(CELL_1); break;
            case 1: mux_set_channel(CELL_2); break;
            case 2: mux_set_channel(CELL_3); break;
            case 3: mux_set_channel(CELL_4); break;
            case 4: mux_set_channel(CELL_5); break;
            case 5: mux_set_channel(CELL_6); break;
            default: break;
        }
                osDelay(50);

        int16_t raw = ads1115_read_voltage(hi2c, 3, cell_pga);
        float sense_voltage = ads1115_raw_to_voltage(raw, cell_pga);
        node_voltage[ch] = sense_voltage * cell_divider_scale[ch];

        if (rawCells != NULL) {
            rawCells[ch] = raw;
        }

        if (raw == 32767) {
            char msg[64];
            int len = snprintf(msg, sizeof(msg), "%schannel=%u raw=32767\r\n", debug_header, ch);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
        }
    }

    float previous_node = 0.0f;
    for (uint8_t ch = 0; ch < 6; ch++) {
        float cell_voltage = node_voltage[ch] - previous_node;
        if (cell_voltage < 0.0f) {
            cell_voltage = 0.0f;
        }
        cells[ch] = cell_voltage * cell_calibration_gain[ch];
        previous_node = node_voltage[ch];
    }
}

static float readBatteryVoltage(I2C_HandleTypeDef *hi2c, float *divider_voltage)
{
    const ADS1115_PGA battery_pga = ADS1115_PGA_4V096;
    int16_t raw = ads1115_read_voltage(hi2c, 2, battery_pga);
    float sense_voltage = ads1115_raw_to_voltage(raw, battery_pga);
    if (divider_voltage != NULL) {
        *divider_voltage = sense_voltage;
    }
    return sense_voltage * battery_divider_scale;
}

static float readBuck(I2C_HandleTypeDef *hi2c)
{
    const ADS1115_PGA buck_pga = ADS1115_PGA_4V096;
    int16_t raw = ads1115_read_voltage(hi2c, 1, buck_pga);
    float sense_voltage = ads1115_raw_to_voltage(raw, buck_pga);
    return sense_voltage * buck_divider_scale;
}

static float readCurrent(I2C_HandleTypeDef *hi2c, float battery_divider_voltage)
{
    const ADS1115_PGA current_pga = ADS1115_PGA_4V096;
    int16_t raw_current = ads1115_read_voltage(hi2c, 0, current_pga);
    float current_high_voltage = ads1115_raw_to_voltage(raw_current, current_pga);
    float shunt_voltage = (current_high_voltage - battery_divider_voltage) * current_divider_scale;
    float shunt_resistance = 0.025f; // Effective shunt resistance (4x 0.1 ohm in parallel)
    float current_voltage = shunt_voltage;
    return current_voltage / shunt_resistance;
}

VoltageValues ads1115_read_all_voltages(I2C_HandleTypeDef *hi2c)
{
    VoltageValues values;
    int16_t rawCells[6] = {0};
    readCells(hi2c, values.cell, rawCells);
    float battery_divider_voltage = 0.0f;
    values.battery_voltage = readBatteryVoltage(hi2c, &battery_divider_voltage);
    values.buck_voltage = readBuck(hi2c);
    values.current = readCurrent(hi2c, battery_divider_voltage);
    for (int i = 0; i < 6; ++i) {
        values.cell_raw[i] = rawCells[i];
    }
    return values;
}

void print_all_voltages_uart(const VoltageValues *values)
{
    char msg[128];
    snprintf(msg, sizeof(msg), "[ADC] Current values:\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    for (int i = 0; i < 6; i++) {
        int int_part = (int)values->cell[i];
        int dec_part = (int)(fabsf((values->cell[i] - int_part) * 1000.0f) + 0.5f);
        snprintf(msg, sizeof(msg), "    CELL%d: %d.%03d V (raw=%d)\r\n", i+1, int_part, dec_part, values->cell_raw[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    int int_bat = (int)values->battery_voltage;
    int dec_bat = (int)(fabsf((values->battery_voltage - int_bat) * 1000.0f) + 0.5f);
    snprintf(msg, sizeof(msg), "    Battery: %d.%03d V\r\n", int_bat, dec_bat);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    int int_buck = (int)values->buck_voltage;
    int dec_buck = (int)(fabsf((values->buck_voltage - int_buck) * 1000.0f) + 0.5f);
    snprintf(msg, sizeof(msg), "    BUCK: %d.%03d V\r\n", int_buck, dec_buck);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    int int_curr = (int)values->current;
    int dec_curr = (int)((values->current - int_curr) * 1000);
    snprintf(msg, sizeof(msg), "    Current: %d.%03d A\r\n", int_curr, dec_curr);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
