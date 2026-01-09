#include "adc_mux.h"
#include "cmsis_os2.h"
#include <math.h> // Add this include

// Exponential Moving Average (EMA) filter for smoother voltage measurements
// Lower alpha = more smoothing (more stable but slower response)
// Higher alpha = less smoothing (faster response but noisier)
#define VOLTAGE_FILTER_ALPHA 0.1f  // 0.0 - 1.0
#define CURRENT_FILTER_ALPHA 0.35f // Smoother filter for current

// Filtered measurement values buffer
static VoltageValues filtered_voltages = {0};
static uint8_t filter_initialized = 0;

// Exponential Moving Average filter
static float apply_ema_filter(float new_value, float previous_filtered, float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    return (alpha * new_value) + ((1.0f - alpha) * previous_filtered);
}

#define ADS1115_ADDR 0x48 // Default I2C address
#define ADS1115_ADDR_2 0x49 // Second I2C address
#define ADS1115_ADDR_3 0x4A // Default I2C address

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
    1.000422f, 1.001702f, 1.000087f, 0.999220f, 0.999481f, 1.00f
};

// AIN1 (buck) and AIN0/AIN2 (current sense high & battery) all use 91k/10k dividers
static const float buck_divider_scale = DIVIDER_SCALE(91000.0f, 10000.0f);
static const float battery_divider_scale = DIVIDER_SCALE(91000.0f, 10000.0f);
static const float current_divider_scale = DIVIDER_SCALE(91000.0f, 10000.0f);
static const float shunt_divider_scale = DIVIDER_SCALE(91000.0f, 10000.0f);

static const float battery_calibration_gain = 0.99349f;
static const float buck_calibration_gain = 1.0f;
static const float current_calibration_gain = 1.10256f; 
static const float shunt_calibration_gain = 0.99534f;

static const float pga_full_scale_table[] = {
    6.144f,
    4.096f, //dla celli
    2.048f,
    1.024f,
    0.512f,
    0.256f //dla shunta
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

int16_t ads1115_read_voltage(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, uint8_t channel, ADS1115_PGA pga)
{
    // Set config for single-shot, single-ended channel (AINx vs GND)
    uint16_t config = 0x8183;              // OS=1, PGA bits cleared, MODE=single-shot, 128 SPS
    config |= ((uint16_t)pga & 0x07) << 9; // apply PGA selection
    config &= ~0x7000;                     // clear MUX bits
    config |= 0x4000 | ((channel & 0x03) << 12); // select AINx single-ended (0b100 + channel)
    uint8_t config_bytes[3] = {0x01, config >> 8, config & 0xFF};
    HAL_I2C_Master_Transmit(hi2c, i2c_addr << 1, config_bytes, 3, HAL_MAX_DELAY);

    // Wait for conversion (minimum 8ms for 128SPS)
    HAL_Delay(10);

    // Read conversion result
    uint8_t reg = 0x00;
    uint8_t data[2];
    HAL_I2C_Master_Transmit(hi2c, i2c_addr << 1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, i2c_addr << 1, data, 2, HAL_MAX_DELAY);

    int16_t raw = (data[0] << 8) | data[1];
    return raw;
}

static void readCells(I2C_HandleTypeDef *hi2c, float *cells, int16_t *rawCells)
{
    const ADS1115_PGA cell_pga = ADS1115_PGA_4V096;
    const ADS1115_VoltageChannel cell_map[6] = {
        CELL_1_VOLTAGE,
        CELL_2_VOLTAGE,
        CELL_3_VOLTAGE,
        CELL_4_VOLTAGE,
        CELL_5_VOLTAGE,
        CELL_6_VOLTAGE
    };
    
    // First read all absolute voltages from GND
    float absolute_voltages[6];
    for (uint8_t ch = 0; ch < 6; ch++) {
        ADS1115_ChannelConfig cfg = get_ads1115_config(cell_map[ch]);
        int16_t raw = ads1115_read_voltage(hi2c, cfg.i2c_addr, cfg.channel, cell_pga);
        float sense_voltage = ads1115_raw_to_voltage(raw, cell_pga);
        absolute_voltages[ch] = sense_voltage * cell_divider_scale[ch] * cell_calibration_gain[ch];

        if (rawCells != NULL) {
            rawCells[ch] = raw;
        }
    }
    
    // Now calculate differential voltages (each cell voltage relative to previous)
    // Cell 1: voltage from GND to Cell1+ (0V to ~3.7V)
    // Cell 2: voltage from Cell1+ to Cell2+ (difference)
    // Cell 3: voltage from Cell2+ to Cell3+ (difference)
    // etc.
    cells[0] = absolute_voltages[0];  // Cell 1 is already correct (measured from GND)
    for (uint8_t ch = 1; ch < 6; ch++) {
        cells[ch] = absolute_voltages[ch] - absolute_voltages[ch - 1];  // Differential voltage
    }
}

static float readBatteryVoltage(I2C_HandleTypeDef *hi2c, float *divider_voltage, int16_t *raw_out)
{
    const ADS1115_PGA battery_pga = ADS1115_PGA_4V096;
    ADS1115_ChannelConfig cfg = get_ads1115_config(SHUNT_MINUS_VOLTAGE);
    int16_t raw = ads1115_read_voltage(hi2c, cfg.i2c_addr, cfg.channel, battery_pga);
    if (raw_out != NULL) {
        *raw_out = raw;
    }
    float sense_voltage = ads1115_raw_to_voltage(raw, battery_pga);
    if (divider_voltage != NULL) {
        *divider_voltage = sense_voltage;
    }
    return sense_voltage * battery_divider_scale * battery_calibration_gain;
}

static float readBuck(I2C_HandleTypeDef *hi2c)
{
    const ADS1115_PGA buck_pga = ADS1115_PGA_4V096;
    ADS1115_ChannelConfig cfg = get_ads1115_config(BUCK_VOLTAGE);
    int16_t raw = ads1115_read_voltage(hi2c, cfg.i2c_addr, cfg.channel, buck_pga);
    float sense_voltage = ads1115_raw_to_voltage(raw, buck_pga);
    return sense_voltage * buck_divider_scale * buck_calibration_gain;
}

static float calculateCurrent(float shunt_voltage, float battery_voltage)
{
    float shunt_resistance = 0.025f; // Effective shunt resistance (4x 0.1 ohm in parallel)
    float current_voltage = shunt_voltage-battery_voltage;
    float current = (current_voltage / shunt_resistance) * current_calibration_gain;
    return (current < 0.0f) ? 0.0f : current;
}

static float __attribute__((unused)) readCurrent(I2C_HandleTypeDef *hi2c,
                         float battery_voltage,
                         float buck_voltage,
                         int16_t battery_raw,
                         int16_t *raw_current_out)
{
    const ADS1115_PGA current_pga = ADS1115_PGA_4V096;
    ADS1115_ChannelConfig cfg_shunt_plus = get_ads1115_config(SHUNT_PLUS_VOLTAGE);
    int16_t raw_current = ads1115_read_voltage(hi2c, cfg_shunt_plus.i2c_addr, cfg_shunt_plus.channel, current_pga);
    if (buck_voltage < battery_voltage) {
        if (raw_current_out != NULL) {
            *raw_current_out = 0;
        }
        return 0.0f;
    }
    float current_high_voltage = ads1115_raw_to_voltage(raw_current, current_pga);
    float battery_voltage_sense = ads1115_raw_to_voltage(battery_raw, ADS1115_PGA_4V096);
    float current_high_actual = current_high_voltage * current_divider_scale;
    float battery_actual = battery_voltage_sense * battery_divider_scale;
    float shunt_voltage = current_high_actual - battery_actual;
    if (raw_current_out != NULL) {
        *raw_current_out = raw_current - battery_raw;
    }
    float shunt_resistance = 0.025f; // Effective shunt resistance (4x 0.1 ohm in parallel)
    float current_voltage = shunt_voltage;
    return (current_voltage / shunt_resistance) * current_calibration_gain;
}

VoltageValues ads1115_read_all_voltages(I2C_HandleTypeDef *hi2c)
{
    VoltageValues values;
    int16_t rawCells[6] = {0};
    readCells(hi2c, values.cell, rawCells);
    float battery_divider_voltage = 0.0f;
    int16_t battery_raw = 0;
    values.battery_voltage = readBatteryVoltage(hi2c, &battery_divider_voltage, &battery_raw);
        ADS1115_ChannelConfig cfg_shunt_plus = get_ads1115_config(SHUNT_PLUS_VOLTAGE);
            const ADS1115_PGA current_pga = ADS1115_PGA_4V096;

    int16_t raw_shunt_voltage = ads1115_read_voltage(hi2c, cfg_shunt_plus.i2c_addr, cfg_shunt_plus.channel, current_pga);

    
    values.shunt_voltage = ads1115_raw_to_voltage(raw_shunt_voltage, current_pga);
        values.shunt_voltage = values.shunt_voltage * shunt_calibration_gain * shunt_divider_scale;

    values.buck_voltage = readBuck(hi2c);

    values.current = calculateCurrent(values.shunt_voltage, values.battery_voltage);  
    for (int i = 0; i < 6; ++i) {
        values.cell_raw[i] = rawCells[i];
    }
    
    // Initialize filter on first call
    if (!filter_initialized) {
        filtered_voltages = values;
        filter_initialized = 1;
        return values;  // Return unfiltered on first read
    }
    
    // Apply exponential filter to all measurements for stability
    for (int i = 0; i < 6; ++i) {
        filtered_voltages.cell[i] = apply_ema_filter(values.cell[i], filtered_voltages.cell[i], VOLTAGE_FILTER_ALPHA);
    }
    filtered_voltages.battery_voltage = apply_ema_filter(values.battery_voltage, filtered_voltages.battery_voltage, VOLTAGE_FILTER_ALPHA);
    filtered_voltages.buck_voltage = apply_ema_filter(values.buck_voltage, filtered_voltages.buck_voltage, VOLTAGE_FILTER_ALPHA);
    filtered_voltages.shunt_voltage = apply_ema_filter(values.shunt_voltage, filtered_voltages.shunt_voltage, VOLTAGE_FILTER_ALPHA);
    filtered_voltages.current = apply_ema_filter(values.current, filtered_voltages.current, CURRENT_FILTER_ALPHA);
    
    // Keep raw ADC values unfiltered
    for (int i = 0; i < 6; ++i) {
        filtered_voltages.cell_raw[i] = values.cell_raw[i];
    }
    filtered_voltages.current_raw = values.current_raw;
    
    return filtered_voltages;
}

