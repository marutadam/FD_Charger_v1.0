
#include "adc_mux.h"

#define ADS1115_ADDR 0x48 // Default I2C address

void mux_set_channel(uint8_t channel)
{
    // Only use lower 3 bits
    channel &= 0x07;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // A
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET); // B
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET); // C
}

float ads1115_read_voltage(I2C_HandleTypeDef *hi2c, uint8_t channel)
{
    // Set config for single-shot, single-ended channel
    uint16_t config = 0x8583 | ((channel & 0x03) << 12); // OS=1, MUX=channel, PGA=±4.096V, MODE=single-shot
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
    // Convert to voltage (assuming ±4.096V range)
    float voltage = (float)raw * 4.096f / 32768.0f;
    return voltage;
}