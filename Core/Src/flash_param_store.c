#include "flash_param_store.h"
#include <stdio.h>

// Save all parameters to flash
void SaveAllParams(FlashParams params) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError;
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector = FLASH_SECTOR_7; // Example: last sector
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }
    // Write struct as raw bytes
    uint32_t *src = (uint32_t*)&params;
    uint32_t *dst = (uint32_t*)FLASH_ADDR_DATA;
    for (size_t i = 0; i < sizeof(FlashParams)/4; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dst + i), src[i]);
    }
    HAL_FLASH_Lock();
}

// Read all parameters from flash
FlashParams ReadAllParams(void) {
    FlashParams params;
    memcpy(&params, (void*)FLASH_ADDR_DATA, sizeof(FlashParams));
    return params;
}

/**
 * @brief Prints all stored parameters to UART1 for debugging/monitoring.
 *
 * This function reads the stored FlashParams and prints each parameter's name and value
 * to UART1, with each line terminated by \r\n for terminal formatting.
 */
void PrintAllParamsToUART(void) {
    FlashParams params = ReadAllParams();
#ifdef huart1
    // If huart1 is defined as a macro or extern, use it. Otherwise, declare extern here.
#else
    extern UART_HandleTypeDef huart1;
#endif
    char buf[64];
    int len;
    len = snprintf(buf, sizeof(buf), "can_id: %u\r\n", params.can_id);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    // Print floats as integer.decimal (2 decimal places)
    int int_part, dec_part;
    int_part = (int)params.Kp;
    dec_part = (int)((params.Kp - int_part) * 100);
    len = snprintf(buf, sizeof(buf), "Kp: %d.%02d\r\n", int_part, dec_part);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    int_part = (int)params.enable_thresh;
    dec_part = (int)((params.enable_thresh - int_part) * 100);
    len = snprintf(buf, sizeof(buf), "enable_thresh: %d.%02d\r\n", int_part, dec_part);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    int_part = (int)params.disable_thresh;
    dec_part = (int)((params.disable_thresh - int_part) * 100);
    len = snprintf(buf, sizeof(buf), "disable_thresh: %d.%02d\r\n", int_part, dec_part);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    // duty_max is uint8_t, print as integer
    len = snprintf(buf, sizeof(buf), "duty_max: %u\r\n", params.duty_max);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    len = snprintf(buf, sizeof(buf), "min_on_ms: %u\r\n", params.min_on_ms);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);

    int_part = (int)params.storage_volt;
    dec_part = (int)((params.storage_volt - int_part) * 100);
    len = snprintf(buf, sizeof(buf), "storage_volt: %d.%02d\r\n", int_part, dec_part);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
}


// Save CAN_ID to flash

void ParamStore_Save_CAN_ID(uint8_t id) {
    FlashParams params = ReadAllParams();
    if (id < 0x71 || id > 0x76) // Valid range check
        id = 0x71;
    params.can_id = id;
    SaveAllParams(params);
}

uint8_t ParamStore_Read_CAN_ID(void) {
    FlashParams params = ReadAllParams();
    return params.can_id;
}
