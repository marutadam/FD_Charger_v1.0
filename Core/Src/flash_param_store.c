#include "flash_param_store.h"

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

