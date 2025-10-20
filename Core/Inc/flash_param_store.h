#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdint.h>
#include "param_types.h"
#include "main.h"
#ifndef FLASH_PARAM_STORE_H
#define FLASH_PARAM_STORE_H


#ifdef __cplusplus
extern "C" {
#endif

// Example parameter keys (expand as needed)
#define FLASH_ADDR_DATA  0x0807F000  // początek zarezerwowanego obszaru (4 kB)


// API
void ParamStore_Save_CAN_ID(uint8_t id);
uint8_t ParamStore_Read_CAN_ID(void);
FlashParams ReadAllParams(void);
void SaveAllParams(FlashParams params);
void SaveBalanceConfig(FlashParams params);
void PrintAllParamsToUART(void);
// Add more save/read functions for other parameters

#ifdef __cplusplus
}
#endif

#endif // FLASH_PARAM_STORE_H
