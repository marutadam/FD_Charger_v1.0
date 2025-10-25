#pragma once
#include "stm32f4xx_hal.h"

#define WS2812_LED_COUNT 6

void WS2812_Send(uint8_t (*colors)[3]);
void ws2812_enable_dwt(void);