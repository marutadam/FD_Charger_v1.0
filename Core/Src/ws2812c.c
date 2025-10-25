#include "ws2812c.h"
#include "stm32f4xx_hal_gpio.h"

// Ustaw odpowiednio do Twojego projektu!
#define WS2812_GPIO_PORT GPIOC
#define WS2812_GPIO_PIN  GPIO_PIN_15

static void ws2812_delay_cycles(uint32_t cycles) {
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

// Call this once in main() before using DWT:
void ws2812_enable_dwt(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void WS2812_Send(uint8_t (*colors)[3]) {
    __disable_irq(); // Wyłącz przerwania na czas transmisji

    for (int i = 0; i < WS2812_LED_COUNT; i++) {
        for (int j = 0; j < 3; j++) { // G, R, B
            uint8_t val = colors[i][j];
            for (int k = 7; k >= 0; k--) {
                if (val & (1 << k)) {
                    // Bit '1': ~0.8us high, ~0.45us low
                    HAL_GPIO_WritePin(WS2812_GPIO_PORT, WS2812_GPIO_PIN, GPIO_PIN_SET);
                    ws2812_delay_cycles(8);
                    HAL_GPIO_WritePin(WS2812_GPIO_PORT, WS2812_GPIO_PIN, GPIO_PIN_RESET);
                    ws2812_delay_cycles(5);
                } else {
                    // Bit '0': ~0.4us high, ~0.85us low
                    HAL_GPIO_WritePin(WS2812_GPIO_PORT, WS2812_GPIO_PIN, GPIO_PIN_SET);
                    ws2812_delay_cycles(4);
                    HAL_GPIO_WritePin(WS2812_GPIO_PORT, WS2812_GPIO_PIN, GPIO_PIN_RESET);
                    ws2812_delay_cycles(9);
                }
            }
        }
    }
    __enable_irq();
    // Reset sygnału: >50us low
    HAL_GPIO_WritePin(WS2812_GPIO_PORT, WS2812_GPIO_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
}