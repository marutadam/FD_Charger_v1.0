#include "ws2812c.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal_gpio.h"
#include "main.h"  // for LED_WS2812C pin mapping

// Use the dedicated WS2812 pin (PC15)
#define WS2812_GPIO_PORT LED_WS2812C_GPIO_Port
#define WS2812_GPIO_PIN  LED_WS2812C_Pin

// Force inline to reduce function call overhead which affects timing
static inline void __attribute__((always_inline)) ws2812_delay_cycles(uint32_t cycles) {
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

// Convert nanoseconds to CPU cycles based on SystemCoreClock
static inline uint32_t ws2812_ns_to_cycles(uint32_t ns) {
    // (SystemCoreClock [Hz] * ns) / 1e9
    uint64_t cycles = ((uint64_t)SystemCoreClock * (uint64_t)ns) / 1000000000ULL;
    return cycles == 0 ? 1U : (uint32_t)cycles;  // Ensure non-zero delays
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
    // Timing derived from WS2812 datasheet (800kHz):
    // Optimized for direct register access (BSRR)
    // Adjusted timings to compensate for loop and function overhead (approx 10-20 cycles)
    // T0H target ~300ns (was 400) to ensure '0' is not read as '1'
    // T1H target ~750ns (was 800)
    // T0L/T1L adjusted to maintain frame timing
    const uint32_t t0h = ws2812_ns_to_cycles(300);
    const uint32_t t0l = ws2812_ns_to_cycles(900);
    const uint32_t t1h = ws2812_ns_to_cycles(750);
    const uint32_t t1l = ws2812_ns_to_cycles(500);

    // Prepare bit masks for BSRR
    const uint32_t pin_set = WS2812_GPIO_PIN;
    const uint32_t pin_reset = (uint32_t)WS2812_GPIO_PIN << 16U;

    __disable_irq(); // Disable interrupts during transmission

    for (int i = 0; i < WS2812_LED_COUNT; i++) {
        for (int j = 0; j < 3; j++) { // G, R, B
            uint8_t val = colors[i][j];
            for (int k = 7; k >= 0; k--) {
                if (val & (1 << k)) {
                    // Bit '1': High then Low
                    WS2812_GPIO_PORT->BSRR = pin_set;
                    ws2812_delay_cycles(t1h);
                    WS2812_GPIO_PORT->BSRR = pin_reset;
                    ws2812_delay_cycles(t1l);
                } else {
                    // Bit '0': High then Low
                    WS2812_GPIO_PORT->BSRR = pin_set;
                    ws2812_delay_cycles(t0h);
                    WS2812_GPIO_PORT->BSRR = pin_reset;
                    ws2812_delay_cycles(t0l);
                }
            }
        }
    }
    __enable_irq();
    // Reset signal: >50us low
    WS2812_GPIO_PORT->BSRR = pin_reset;
    HAL_Delay(1);
}

// Fill all LEDs with one color (GRB order expected by WS2812)
void WS2812_FillColor(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t colors[WS2812_LED_COUNT][3];
    for (int i = 0; i < WS2812_LED_COUNT; ++i) {
        colors[i][0] = g;  // G
        colors[i][1] = r;  // R
        colors[i][2] = b;  // B
    }
    WS2812_Send(colors);
}

// Light only the selected LED, turn others off
void WS2812_SetOne(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    uint8_t colors[WS2812_LED_COUNT][3] = {0};
    if (index < WS2812_LED_COUNT) {
        colors[index][0] = g;  // G
        colors[index][1] = r;  // R
        colors[index][2] = b;  // B
    }
    WS2812_Send(colors);
}