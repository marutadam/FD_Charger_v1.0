#include "commands.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
static uint32_t last_ping_ts = 0;
#define PING_MIN_INTERVAL_MS 2000

// Minimal command parser stub to satisfy linker and provide basic handling
void ParseCommand(char *line) {
    if (!line) return;

    // trim trailing whitespace/newlines (in case)
    size_t len = strlen(line);
    while (len && (line[len-1] == '\r' || line[len-1] == '\n' || line[len-1] == ' ')) {
        line[--len] = '\0';
    }

    if (strcmp(line, "ping") == 0) {
        uint32_t now = HAL_GetTick();
        if (now - last_ping_ts >= PING_MIN_INTERVAL_MS) {
            const char* msg = "pong\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            last_ping_ts = now;
        }
        return;
    } else {
        // Echo back
        char buf[128];
        int n = snprintf(buf, sizeof(buf), "Unknown cmd: %s\r\n", line);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, HAL_MAX_DELAY);
    }
}


