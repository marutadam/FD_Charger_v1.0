#include "mcp2515.h"
#include "can_process.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "mcp2515.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const char* CAN_GetCommandName(uint8_t cmd);
#ifdef __cplusplus
}
#endif

// Redundant externs removed as they are declared in main.h


void ProcessCanFrame(CAN_Frame *rxFrame)
{
    char uart_buffer[100];
    // Accept command messages on 0x70 and CAN_ID
    if ((rxFrame->id == 0x70 || rxFrame->id == CAN_ID) && !rxFrame->extended) {
        const char* cmd_name = CAN_GetCommandName(rxFrame->data[0]);
        int len = sprintf(uart_buffer, "[0x%02X] %s | Data: ", rxFrame->data[0], cmd_name);
        for (uint8_t i = 1; i < rxFrame->dlc && i < 8; i++) {
            len += sprintf(uart_buffer + len, "%02X ", rxFrame->data[i]);
        }
        len += sprintf(uart_buffer + len, "\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
        // ...existing command handling logic from CanTaskHandler goes here...
        // For brevity, you can copy the command parsing blocks from your current CanTaskHandler
    } else {
        // Print basic CAN frame info
        int len = sprintf(uart_buffer, "CAN ID: 0x%03lX | DLC: %d | Data: ", (unsigned long)rxFrame->id, rxFrame->dlc);
        for (uint8_t i = 0; i < rxFrame->dlc && i < 8; i++) {
            len += sprintf(uart_buffer + len, "%02X ", rxFrame->data[i]);
        }
        if (rxFrame->extended) {
            len += sprintf(uart_buffer + len, "| EXT");
        }
        if (rxFrame->rtr) {
            len += sprintf(uart_buffer + len, " RTR");
        }
        len += sprintf(uart_buffer + len, "\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
    }
}

const char* CAN_GetCommandName(uint8_t cmd) {
    switch ((CAN_COMMAND)cmd) {
        case CMD_CHECK: return "CHECK";
        case CMD_START: return "START";
        case CMD_STOP: return "STOP";
        case CMD_RESET: return "RESET";
        case CMD_SET_END_VOLTAGE: return "SET_END_VOLTAGE";
        case CMD_SET_CURRENT: return "SET_CURRENT";
        case CMD_IS_BATT_PRESENT: return "IS_BATT_PRESENT";
        case CMD_READ_CURRENT_VOLTAGE: return "READ_CURRENT_VOLTAGE";
        case CMD_READ_CURRENT_CURRENT: return "READ_CURRENT_CURRENT";
        case CMD_READ_CURRENT_MAH: return "READ_CURRENT_MAH";
        case CMD_READ_CURRENT_POWER: return "READ_CURRENT_POWER";
        case CMD_UNKNOWN: return "UNKNOWN";
        default: return "UNKNOWN";
    }
}
