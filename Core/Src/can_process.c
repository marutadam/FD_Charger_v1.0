#include "main.h"
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
#include "can_process.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <cmsis_os.h>

#include "main.h"

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
        CAN_COMMAND cmd = (CAN_COMMAND)rxFrame->data[0];
        int len = sprintf(uart_buffer, "[0x%02X] | Data: ", rxFrame->data[0]);
        for (uint8_t i = 1; i < rxFrame->dlc && i < 8; i++) {
            len += sprintf(uart_buffer + len, "%02X ", rxFrame->data[i]);
        }
        len += sprintf(uart_buffer + len, "\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
        osDelay(5);
        // Example: handle commands using enum
        CAN_Frame response;
        switch (cmd) {
            case CMD_START:
                response = StartCharging();
                break;
            case CMD_STOP:
                response = StopCharging();
                break;
            case CMD_RESET:
                ResetSystem();
                break;
            // case CMD_CHECK:
            //     response = CheckSystem();
            //     break; 
            case CMD_SET_END_VOLTAGE:
                response = SetEndVoltage(rxFrame->data[7]);
                break;
            case CMD_SET_CURRENT:
                response = SetCurrent(rxFrame->data[7]);
                break;
            // case CMD_IS_BATT_PRESENT:
            //     response = IsBatteryPresent();
            //     break;
            // case CMD_READ_CURRENT_VOLTAGE:
            //     response = ReadCurrentVoltage();
            //     break;
            // case CMD_READ_CURRENT_CURRENT:
            //     response = ReadCurrentCurrent();
            //     break;
            // case CMD_READ_CURRENT_MAH:
            //     response = ReadCurrentmAh();
            //     break;
            // case CMD_READ_CURRENT_POWER: 
            //     response = ReadCurrentPower();   
            //     break;
            default:
                // Unknown command
                return;
                break;
        }
        if (MCP2515_SendMessage(&hspi1, &response) == MCP2515_OK) {
                        // osDelay(5);
                        uart_send_frame("[INFO] Response sent: ", &response);

                    } else {
                        // osDelay(5);
                        const char* err_msg = "[WARN] Failed to send response\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, strlen(err_msg), HAL_MAX_DELAY);
                    }
    } 
}

// CAN_GetCommandName removed; use CAN_COMMAND enum directly

CAN_Frame StartCharging() {
    CAN_Frame response = CreateResponse(CMD_START);
    response.data[7] = 0x01;
    return response;
    // Additional logic to start charging can be added here
}
CAN_Frame StopCharging() {
    CAN_Frame response = CreateResponse(CMD_STOP);
    response.data[7] = 0x01;
    return response;
    // Additional logic to stop charging can be added here
}  
void ResetSystem() {
    NVIC_SystemReset(); // Perform a system reset
}
CAN_Frame SetEndVoltage(uint8_t voltage) {
    end_voltage = (float)voltage / 10.0f;
    

    CAN_Frame response = CreateResponse(CMD_SET_END_VOLTAGE);
    float voltage_f = (float)end_voltage * 10.0f;
    uint16_t voltage_uint = (uint16_t)voltage_f;
           // Log the voltage (avoid float printf, use integer math)
                    uint16_t voltage_int = voltage_uint / 10;      // Integer part (5)
                    uint16_t voltage_dec = voltage_uint % 10;      // Decimal part (0)
            char buffer[60];
                    int curr_len = sprintf(buffer, "[INFO] Set Voltage: %u.%u V (raw=0x%02X)\r\n", 
                                          voltage_int, voltage_dec, voltage);
                    

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, curr_len, HAL_MAX_DELAY);

    response.data[7] = (uint8_t)voltage_f;
    return response;
}

CAN_Frame SetCurrent(uint8_t current) {
    set_current = (float)current / 10.0f;    

    CAN_Frame response = CreateResponse(CMD_SET_CURRENT);
    float current_f = (float)set_current * 10.0f;
    uint16_t current_uint = (uint16_t)current_f;
           // Log the voltage (avoid float printf, use integer math)
                    uint16_t current_int = current_uint / 10;      // Integer part (5)
                    uint16_t current_dec = current_uint % 10;      // Decimal part (0)
            char buffer[60];
                    int curr_len = sprintf(buffer, "[INFO] Set Current: %u.%u A (raw=0x%02X)\r\n", 
                                          current_int, current_dec, current);
                    

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, curr_len, HAL_MAX_DELAY);

    response.data[7] = (uint8_t)current_f; // Acknowledge
    return response;
}

// CAN_Frame IsBatteryPresent() {
//     // Placeholder logic, replace with actual battery presence detection
//     return 1; // Assume battery is always present for now
// }
// CAN_Frame ReadCurrentVoltage() {
//     return battery_voltage; // Return the current battery voltage      
// }
// CAN_Frame ReadCurrentCurrent() {
//     return set_current; // Return the current charging current
// }

// CAN_Frame ReadCurrentmAh() {
//     // Placeholder logic, replace with actual mAh reading
//     return 1000.0f; // Example value
// }

// CAN_Frame ReadCurrentPower() {
//     return battery_voltage * set_current; // Power = Voltage * Current
// }

CAN_Frame CreateResponse(CAN_COMMAND cmd) {
    CAN_Frame response;
    response.id = (uint8_t)CAN_ID; // Set appropriate ID
    response.dlc = 8; // Set appropriate DLC
    response.extended = 0; // Standard ID
    response.rtr = 0; // Data frame
    response.data[0] = cmd; // Echo command
    for (int i = 1; i < 8; i++) {
        response.data[i] = 0x00; // Placeholder data
    }
    return response;
}