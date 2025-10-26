#include "main.h"
#include "param_types.h"
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
#include "can_process.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <cmsis_os.h>

#include "battery_charge.h"

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
            case CMD_CHECK:
                response = CheckSystem();
                break; 
            case CMD_SET_END_VOLTAGE:
                response = SetEndVoltage(rxFrame->data[7]);
                break;
            case CMD_SET_CURRENT:
                response = SetCurrent(rxFrame->data[7]);
                break;
            case CMD_IS_BATT_PRESENT:
                response = IsBatteryPresent();
                break;
            case CMD_READ_CURRENT_VOLTAGE:
                response = ReadCurrentVoltage();
                break;
            case CMD_READ_CURRENT_CURRENT:
                response = ReadCurrentCurrent();
                break;
            case CMD_READ_CURRENT_MAH:
                response = ReadCurrentmAh();
                break;
            case CMD_READ_CURRENT_POWER: 
                response = ReadCurrentPower();   
                break;
            case CMD_CONFIG_BALANCE:
                response = ConfigBalance(rxFrame);
                break;
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
    charger_set_targets(end_voltage, set_current);
    charger_enable();
    return response;
    // Additional logic to start charging can be added here
}
CAN_Frame StopCharging() {
    CAN_Frame response = CreateResponse(CMD_STOP);
    response.data[7] = 0x01;
    charger_disable();
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
    charger_set_targets(end_voltage, set_current);
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
    charger_set_targets(end_voltage, set_current);
    return response;
}

CAN_Frame CheckSystem(void){
    CAN_Frame response = CreateResponse(CMD_CHECK);

    for (int i = 1; i < 7; i++) {
        response.data[i] = 0x00; 
    }
    response.data[7] = system_state; 
    return response;
}


CAN_Frame IsBatteryPresent() {
    CAN_Frame response = CreateResponse(CMD_IS_BATT_PRESENT);
    for (int i = 1; i < 7; i++) {
        response.data[i] = 0x00;
    }
    response.data[7] = is_battery_present;
    return response;
}

CAN_Frame ReadCurrentVoltage() {
    CAN_Frame response = CreateResponse(CMD_READ_CURRENT_VOLTAGE);
    for (int i = 1; i < 7; i++) {
        float v = current_battery_voltages.cell[i-1];
        // Encode: byte_value = (voltage - 1.80) / 0.01
        int byte_value = (uint8_t)((v - 1.80f) * 100.0f);
        if (byte_value < 0) byte_value = 0;
        if (byte_value > 255) byte_value = 255;
        response.data[i] = (uint8_t)byte_value;
    }
    response.data[7] = (uint8_t)(current_battery_voltages.battery_voltage * 10.0f);
    return response;
}

CAN_Frame ReadCurrentCurrent() {
    CAN_Frame response = CreateResponse(CMD_READ_CURRENT_CURRENT);
    for (int i = 1; i < 7; i++) {
        response.data[i] = 0x00;
    }
    response.data[7] = (uint8_t)(charging_current * 10.0f);
    return response;
}

CAN_Frame ReadCurrentmAh() {
    CAN_Frame response = CreateResponse(CMD_READ_CURRENT_MAH);
    for (int i = 1; i < 7; i++) {
        response.data[i] = 0x00;
    }
    uint8_t high_byte = (charged_mah >> 8) & 0xFF;  
    uint8_t low_byte  = charged_mah & 0xFF;         
    response.data[6] = high_byte;
    response.data[7] = low_byte;
    return response;
}

CAN_Frame ReadCurrentPower() {
 CAN_Frame response = CreateResponse(CMD_READ_CURRENT_POWER);
    for (int i = 1; i < 7; i++) {
        response.data[i] = 0x00;
    }
    uint8_t high_byte = (charging_power >> 8) & 0xFF;  
    uint8_t low_byte  = charging_power & 0xFF;         
    response.data[6] = high_byte;
    response.data[7] = low_byte;
    return response;}

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

CAN_Frame ConfigBalance(CAN_Frame *rxFrame) {
    CAN_Frame response = CreateResponse(CMD_CONFIG_BALANCE);
    char buffer[100];
    int len = sprintf(buffer, "[INFO] Balance Config: ");
    FlashParams cfg;
    cfg.enable_thresh = (float)rxFrame->data[2] / 1000.0f; // mV to V
    cfg.disable_thresh = (float)rxFrame->data[3] / 1000.0f; // mV to V
    cfg.Kp = (float)(rxFrame->data[4] * 4); 
    cfg.duty_max = rxFrame->data[5]; // percent
    cfg.min_on_ms = (uint16_t)(rxFrame->data[6] * 100); // ms
    cfg.storage_volt = (float)rxFrame->data[7] / 10.0f; // 0.1V to V
    cfg.can_id = ParamStore_Read_CAN_ID();

    SaveAllParams(cfg);
    
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    // Here you would apply the configuration to your balancing controllers
    response.data[1] = 0x00; // Acknowledge
    return response;
}
