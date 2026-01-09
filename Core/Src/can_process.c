#include "battery_balance.h"
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

// Magic number constants
#define CAN_BROADCAST_ID 0x70
#define VOLTAGE_OFFSET_V 1.80f
#define VOLTAGE_SCALE 100.0f
#define UART_TIMEOUT_MS 100

// Helper function to clear response data bytes
static inline void clear_response_data(CAN_Frame *frame) {
    for (int i = 1; i < 8; i++) {
        frame->data[i] = 0x00;
    }
}


void ProcessCanFrame(CAN_Frame *rxFrame)
{
    // Validate DLC
    if (rxFrame->dlc == 0 || rxFrame->dlc > 8) {
        return;
    }
    
    // Accept command messages on broadcast or device-specific ID
    if ((rxFrame->id == CAN_BROADCAST_ID || rxFrame->id == CAN_ID) && !rxFrame->extended) {
        CAN_COMMAND cmd = (CAN_COMMAND)rxFrame->data[0];
        // Command handling; UART debug removed (JSON telemetry used)
        CAN_Frame response;
        switch (cmd) {
            case CMD_START_CHARGING:
                response = StartCharging(rxFrame->data[7]);
                is_battery_charging = true;
                break;
            case CMD_STOP_CHARGING:
                response = StopCharging();
                is_battery_charging = false;
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
            case CMD_STORAGE:
                response = StartStorage();
                break;
            case CMD_STORAGE_STOP:
                response = StopStorage();
                break;
            case CMD_DISCHARGE:
                response = StartDischarge();
                break;
            case CMD_SET_CELL_BALANCE:
                response = SetCellBalance(rxFrame->data[6], rxFrame->data[7]);
                break;
            case CMD_MANUAL_BALANCE_CTRL:
                response = ManualBalanceCtrl(rxFrame->data[7]);
                break;
            default:
                // Unknown command
                return;
        }
        if (MCP2515_SendMessage(&hspi1, &response) == MCP2515_OK) {
                        uart_send_frame("[INFO] Response sent: ", &response);
                    } else {
                        const char* err_msg = "[WARN] Failed to send response\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, strlen(err_msg), UART_TIMEOUT_MS);
                    }
        return;  // Exit after sending response
    } 
}

// CAN_GetCommandName removed; use CAN_COMMAND enum directly

CAN_Frame StartCharging(uint8_t balance_flag) {
    extern volatile uint8_t can_balance_enabled;
    CAN_Frame response = CreateResponse(CMD_START_CHARGING);
    charger_set_targets(end_voltage, set_current);
    charger_enable();
    is_battery_charging = true;
    
    // Handle balance flag: 0x02 = enable balance, 0x01 = disable balance
    if (balance_flag == 0x02) {
        can_balance_enabled = 1;
        response.data[7] = 0x02;
    } else {
        can_balance_enabled = 0;
        response.data[7] = 0x01;
    }
    
    return response;
}
CAN_Frame StopCharging() {
    extern volatile uint8_t can_balance_enabled;

    CAN_Frame response = CreateResponse(CMD_STOP_CHARGING);
    response.data[7] = 0x01;
    charger_disable_with_reason("CAN stop command");
    is_battery_charging = false;
    can_balance_enabled = 0;
    return response;
    // Additional logic to stop charging can be added here
}  
void ResetSystem() {
    NVIC_SystemReset(); // Perform a system reset
}

CAN_Frame SetEndVoltage(uint8_t voltage) {
    end_voltage = (float)voltage / 10.0f;

    CAN_Frame response = CreateResponse(CMD_SET_END_VOLTAGE);
    float voltage_f = end_voltage * 10.0f;
    response.data[7] = (uint8_t)voltage_f;
    charger_set_targets(end_voltage, set_current);
    return response;
}

CAN_Frame SetCurrent(uint8_t current) {
    set_current = (float)current / 10.0f;
    CAN_Frame response = CreateResponse(CMD_SET_CURRENT);
    float current_f = set_current * 10.0f;
    response.data[7] = (uint8_t)current_f; // Acknowledge
    charger_set_targets(end_voltage, set_current);
    return response;
}

CAN_Frame CheckSystem(void){
    CAN_Frame response = CreateResponse(CMD_CHECK);
    clear_response_data(&response);
    response.data[7] = system_state; 
    return response;
}


CAN_Frame IsBatteryPresent() {
    CAN_Frame response = CreateResponse(CMD_IS_BATT_PRESENT);
    clear_response_data(&response);
    response.data[7] = is_battery_present;
    return response;
}

CAN_Frame ReadCurrentVoltage() {
    CAN_Frame response = CreateResponse(CMD_READ_CURRENT_VOLTAGE);
    VoltageValues voltages = get_battery_voltages_safe();  // Thread-safe read

    // If battery absent, force zeros for pack and cells
    if (!is_battery_present) {
        voltages.battery_voltage = 0.0f;
        for (int i = 0; i < 6; ++i) {
            voltages.cell[i] = 0.0f;
        }
    }
    
    for (int i = 1; i < 7; i++) {
        float v = voltages.cell[i-1];
        // Encode: byte_value = (voltage - 1.80) / 0.01
        float scaled = (v - VOLTAGE_OFFSET_V) * VOLTAGE_SCALE;
        int byte_value = (int)scaled;
        if (byte_value < 0) byte_value = 0;
        if (byte_value > 255) byte_value = 255;
        response.data[i] = (uint8_t)byte_value;
    }
    response.data[7] = (uint8_t)(voltages.battery_voltage * 10.0f);
    return response;
}

CAN_Frame ReadCurrentCurrent() {
    CAN_Frame response = CreateResponse(CMD_READ_CURRENT_CURRENT);
    clear_response_data(&response);

    // Use latest measured current from ADC (current_battery_voltages)
    VoltageValues voltages = get_battery_voltages_safe();
    float curr = voltages.current;
    if (!is_battery_present) {
        curr = 0.0f;
    }
    int scaled = (int)(curr * 10.0f); // 0.1 A resolution
    if (scaled < 0) scaled = 0;
    if (scaled > 255) scaled = 255;
    response.data[7] = (uint8_t)scaled;
    return response;
}

CAN_Frame ReadCurrentmAh() {
    CAN_Frame response = CreateResponse(CMD_READ_CURRENT_MAH);
    clear_response_data(&response);
    // Encode 16-bit value: high byte at [6], low byte at [7]
    response.data[6] = (uint8_t)((charged_mah >> 8) & 0xFF);
    response.data[7] = (uint8_t)(charged_mah & 0xFF);
    return response;
}

CAN_Frame ReadCurrentPower() {
    CAN_Frame response = CreateResponse(CMD_READ_CURRENT_POWER);
    clear_response_data(&response);
    // Encode 16-bit value: high byte at [6], low byte at [7]
    response.data[6] = (uint8_t)((charging_power >> 8) & 0xFF);
    response.data[7] = (uint8_t)(charging_power & 0xFF);
    return response;
}

CAN_Frame CreateResponse(CAN_COMMAND cmd) {
    CAN_Frame response;
    response.id = CAN_ID;
    response.dlc = 8;
    response.extended = 0; // Standard ID
    response.rtr = 0; // Data frame
    response.data[0] = cmd; // Echo command
    clear_response_data(&response);
    return response;
}

CAN_Frame ConfigBalance(CAN_Frame *rxFrame) {
    CAN_Frame response = CreateResponse(CMD_CONFIG_BALANCE);
    
    // Validate DLC for all required data bytes
    if (rxFrame->dlc < 8) {
        response.data[1] = 0xFF; // Error: insufficient data
        return response;
    }
    
    FlashParams cfg;
    cfg.enable_thresh = (float)rxFrame->data[2] / 1000.0f; // mV to V
    cfg.disable_thresh = (float)rxFrame->data[3] / 1000.0f; // mV to V
    cfg.Kp = (float)(rxFrame->data[4] * 4); 
    cfg.duty_max = rxFrame->data[5]; // percent
    if (cfg.duty_max == 0) {
        cfg.duty_max = 60;
    }
    cfg.min_on_ms = (uint16_t)(rxFrame->data[6] * 100); // ms
    cfg.storage_volt = (float)rxFrame->data[7] / 10.0f; // 0.1V to V
    if (cfg.storage_volt <= 0.0f) {
        cfg.storage_volt = end_voltage_storage;
    }
    cfg.can_id = ParamStore_Read_CAN_ID();

    SaveAllParams(cfg);
    end_voltage_storage = cfg.storage_volt;
    
    // UART log removed (config acknowledged via CAN)
    // Here you would apply the configuration to your balancing controllers
    response.data[1] = 0x00; // Acknowledge
    return response;
}

CAN_Frame StartStorage() {
    CAN_Frame response = CreateResponse(CMD_STORAGE);
    response.data[7] = 0x01;
    charger_set_targets(end_voltage_storage, set_current);
    StartStorageMode();
    is_battery_charging = true;
    return response;
}
CAN_Frame StopStorage() {
    CAN_Frame response = CreateResponse(CMD_STORAGE_STOP);
    response.data[7] = 0x01;
    charger_set_targets(end_voltage_storage, set_current);
    balance_disable_all_cells();
    charger_disable_with_reason("Stop storage mode");
    is_battery_charging = false;
    return response;
}

CAN_Frame StartDischarge() {
    CAN_Frame response = CreateResponse(CMD_DISCHARGE);
    clear_response_data(&response);
    
    // TODO: Implement discharge logic
    // For now, stop charging and enable discharge resistors
    charger_disable_with_reason("Discharge mode");
    is_battery_charging = false;
    
    response.data[7] = 0x01; // Acknowledge
    return response;
}

CAN_Frame SetCellBalance(uint8_t cell_num, uint8_t duty_percent) {
    CAN_Frame response = CreateResponse(CMD_SET_CELL_BALANCE);
    clear_response_data(&response);
    
    // Validate cell number (1-6 in user terms, 0-5 internally)
    if (cell_num < 1 || cell_num > 6) {
        response.data[6] = cell_num;
        response.data[7] = 0xFE; // Invalid cell number
        return response;
    }
    
    // Validate duty percent (0-100)
    if (duty_percent > 100) {
        response.data[6] = cell_num;
        response.data[7] = 0xFD; // Invalid duty percent
        return response;
    }
    
    // Convert to 0-based index
    uint8_t cell_index = cell_num - 1;
    
    // Set the balance PWM
    if (duty_percent == 0) {
        disable_cell_balance(cell_index);
    } else {
        enable_cell_balance(cell_index, duty_percent);
    }
    
    // Echo back the values and success status
    response.data[6] = cell_num;
    response.data[7] = duty_percent;
    
    return response;
}

CAN_Frame ManualBalanceCtrl(uint8_t enable) {
    CAN_Frame response = CreateResponse(CMD_MANUAL_BALANCE_CTRL);
    clear_response_data(&response);
    extern volatile uint8_t manual_balance_mode;

    if (enable == 0x01) {
        manual_balance_mode = 1;      // Allow SetCellBalance to hold duty
        balance_disable_all_cells();  // Start from known-off state
        response.data[7] = 0x01;      // Success
    } else if (enable == 0x00) {
        manual_balance_mode = 0;
        balance_disable_all_cells();  // Turn everything off when leaving manual
        response.data[7] = 0x00;
              // Success
    } else {
        response.data[7] = 0xFF;      // Invalid parameter
    }
    
    return response;
}
