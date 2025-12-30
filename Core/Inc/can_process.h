#ifndef CAN_PROCESS_H
#define CAN_PROCESS_H

#include "mcp2515.h"
#include <stm32f4xx_hal.h>
#include <stdint.h>
#include "battery_balance.h"
#include "param_types.h"
#include "main.h"
#include <stdbool.h>


typedef enum {
    CMD_CHECK = 0x01,
    CMD_START = 0x02,
    CMD_STOP = 0x03,
    CMD_RESET = 0x04,
    CMD_SET_END_VOLTAGE = 0x05,
    CMD_SET_CURRENT = 0x06,
    CMD_IS_BATT_PRESENT = 0x07,
    CMD_READ_CURRENT_VOLTAGE = 0x08,
    CMD_READ_CURRENT_CURRENT = 0x09,
    CMD_READ_CURRENT_MAH = 0x0A,
    CMD_READ_CURRENT_POWER = 0x0B,
    CMD_DISCHARGE = 0x0C,
    CMD_STORAGE = 0x0D,
    CMD_STORAGE_STOP = 0x0E,
    CMD_CONFIG_BALANCE = 0xF0,
    CMD_SET_CELL_BALANCE = 0x56,
    CMD_MANUAL_BALANCE_CTRL = 0x57,
    CMD_UNKNOWN = 0xFF
} CAN_COMMAND;

/* CAN Frame Structure */
typedef struct CAN_Frame {
    uint8_t id;            // CAN ID (11-bit or 29-bit)
    uint8_t extended;       // 0 = Standard ID, 1 = Extended ID
    uint8_t rtr;            // Remote Transmission Request
    uint8_t dlc;            // Data Length Code (0-8)
    uint8_t data[8];        // Data bytes
} CAN_Frame;


#ifdef __cplusplus
extern "C" {
#endif

void ProcessCanFrame(CAN_Frame *rxFrame);
CAN_Frame StartCharging();
CAN_Frame StopCharging();
void ResetSystem(void);
CAN_Frame SetEndVoltage(uint8_t voltage);
CAN_Frame SetCurrent(uint8_t current);
CAN_Frame ReadCurrentVoltage(void);
CAN_Frame ReadCurrentCurrent(void);
CAN_Frame ReadCurrentmAh(void);
CAN_Frame ReadCurrentPower(void);
CAN_Frame IsBatteryPresent(void);
CAN_Frame CreateResponse(CAN_COMMAND cmd);
CAN_Frame CheckSystem(void);
CAN_Frame ConfigBalance(CAN_Frame *rxFrame);
CAN_Frame StartStorage(void);
CAN_Frame SetCellBalance(uint8_t cell_num, uint8_t duty_percent);
CAN_Frame ManualBalanceCtrl(uint8_t enable);
CAN_Frame StopStorage(void);
CAN_Frame StartDischarge(void);
extern volatile uint8_t system_state; // 0x00 = System ok, 0x0X = error codes
extern volatile float set_current;
extern volatile float charging_current; 
extern volatile uint16_t charged_mah;
extern volatile uint16_t charging_power;
extern volatile uint8_t is_battery_present;
extern volatile uint8_t CAN_ID;
extern volatile float cell_voltages[6];
extern volatile float end_voltage;
extern volatile float end_voltage_storage;
extern volatile float battery_voltage;
extern VoltageValues current_battery_voltages;
extern volatile bool is_battery_charging;

#ifdef __cplusplus
}
#endif
#endif // CAN_PROCESS_H
