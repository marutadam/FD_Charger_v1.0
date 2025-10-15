#ifndef CAN_PROCESS_H
#define CAN_PROCESS_H

#include "mcp2515.h"
#include <stm32f4xx_hal.h>
#include <stdint.h>

/* CAN Command Codes (data[0] byte for ID 0x72) */
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
    CMD_UNKNOWN = 0xFF
} CAN_COMMAND;

/* CAN Frame Structure */
typedef struct {
    uint8_t id;            // CAN ID (11-bit or 29-bit)
    uint8_t extended;       // 0 = Standard ID, 1 = Extended ID
    uint8_t rtr;            // Remote Transmission Request
    uint8_t dlc;            // Data Length Code (0-8)
    uint8_t data[8];        // Data bytes
} CAN_Frame;

void ProcessCanFrame(CAN_Frame *rxFrame);
const char* CAN_GetCommandName(uint8_t cmd);
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
extern volatile uint8_t system_state; // 0x00 = System ok, 0x0X = error codes
extern volatile float set_current;
extern volatile float charging_current; 
extern volatile uint16_t charged_mah;
extern volatile uint16_t charging_power;
extern volatile uint8_t is_battery_present;
#endif // CAN_PROCESS_H
