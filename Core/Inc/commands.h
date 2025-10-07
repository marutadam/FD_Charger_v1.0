#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

#define CAN_ID 0x71
//  Charger command codes
#define CMD_GET_STATUS              0x01
#define CMD_START_CHARGING          0x02
#define CMD_STOP_CHARGING           0x03
#define CMD_RESET                   0x04
#define CMD_SET_END_VOLTAGE         0x05
#define CMD_SET_CURRENT             0x06
#define CMD_IS_BATT_PRESENT         0x07
#define CMD_READ_CURRENT_VOLTAGE    0x08
#define CMD_READ_CURRENT_CURRENT    0x09
#define CMD_READ_MAH                0x0A
#define CMD_READ_POWER              0x0B


static inline const char* CAN_GetCommandName(uint8_t cmd_code) {
    switch (cmd_code) {
        case CMD_GET_STATUS:            return "GET_STATUS";
        case CMD_START_CHARGING:        return "START_CHARGING";
        case CMD_STOP_CHARGING:         return "STOP_CHARGING";
        case CMD_RESET:                 return "RESET";
        case CMD_SET_END_VOLTAGE:       return "SET_END_VOLTAGE";
        case CMD_SET_CURRENT:           return "SET_CURRENT";
        case CMD_IS_BATT_PRESENT:       return "IS_BATT_PRESENT";
        case CMD_READ_CURRENT_VOLTAGE:  return "READ_CURRENT_VOLTAGE";
        case CMD_READ_CURRENT_CURRENT:  return "READ_CURRENT_CURRENT";
        case CMD_READ_MAH:              return "READ_MAH";
        case CMD_READ_POWER:            return "READ_POWER";
        default:                        return "UNKNOWN_COMMAND";
    }
}


#endif // COMMANDS_H
