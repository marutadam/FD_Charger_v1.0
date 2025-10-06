/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mcp2515.h
  * @brief          : MCP2515 CAN Controller Driver Header
  ******************************************************************************
  * @attention
  *
  * MCP2515 CAN controller driver for STM32 HAL
  * This driver provides functions to initialize and communicate with MCP2515
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MCP2515_H
#define __MCP2515_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* MCP2515 SPI Instructions */
#define MCP2515_RESET           0xC0
#define MCP2515_READ            0x03
#define MCP2515_WRITE           0x02
#define MCP2515_RTS             0x80
#define MCP2515_READ_STATUS     0xA0
#define MCP2515_RX_STATUS       0xB0
#define MCP2515_BIT_MODIFY      0x05

/* MCP2515 Registers */
#define MCP2515_CANSTAT         0x0E
#define MCP2515_CANCTRL         0x0F
#define MCP2515_CNF3            0x28
#define MCP2515_CNF2            0x29
#define MCP2515_CNF1            0x2A
#define MCP2515_CANINTE         0x2B
#define MCP2515_CANINTF         0x2C
#define MCP2515_EFLG            0x2D

/* RX Buffer 0 */
#define MCP2515_RXB0CTRL        0x60
#define MCP2515_RXB0SIDH        0x61
#define MCP2515_RXB0SIDL        0x62
#define MCP2515_RXB0EID8        0x63
#define MCP2515_RXB0EID0        0x64
#define MCP2515_RXB0DLC         0x65
#define MCP2515_RXB0DATA        0x66

/* RX Buffer 1 */
#define MCP2515_RXB1CTRL        0x70
#define MCP2515_RXB1SIDH        0x71
#define MCP2515_RXB1SIDL        0x72
#define MCP2515_RXB1EID8        0x73
#define MCP2515_RXB1EID0        0x74
#define MCP2515_RXB1DLC         0x75
#define MCP2515_RXB1DATA        0x76

/* TX Buffer 0 */
#define MCP2515_TXB0CTRL        0x30
#define MCP2515_TXB0SIDH        0x31
#define MCP2515_TXB0SIDL        0x32
#define MCP2515_TXB0EID8        0x33
#define MCP2515_TXB0EID0        0x34
#define MCP2515_TXB0DLC         0x35
#define MCP2515_TXB0DATA        0x36

/* Mode Configuration */
#define MCP2515_MODE_NORMAL     0x00
#define MCP2515_MODE_SLEEP      0x20
#define MCP2515_MODE_LOOPBACK   0x40
#define MCP2515_MODE_LISTENONLY 0x60
#define MCP2515_MODE_CONFIG     0x80

/* Interrupt Flags */
#define MCP2515_RX0IF           0x01
#define MCP2515_RX1IF           0x02
#define MCP2515_TX0IF           0x04
#define MCP2515_TX1IF           0x08
#define MCP2515_TX2IF           0x10
#define MCP2515_ERRIF           0x20
#define MCP2515_WAKIF           0x40
#define MCP2515_MERRF           0x80

/* CAN Speed Settings (for 8MHz crystal) */
typedef enum {
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_50KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
} CAN_SPEED;

/* CAN Frame Structure */
typedef struct {
    uint32_t id;            // CAN ID (11-bit or 29-bit)
    uint8_t extended;       // 0 = Standard ID, 1 = Extended ID
    uint8_t rtr;            // Remote Transmission Request
    uint8_t dlc;            // Data Length Code (0-8)
    uint8_t data[8];        // Data bytes
} CAN_Frame;

/* Error codes */
typedef enum {
    MCP2515_OK = 0,
    MCP2515_FAIL,
    MCP2515_ALLTXBUSY,
    MCP2515_NOMSG
} MCP2515_ERROR;

/* Function Prototypes */
MCP2515_ERROR MCP2515_Init(SPI_HandleTypeDef *hspi, CAN_SPEED speed);
MCP2515_ERROR MCP2515_Reset(SPI_HandleTypeDef *hspi);
MCP2515_ERROR MCP2515_SetMode(SPI_HandleTypeDef *hspi, uint8_t mode);
MCP2515_ERROR MCP2515_ReadMessage(SPI_HandleTypeDef *hspi, CAN_Frame *frame);
MCP2515_ERROR MCP2515_SendMessage(SPI_HandleTypeDef *hspi, CAN_Frame *frame);
uint8_t MCP2515_CheckReceive(SPI_HandleTypeDef *hspi);
uint8_t MCP2515_CheckError(SPI_HandleTypeDef *hspi);

/* Low-level SPI functions */
void MCP2515_Select(void);
void MCP2515_Deselect(void);
void MCP2515_WriteByte(SPI_HandleTypeDef *hspi, uint8_t data);
uint8_t MCP2515_ReadByte(SPI_HandleTypeDef *hspi);
void MCP2515_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t value);
uint8_t MCP2515_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t address);
void MCP2515_ModifyRegister(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t mask, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* __MCP2515_H */
