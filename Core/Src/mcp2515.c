/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mcp2515.c
  * @brief          : MCP2515 CAN Controller Driver Implementation
  ******************************************************************************
  * @attention
  *
  * MCP2515 CAN controller driver for STM32 HAL
  * This driver provides functions to initialize and communicate with MCP2515
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mcp2515.h"
#include "main.h"

/* CS Pin Configuration - PA4 is used as CS */
#define MCP2515_CS_PORT GPIOA
#define MCP2515_CS_PIN  GPIO_PIN_4

/* Private variables */
static SPI_HandleTypeDef *mcp2515_spi;

/* Chip Select Functions */
void MCP2515_Select(void) {
    HAL_GPIO_WritePin(MCP2515_CS_PORT, MCP2515_CS_PIN, GPIO_PIN_RESET);
}

void MCP2515_Deselect(void) {
    HAL_GPIO_WritePin(MCP2515_CS_PORT, MCP2515_CS_PIN, GPIO_PIN_SET);
}

/* Low-level SPI Functions */
void MCP2515_WriteByte(SPI_HandleTypeDef *hspi, uint8_t data) {
    HAL_SPI_Transmit(hspi, &data, 1, HAL_MAX_DELAY);
}

uint8_t MCP2515_ReadByte(SPI_HandleTypeDef *hspi) {
    uint8_t data = 0;
    HAL_SPI_Receive(hspi, &data, 1, HAL_MAX_DELAY);
    return data;
}

void MCP2515_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t value) {
    MCP2515_Select();
    MCP2515_WriteByte(hspi, MCP2515_WRITE);
    MCP2515_WriteByte(hspi, address);
    MCP2515_WriteByte(hspi, value);
    MCP2515_Deselect();
}

uint8_t MCP2515_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t address) {
    uint8_t value;
    MCP2515_Select();
    MCP2515_WriteByte(hspi, MCP2515_READ);
    MCP2515_WriteByte(hspi, address);
    value = MCP2515_ReadByte(hspi);
    MCP2515_Deselect();
    return value;
}

void MCP2515_ModifyRegister(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t mask, uint8_t value) {
    MCP2515_Select();
    MCP2515_WriteByte(hspi, MCP2515_BIT_MODIFY);
    MCP2515_WriteByte(hspi, address);
    MCP2515_WriteByte(hspi, mask);
    MCP2515_WriteByte(hspi, value);
    MCP2515_Deselect();
}

/* Reset MCP2515 */
MCP2515_ERROR MCP2515_Reset(SPI_HandleTypeDef *hspi) {
    MCP2515_Select();
    MCP2515_WriteByte(hspi, MCP2515_RESET);
    MCP2515_Deselect();
    HAL_Delay(10);
    return MCP2515_OK;
}

/* Set Operating Mode */
MCP2515_ERROR MCP2515_SetMode(SPI_HandleTypeDef *hspi, uint8_t mode) {
    MCP2515_ModifyRegister(hspi, MCP2515_CANCTRL, 0xE0, mode);
    HAL_Delay(10);
    
    // Verify mode change
    uint8_t regValue = MCP2515_ReadRegister(hspi, MCP2515_CANCTRL);
    if ((regValue & 0xE0) == mode) {
        return MCP2515_OK;
    }
    return MCP2515_FAIL;
}

/* Configure CAN Speed */
static MCP2515_ERROR MCP2515_ConfigRate(SPI_HandleTypeDef *hspi, CAN_SPEED speed) {
    uint8_t cfg1, cfg2, cfg3;
    
    // Configuration for 8MHz crystal
    switch (speed) {
        case CAN_125KBPS:
            cfg1 = 0x01; // BRP = 1, SJW = 1
            cfg2 = 0xB1; // BTLMODE = 1, SAM = 0, PHSEG1 = 3, PRSEG = 1
            cfg3 = 0x85; // PHSEG2 = 5, WAKFIL = 0
            break;
        case CAN_250KBPS:
            cfg1 = 0x00; // BRP = 0, SJW = 1
            cfg2 = 0xB1;
            cfg3 = 0x85;
            break;
        case CAN_500KBPS:
            cfg1 = 0x00; // BRP = 0, SJW = 1
            cfg2 = 0x90; // BTLMODE = 1, SAM = 0, PHSEG1 = 2, PRSEG = 0
            cfg3 = 0x82; // PHSEG2 = 2, WAKFIL = 0
            break;
        case CAN_1000KBPS:
            cfg1 = 0x00;
            cfg2 = 0x80;
            cfg3 = 0x80;
            break;
        default:
            // Default to 125kbps
            cfg1 = 0x01;
            cfg2 = 0xB1;
            cfg3 = 0x85;
            break;
    }
    
    MCP2515_WriteRegister(hspi, MCP2515_CNF1, cfg1);
    MCP2515_WriteRegister(hspi, MCP2515_CNF2, cfg2);
    MCP2515_WriteRegister(hspi, MCP2515_CNF3, cfg3);
    
    return MCP2515_OK;
}

/* Initialize MCP2515 */
MCP2515_ERROR MCP2515_Init(SPI_HandleTypeDef *hspi, CAN_SPEED speed) {
    mcp2515_spi = hspi;
    
    // Ensure CS is high initially
    MCP2515_Deselect();
    HAL_Delay(50);
    
    // Reset MCP2515
    MCP2515_Reset(hspi);
    HAL_Delay(50);  // Longer delay after reset
    
    // Set configuration mode
    if (MCP2515_SetMode(hspi, MCP2515_MODE_CONFIG) != MCP2515_OK) {
        return MCP2515_FAIL;
    }
    
    // Configure CAN speed
    MCP2515_ConfigRate(hspi, speed);
    
    // Enable interrupts for RX buffers
    MCP2515_WriteRegister(hspi, MCP2515_CANINTE, MCP2515_RX0IF | MCP2515_RX1IF);
    
    // DISABLE FILTERS - Accept ALL CAN messages (for debugging)
    // Configure RX buffer 0 - Receive all messages (turn off filters)
    MCP2515_WriteRegister(hspi, MCP2515_RXB0CTRL, 0x60); // 0x60 = receive all valid messages
    
    // Configure RX buffer 1 - Receive all messages (turn off filters)
    MCP2515_WriteRegister(hspi, MCP2515_RXB1CTRL, 0x60); // 0x60 = receive all valid messages
    
    // Set normal mode
    if (MCP2515_SetMode(hspi, MCP2515_MODE_NORMAL) != MCP2515_OK) {
        return MCP2515_FAIL;
    }
    
    return MCP2515_OK;
}

/* Check if message received */
uint8_t MCP2515_CheckReceive(SPI_HandleTypeDef *hspi) {
    uint8_t status = MCP2515_ReadRegister(hspi, MCP2515_CANINTF);
    return (status & (MCP2515_RX0IF | MCP2515_RX1IF));
}

/* Check for errors */
uint8_t MCP2515_CheckError(SPI_HandleTypeDef *hspi) {
    uint8_t eflg = MCP2515_ReadRegister(hspi, MCP2515_EFLG);
    return eflg;
}

/* Read CAN Message */
MCP2515_ERROR MCP2515_ReadMessage(SPI_HandleTypeDef *hspi, CAN_Frame *frame) {
    uint8_t status = MCP2515_ReadRegister(hspi, MCP2515_CANINTF);
    uint8_t buffer_offset;
    
    // Check which buffer has data
    if (status & MCP2515_RX0IF) {
        buffer_offset = MCP2515_RXB0SIDH;
    } else if (status & MCP2515_RX1IF) {
        buffer_offset = MCP2515_RXB1SIDH;
    } else {
        return MCP2515_NOMSG;
    }
    
    // Read message
    MCP2515_Select();
    MCP2515_WriteByte(hspi, MCP2515_READ);
    MCP2515_WriteByte(hspi, buffer_offset);
    
    // Read ID
    uint8_t sidh = MCP2515_ReadByte(hspi);
    uint8_t sidl = MCP2515_ReadByte(hspi);
    uint8_t eid8 = MCP2515_ReadByte(hspi);
    uint8_t eid0 = MCP2515_ReadByte(hspi);
    
    // Check if extended frame
    frame->extended = (sidl & 0x08) ? 1 : 0;
    
    if (frame->extended) {
        // Extended ID (29-bit)
        frame->id = ((uint32_t)sidh << 21) | ((uint32_t)(sidl & 0xE0) << 13) |
                    ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
    } else {
        // Standard ID (11-bit)
        frame->id = ((uint32_t)sidh << 3) | ((uint32_t)sidl >> 5);
    }
    
    // Read DLC
    uint8_t dlc_reg = MCP2515_ReadByte(hspi);
    frame->dlc = dlc_reg & 0x0F;
    frame->rtr = (dlc_reg & 0x40) ? 1 : 0;
    
    // Read data
    for (uint8_t i = 0; i < frame->dlc && i < 8; i++) {
        frame->data[i] = MCP2515_ReadByte(hspi);
    }
    
    MCP2515_Deselect();
    
    // Clear interrupt flag
    if (status & MCP2515_RX0IF) {
        MCP2515_ModifyRegister(hspi, MCP2515_CANINTF, MCP2515_RX0IF, 0x00);
    } else if (status & MCP2515_RX1IF) {
        MCP2515_ModifyRegister(hspi, MCP2515_CANINTF, MCP2515_RX1IF, 0x00);
    }
    
    return MCP2515_OK;
}

/* Send CAN Message */
MCP2515_ERROR MCP2515_SendMessage(SPI_HandleTypeDef *hspi, CAN_Frame *frame) {
    // Check if TX buffer 0 is free
    uint8_t status = MCP2515_ReadRegister(hspi, MCP2515_TXB0CTRL);
    if (status & 0x08) { // TXREQ bit
        return MCP2515_ALLTXBUSY;
    }
    
    // Write message to TX buffer 0
    MCP2515_Select();
    MCP2515_WriteByte(hspi, MCP2515_WRITE);
    MCP2515_WriteByte(hspi, MCP2515_TXB0SIDH);
    
    if (frame->extended) {
        // Extended ID
        MCP2515_WriteByte(hspi, (uint8_t)(frame->id >> 21));
        MCP2515_WriteByte(hspi, (uint8_t)(((frame->id >> 13) & 0xE0) | 0x08 | ((frame->id >> 16) & 0x03)));
        MCP2515_WriteByte(hspi, (uint8_t)(frame->id >> 8));
        MCP2515_WriteByte(hspi, (uint8_t)frame->id);
    } else {
        // Standard ID
        MCP2515_WriteByte(hspi, (uint8_t)(frame->id >> 3));
        MCP2515_WriteByte(hspi, (uint8_t)(frame->id << 5));
        MCP2515_WriteByte(hspi, 0x00);
        MCP2515_WriteByte(hspi, 0x00);
    }
    
    // Write DLC
    uint8_t dlc = frame->dlc & 0x0F;
    if (frame->rtr) {
        dlc |= 0x40;
    }
    MCP2515_WriteByte(hspi, dlc);
    
    // Write data
    for (uint8_t i = 0; i < frame->dlc && i < 8; i++) {
        MCP2515_WriteByte(hspi, frame->data[i]);
    }
    
    MCP2515_Deselect();
    
    // Request transmission
    MCP2515_WriteRegister(hspi, MCP2515_TXB0CTRL, 0x08);
    
    return MCP2515_OK;
}
