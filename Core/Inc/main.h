/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

// #include "can_process.h"
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Ensure CAN_Frame is defined for uart_send_frame
#include "mcp2515.h"
#include "battery_balance.h"
#include "battery_charge.h"
#include "flash_param_store.h"
#include "param_types.h"
#include "adc_mux.h" 
#include "stm32f4xx_hal_gpio.h"
#include "ws2812c.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUILTIN_LED_Pin GPIO_PIN_13
#define BUILTIN_LED_GPIO_Port GPIOC
#define CAN_PROG_BTN_Pin GPIO_PIN_14
#define CAN_PROG_BTN_GPIO_Port GPIOC
#define LED_WS2812C_Pin GPIO_PIN_15
#define LED_WS2812C_GPIO_Port GPIOC
#define PWM_BUCK_Pin GPIO_PIN_0
#define PWM_BUCK_GPIO_Port GPIOA
#define FAN_TACH_Pin GPIO_PIN_2
#define FAN_TACH_GPIO_Port GPIOA
#define SPI_INT_Pin GPIO_PIN_3
#define SPI_INT_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define CELL1_PWM_Pin GPIO_PIN_0
#define CELL1_PWM_GPIO_Port GPIOB
#define CELL2_PWM_Pin GPIO_PIN_1
#define CELL2_PWM_GPIO_Port GPIOB
#define A_Pin GPIO_PIN_12
#define A_GPIO_Port GPIOB
#define B_Pin GPIO_PIN_13
#define B_GPIO_Port GPIOB
#define C_Pin GPIO_PIN_14
#define C_GPIO_Port GPIOB
#define CELL3_PWM_Pin GPIO_PIN_4
#define CELL3_PWM_GPIO_Port GPIOB
#define CELL4_PWM_Pin GPIO_PIN_5
#define CELL4_PWM_GPIO_Port GPIOB
#define CELL5_PWM_Pin GPIO_PIN_6
#define CELL5_PWM_GPIO_Port GPIOB
#define CELL6_PWM_Pin GPIO_PIN_7
#define CELL6_PWM_GPIO_Port GPIOB
#define FLASH_ADDR_CAN_ID  0x0807FFF0


//Debugging macros
#define DEBUG_INFO 1
#define DEBUG_UART_TASK     1
#define DEBUG_CAN_TASK      1
#define DEBUG_CHARGER_TASK  1
#define DEBUG_ADC_TASK      1

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

// Externs for CAN processing module
extern UART_HandleTypeDef huart1;
extern volatile uint8_t CAN_ID;
extern volatile float end_voltage;
extern volatile float set_current;
extern volatile float battery_voltage;
extern volatile float cell_voltages[6];
extern volatile bool is_battery_charging;
void uart_send_frame(const char *prefix, CAN_Frame *frame);
void ProcessCanFrame(CAN_Frame *rxFrame);
