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

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
