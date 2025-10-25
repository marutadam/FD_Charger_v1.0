/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "mcp2515.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "can_process.h"
#include "param_types.h"
#include <stdlib.h>
#include "battery_charge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canTask */
osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chargerTask */
osThreadId_t chargerTaskHandle;
const osThreadAttr_t chargerTask_attributes = {
  .name = "chargerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
  .name = "adcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartRxQueue */
osMessageQueueId_t uartRxQueueHandle;
const osMessageQueueAttr_t uartRxQueue_attributes = {
  .name = "uartRxQueue"
};
/* USER CODE BEGIN PV */
// Global variables to store settings
volatile float end_voltage = 0.0f;
volatile float set_current = 0.0f;
volatile float battery_voltage = 20.0f;  // Current battery voltage
volatile float cell_voltages[6] = {3.5f, 3.6f, 3.55f, 3.58f, 3.52f, 3.54f};  // Individual cell voltages (for testing)
volatile uint8_t system_state = 0x03; // 0x00 = System ok, 0x0X = error codes
volatile float charging_current = 3.5f;
volatile uint16_t charged_mah = 12345;
volatile uint16_t charging_power = 6572;
volatile uint8_t is_battery_present = 1;
    static uint32_t fan_int_count = 0; 

volatile uint8_t rxByte;
#define CMD_MAX_LEN 64


 void uart_send_frame(const char *prefix, CAN_Frame *frame)
{
  char buffer[128];
  int len = 0;
  len += sprintf(buffer, "%sCAN ID: 0x%03lX | DLC: %d | Data: ",
           prefix ? prefix : "", (unsigned long)frame->id, frame->dlc);
  for (uint8_t i = 0; i < frame->dlc && i < 8; i++) {
    len += sprintf(buffer + len, "%02X ", frame->data[i]);
  }
  if (frame->extended) {
    len += sprintf(buffer + len, "| EXT");
  }
  if (frame->rtr) {
    len += sprintf(buffer + len, " RTR");
  }
  len += sprintf(buffer + len, "\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}
static void start_pwm_or_error(TIM_HandleTypeDef *htim, uint32_t channel);
uint8_t Flash_Read_CAN_ID(void);

volatile uint8_t CAN_ID = 0x71; // Default value, will be overwritten on startup

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void UartTask(void *argument);
void CanTaskHandler(void *argument);
void ChargerTaskHandler(void *argument);
void AdcTaskHandler(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  ChargerControllerCfg charger_cfg = {
    .current_kp = 8.0f,
    .current_ki = 2.5f,
    .voltage_kp = 6.0f,
    .voltage_ki = 1.5f,
    .integral_limit = 5.0f,
    .duty_min = 3.0f,
    .duty_max = 95.0f,
    .voltage_hysteresis = 0.1f,
    .termination_current = 0.5f,
    .termination_hold_ms = 5000,
    .cell_overvoltage_limit = 4.25f,
    .update_period_ms = 10
  };
  charger_controller_init(charger_cfg);
  charger_set_targets(end_voltage, set_current);

  /* Start UART interrupt-driven receive for 1 byte */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, 1);

  // Read CAN ID from flash on startup
  //CAN_ID = Flash_Read_CAN_ID();
  CAN_ID = ParamStore_Read_CAN_ID();
  if(DEBUG_INFO){
  char canid_msg[64];
  sprintf(canid_msg, "[INIT] Startup CAN_ID from flash: 0x%02X\r\n", CAN_ID);
  HAL_UART_Transmit(&huart1, (uint8_t*)canid_msg, strlen(canid_msg), HAL_MAX_DELAY);
  // Initialize MCP2515 with 125kbps CAN speed
  const char* init_msg = "[INIT] Initializing MCP2515 at 125kbps...\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)init_msg, strlen(init_msg), HAL_MAX_DELAY);
  }
  // Add small delay for MCP2515 power-up
  HAL_Delay(100);

  MCP2515_ERROR result = MCP2515_Init(&hspi1, CAN_125KBPS);

  if (result == MCP2515_OK) {
    if(DEBUG_INFO){
      const char* success_init_msg = "[INIT] MCP2515 initialization successful.\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)success_init_msg, strlen(success_init_msg), HAL_MAX_DELAY);
    }
    const char* success_msg = "[INIT] MCP2515 initialized successfully!\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)success_msg, strlen(success_msg), HAL_MAX_DELAY);
  } else {
    char error_msg[100];  
      sprintf(error_msg, "[ERROR] MCP2515 initialization failed! Error: %d\r\n", result);
      HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
    // Try to read a register to test SPI communication
    HAL_Delay(10);
    uint8_t test_read = MCP2515_ReadRegister(&hspi1, MCP2515_CANSTAT);
    if(DEBUG_INFO) {
      sprintf(error_msg, "[INIT] CANSTAT register read: 0x%02X\r\n", test_read);
      HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
    }
    // Continue anyway to allow debugging
  }
if (DEBUG_INFO) {
  const char* ready_msg = "[INIT] Ready to receive CAN messages...\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)ready_msg, strlen(ready_msg), HAL_MAX_DELAY);
}


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartRxQueue */
  uartRxQueueHandle = osMessageQueueNew (5, 20, &uartRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of uartTask */
  uartTaskHandle = osThreadNew(UartTask, NULL, &uartTask_attributes);
  if (uartTaskHandle == NULL) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"[ERROR] UART thread error!\r\n", 29, HAL_MAX_DELAY);
  }

  /* creation of canTask */
  canTaskHandle = osThreadNew(CanTaskHandler, NULL, &canTask_attributes);
if (canTaskHandle == NULL) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"[ERROR] CAN thread error!\r\n", 28, HAL_MAX_DELAY);
}
  /* creation of chargerTask */
  chargerTaskHandle = osThreadNew(ChargerTaskHandler, NULL, &chargerTask_attributes);
  if (chargerTaskHandle == NULL) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"[ERROR] Charger thread error!\r\n", 32, HAL_MAX_DELAY);
  }

  /* creation of adcTask */
  adcTaskHandle = osThreadNew(AdcTaskHandler, NULL, &adcTask_attributes);
  if (adcTaskHandle == NULL) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"[ERROR] ADC thread error!\r\n", 28, HAL_MAX_DELAY);
}

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, 1); // uruchom przerwanie odbioru

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // This should never be reached as control is taken by FreeRTOS scheduler
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 666;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  start_pwm_or_error(&htim4, TIM_CHANNEL_2);
  start_pwm_or_error(&htim4, TIM_CHANNEL_1);
  start_pwm_or_error(&htim3, TIM_CHANNEL_1);
  start_pwm_or_error(&htim3, TIM_CHANNEL_2);
  start_pwm_or_error(&htim3, TIM_CHANNEL_3);
  start_pwm_or_error(&htim3, TIM_CHANNEL_4);
  start_pwm_or_error(&htim2, TIM_CHANNEL_1);
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUILTIN_LED_Pin|LED_WS2812C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A_Pin|B_Pin|C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUILTIN_LED_Pin LED_WS2812C_Pin */
  GPIO_InitStruct.Pin = BUILTIN_LED_Pin|LED_WS2812C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_PROG_BTN_Pin */
  GPIO_InitStruct.Pin = CAN_PROG_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAN_PROG_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_TACH_Pin */
  GPIO_InitStruct.Pin = FAN_TACH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FAN_TACH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_INT_Pin */
  GPIO_InitStruct.Pin = SPI_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin C_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  /*Configure GPIO pin : PA3 (MCP2515 INT) */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on falling edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // Internal pull-up
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Enable EXTI3 interrupt */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Helper to start PWM and call Error_Handler on failure
static void start_pwm_or_error(TIM_HandleTypeDef *htim, uint32_t channel)
{
  if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  EXTI line detection callback
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
        // char msg[64];

      
  if (GPIO_Pin == FAN_TACH_Pin) {
    fan_int_count++;
    if (fan_int_count % 100 == 0) {
      HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);
      // char msg[64];
      // sprintf(msg, "FAN INT occurred %lu times\r\n", fan_int_count);
      // HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
  }

  if (GPIO_Pin == GPIO_PIN_3) {
    // MCP2515 INT pin triggered - notify CAN task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(canTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/**
 * @brief UART Rx complete callback
 * This is called from HAL when one byte is received via interrupt.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   // Toggle onboard LED (PC13) on each received byte


 if (huart->Instance == USART1) {
  // Wysyłamy bajt do kolejki (z ISR)
  osMessageQueuePut(uartRxQueueHandle, (const void *)&rxByte, 0, 0);

  // Uruchamiamy ponownie odbiór następnego bajtu
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, 1);
  }
}
void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_UartTask */
/**
  * @brief  Function implementing the uartTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_UartTask */
void UartTask(void *argument)
{
  /* USER CODE BEGIN 5 */

 uint8_t c;
    char cmd[CMD_MAX_LEN];
    uint8_t idx = 0;
    uint8_t last_was_eol = 0;

    if (DEBUG_UART_TASK) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] UART Task started\r\n", 26, HAL_MAX_DELAY);
}
for (;;) {
    if (osMessageQueueGet(uartRxQueueHandle, &c, NULL, osWaitForever) == osOK) {
        if ((c == '\r' || c == '\n')) {
            if (!last_was_eol) {
                cmd[idx] = '\0';

                // Parse SETID command
                if ((strncmp(cmd, "SETID=0x", 8) == 0 || strncmp(cmd, "SETID=0X", 8) == 0) && strlen(cmd) == 10) {
                    uint8_t new_id = (uint8_t)strtol(cmd + 8, NULL, 16);
                    ParamStore_Save_CAN_ID(new_id);
                    CAN_ID = ParamStore_Read_CAN_ID(); // update global CAN_ID
                    char msg[32];
                    sprintf(msg, "[RESPONSE] CAN_ID set to 0x%02X\r\n", CAN_ID);
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                }
                // Parse GETID command
                else if (strcmp(cmd, "GETID") == 0) {
                    uint8_t current_id = ParamStore_Read_CAN_ID();
                    PrintAllParamsToUART();
                    char msg[32];
                    sprintf(msg, "[RESPONSE] CAN_ID is 0x%02X\r\n", current_id);
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                }
                else {
                    // Respond with error for unknown command
                    const char *err_msg = "[RESPONSE] Unknown command\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, strlen(err_msg), HAL_MAX_DELAY);
                }
                idx = 0;
                last_was_eol = 1;
            }
        } else if (idx < CMD_MAX_LEN - 1) {
            cmd[idx++] = c;
            last_was_eol = 0;
        }
    }
    osDelay(10);
}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CanTaskHandler */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanTaskHandler */
void CanTaskHandler(void *argument)
{
  /* USER CODE BEGIN CanTaskHandler */
  CAN_Frame rxFrame;
  char uart_buffer[100];
  
  // Send startup message
  const char* task_start = "[INIT] CAN Task started\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)task_start, strlen(task_start), HAL_MAX_DELAY);
  
  
  /* Infinite loop */
  for(;;)
  {

    // Wait for notification from ISR (blocking, efficient)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Process all available CAN messages
    while (MCP2515_CheckReceive(&hspi1)) {
        // Read CAN message
        if (MCP2515_ReadMessage(&hspi1, &rxFrame) == MCP2515_OK) {


    // Modular CAN frame processing
    ProcessCanFrame(&rxFrame);
        }
    }
    
    // Check for CAN errors
    uint8_t error = MCP2515_CheckError(&hspi1);
    if (error != 0) {
        int len = sprintf(uart_buffer, "CAN Error: 0x%02X\r\n", error);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
        
        // Clear error flags by writing 0 to EFLG register
        MCP2515_WriteRegister(&hspi1, MCP2515_EFLG, 0x00);
        
        // Also clear interrupt flags
        MCP2515_WriteRegister(&hspi1, MCP2515_CANINTF, 0x00);
    }
  }
  /* USER CODE END CanTaskHandler */
}

/* USER CODE BEGIN Header_ChargerTaskHandler */
/**
* @brief Function implementing the chargerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChargerTaskHandler */
void ChargerTaskHandler(void *argument)
{
  /* USER CODE BEGIN ChargerTaskHandler */
  /* Infinite loop */
  for(;;)
  {
    // ChargerMeasurements meas;
    // meas.pack_voltage = battery_voltage;
    // meas.charge_current = charging_current;
    // float max_cell = cell_voltages[0];
    // for (uint8_t i = 1; i < 6; ++i) {
    //   if (cell_voltages[i] > max_cell) {
    //     max_cell = cell_voltages[i];
    //   }
    // }
    // meas.max_cell_voltage = max_cell;
    // charger_update(&meas);
    // uint16_t delay_ms = charger_get_update_period_ms();
    // if (delay_ms == 0) {
    //   delay_ms = 10;
    // }

    // HAL_UART_Transmit(&huart1, (uint8_t*)"[CHARGER] PWM ON (Pulse=500)\r\n", 29, HAL_MAX_DELAY);
    // __disable_irq();
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
    // __enable_irq();
    // osDelay(500); // 500 ms

    // HAL_UART_Transmit(&huart1, (uint8_t*)"[CHARGER] PWM OFF (Pulse=0)\r\n", 28, HAL_MAX_DELAY);
    // __disable_irq();
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    // __enable_irq();
    osDelay(1000); // 1 sekunda
}
  /* USER CODE END ChargerTaskHandler */
}

/* USER CODE BEGIN Header_AdcTaskHandler */
/**
* @brief Function implementing the adcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AdcTaskHandler */
void AdcTaskHandler(void *argument)
{
  /* USER CODE BEGIN AdcTaskHandler */
  char msg[64];
  uint8_t found = 0;
  // Scan I2C addresses 0x03 to 0x77
  HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] AdcTaskHandler started\r\n", 30, HAL_MAX_DELAY);
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 10) == HAL_OK) {
      int len = sprintf(msg, "[ADC] I2C device found at 0x%02X\r\n", addr);
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
      if (addr == 0x48) {
        found = 1;
      }
    }
    osDelay(2);
  }
  if (found) {
    sprintf(msg, "[ADC] ADS1115 detected at 0x48!\r\n");
  } else {
    sprintf(msg, "[ERROR] ADS1115 NOT detected!\r\n");
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

mux_set_channel(CELL_1);


  // Infinite loop (or repeat scan if you want)
  for(;;) {
    // Measure voltage on ADS1115 channel 3 (AIN3)
        VoltageValues values = ads1115_read_all_voltages(&hi2c1);
        if(DEBUG_ADC_TASK)
          print_all_voltages_uart(&values);
        osDelay(1000);
  }
  /* USER CODE END AdcTaskHandler */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  char uart_buffer[100];
  int len = sprintf(uart_buffer, "[ERROR] PWM Error!\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

