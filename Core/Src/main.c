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
#include <math.h>
#include "can_process.h"
#include "param_types.h"
#include <stdlib.h>
#include "battery_charge.h"
#include "battery_balance.h"
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

// Global balance enable flag
volatile uint8_t can_balance_enabled = 0;

/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 512 * 4,  // 2KB for JSON telemetry buffer (~600 bytes)
  .priority = (osPriority_t) osPriorityBelowNormal,  // Non-critical, can be preempted
};
/* Definitions for canTask */
osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,  // CAN is time-critical
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
/* Definitions for balanceTask */
osThreadId_t balanceTaskHandle;
const osThreadAttr_t balanceTask_attributes = {
  .name = "balanceTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartRxQueue */
osMessageQueueId_t uartRxQueueHandle;
const osMessageQueueAttr_t uartRxQueue_attributes = {
  .name = "uartRxQueue"
};
/* Definitions for voltage data mutex */
osMutexId_t voltageDataMutexHandle;
const osMutexAttr_t voltageDataMutex_attributes = {
  .name = "voltageDataMutex"
};
/* USER CODE BEGIN PV */
// Global variables to store settings
volatile float end_voltage = 24.6f;
volatile float end_voltage_storage = 20.2f; // default ~3.7V per cell for 6S
volatile float set_current = 2.0f;
volatile uint8_t system_state = 0x00; // 0x00 = System ok, 0x0X = error codes
volatile uint16_t charged_mah = 0;
volatile uint16_t charging_power = 0;
volatile uint8_t is_battery_present = 0;
volatile bool is_battery_charging = false;
volatile uint8_t manual_balance_mode = 0;  // 1 = manual PWM via CAN, skip auto logic


volatile ChargerFaultStatus charger_faults = {0};
//Fan RPM measurement variables
volatile uint32_t fan_int_count = 0;
volatile uint32_t fan_rpm = 0;

volatile uint8_t rxByte;
VoltageValues current_battery_voltages = {
  .cell = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  .cell_raw = {0, 0, 0, 0, 0, 0},
  .battery_voltage = 0.0f,
  .shunt_voltage = 0.0f,
  .buck_voltage = 0.0f,
  .current = 0.0f
};
#define CMD_MAX_LEN 64

 void uart_send_frame(const char *prefix, CAN_Frame *frame)
{
  char buffer[128];
  int len = 0;
  len += snprintf(buffer, sizeof(buffer), "%sCAN ID: 0x%03lX | DLC: %d | Data: ",
           prefix ? prefix : "", (unsigned long)frame->id, frame->dlc);
  for (uint8_t i = 0; i < frame->dlc && i < 8; i++) {
    len += snprintf(buffer + len, sizeof(buffer) - len, "%02X ", frame->data[i]);
  }
  if (frame->extended) {
    len += snprintf(buffer + len, sizeof(buffer) - len, "| EXT");
  }
  if (frame->rtr) {
    len += snprintf(buffer + len, sizeof(buffer) - len, " RTR");
  }
  len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}
static void start_pwm_or_error(TIM_HandleTypeDef *htim, uint32_t channel);

// Helper function to safely disable all PWM outputs
static inline void disable_all_pwm(void)
{
  __disable_irq();
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __enable_irq();
}

// Helper function to safely read voltage data
VoltageValues get_battery_voltages_safe(void)
{
  VoltageValues snapshot;
  osMutexAcquire(voltageDataMutexHandle, osWaitForever);
  snapshot = current_battery_voltages;
  osMutexRelease(voltageDataMutexHandle);
  return snapshot;
}

// Helper function to safely write voltage data
static inline void set_battery_voltages_safe(const VoltageValues *voltages)
{
  osMutexAcquire(voltageDataMutexHandle, osWaitForever);
  current_battery_voltages = *voltages;
  osMutexRelease(voltageDataMutexHandle);
}

#if DEBUG_TELEMETRY
static const char *charger_state_to_string_public(ChargerState state)
{
  switch (state) {
    case CHARGER_STATE_IDLE: return "IDLE";
    case CHARGER_STATE_CC: return "CC";
    case CHARGER_STATE_CV: return "CV";
    case CHARGER_STATE_COMPLETE: return "COMPLETE";
    case CHARGER_STATE_FAULT: return "FAULT";
    case CHARGER_STATE_STORAGE: return "STORAGE";
    default: return "UNKNOWN";
  }
}

static inline float safe_float(float val) {
  // Simple: if exponent bits are all 1s (NaN/inf), return 0
  // Otherwise return value as-is (even if garbage - will show in JSON)
  union { float f; uint32_t u; } conv;
  conv.f = val;
  uint32_t exp = (conv.u >> 23) & 0xFF;
  if (exp == 0xFF || exp == 0) {
    return 0.0f;  // NaN, inf, or denormal
  }
  return val;
}

static void uart_send_status_json(void)
{
  static uint32_t call_counter = 0;
  call_counter++;

  VoltageValues v = get_battery_voltages_safe();
  
  float pack_v = 0.0f;
  for (uint8_t i = 0; i < 6; ++i) {
    pack_v += safe_float(v.cell[i]);
  }

  // If battery absent, zero out pack and cell readings
  if (!is_battery_present) {
    pack_v = 0.0f;
    v.battery_voltage = 0.0f;
    v.shunt_voltage = 0.0f;
    for (uint8_t i = 0; i < 6; ++i) {
      v.cell[i] = 0.0f;
    }
  }

  uint32_t ts_ms = HAL_GetTick();

  uint8_t duty[6];
  uint32_t pwm_raw[6];
  for (uint8_t i = 0; i < 6; ++i) {
    duty[i] = balance_get_last_duty(i);
    pwm_raw[i] = balance_get_last_pulse(i);
  }

  ChargerState st = charger_get_state();
  
  float pwm = safe_float(charger_get_pwm_duty());
  float pwm_counts = safe_float(charger_get_pwm_counts_raw());
  float pwm_counts_max = safe_float(charger_get_pwm_counts_max());

  // Convert floats to int.frac manually (snprintf doesn't support %f without -u _printf_float)
  // Use abs() for fractional part to handle negative values
  
  float pack_val = pack_v;
  int pack_v_i = (int)pack_val;
  int pack_v_f = (int)(fabsf(pack_val - pack_v_i) * 1000.0f);
  
  float curr_val = safe_float(v.current);
  int curr_i = (int)curr_val;
  int curr_f = (int)(fabsf(curr_val - curr_i) * 1000.0f);
  
  float buck_val = safe_float(v.buck_voltage);
  int buck_i = (int)buck_val;
  int buck_f = (int)(fabsf(buck_val - buck_i) * 1000.0f);
  
  float batt_val = safe_float(v.battery_voltage);
  int batt_i = (int)batt_val;
  int batt_f = (int)(fabsf(batt_val - batt_i) * 1000.0f);
  
  float shunt_val = safe_float(v.shunt_voltage);
  int shunt_i = (int)shunt_val;
  int shunt_f = (int)(fabsf(shunt_val - shunt_i) * 1000.0f);
  
  float c0_val = safe_float(v.cell[0]); int c0_i = (int)c0_val; int c0_f = (int)(fabsf(c0_val - c0_i) * 1000.0f);
  float c1_val = safe_float(v.cell[1]); int c1_i = (int)c1_val; int c1_f = (int)(fabsf(c1_val - c1_i) * 1000.0f);
  float c2_val = safe_float(v.cell[2]); int c2_i = (int)c2_val; int c2_f = (int)(fabsf(c2_val - c2_i) * 1000.0f);
  float c3_val = safe_float(v.cell[3]); int c3_i = (int)c3_val; int c3_f = (int)(fabsf(c3_val - c3_i) * 1000.0f);
  float c4_val = safe_float(v.cell[4]); int c4_i = (int)c4_val; int c4_f = (int)(fabsf(c4_val - c4_i) * 1000.0f);
  float c5_val = safe_float(v.cell[5]); int c5_i = (int)c5_val; int c5_f = (int)(fabsf(c5_val - c5_i) * 1000.0f);
  
  int pwm_i = (int)pwm; int pwm_f = (int)(fabsf(pwm - pwm_i) * 100.0f);
  int pwm_cnt = (int)pwm_counts; int pwm_max = (int)pwm_counts_max;
  
  float tgt_v_val = safe_float(end_voltage); int tgt_v_i = (int)tgt_v_val; int tgt_v_f = (int)(fabsf(tgt_v_val - tgt_v_i) * 100.0f);
  float tgt_i_val = safe_float(set_current); int tgt_i_i = (int)tgt_i_val; int tgt_i_f = (int)(fabsf(tgt_i_val - tgt_i_i) * 100.0f);

    char json[600];
  int len = snprintf(
      json,
      sizeof(json),
      "{\"ts_ms\":%lu,\"pack\":{\"present\":%u,\"v\":%d.%03d,\"i\":%d.%03d,\"buck\":%d.%03d,\"battery\":%d.%03d,\"shunt\":%d.%03d},"
      "\"cells\":[%d.%03d,%d.%03d,%d.%03d,%d.%03d,%d.%03d,%d.%03d],"
      "\"balance\":{\"enabled\":%u,\"duties\":[%u,%u,%u,%u,%u,%u],\"pwm_raw\":[%lu,%lu,%lu,%lu,%lu,%lu]},"
      "\"charger\":{\"state\":\"%s\",\"pwm\":%d.%02d,\"pwm_raw\":%d,\"pwm_max\":%d,\"target_v\":%d.%02d,\"target_i\":%d.%02d,\"charged_mah\":%u,\"power_w\":%u},"
      "\"fan\":{\"rpm\":%lu,\"status\":\"%s\"}}\r\n",
      ts_ms,
      is_battery_present,
      pack_v_i, pack_v_f, curr_i, curr_f, buck_i, buck_f, batt_i, batt_f, shunt_i, shunt_f,
      c0_i, c0_f, c1_i, c1_f, c2_i, c2_f, c3_i, c3_f, c4_i, c4_f, c5_i, c5_f,
      can_balance_enabled,
      duty[0], duty[1], duty[2], duty[3], duty[4], duty[5],
      pwm_raw[0], pwm_raw[1], pwm_raw[2], pwm_raw[3], pwm_raw[4], pwm_raw[5],
      charger_state_to_string_public(st),
      pwm_i, pwm_f, pwm_cnt, pwm_max, tgt_v_i, tgt_v_f, tgt_i_i, tgt_i_f, charged_mah, charging_power,
      fan_rpm, charger_faults.fan_error ? "FAULT" : "OK");

  if (len > 0 && len < (int)sizeof(json)) {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)json, (uint16_t)len, 200);
  }
}
#else
static inline void uart_send_status_json(void) {(void)0;}
#endif

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
void BalanceTaskHandler(void *argument);

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
  
  // Disable all balancer PWM channels on startup
  balance_disable_all_cells();

  // WS2812 LEDs completely disabled - pin set to ANALOG in MX_GPIO_Init
  // HAL_GPIO_WritePin(LED_WS2812C_GPIO_Port, LED_WS2812C_Pin, GPIO_PIN_RESET);

  ChargerControllerCfg charger_cfg = {
    // PI gains tuned down to prevent oscillation and overshoot.
    // Kp provides primary response, Ki corrects for steady-state error.
    .current_kp = 15.0f,      // Reduced from 50.0f to be less aggressive
    .current_ki = 2.0f,       // Reduced from 10.0f
    .voltage_kp = 0.2f,
    .voltage_ki = 0.05f,
    .integral_limit = 600.0f,
    .duty_min = 3.0f,
    .duty_max = 95.0f,
    .voltage_hysteresis = 0.2f,
    .termination_current = 0.5f,
    .termination_hold_ms = 5000,
    .cell_overvoltage_limit = 4.25f,
    .update_period_ms = 100,
    .control_mode = CHARGER_CTRL_HYSTERESIS
  };
  charger_controller_init(charger_cfg);
  charger_set_targets(end_voltage, set_current);

  /* Start UART interrupt-driven receive for 1 byte */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, 1);

  // Read CAN ID from flash on startup
  CAN_ID = ParamStore_Read_CAN_ID();
  FlashParams startup_params = ReadAllParams();
  if (startup_params.storage_volt > 0.0f) {
    end_voltage_storage = startup_params.storage_volt;
  }
  if(DEBUG_INFO){
  char canid_msg[64];
  snprintf(canid_msg, sizeof(canid_msg), "[INIT] Startup CAN_ID from flash: 0x%02X\r\n", CAN_ID);
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
      snprintf(error_msg, sizeof(error_msg), "[ERROR] MCP2515 initialization failed! Error: %d\r\n", result);
      HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
    // Try to read a register to test SPI communication
    HAL_Delay(10);
    uint8_t test_read = MCP2515_ReadRegister(&hspi1, MCP2515_CANSTAT);
    if(DEBUG_INFO) {
      snprintf(error_msg, sizeof(error_msg), "[INIT] CANSTAT register read: 0x%02X\r\n", test_read);
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
  voltageDataMutexHandle = osMutexNew(&voltageDataMutex_attributes);
  if (voltageDataMutexHandle == NULL) {
    Error_Handler();  // Failed to create mutex
  }
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
  if (uartRxQueueHandle == NULL) {
    Error_Handler();  // Failed to create queue
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of uartTask */
  uartTaskHandle = osThreadNew(UartTask, NULL, &uartTask_attributes);

  /* creation of canTask */
  canTaskHandle = osThreadNew(CanTaskHandler, NULL, &canTask_attributes);

  /* creation of chargerTask */
  chargerTaskHandle = osThreadNew(ChargerTaskHandler, NULL, &chargerTask_attributes);

  /* creation of adcTask */
  adcTaskHandle = osThreadNew(AdcTaskHandler, NULL, &adcTask_attributes);

  /* creation of balanceTask */
  balanceTaskHandle = osThreadNew(BalanceTaskHandler, NULL, &balanceTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Validate all tasks were created successfully */
  if (uartTaskHandle == NULL || canTaskHandle == NULL || 
      chargerTaskHandle == NULL || adcTaskHandle == NULL || 
      balanceTaskHandle == NULL) {
    Error_Handler();  // Failed to create one or more tasks
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  // HAL_UART_Receive_IT already called in USER CODE BEGIN 2

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
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim4.Init.Prescaler = 9999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, BUILTIN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|LED_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A_Pin|B_Pin|C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUILTIN_LED_Pin */
  GPIO_InitStruct.Pin = BUILTIN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_WS2812C_Pin - DISABLED, set as analog to prevent driving LEDs */
  GPIO_InitStruct.Pin = LED_WS2812C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_PROG_BTN_Pin */
  GPIO_InitStruct.Pin = CAN_PROG_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAN_PROG_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_TACH_Pin */
  GPIO_InitStruct.Pin = FAN_TACH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
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

  /*Configure GPIO pin : LED_DATA_Pin */
  GPIO_InitStruct.Pin = LED_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_DATA_GPIO_Port, &GPIO_InitStruct);

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
  uint32_t last_status_tick = HAL_GetTick();

  // Always announce UART task startup
  HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] UART Task started\r\n", 26, 100);
#if DEBUG_TELEMETRY
  HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] Telemetry enabled (500ms)\r\n", 34, 100);
#else
  HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] Telemetry disabled\r\n", 27, 100);
#endif
for (;;) {
  // Use shorter timeout to allow periodic status JSON
  if (osMessageQueueGet(uartRxQueueHandle, &c, NULL, 50) == osOK) {
        if ((c == '\r' || c == '\n')) {
            if (!last_was_eol) {
                cmd[idx] = '\0';

                // Parse SETID command (accept SETID=0xNN or SETID=NN)
                if (strncmp(cmd, "SETID=", 6) == 0) {
                  long parsed = strtol(cmd + 6, NULL, 0); // base 0 accepts 0x/0X or decimal
                  if (parsed < 0 || parsed > 0xFF) {
                    const char *err = "[RESPONSE] Invalid CAN_ID (must be 0x00-0xFF)\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
                  } else {
                    uint8_t new_id = (uint8_t)parsed;
                    ParamStore_Save_CAN_ID(new_id);
                    CAN_ID = ParamStore_Read_CAN_ID(); // update global CAN_ID
                    char msg[48];
                    snprintf(msg, sizeof(msg), "[RESPONSE] CAN_ID set to 0x%02X\r\n", CAN_ID);
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                  }
                }
                // Parse GETID command
                else if (strcmp(cmd, "GETID") == 0) {
                    uint8_t current_id = ParamStore_Read_CAN_ID();
                    PrintAllParamsToUART();
                    char msg[32];
                    snprintf(msg, sizeof(msg), "[RESPONSE] CAN_ID is 0x%02X\r\n", current_id);
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
    
    // Moved outside queue check - always runs even if no RX data
    uint32_t now = HAL_GetTick();
#if DEBUG_TELEMETRY
    static uint32_t loop_counter = 0;
    loop_counter++;
    
    if ((now - last_status_tick) >= 200U) {
      uart_send_status_json();
      last_status_tick = now;
    }
#endif
    
    // Yield to other tasks to prevent starvation
    osDelay(1);
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
    
    // Check for CAN errors less frequently
    static uint32_t error_check_count = 0;
    if (++error_check_count >= 10) {  // Check errors every 10 iterations
        error_check_count = 0;
        uint8_t error = MCP2515_CheckError(&hspi1);
        if (error != 0) {
          // Clear error flags by writing 0 to EFLG register
          MCP2515_WriteRegister(&hspi1, MCP2515_EFLG, 0x00);
          // Also clear interrupt flags
          MCP2515_WriteRegister(&hspi1, MCP2515_CANINTF, 0x00);
        }
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
  static uint8_t was_charging = 0;
  static float accumulated_mah = 0.0f;
  static float filtered_power = 0.0f;
  const float alpha = 0.2f;  // Low-pass filter coefficient (0.0 = no change, 1.0 = instant)
  
  /* Infinite loop */
  for(;;)
  {
    if(is_battery_charging && is_battery_present)
    {
      // Detect start of new charging session and reset counter
      if (!was_charging) {
        charged_mah = 0;
        accumulated_mah = 0.0f;
        filtered_power = 0.0f;
        charging_power = 0;
        was_charging = 1;
      }
      
      // The PI controller is now active.
      // It will handle CC/CV phases and PWM duty cycle adjustments.
      charger_update(&current_battery_voltages);
      
      // Accumulate charged mAh: mAh = (current_A * time_ms / 3600.0)
      VoltageValues v = get_battery_voltages_safe();
      float current_a = v.current;
      float voltage_v = v.battery_voltage;
      uint32_t period_ms = charger_get_update_period_ms();
      
      if (current_a > 0.0f) {
        float mah_delta = (current_a * (float)period_ms) / 3600.0f;
        accumulated_mah += mah_delta;
        charged_mah = (uint16_t)accumulated_mah;
      }
      
      // Calculate instantaneous power and apply exponential moving average filter
      float instant_power = voltage_v * current_a;  // Watts
      filtered_power = alpha * instant_power + (1.0f - alpha) * filtered_power;
      charging_power = (uint16_t)filtered_power;  // Store as Watts
    }
    else
    {
      // If charging is globally disabled or battery is unplugged,
      // ensure the hardware is turned off.
      const char *reason = NULL;
      if (!is_battery_present) {
        reason = "battery not present";
      } else if (!is_battery_charging) {
        reason = "charging disabled flag";
      } else {
        reason = "charger idle guard";
      }
      charger_disable_with_reason(reason);
      was_charging = 0;  // Reset flag when not charging
      charging_power = 0;  // Reset power when not charging
    }

    // This function checks fan speed and can set a fault flag.
    CalculateFanRPM(charger_get_update_period_ms());

    // Print balancer duty status
    if (DEBUG_BALANCE) {
      printBalanceDuty();
    }

    // The task will now sleep for the duration configured in the charger settings.
    osDelay(charger_get_update_period_ms());
  }
  /* USER CODE END ChargerTaskHandler */
}

/* USER CODE BEGIN Header_AdcTaskHandler */
/**
* @brief Function implementing the adscTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AdcTaskHandler */
void AdcTaskHandler(void *argument)
{
  /* USER CODE BEGIN AdcTaskHandler */
  char msg[64];
  uint8_t found = 0;
  uint8_t fan_error_sent = 0;
  // Scan I2C addresses 0x03 to 0x77 only once on startup
  HAL_UART_Transmit(&huart1, (uint8_t*)"[INIT] AdcTaskHandler started\r\n", 30, HAL_MAX_DELAY);
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 10) == HAL_OK) {
      int len = snprintf(msg, sizeof(msg), "[ADC] ADS1115 device found at 0x%02X\r\n", addr);
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
      if (addr == 0x48) {
        found = 1;
      }
    }
    osDelay(2);
  }
  if (!found) {
    snprintf(msg, sizeof(msg), "[ERROR] ADS1115 NOT detected!\r\n");
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  // Infinite loop
  for (;;) {
    VoltageValues new_voltages = ads1115_read_all_voltages(&hi2c1);
    set_battery_voltages_safe(&new_voltages);  // Thread-safe update with mutex
    
    is_battery_present = (new_voltages.battery_voltage > 12.0f) ? 1U : 0U;

    if (!is_battery_present) {
      charger_faults.battery_not_present = true;
    } else {
      charger_faults.battery_not_present = false;
    }

    if (new_voltages.buck_voltage > 26.0f) {
      HAL_UART_Transmit(&huart1,
                        (const uint8_t *)"[ERROR] Buck voltage too high!\r\n",
                        32,
                        HAL_MAX_DELAY);
      disable_all_pwm();
      charger_disable_with_reason("buck overvoltage");
    }

    if (charger_faults.fan_error && fan_error_sent == 0) {
      HAL_UART_Transmit(&huart1,
                        (const uint8_t *)"[ERROR] Fan error detected!\r\n",
                        30,
                        HAL_MAX_DELAY);
      fan_error_sent = 1;
      disable_all_pwm();
      charger_disable_with_reason("fan error detected");
    }
    else if (!charger_faults.fan_error && fan_error_sent) {
      HAL_UART_Transmit(&huart1,
                        (const uint8_t *)"[FAN] Fan error cleared!\r\n",
                        27,
                        HAL_MAX_DELAY);
      fan_error_sent = 0;
    }

    osDelay(50);
  }
  /* USER CODE END AdcTaskHandler */
}

/* USER CODE BEGIN Header_BalanceTaskHandler */
/**
* @brief Function implementing the balanceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BalanceTaskHandler */
void BalanceTaskHandler(void *argument)
{
  /* USER CODE BEGIN BalanceTaskHandler */
  const uint32_t balance_period_ms = 1000U;
  const float balance_deadband_v = 0.010f;   // 10 mV window around the lowest cell
  const float min_cell_for_balance = 3.0f;   // skip balancing if pack is deeply discharged

  balance_controller_configure(200.0f,   // ~20% duty for a 50 mV delta
                               0.02f,    // enable threshold in volts (20mV)
                               0.01f,    // disable threshold (10mV)
                               250U,     // minimum on-time in ms
                               60U);     // max PWM duty percent

  // IMPORTANT: Balancing is DISABLED by default and only enabled via CAN command
  can_balance_enabled = 0;

  /* Infinite loop */
  for(;;)
  {
    // In manual mode we leave CAN-set PWM values untouched
    if (manual_balance_mode) {
      osDelay(balance_period_ms);
      continue;
    }

    VoltageValues snapshot = get_battery_voltages_safe();  // Thread-safe read with mutex
    bool can_balance = can_balance_enabled;  // Use global enable flag

    // Track the minimum and maximum cell to understand pack imbalance.
    float lowest = snapshot.cell[0];
    float highest = snapshot.cell[0];
    for (uint8_t i = 1; i < 6; ++i) {
      if (snapshot.cell[i] < lowest) {
        lowest = snapshot.cell[i];
      }
      if (snapshot.cell[i] > highest) {
        highest = snapshot.cell[i];
      }
    }

    // Calculate spread for all cells (needed for debug regardless of balance state)
    float spread = highest - lowest;

    // Skip balancing when the pack is disconnected or deeply discharged.
    if (!is_battery_present || lowest < min_cell_for_balance) {
      can_balance = false;
    }

    // Require sufficient spread before enabling balancing (cells are imbalanced enough to need balancing)
    // Disable when spread drops below deadband (cells are sufficiently balanced)
    if (can_balance) {
      if (spread < balance_deadband_v) {
        can_balance = false;  // Spread too small, balancing done
      }
    }


    // Turn off balancing when not allowed, otherwise run balance controllers
    if (!can_balance) {
      balance_disable_all_cells();  // Turns off all PWM and resets state
    } else {
      // Actively balance all cells toward the lowest cell + deadband window.
      // (debug output is in balance_all_cells() to avoid UART collision)
      balance_all_cells(snapshot.cell, 6, balance_deadband_v);
    }

    osDelay(balance_period_ms);
  }
  /* USER CODE END BalanceTaskHandler */
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
  int len = snprintf(uart_buffer, sizeof(uart_buffer), "[ERROR] Critical error occurred!\r\n");
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
