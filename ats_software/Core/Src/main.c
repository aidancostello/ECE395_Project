/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "structs.h"
#include "self_gps.h"
#include "target_gps.h"
#include "calculate.h"
#include "encoder.h"
#include "stepper.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for task_SelfGps */
osThreadId_t task_SelfGpsHandle;
uint32_t task_SelfGpsBuffer[ 512 ];
osStaticThreadDef_t task_SelfGpsControlBlock;
const osThreadAttr_t task_SelfGps_attributes = {
  .name = "task_SelfGps",
  .cb_mem = &task_SelfGpsControlBlock,
  .cb_size = sizeof(task_SelfGpsControlBlock),
  .stack_mem = &task_SelfGpsBuffer[0],
  .stack_size = sizeof(task_SelfGpsBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_TargetGps */
osThreadId_t task_TargetGpsHandle;
uint32_t task_TargetGpsBuffer[ 512 ];
osStaticThreadDef_t task_TargetGpsControlBlock;
const osThreadAttr_t task_TargetGps_attributes = {
  .name = "task_TargetGps",
  .cb_mem = &task_TargetGpsControlBlock,
  .cb_size = sizeof(task_TargetGpsControlBlock),
  .stack_mem = &task_TargetGpsBuffer[0],
  .stack_size = sizeof(task_TargetGpsBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_Calculate */
osThreadId_t task_CalculateHandle;
uint32_t task_CalculateBuffer[ 512 ];
osStaticThreadDef_t task_CalculateControlBlock;
const osThreadAttr_t task_Calculate_attributes = {
  .name = "task_Calculate",
  .cb_mem = &task_CalculateControlBlock,
  .cb_size = sizeof(task_CalculateControlBlock),
  .stack_mem = &task_CalculateBuffer[0],
  .stack_size = sizeof(task_CalculateBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_Encoder */
osThreadId_t task_EncoderHandle;
uint32_t task_EncoderBuffer[ 512 ];
osStaticThreadDef_t task_EncoderControlBlock;
const osThreadAttr_t task_Encoder_attributes = {
  .name = "task_Encoder",
  .cb_mem = &task_EncoderControlBlock,
  .cb_size = sizeof(task_EncoderControlBlock),
  .stack_mem = &task_EncoderBuffer[0],
  .stack_size = sizeof(task_EncoderBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_Stepper */
osThreadId_t task_StepperHandle;
uint32_t task_StepperBuffer[ 512 ];
osStaticThreadDef_t task_StepperControlBlock;
const osThreadAttr_t task_Stepper_attributes = {
  .name = "task_Stepper",
  .cb_mem = &task_StepperControlBlock,
  .cb_size = sizeof(task_StepperControlBlock),
  .stack_mem = &task_StepperBuffer[0],
  .stack_size = sizeof(task_StepperBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_Servo */
osThreadId_t task_ServoHandle;
uint32_t task_ServoBuffer[ 512 ];
osStaticThreadDef_t task_ServoControlBlock;
const osThreadAttr_t task_Servo_attributes = {
  .name = "task_Servo",
  .cb_mem = &task_ServoControlBlock,
  .cb_size = sizeof(task_ServoControlBlock),
  .stack_mem = &task_ServoBuffer[0],
  .stack_size = sizeof(task_ServoBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
osStaticMutexDef_t uartMutexControlBlock;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex",
  .cb_mem = &uartMutexControlBlock,
  .cb_size = sizeof(uartMutexControlBlock),
};
/* Definitions for gpsDataMutex */
osMutexId_t gpsDataMutexHandle;
osStaticMutexDef_t gpsDataMutexControlBlock;
const osMutexAttr_t gpsDataMutex_attributes = {
  .name = "gpsDataMutex",
  .cb_mem = &gpsDataMutexControlBlock,
  .cb_size = sizeof(gpsDataMutexControlBlock),
};
/* Definitions for targetPositionMutex */
osMutexId_t targetPositionMutexHandle;
osStaticMutexDef_t targetPositionMutexControlBlock;
const osMutexAttr_t targetPositionMutex_attributes = {
  .name = "targetPositionMutex",
  .cb_mem = &targetPositionMutexControlBlock,
  .cb_size = sizeof(targetPositionMutexControlBlock),
};
/* Definitions for encoderPositionMutex */
osMutexId_t encoderPositionMutexHandle;
osStaticMutexDef_t encoderPositionMutexControlBlock;
const osMutexAttr_t encoderPositionMutex_attributes = {
  .name = "encoderPositionMutex",
  .cb_mem = &encoderPositionMutexControlBlock,
  .cb_size = sizeof(encoderPositionMutexControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void task_entry_SelfGps(void *argument);
void task_entry_TargetGps(void *argument);
void task_entry_Calculate(void *argument);
void task_entry_Encoder(void *argument);
void task_entry_Stepper(void *argument);
void task_entry_Servo(void *argument);

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* creation of gpsDataMutex */
  gpsDataMutexHandle = osMutexNew(&gpsDataMutex_attributes);

  /* creation of targetPositionMutex */
  targetPositionMutexHandle = osMutexNew(&targetPositionMutex_attributes);

  /* creation of encoderPositionMutex */
  encoderPositionMutexHandle = osMutexNew(&encoderPositionMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  struct GpsData gps_data = {0, 0, 0, 0, 0, 0, &gpsDataMutexHandle};
  struct TargetPosition target_position = {0, 0, &targetPositionMutexHandle};
  struct EncoderPosition encoder_position = {0, &encoderPositionMutexHandle};
  struct DataPointers data_pointers = {&gps_data, &target_position, &encoder_position};
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task_SelfGps */
  task_SelfGpsHandle = osThreadNew(task_entry_SelfGps, (void*) &data_pointers, &task_SelfGps_attributes);

  /* creation of task_TargetGps */
  task_TargetGpsHandle = osThreadNew(task_entry_TargetGps, (void*) &data_pointers, &task_TargetGps_attributes);

  /* creation of task_Calculate */
  task_CalculateHandle = osThreadNew(task_entry_Calculate, (void*) &data_pointers, &task_Calculate_attributes);

  /* creation of task_Encoder */
  task_EncoderHandle = osThreadNew(task_entry_Encoder, (void*) &data_pointers, &task_Encoder_attributes);

  /* creation of task_Stepper */
  task_StepperHandle = osThreadNew(task_entry_Stepper, (void*) &data_pointers, &task_Stepper_attributes);

  /* creation of task_Servo */
  task_ServoHandle = osThreadNew(task_entry_Servo, (void*) &data_pointers, &task_Servo_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00100D14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           PA7 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_entry_SelfGps */
/**
  * @brief  Function implementing the task_SelfGps thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_entry_SelfGps */
void task_entry_SelfGps(void *argument)
{
  /* USER CODE BEGIN 5 */
  self_gps_init(&hi2c1);
  /* Infinite loop */
  for(;;)
  {
    self_gps_update(&hi2c1, ((struct DataPointers*)argument)->gps_data);
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task_entry_TargetGps */
/**
* @brief Function implementing the task_TargetGps thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_entry_TargetGps */
void task_entry_TargetGps(void *argument)
{
  /* USER CODE BEGIN task_entry_TargetGps */
  target_gps_init();
  /* Infinite loop */
  for(;;)
  {
    target_gps_update(((struct DataPointers*)argument)->gps_data);
    osDelay(10);
  }
  /* USER CODE END task_entry_TargetGps */
}

/* USER CODE BEGIN Header_task_entry_Calculate */
/**
* @brief Function implementing the task_Calculate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_entry_Calculate */
void task_entry_Calculate(void *argument)
{
  /* USER CODE BEGIN task_entry_Calculate */
  /* Infinite loop */
  for(;;)
  {
    calculate_update(((struct DataPointers*)argument)->gps_data, ((struct DataPointers*)argument)->target_position);
    osDelay(10);
  }
  /* USER CODE END task_entry_Calculate */
}

/* USER CODE BEGIN Header_task_entry_Encoder */
/**
* @brief Function implementing the task_Encoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_entry_Encoder */
void task_entry_Encoder(void *argument)
{
  /* USER CODE BEGIN task_entry_Encoder */
  encoder_init();
  /* Infinite loop */
  for(;;)
  {
    encoder_update(((struct DataPointers*)argument)->encoder_position);
    osDelay(10);
  }
  /* USER CODE END task_entry_Encoder */
}

/* USER CODE BEGIN Header_task_entry_Stepper */
/**
* @brief Function implementing the task_Stepper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_entry_Stepper */
void task_entry_Stepper(void *argument)
{
  /* USER CODE BEGIN task_entry_Stepper */
  stepper_init();
  /* Infinite loop */
  for(;;)
  {
    stepper_update(((struct DataPointers*)argument)->encoder_position, ((struct DataPointers*)argument)->target_position);
    osDelay(10);
  }
  /* USER CODE END task_entry_Stepper */
}

/* USER CODE BEGIN Header_task_entry_Servo */
/**
* @brief Function implementing the task_Servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_entry_Servo */
void task_entry_Servo(void *argument)
{
  /* USER CODE BEGIN task_entry_Servo */
  servo_init(&htim2);
  /* Infinite loop */
  for(;;)
  {
    servo_update(&htim2, ((struct DataPointers*)argument)->target_position);
    osDelay(10);
  }
  /* USER CODE END task_entry_Servo */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
