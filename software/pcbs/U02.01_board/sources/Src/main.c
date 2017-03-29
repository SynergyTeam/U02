/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "global.h"
#include "cli.h"
#include "mb.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;

osThreadId cliTaskHandle;
osThreadId modbusTaskHandle;
osThreadId sensTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#ifdef DEBUG

DebugWindow dwnd;

#endif

// сигнал из прерываний в задачу, что АЦП завершил последовательность преобразований
#define ADC_SIGNAL 0
// сигнал из прерываний в задачу, что приём данных по I2C завершен
#define I2C_RX_SIGNAL 1

osThreadId testTaskHandle;
// данные полученные с LM75A
volatile int16_t lm75buf = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
void cliTaskRoutine(void const * argument);
void modbusTaskRoutine(void const * argument);
void sensTaskRoutine(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void testTaskRoutine(void const * argument);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  // выключаем светодиод
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);

  // загружаем настройки устройства
  loadSettings();
  if(getCurSettings()->neverStored)
    applyDefaultSettings();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of cliTask */
  osThreadDef(cliTask, cliTaskRoutine, osPriorityNormal, 0, 180);
  cliTaskHandle = osThreadCreate(osThread(cliTask), NULL);

  /* definition and creation of modbusTask */
  osThreadDef(modbusTask, modbusTaskRoutine, osPriorityNormal, 0, 150);
  modbusTaskHandle = osThreadCreate(osThread(modbusTask), NULL);

  /* definition and creation of sensTask */
  osThreadDef(sensTask, sensTaskRoutine, osPriorityNormal, 0, 96);
  sensTaskHandle = osThreadCreate(osThread(sensTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(testTask, testTaskRoutine, osPriorityNormal, 0, 64);
  testTaskHandle = osThreadCreate(osThread(testTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
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

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 479;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PWR_12V_ENABLE_Pin|GAS_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, XF003_Pin|XF002_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FOG_ENABLE_Pin|LED1_Pin|RELAY2_Pin|RELAY1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_12V_ENABLE_Pin GAS_ENABLE_Pin */
  GPIO_InitStruct.Pin = PWR_12V_ENABLE_Pin|GAS_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : XF003_Pin XF002_Pin */
  GPIO_InitStruct.Pin = XF003_Pin|XF002_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_CONNECTED_Pin BOX_CLOSED_Pin */
  GPIO_InitStruct.Pin = USB_CONNECTED_Pin|BOX_CLOSED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FOG_ENABLE_Pin LED1_Pin RELAY2_Pin RELAY1_Pin */
  GPIO_InitStruct.Pin = FOG_ENABLE_Pin|LED1_Pin|RELAY2_Pin|RELAY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 PB13 PB14 
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

UART_HandleTypeDef* getModbusUart()
{
  return &huart1;
}

/*
UART_HandleTypeDef* getCliUart()
{
  return &huart3;
}
*/

TIM_HandleTypeDef* getModbusTimer(void)
{
  return &htim15;
}

void modbusInitUart(uint32_t baudRate, uint8_t dataBits, uint8_t parity, uint8_t stopbits)
{
  UART_HandleTypeDef *pModbusUart = getModbusUart();

  if(HAL_UART_DeInit(pModbusUart) != HAL_OK)
  {
    Error_Handler();
  }

  /* Внимание: нельзя в этой функции присваивать указатель на UART,
   * потому что мы можем использовать далее другой модуль UART и
   * какой именно мы используем, задаётся в методе getModbusUart,
   * hInstance не всегда будет USART2! */
  //pModbusUart->Instance = USART2;

  pModbusUart->Init.BaudRate = baudRate;

  switch(dataBits)
  {
  case 7:
    pModbusUart->Init.WordLength = UART_WORDLENGTH_7B;
    break;
  case 8:
    pModbusUart->Init.WordLength = UART_WORDLENGTH_8B;
    break;
  case 9:
    pModbusUart->Init.WordLength = UART_WORDLENGTH_9B;
    break;
  default:
    pModbusUart->Init.WordLength = UART_WORDLENGTH_8B;
    break;
  }

  switch(stopbits)
  {
  case 0:
    //pModbusUart->Init.StopBits = UART_STOPBITS_0_5;
    // XXX Для F070 нет UART с поддержкой стопбита 0.5
    pModbusUart->Init.StopBits = UART_STOPBITS_1;
    break;
  case 1:
    pModbusUart->Init.StopBits = UART_STOPBITS_1;
    break;
  case 2:
    pModbusUart->Init.StopBits = UART_STOPBITS_2;
    break;
  default:
    pModbusUart->Init.StopBits = UART_STOPBITS_1;
  }


  switch(parity)
  {
  case MB_PAR_NONE:
    pModbusUart->Init.Parity = UART_PARITY_NONE;
    break;
  case MB_PAR_EVEN:
    pModbusUart->Init.Parity = UART_PARITY_EVEN;
    break;
  case MB_PAR_ODD:
    break;
  default:
    pModbusUart->Init.Parity = UART_PARITY_ODD;
    break;
  }

  pModbusUart->Init.Mode = UART_MODE_TX_RX;
  pModbusUart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  pModbusUart->Init.OverSampling = UART_OVERSAMPLING_16;
  pModbusUart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  pModbusUart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(pModbusUart) != HAL_OK)
  {
    Error_Handler();
  }
}

void modbusInitTimer(uint32_t tick50usCount)
{
  TIM_HandleTypeDef *pModbusTimer = getModbusTimer();

  if(HAL_TIM_Base_DeInit(pModbusTimer) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  /* Внимание: нельзя в этой функции присваивать указатель на Timer,
   * потому что мы можем использовать далее другой Timer и
   * какой именно мы используем, задаётся в методе getModbusTimer,
   * hInstance не всегда будет TIM2! */
  //pModbusTimer->Instance = TIM2;

  pModbusTimer->Init.Prescaler = 2399; // настраиваем таймер на тикание с периодом 50 мкс
  pModbusTimer->Init.CounterMode = TIM_COUNTERMODE_UP;
  pModbusTimer->Init.Period = tick50usCount;
  pModbusTimer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  pModbusTimer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(pModbusTimer) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(pModbusTimer, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(pModbusTimer, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == getModbusTimer()->Instance)
  {
    pxMBPortCBTimerExpired();
  }
}

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /*if (huart->Instance == getCliUart()->Instance)
  {
    osSignalSet(cliTaskHandle, CLI_UART_SIGNAL);
  }*/

  if (huart->Instance == getModbusUart()->Instance)
  {
    // говорим стеку модбас, что мы получили новый байт
    pxMBFrameCBByteReceived();
    // включаем прерывания и ждём следующий байт
    vMBPortSerialEnable(TRUE,FALSE);
  }
}

/**
 * @brief Tx Transfer completed callback.
 * @param huart: UART handle.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == getModbusUart()->Instance)
  {
    // говорим стеку модбас, что мы готовы отправить следующий байт сообщения
    pxMBFrameCBTransmitterEmpty();
  }
}

/**
 * @brief  UART error callbacks
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  // nothing
  return;
}

/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  hadc: ADC handle
 * @retval None
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  osSignalSet(sensTaskHandle,ADC_SIGNAL);
}

/**
 * @brief  Master Rx Transfer completed callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  osSignalSet(sensTaskHandle,I2C_RX_SIGNAL);
}

void testTaskRoutine(void const * argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    osDelay(500);
  }
}

/* USER CODE END 4 */

/* cliTaskRoutine function */
void cliTaskRoutine(void const * argument)
{
  /* Prepare USB to work */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  cliInit();
  /* Infinite loop */
  for (;;)
  {
    cliRecieveSymbol();

    if(osSignalWait(CLI_UART_SIGNAL, 50).status == osEventSignal)
    {
      cliProcessLastSymbol();
    }

#ifdef DEBUG
    dwnd.cliTaskStackSize = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}

/* modbusTaskRoutine function */
void modbusTaskRoutine(void const * argument)
{
  /* USER CODE BEGIN modbusTaskRoutine */
  // записываем Modbus Report Id
  uint32_t *pId = (uint32_t*)UID_BASE;
  char* modbusSlaveId = (char*)pvPortMalloc(128);
  snprintf(modbusSlaveId,128,"%s,%s,%X%X%X",DEVICE_NAME,VERSION_NAME,*pId,*(pId+4),*(pId+8));
  eMBSetSlaveID(0,TRUE,(const UCHAR*)modbusSlaveId,(USHORT)strlen(modbusSlaveId));
  vPortFree(modbusSlaveId);
  modbusSlaveId = 0;

  for (;;)
  {
    if(reqModbusInit && !flagModbusTxEnabled)
    {
      // получаем указатель на настройки
      Settings *pSettings = getCurSettings();
      // перенастраиваем порт UART и Таймер для Модбаса
      eMBInit(  MB_RTU,
                pSettings->modbusAddr,
                pSettings->modbusStopbits,
                pSettings->modbusBaudrate,
                pSettings->modbusParity);
      // включаем приёмник UART и Таймер Модбас
      eMBEnable();
      // говорим системе, что Модбас настроен
      reqModbusInit = FALSE;
    }

    // обрабатываем сообщения модбас
    /* Внимание: эта функция блокирует поток на 50мс в ожидании
     * сообщения от Таймера Модбаса, который срабатывает по истечении
     * периода в 3.5 символа Модбас */
    eMBPoll();

#ifdef DEBUG
    dwnd.modbusTaskStackSize = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
  /* USER CODE END modbusTaskRoutine */
}

/* sensTaskRoutine function */
void sensTaskRoutine(void const * argument)
{
  /* USER CODE BEGIN sensTaskRoutine */
  uint32_t calibrationTimer = 0;
  // буфер для расчета температуры
  int32_t temp = 0;
  // перед началом работы калибруем АЦП
  HAL_ADCEx_Calibration_Start(&hadc);
  // Включаем БП12В
  set12vEnable(1);
  // Включаем датчики задымления
  setFogEnable(1);
  /* Infinite loop */
  for(;;)
  {
    // примерно через каждые 10-20 минут, выполняем калибровку АЦП
    if(calibrationTimer > 0x1FFFF)
    {
      HAL_ADCEx_Calibration_Start(&hadc);
      calibrationTimer = 0;
    }
    else
    {
      calibrationTimer++;
    }

    // выполняем сброс датчиков задымления
    if(reqResetFog)
    {
      // выключаем питание датчиков
      setFogEnable(0);
      // ждём 3 секунды
      osDelay(3000);
      // всё по новой
      setFogEnable(1);
      reqResetFog = FALSE;

      // TODO подумать: когда делаем сброс задымления, останавливается опрос остальных датчиков
    }

    // TODO подумать над включением датчика газа
    // включаем датчик газа
    setGasEnable(1);
    // включаем АЦП
    HAL_ADC_Start_DMA(&hadc,(uint32_t*)gAdcValue,ADC_USED_CHANNELS_COUNT);
    // ждём завершения измерений
    if(osSignalWait(ADC_SIGNAL,500).status == osEventSignal)
    {
      // обрабатываем датчики
      Settings *pSettings = getCurSettings();

      updateSensor(getDryContacts1(), gAdcValue[ADC_DRY1_IDX], pSettings->dryLevel, 75, TRUE);
      updateSensor(getDryContacts2(), gAdcValue[ADC_DRY2_IDX], pSettings->dryLevel, 75, TRUE);
      updateSensor(getWaterSensor1(), gAdcValue[ADC_WATER1_IDX], pSettings->waterLevel, pSettings->waterHyst, FALSE);
      updateSensor(getWaterSensor2(), gAdcValue[ADC_WATER2_IDX], pSettings->waterLevel, pSettings->waterHyst, FALSE);
      updateSensor(getFogSensor1(), gAdcValue[ADC_FOG1_IDX], pSettings->fogLevel, pSettings->fogHyst, FALSE);
      updateSensor(getFogSensor2(), gAdcValue[ADC_FOG2_IDX], pSettings->fogLevel, pSettings->fogHyst, FALSE);
      updateSensor(getGasSensor1(), gAdcValue[ADC_GAS_IDX], pSettings->gasLevel, pSettings->gasHyst, FALSE);

      // системные данные
      gVdd = getSystemVdd(gAdcValue[ADC_VREFINT_IDX]);
      gMcuTemperature = getSystemTemperature(gAdcValue[ADC_TEMPERATURE_IDX],gAdcValue[ADC_VREFINT_IDX]);
    }
    // выключаем датчик газа
    setGasEnable(0);

    // TODO подумать, когда делаем опрос датчика температуры, то останавливается опрос остальных датчиков
    // Получение значений температуры по I2C
    HAL_I2C_Master_Receive_IT(&hi2c1,LM75_ADDR,(uint8_t*)&lm75buf,2);
    if(osSignalWait(I2C_RX_SIGNAL,100).status == osEventSignal)
    //if(HAL_I2C_Master_Receive(&hi2c1,LM75_ADDR,(uint8_t*)&lm75buf,2,100) == HAL_OK)
    {
      // меняем местами байты, так как первым приходит старший байт
      lm75buf = ((lm75buf >> 8) & 0x00FF) | ((lm75buf << 8) & 0xFF00);
      // преобразуем полученные данные в температуру
      lm75buf >>= 5;
      temp = lm75buf;
      temp *= 125;
      temp /= 100;
      gTemperature = temp;
    }

#ifdef DEBUG
    dwnd.sensTaskStackSize = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
  /* USER CODE END sensTaskRoutine */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
