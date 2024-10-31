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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_COUNT 40U
#define NUM_TASKS 2U
#define NUM_BUFFERS 2U

#define NO_TRANSMISSION_IN_PROGRESS 255U

#define BUFFER_EMPTY 0U
#define BUFFER_FULL 1U

#define SAMPLING_INACTIVE 0U
#define SAMPLING_ACTIVE 1U

/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* This is what we send over UART to the PC. */
typedef struct {
	uint32_t uniqueHeader;
	uint16_t adcSamples[SAMPLE_COUNT];
	uint32_t uniqueEnder;	//The future vision here is a checksum.
} DataPacket_t;

/* This is an object that is being passed around. It mimics OOP but in C instead. */
typedef struct {
	DataPacket_t dataPacket;
	volatile uint8_t bufferFullFlag;
} DataBuffer_t;

/* USER CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/**
 * @brief This index is used to iterate through the buffer storing ADC samples.
 */
volatile uint8_t adcSampleIndex = 0;

/**
 * @brief This index is used to denote which buffer (0 or 1) is being filled by the ADC.
 * This is used by dataBuffers array to switch between which buffer is being referred to.
 */
volatile uint8_t bufferNumIndex = 0;

/**
 * @brief This is a flag that allows TIMER3 to do its job of collecting ADC data or switching buffer.
 * This flag is lowered once 40 ADC samples have been collected, effectively rendering
 * TIMER3 idle until the next 10ms sampling interval. This flag approach is a bandaid method
 * since you cannot disable TIMER3 (200us) and start it again within the IRQ handler of TIMER2 (10ms).
 */
volatile uint8_t samplingActive = SAMPLING_ACTIVE;

/**
 * @brief This flag indicates whether a UART transmission is in progress or not.
 */
uint8_t transmittingBuffer = NO_TRANSMISSION_IN_PROGRESS; // Invalid index to indicate no transmission in progress

/*
 *	TODO write a description here.
 */
DataBuffer_t dataBuffers[NUM_BUFFERS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Check_Buffer_Full_Task(void);
void Buffers_Overflow_Error_Task(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Task Array for Round-Robin Scheduler
 */
void (*TaskArray[NUM_TASKS])(void) = 
{
	Check_Buffer_Full_Task,
	Buffers_Overflow_Error_Task
};

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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	/* Initializes the two buffer objects with data. */
	for (uint8_t i = 0; i < NUM_BUFFERS; i++)
	{
		dataBuffers[i].dataPacket.uniqueHeader = 0xFFFFFFFF;
		dataBuffers[i].dataPacket.uniqueEnder = 0xDEADBEEF;
		dataBuffers[i].bufferFullFlag = BUFFER_EMPTY;
	}

	// TODO Write description
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK){ Error_Handler(); }

	// TODO Write Description
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK){ Error_Handler(); }

	// TODO Write Description
	if (HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK){ Error_Handler(); }

	HAL_GPIO_WritePin(TIA_RST_A_GPIO_Port, TIA_RST_A_Pin, GPIO_PIN_RESET);

  //HAL_GPIO_WritePin(IR_LED_850_S1_GPIO_Port, IR_LED_850_S1_Pin, GPIO_PIN_SET);

  //HAL_GPIO_WritePin(MCU_BOARD_LED_Port, MCU_BOARD_LED_Pin, SET);

	//HAL_GPIO_WritePin(IR_LED_850_S2_GPIO_Port, IR_LED_850_S2_Pin, GPIO_PIN_SET);

	// Round-Robin Scheduler Variables
  uint8_t currentTask = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Call the current task
		TaskArray[currentTask]();

		// Move to the next task
		currentTask = (currentTask + 1) % NUM_TASKS;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 43;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * TIM2 is on APB1 bus matrix. Refer to page 139 of STM32H723ZG ref. manual
  * Timer Period = (Prescaler + 1) * (ARR + 1) / f_clk (Hz)
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1749;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * TIM3 is on APB1 bus matrix. Refer to page 139 of STM32H723ZG ref. manual
  * Timer Period = (Prescaler + 1) * (ARR + 1) / f_clk (Hz)
  * OC Match Time = (Prescaler + 1) * OC.Pulse / f_clk (Hz)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 69;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 225;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TIA_RST_A_GPIO_Port, TIA_RST_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IR_LED_850_S1_Pin|IR_LED_735_S1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_BOARD_LED_Port, MCU_BOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TIA_RST_A_Pin */
  GPIO_InitStruct.Pin = TIA_RST_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TIA_RST_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_LED_850_S1_Pin IR_LED_735_S1_Pin */
  GPIO_InitStruct.Pin = IR_LED_850_S1_Pin|IR_LED_735_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_YELLOW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_Board_LED */
  GPIO_InitStruct.Pin = MCU_BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MCU_BOARD_LED_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
HAL_GPIO_WritePin(TIA_RST_A_GPIO_Port, TIA_RST_A_Pin, GPIO_PIN_SET);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Control Portion */
void Check_Buffer_Full_Task(void)
{
  // Iterate over the buffers to check if any buffer is full and not being transmitted
  for (uint8_t i = 0; i < NUM_BUFFERS; i++) {
    if ((dataBuffers[i].bufferFullFlag == BUFFER_FULL) && (transmittingBuffer == NO_TRANSMISSION_IN_PROGRESS)) {
        // Disable interrupts to protect shared data
        __disable_irq();

        dataBuffers[i].bufferFullFlag = BUFFER_EMPTY; // Reset the buffer full flag

        __enable_irq();

        // Start UART transmission
        transmittingBuffer = i;
        HAL_UART_Transmit_IT(&huart4, (uint8_t*)&dataBuffers[i].dataPacket, sizeof(DataPacket_t));
        break; // Only handle one buffer at a time
    }
  }
}

/* UART Portion */
/*
 *  TODO: Explain what this is used for.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Transmission is complete
    transmittingBuffer = NO_TRANSMISSION_IN_PROGRESS;
}

/* Timer Portion */
/**
 * @brief IRQ handler callback function for when the timers have overflown.
 * For 10ms overflown event:
 * 1) Sets sampling active flag to begin the 40x ADC sampling cycle again.
 * 2) Start charging the TIA for the first 200us cycle.
 * 
 * For 200us overflown event:
 * 1) Start charging the TIA for the next 200us cycle.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		// Code to execute every 10 ms (TIM2)
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

    // Start charging the TIA.
    HAL_GPIO_WritePin(IR_LED_850_S1_GPIO_Port, IR_LED_850_S1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TIA_RST_A_GPIO_Port, TIA_RST_A_Pin, GPIO_PIN_RESET);

		// Raises the flag for TIMER3 (200us) to start doing its job again.
		samplingActive = SAMPLING_ACTIVE;

	}
	else if (htim->Instance == TIM3)
	{
		if (samplingActive == SAMPLING_ACTIVE)
		{
			// Code to execute every 200 µs (TIM3)
			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

      // Start charging the TIA.
      HAL_GPIO_WritePin(IR_LED_850_S1_GPIO_Port, IR_LED_850_S1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(TIA_RST_A_GPIO_Port, TIA_RST_A_Pin, GPIO_PIN_RESET);
		}
	}
}

/**
 * @brief TODO write something about this function
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
			if (samplingActive == SAMPLING_ACTIVE)
			{
						// Start ADC conversion at 180 µs
            HAL_ADC_Start_IT(&hadc1);
            HAL_GPIO_WritePin(MCU_BOARD_LED_Port, MCU_BOARD_LED_Pin, SET);
			}
    }
  }
}

/* ADC Portion */
/**
 * @brief TODO write something about this function
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		// Process the ADC conversion result
		uint16_t adcValue = HAL_ADC_GetValue(hadc);
    HAL_GPIO_WritePin(MCU_BOARD_LED_Port, MCU_BOARD_LED_Pin, RESET);

		// Store ADC value in the current buffer
		dataBuffers[bufferNumIndex].dataPacket.adcSamples[adcSampleIndex] = adcValue;

    // Resets the TIA.
    HAL_GPIO_WritePin(IR_LED_850_S1_GPIO_Port, IR_LED_850_S1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TIA_RST_A_GPIO_Port, TIA_RST_A_Pin, GPIO_PIN_SET);

		// Increment adcSampleIndex after storing the value.
    adcSampleIndex++;

    // Check if buffer is full.
    if (adcSampleIndex >= SAMPLE_COUNT)
    {
      __disable_irq();

      // Buffer is full
      dataBuffers[bufferNumIndex].bufferFullFlag = BUFFER_FULL; // Set buffer full flag.
      adcSampleIndex = 0;          // Reset sample index.

      // Switch to the next buffer.
      bufferNumIndex = (bufferNumIndex + 1) % NUM_BUFFERS;

			// Sampling is complete.
      samplingActive = SAMPLING_INACTIVE;

      __enable_irq();
		}
	}
}

/* Error Handling Portion */
/**
 * @brief TODO write something about this function
 */
void Buffers_Overflow_Error_Task(void)
{
	if(dataBuffers[0].bufferFullFlag == BUFFER_FULL && dataBuffers[1].bufferFullFlag == BUFFER_FULL)
	{
		//HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); Disabled by fault. Only enable for Nucleo board.
		__disable_irq();
		//Probably disable TIMERx, and ADC here.
		while (1)
		{
		}
	}
}

/* USER CODE END 4 */

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
