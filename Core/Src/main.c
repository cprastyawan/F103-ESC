/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <dwt_delay.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESC_PWM_MIN 1000
#define ESC_PWM_MAX 2000
#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )
#define CHANNEL_AH TIM_CHANNEL_1
#define CHANNEL_BH TIM_CHANNEL_2
#define CHANNEL_CH TIM_CHANNEL_3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//const uint32_t GPIO_PORT_MOTOR = GPIOB;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint32_t Input_DutyCycle;
static uint32_t Frequency;
ESC_MODE esc_mode;

uint32_t pwm_cal_min;
uint32_t pwm_cal_max;

static unsigned char buffer[200];
static int strSize;
uint32_t bemfA = 0, bemfB = 0, bemfC = 0;

uint32_t adcValue[3];
bool ADCDataStatus;

int step = 1;

static uint32_t delta, lastDelta;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void sound(uint32_t Frequency, uint32_t milliseconds);
void phase(int step, uint16_t speed);
void outPWM(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t dutyCycle){
	dutyCycle = map(dutyCycle, 0, 0xFFFF, 0, 7200);
	__HAL_TIM_SET_COMPARE(htim, channel, dutyCycle);
}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adcValue, 3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  DWT_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  if(Input_DutyCycle > ESC_PWM_MIN + 100){
	  esc_mode = ESC_CALIBRATE;
  } else esc_mode = ESC_DISARMED;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //if(ADCDataStatus){
		  //ADCDataStatus = false;

		 /* bemfA = adcValue[0];
		  bemfB = adcValue[1];
		  bemfC = adcValue[2];

		  phase(step);

		  if(lastDelta < 0){
			if(delta > 0){
				lastDelta = delta;
				step = step + 1;
				if(step > 6) step = 1;
			}
		  }

		  if(lastDelta > 0){
			  if(delta < 0){
				  lastDelta = delta;
				  step = step + 1;
				  if(step > 6) step = 1;
			  }
		  }*/
	  	  phase(step, 0xFFFF / 0x2);
		  step++;
		  if(step > 6) step = 1;

		 /* strSize = sprintf((char*)buffer, "BEMF-A: %lu\r\n", bemfA);
		  HAL_UART_Transmit(&huart1, buffer, strSize, 30);

		  strSize = sprintf((char*)buffer, "BEMF-B: %lu\r\n", bemfB);
		  HAL_UART_Transmit(&huart1, buffer, strSize, 30);

		  strSize = sprintf((char*)buffer, "BEMF-C: %lu\r\n", bemfC);
		  HAL_UART_Transmit(&huart1, buffer, strSize, 30);

		  HAL_UART_Transmit(&huart1, "\r\n\r\n", 4, 30);*/



	  //}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7200 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|AL_Pin|BL_Pin|CL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin AL_Pin BL_Pin CL_Pin */
  GPIO_InitStruct.Pin = LED_Pin|AL_Pin|BL_Pin|CL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void phase(int step, uint16_t speed){
	uint32_t bemfSum = (bemfA + bemfB + bemfC) / 3;

	//strSize = sprintf((char*)buffer, "bemfA: %lu, bemfB: %lu, bemfC: %lu, bemfSum: %lu\t",bemfA, bemfB, bemfC, bemfSum);
	//HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	switch(step){
	case 1:

		outPWM(&htim1, CHANNEL_BH, 0);
		outPWM(&htim1, CHANNEL_CH, 0);
		HAL_GPIO_WritePin(GPIOA, AL_Pin | CL_Pin, GPIO_PIN_RESET);

		outPWM(&htim1, CHANNEL_AH, speed);
		HAL_GPIO_WritePin(GPIOA, BL_Pin, GPIO_PIN_SET);

		delta = bemfA - bemfSum;
		strSize = sprintf((char*)buffer, "STEP: %d AH-BL\r\n", step);
		HAL_UART_Transmit(&huart1, buffer, strSize, 100);
		break;

	case 2:
		outPWM(&htim1, CHANNEL_BH, 0);
		outPWM(&htim1, CHANNEL_CH, 0);
		HAL_GPIO_WritePin(GPIOA, AL_Pin | BL_Pin, GPIO_PIN_RESET);


		outPWM(&htim1, CHANNEL_AH, speed);
		HAL_GPIO_WritePin(GPIOA, CL_Pin, GPIO_PIN_SET);

		delta = bemfC - bemfSum;
		strSize = sprintf((char*)buffer, "STEP: %d AH-CL\r\n", step);
		HAL_UART_Transmit(&huart1, buffer, strSize, 100);
		break;

	case 3:
		outPWM(&htim1, CHANNEL_CH, 0);
		outPWM(&htim1, CHANNEL_AH, 0);
		HAL_GPIO_WritePin(GPIOA, AL_Pin | BL_Pin, GPIO_PIN_RESET);


		outPWM(&htim1, CHANNEL_BH, speed);
		HAL_GPIO_WritePin(GPIOA, CL_Pin, GPIO_PIN_SET);

		strSize = sprintf((char*)buffer, "STEP: %d BH-CL\r\n", step);
		HAL_UART_Transmit(&huart1, buffer, strSize, 100);
		delta = bemfB - bemfSum;
		break;

	case 4:
		outPWM(&htim1, CHANNEL_CH, 0);
		outPWM(&htim1, CHANNEL_AH, 0);
		HAL_GPIO_WritePin(GPIOA, CL_Pin | BL_Pin, GPIO_PIN_RESET);

		outPWM(&htim1, CHANNEL_BH, speed);
		HAL_GPIO_WritePin(GPIOA, AL_Pin, GPIO_PIN_SET);

		strSize = sprintf((char*)buffer, "STEP: %d BH-AL\r\n", step);
		HAL_UART_Transmit(&huart1, buffer, strSize, 100);
		delta = bemfA - bemfSum;
		break;

	case 5:
		outPWM(&htim1, CHANNEL_AH, 0);
		outPWM(&htim1, CHANNEL_BH, 0);
		HAL_GPIO_WritePin(GPIOA, CL_Pin | BL_Pin, GPIO_PIN_RESET);

		outPWM(&htim1, CHANNEL_CH, speed);
		HAL_GPIO_WritePin(GPIOA, AL_Pin, GPIO_PIN_SET);

		strSize = sprintf((char*)buffer, "STEP: %d CH-AL\r\n", step);
		HAL_UART_Transmit(&huart1, buffer, strSize, 100);
		delta = bemfC - bemfSum;
		break;

	case 6:
		outPWM(&htim1, CHANNEL_BH, 0);
		outPWM(&htim1, CHANNEL_AH, 0);
		HAL_GPIO_WritePin(GPIOA, CL_Pin | AL_Pin, GPIO_PIN_RESET);

		outPWM(&htim1, CHANNEL_CH, speed);
		HAL_GPIO_WritePin(GPIOA, BL_Pin, GPIO_PIN_SET);

		strSize = sprintf((char*)buffer, "STEP: %d CH-BL\r\n", step);
		HAL_UART_Transmit(&huart1, buffer, strSize, 100);
		delta = bemfB - bemfSum;
		break;
	}
	//HAL_Delay(100);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(!ADCDataStatus) ADCDataStatus = true;
}

void sound(uint32_t Frequency, uint32_t milliseconds){
/*	int x = 0;

	uint32_t tOnOff = (1000000 / Frequency) / 2;

	while(i < milliseconds){
		for(int i = 0; i < 1000 / tOnOff; i++){
			HAL_GPIO_WritePin(PORTMotor, GPIO_PIN_PHASEA, GPIO_PIN_SET);
			DWT_Delay(tOnOff / 4);
			HAL_GPIO_WritePin(PORTMotor, GPIO_PIN_PHASEA, GPIO_PIN_RESET);
			DWT_Delay(tOnOff / 4);

			HAL_GPIO_WritePin(PORTMotor, GPIO_PIN_PHASEB, GPIO_PIN_SET);
			DWT_Delay(tOnOff / 4);
			HAL_GPIO_WritePin(PORTMotor, GPIO_PIN_PHASEB, GPIO_PIN_RESET);
			DWT_Delay(tOnOff / 4);
		}
		++x;
	}*/
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	static uint32_t Count_RisingEdge;
	static uint32_t Count_FallingEdge;
	static uint32_t Count_Freq1;
	static uint32_t Count_Freq2;

	static bool Freq_State;

	if(htim->Instance == TIM2){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			Count_RisingEdge = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);

			if(Freq_State == 0){
				Freq_State = 1;
				Count_Freq1 = Count_RisingEdge;
			}
			else if(Freq_State == 1){
				Freq_State = 0;
				Count_Freq2 = Count_RisingEdge;
				Frequency = 1 / ((Count_Freq2 - Count_Freq1) / 1000); //in kHz
				Frequency = Frequency * 1000; //Convert to Hz
			}
		}

		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			Count_FallingEdge = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
		}

		if(Count_FallingEdge > Count_RisingEdge){
			Input_DutyCycle = Count_FallingEdge - Count_RisingEdge;
			Count_RisingEdge = 0;
			Count_FallingEdge = 0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
