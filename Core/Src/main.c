/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_LEN 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
DMA_HandleTypeDef hdma_tim2_up_ch3;

/* USER CODE BEGIN PV */
const uint16_t CH1_FREQ = 270; // 135 for 400kHz; 270 for 200kHz
uint32_t pMem0DataCh1[BUFFER_LEN];
uint32_t pMem1DataCh1[BUFFER_LEN];
uint32_t pMem0DataCh2[BUFFER_LEN];
uint32_t pMem1DataCh2[BUFFER_LEN];
uint32_t pMem0DataCh3[BUFFER_LEN];
uint32_t pMem1DataCh3[BUFFER_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void UpdateBuffer(uint32_t *buffer, uint32_t start_index, uint32_t start_value, uint32_t increment, size_t length);

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  UpdateBuffer(pMem0DataCh1, 0, CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  UpdateBuffer(pMem1DataCh1, 0, pMem0DataCh1[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  UpdateBuffer(pMem0DataCh2, 0, CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  UpdateBuffer(pMem1DataCh2, 0, pMem0DataCh2[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  UpdateBuffer(pMem0DataCh3, 0, CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  UpdateBuffer(pMem1DataCh3, 0, pMem0DataCh2[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);

  TIM_OC_Start_DMA_Double_Buffer(&htim2, TIM_CHANNEL_1, (uint32_t)pMem0DataCh1, (uint32_t)&htim2.Instance->CCR1, (uint32_t)pMem1DataCh1, BUFFER_LEN);
  TIM_OC_Start_DMA_Double_Buffer(&htim2, TIM_CHANNEL_2, (uint32_t)pMem0DataCh2, (uint32_t)&htim2.Instance->CCR2, (uint32_t)pMem1DataCh2, BUFFER_LEN);
  TIM_OC_Start_DMA_Double_Buffer(&htim2, TIM_CHANNEL_3, (uint32_t)pMem0DataCh2, (uint32_t)&htim2.Instance->CCR3, (uint32_t)pMem1DataCh3, BUFFER_LEN);

  // Reset Counter, CNT, as 0
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  /* USER CODE END 2 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief This function is executed in case of the DMA M0 buffer transfer complete event occurrence.
 */
void PingPongM0TransferCompleteCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    /* Current memory buffer used is Memory 1 */
    UpdateBuffer(pMem0DataCh1, 0, pMem1DataCh1[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* Current memory buffer used is Memory 1 */
    UpdateBuffer(pMem0DataCh2, 0, pMem1DataCh2[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    /* Current memory buffer used is Memory 1 */
    UpdateBuffer(pMem0DataCh3, 0, pMem1DataCh3[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  }
}

/**
 * @brief This function is executed in case of the DMA M1 buffer transfer complete event occurrence.
 */
void PingPongM1TransferCompleteCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    /* Current memory buffer used is Memory 0 */
    UpdateBuffer(pMem1DataCh1, 0, pMem0DataCh1[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* Current memory buffer used is Memory 0 */
    UpdateBuffer(pMem1DataCh2, 0, pMem0DataCh2[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    /* Current memory buffer used is Memory 0 */
    UpdateBuffer(pMem1DataCh3, 0, pMem0DataCh3[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
  }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    /* Current memory buffer used is Memory 0 */
    if ((htim->hdma[TIM_DMA_ID_CC1]->Instance->CR & DMA_SxCR_CT) == RESET)
    {
      UpdateBuffer(pMem1DataCh1, 0, pMem0DataCh1[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
    }
    /* Current memory buffer used is Memory 1 */
    else
    {
      UpdateBuffer(pMem0DataCh1, 0, pMem1DataCh1[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
    }
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* Current memory buffer used is Memory 0 */
    if ((htim->hdma[TIM_DMA_ID_CC2]->Instance->CR & DMA_SxCR_CT) == RESET)
    {
      UpdateBuffer(pMem1DataCh2, 0, pMem0DataCh2[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
    }
    /* Current memory buffer used is Memory 1 */
    else
    {
      UpdateBuffer(pMem0DataCh2, 0, pMem1DataCh2[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
    }
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    /* Current memory buffer used is Memory 0 */
    if ((htim->hdma[TIM_DMA_ID_CC1]->Instance->CR & DMA_SxCR_CT) == RESET)
    {
      UpdateBuffer(pMem1DataCh3, 0, pMem0DataCh3[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
    }
    /* Current memory buffer used is Memory 1 */
    else
    {
      UpdateBuffer(pMem0DataCh3, 0, pMem1DataCh3[BUFFER_LEN - 1] + CH1_FREQ, CH1_FREQ, BUFFER_LEN);
    }
  }
}

void UpdateBuffer(uint32_t *buffer, uint32_t start_index, uint32_t start_value, uint32_t increment, size_t length)
{
  for (uint32_t i = start_index; i < (start_index + length); i++)
  {
    buffer[i] = start_value + increment * (i - start_index);
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
}

/* USER CODE END Error_Handler_Debug */

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
