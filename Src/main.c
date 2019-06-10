/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define DISP_1_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define DISP_1_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define DISP_2_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define DISP_2_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

#define DISP_VAL_NULL HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                               GPIO_PIN_2 | GPIO_PIN_3 | \
                                               GPIO_PIN_4 | GPIO_PIN_5 | \
                                               GPIO_PIN_6 | GPIO_PIN_7 , \
                                               GPIO_PIN_SET)

#define DISP_VAL_0	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 | GPIO_PIN_3 | \
                                              GPIO_PIN_4 | GPIO_PIN_5 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_1	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 | GPIO_PIN_2 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_2	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_3 | GPIO_PIN_4 | \
                                              GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_3	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 | GPIO_PIN_3 | \
                                              GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_4	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 | GPIO_PIN_2 | \
    	                                       GPIO_PIN_5 | GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_5	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_2 | \
                                              GPIO_PIN_3 | GPIO_PIN_5 | \
                                              GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_6	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_2 | \
                                              GPIO_PIN_3 | GPIO_PIN_4 | \
                                              GPIO_PIN_5 | GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_7	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_8	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 | GPIO_PIN_3 | \
                                              GPIO_PIN_4 | GPIO_PIN_5 | \
                                              GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)
#define DISP_VAL_9	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | \
                                              GPIO_PIN_2 | GPIO_PIN_3 | \
                                              GPIO_PIN_5 | GPIO_PIN_6 , \
                                              GPIO_PIN_RESET)

#define DISP_DOT  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 , \
       GPIO_PIN_RESET)

//licznik oktawy
int counter;

uint8_t value1;

uint8_t value2;
uint8_t check_value2 = 0;

uint8_t value3;
uint8_t check_value3 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/*Obs³uga potencjometrów*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
   	if(htim->Instance == TIM4)
   	{
   		DISP_2_OFF;
   		DISP_VAL_NULL;
		DISP_1_ON;
		switch(counter){
		    case 0: DISP_VAL_0; break;
		    case 1: DISP_VAL_1; break;
		    case 2: DISP_VAL_2; break;
		    case 3: DISP_VAL_3; break;
		    case 4: DISP_VAL_4; break;
		    case 5: DISP_VAL_5; break;
		    case 6: DISP_VAL_6; break;
		    case 7: DISP_VAL_7; break;
		    case 8: DISP_VAL_8; break;
		    case 9: DISP_VAL_9; break;
		    case 10: {
		    	DISP_1_OFF;
		    	DISP_VAL_NULL;
		    	DISP_2_ON;
		    	DISP_DOT;
		    	break;
		    }
		}
   		HAL_ADC_Start(&hadc1);
   		HAL_ADC_Start(&hadc2);
   		HAL_ADC_Start(&hadc3);
   		if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
   		{
   			value1 = HAL_ADC_GetValue(&hadc1)/32;
   			if(value1<12) counter = 0;
   			else if(value1>=12 && value1<24) counter = 1;
   			else if(value1>=24 && value1<36) counter = 2;
   			else if(value1>=36 && value1<48) counter = 3;
   			else if(value1>=48 && value1<60) counter = 4;
   			else if(value1>=60 && value1<72) counter = 5;
   			else if(value1>=72 && value1<84) counter = 6;
   			else if(value1>=84 && value1<96) counter = 7;
   			else if(value1>=96 && value1<108) counter = 8;
   			else if(value1>=108 && value1<120) counter = 9;
   			else if(value1>=120) counter = 10;
   		}

   		if(HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
   		{
   		   	value2 = HAL_ADC_GetValue(&hadc2)/32;
   		   	if(fabs(value2 - check_value2)>1)
   		   	{
   		   		uint8_t sendControl[3] = {177, 1, value2};
   		   	    HAL_UART_Transmit_IT(&huart3, sendControl, 3);
   				check_value2 = value2;
   		   	}
   		}

   		if(HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK)
   		{
   		    value3 = HAL_ADC_GetValue(&hadc3)/32;
   		    if(fabs(value3 - check_value3)>1)
   		   	{
   		   		uint8_t sendControl[3] = {178, 1, value3};
   		   		HAL_UART_Transmit_IT(&huart3, sendControl, 3);
   		   		check_value3 = value3;
   		   	}
   		}
    }
}

/*Obs³uga przycisków*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
/*Obs³uga pianina*/
	if(GPIO_Pin == GPIO_PIN_2 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 0)
	{
			uint8_t sendControl[3] = {144, 0 + (12 * counter), 64};
		   	HAL_UART_Transmit_IT(&huart3, sendControl, 3);

	}
	else if(GPIO_Pin == GPIO_PIN_2 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 1)
	{
			uint8_t sendControl[3] = {129, 0 + (12 * counter), 0};
		    HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_3 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == 0)
	{
			uint8_t sendControl[3] = {144, 1 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);

	}
	else if(GPIO_Pin == GPIO_PIN_3 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == 1)
	{
			uint8_t sendControl[3] = {129, 1 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_4 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == 0)
	{
			uint8_t sendControl[3] = {144, 2 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_4 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == 1)
	{
			uint8_t sendControl[3] = {129, 2 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_5 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == 0)
	{
			uint8_t sendControl[3] = {144, 3 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_5 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == 1)
	{
			uint8_t sendControl[3] = {129, 3 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == 0)
	{
			uint8_t sendControl[3] = {144, 4 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == 1)
	{
			uint8_t sendControl[3] = {129, 4 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == 0)
	{
			uint8_t sendControl[3] = {144, 5 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == 1)
	{
			uint8_t sendControl[3] = {129, 5 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == 0)
	{
			uint8_t sendControl[3] = {144, 6 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == 1)
	{
			uint8_t sendControl[3] = {129, 6 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9) == 0)
	{
			uint8_t sendControl[3] = {144, 7 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9) == 1)
	{
			uint8_t sendControl[3] = {129, 7 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_10 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == 0)
	{
			uint8_t sendControl[3] = {144, 8 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_10 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == 1)
	{
			uint8_t sendControl[3] = {129, 8 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_11 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == 0)
	{
			uint8_t sendControl[3] = {144, 9 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_11 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == 1)
	{
			uint8_t sendControl[3] = {129, 9 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_12 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 0)
	{
			uint8_t sendControl[3] = {144, 10 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_12 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 1)
	{
			uint8_t sendControl[3] = {129, 10 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_13 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == 0)
	{
			uint8_t sendControl[3] = {144, 11 + (12 * counter), 64};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
	else if(GPIO_Pin == GPIO_PIN_13 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == 1)
	{
			uint8_t sendControl[3] = {129, 11 + (12 * counter), 0};
			HAL_UART_Transmit_IT(&huart3, sendControl, 3);
	}
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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
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
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 839;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
