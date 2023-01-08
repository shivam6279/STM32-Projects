/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);

static void delay_us(unsigned long);
static void ADC_Select_CH(unsigned char);
static void BLDC_phase(unsigned char, float);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
volatile uint16_t adc_uvw[5000], adc_x[5000], i_1 = 0, i_2 = 0;
unsigned int p = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1 && i_1 < 5000) {
//    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    	adc_uvw[0] = HAL_ADC_GetValue(&hadc1);
//    	adc_uvw[i_1++] = HAL_ADC_GetValue(&hadc1);
    } else if(hadc->Instance == ADC2 && i_2 < 5000) {
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		adc_x[0] = HAL_ADC_GetValue(&hadc2);
//		adc_x[i_2++] = HAL_ADC_GetValue(&hadc2);
	}
}

void ADC_Select_CH(unsigned char ch)
{
	ADC1->CR &= ~ADC_CR_ADEN;
	ADC1->SQR1 &= 0xFFFFF83F;
	ADC1->SQR1 |= ch << ADC_SQR1_SQ1_Pos ;
	ADC1->CR |= ADC_CR_ADEN;
}

void delay_us(unsigned long t) {
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < t);
}

unsigned char wait_bemf_rising(float t) {
	unsigned int initial_delay = 100, tim_temp;
	unsigned int advance = 10;
	if(t/4 > 100) {
		initial_delay = t/4;
	}
	delay_us(initial_delay);

	if(p > 1000) {
		while(adc_uvw[0] < adc_x[0] && __HAL_TIM_GET_COUNTER(&htim2) < t*0.65);
	} else {
		while(adc_uvw[0] < 1732 && __HAL_TIM_GET_COUNTER(&htim2) < t*0.65);
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);

	tim_temp = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < (tim_temp - advance));

	return 1;
}

unsigned char wait_bemf_falling(float t) {
	unsigned int initial_delay = 100, tim_temp;
	unsigned int advance = 10;
	if(t/4 > 100) {
		initial_delay = t/4;
	}
	delay_us(initial_delay);

	if(p > 1000) {
		while(adc_uvw[0] > adc_x[0] && __HAL_TIM_GET_COUNTER(&htim2) < t*0.65);
	} else {
		while(adc_uvw[0] > 1732 && __HAL_TIM_GET_COUNTER(&htim2) < t*0.65);
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);

	tim_temp = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < (tim_temp - advance));

	return 1;
}

void BLDC_phase(unsigned char phase, float p) {
	if(phase == 1) {
		TIM1->CCER |= TIM_CCER_CC1NP;
		TIM1->CCER &= ~TIM_CCER_CC2NP;
		TIM1->CCER &= ~TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = 0;
		TIM1->CCR2 = p;
		TIM1->CCR3 = 0;
	} else if(phase == 2) {
		TIM1->CCER &= ~TIM_CCER_CC1NP;
		TIM1->CCER &= ~TIM_CCER_CC2NP;
		TIM1->CCER |= TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = 0;
		TIM1->CCR2 = p;
		TIM1->CCR3 = 0;
	} else if(phase == 3) {
		TIM1->CCER &= ~TIM_CCER_CC1NP;
		TIM1->CCER |= TIM_CCER_CC2NP;
		TIM1->CCER &= ~TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = p;
	} else if(phase == 4) {
 		TIM1->CCER |= TIM_CCER_CC1NP;
 		TIM1->CCER &= ~TIM_CCER_CC2NP;
 		TIM1->CCER &= ~TIM_CCER_CC3NP;
 		TIM1->EGR |= TIM_EGR_COMG;
 		TIM1->CCR1 = 0;
 		TIM1->CCR2 = 0;
 		TIM1->CCR3 = p;
	} else if(phase == 5) {
		TIM1->CCER &= ~TIM_CCER_CC1NP;
		TIM1->CCER &= ~TIM_CCER_CC2NP;
		TIM1->CCER |= TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = p;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
	} else if(phase == 6) {
		TIM1->CCER &= ~TIM_CCER_CC1NP;
		TIM1->CCER |= TIM_CCER_CC2NP;
		TIM1->CCER &= ~TIM_CCER_CC3NP;
		TIM1->EGR |= TIM_EGR_COMG;
		TIM1->CCR1 = p;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
	}
}

#define LPF 0.4f

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
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
//	MX_TIM6_Init();
	MX_USART2_UART_Init();

//	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim4);
//	HAL_TIM_Base_Start(&htim6);

// 	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)val1, 2);
// 	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)val2, 2);

	//U, V, W:
	//U, V, W, X:

	unsigned int i;
	unsigned char temp[100];
	unsigned int save[5000], save_i = 0;

// 	c = snprintf(temp, sizeof(temp),"U, V, W, X\n");
//	HAL_UART_Transmit(&huart2, temp, c, 100);

//	while(1) {
//		c = snprintf(temp, sizeof(temp),"%d, %d, %d, %d\n", val2[0], val1[0], val1[1], val2[1]);
//		HAL_UART_Transmit(&huart2, temp, c, 100);
//		HAL_Delay(2);
//	}

	TIM1->CCR4 = 450;
	ADC_Select_CH(1);

//	HAL_ADC_Start_IT(&hadc1);
//	HAL_ADC_Start_IT(&hadc2);
//	TIM1->CCER &= ~TIM_CCER_CC1NP;
//	TIM1->CCER &= ~TIM_CCER_CC2NP;
//	TIM1->CCER &= ~TIM_CCER_CC3NP;
//	TIM1->EGR |= TIM_EGR_COMG;
//	TIM1->CCR1 = 0; //CH1
//	TIM1->CCR2 = 0; //CH2
//	TIM1->CCR3 = 1000; //CH4
//	while(1);

	TIM1->CCER |= TIM_CCER_CC1NP;
	TIM1->CCER |= TIM_CCER_CC2NP;
	TIM1->CCER |= TIM_CCER_CC3NP;
	TIM1->EGR |= TIM_EGR_COMG;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	for(i = 0 ; i < 5000; i++) {
		__HAL_TIM_SET_COUNTER(&htim2,0);
		while (__HAL_TIM_GET_COUNTER(&htim2) < 1000);
	}

	unsigned int c = 0, d = 0, t = 2000, tt = 100, max_time = 50000;
	signed int tim_temp;
	float commutation_time = 6000;

//	HAL_ADC_Start_IT(&hadc1);
//	HAL_ADC_Start_IT(&hadc2);

	__HAL_TIM_SET_COUNTER(&htim4,0);
	p = 550;

 	while (1) {
 		for(i = 1; i <= 6; i++) {

 			if(i == 1 || i == 4) {
 				ADC_Select_CH(1);
 			} else if (i == 2 || i == 5) {
 				ADC_Select_CH(4);
 			} else if(i == 3 || i == 6) {
 				ADC_Select_CH(2);
 			}
 			BLDC_phase(i, p);

// 			delay_ms(t);
			if(!d) {
				delay_us(t);
			} else {
				unsigned initial_delay = 100, advance = 10;
				float t;

				if(i % 2 == 1) {
//					wait_bemf_falling(commutation_time);

					t = commutation_time;

					if(t/4 > 100) {
						initial_delay = t/4;
					}

					__HAL_TIM_SET_COUNTER(&htim2,0);
					while (__HAL_TIM_GET_COUNTER(&htim2) < initial_delay);

					if(p > 3000) {
						while(adc_uvw[0] > adc_x[0] && __HAL_TIM_GET_COUNTER(&htim2) < t*0.65);
					} else {
						while(adc_uvw[0] > 1732 && __HAL_TIM_GET_COUNTER(&htim2) < t*0.65);
					}
					commutation_time = LPF * (float)__HAL_TIM_GET_COUNTER(&htim4) + (1.0-LPF) * commutation_time;
					__HAL_TIM_SET_COUNTER(&htim4,0);

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);

					tim_temp = __HAL_TIM_GET_COUNTER(&htim2);
					__HAL_TIM_SET_COUNTER(&htim2,0);
					tim_temp -= advance;
					if(tim_temp > 0) {
						while (__HAL_TIM_GET_COUNTER(&htim2) < tim_temp);
					}

//					while (__HAL_TIM_GET_COUNTER(&htim2) < commutation_time);

				} else {
//					wait_bemf_rising(commutation_time);

					t = commutation_time;

					if(t/4 > 100) {
						initial_delay = t/4;
					}

					__HAL_TIM_SET_COUNTER(&htim2,0);
					while (__HAL_TIM_GET_COUNTER(&htim2) < initial_delay);

					if(p > 3000) {
						while(adc_uvw[0] < adc_x[0] && __HAL_TIM_GET_COUNTER(&htim2) < t*0.65);
					} else {
						while(adc_uvw[0] < 1732 && __HAL_TIM_GET_COUNTER(&htim2) < t*0.65);
					}
					commutation_time = LPF * (float)__HAL_TIM_GET_COUNTER(&htim4) + (1.0-LPF) * commutation_time;
					__HAL_TIM_SET_COUNTER(&htim4,0);

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);

					tim_temp = __HAL_TIM_GET_COUNTER(&htim2);
					__HAL_TIM_SET_COUNTER(&htim2,0);
					tim_temp -= advance;
					if(tim_temp > 0) {
						while (__HAL_TIM_GET_COUNTER(&htim2) < tim_temp);
					}

//					while (__HAL_TIM_GET_COUNTER(&htim2) < commutation_time);
				}
			}
 		}

//		if(d == 1) {
//			HAL_ADC_Stop_IT(&hadc1);
//			HAL_ADC_Stop_IT(&hadc2);
//			break;
//		}

		if(d == 0) {
			if(++c >= 250) {
				d = 1;
				i_1 = 0;
				i_2 = 0;
				HAL_ADC_Start_IT(&hadc1);
				HAL_ADC_Start_IT(&hadc2);
			}
		}

		if(d == 1) {
			d++;
//			p = 1000;
		}
		else if(d > 1) {
			d++;
			if(d % 350 == 0 && p <= 1500) {
				p += 50;
//				if(p == 1500) {
//					TIM1->CCR4 = 1000;
//				}
//				TIM1->CCR4 += 50;
			}
			// 2000: 1.4ms

		}

//		if(i_1 >= 1500 || i_2 >= 1500) {
//			break;
//		}
 	}

 	HAL_ADC_Stop_IT(&hadc1);
	HAL_ADC_Stop_IT(&hadc2);
 	TIM1->CCER |= TIM_CCER_CC1NP;
 	TIM1->CCER |= TIM_CCER_CC2NP;
 	TIM1->CCER |= TIM_CCER_CC3NP;
	TIM1->EGR |= TIM_EGR_COMG;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	for(i = 0; i < i_1; i++) {

		c = snprintf(temp, sizeof(temp),"%d\n", save[i]);
		HAL_UART_Transmit(&huart2, temp, c, 100);

		delay_us(100);
	}
	while(1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

//  /** Configure the ADC multi-mode
//  */
//  multimode.Mode = ADC_MODE_INDEPENDENT;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim1.Init.Period = 7000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIMEx_OC_ConfigPulseOnCompare(&htim1, 0, 0);
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_4);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 50;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 170;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4.294967295E9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

}

static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 170;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
