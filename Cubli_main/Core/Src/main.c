#include "main.h"
#include <stdio.h>
#include "ADC.h"
#include "diags.h"
#include "USART.h"
#include "EEPROM.h"

DCACHE_HandleTypeDef hdcache1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim13;

FDCAN_TxHeaderTypeDef CAN_TxHeader;
volatile FDCAN_RxHeaderTypeDef RxHeader;
volatile uint8_t CAN_RxData[64];

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DCACHE1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM13_Init(void);

volatile uint8_t initial_press = 0, initial_delay = 0;

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_3) {
		GPIOA->ODR &= ~(1 << 7);

		if(initial_press) {
			TIM13->CNT = 0;
			TIM13->SR &= ~(0x1);
			HAL_TIM_Base_Start_IT(&htim13);
		}
	}
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_3) {
		GPIOA->ODR |= (1 << 7);
		if(initial_delay) {
			initial_press = 1;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM13 && initial_press) {
		GPIOA->ODR &= ~(1 << 2);
	}
}

CanMessage_t can_rxBuffer[CAN_BUFFER_SIZE];
volatile uint8_t can_buffer_head = 0;
volatile uint8_t can_buffer_tail = 0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	static uint8_t next_head;
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		 while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
			next_head = (can_buffer_head + 1) % CAN_BUFFER_SIZE;
			if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, can_rxBuffer[can_buffer_head].Data) == HAL_OK) {
				can_rxBuffer[can_buffer_head].Identifier = RxHeader.Identifier;
				can_rxBuffer[can_buffer_head].DataLength = RxHeader.DataLength;
				can_buffer_head = next_head;
			} else {
				break;
			}
		 }
	}
}

uint8_t can_rxbuffer_available() {
	return (can_buffer_head + CAN_BUFFER_SIZE - can_buffer_tail) % CAN_BUFFER_SIZE;
}

void pop_can_rxbuffer(CanMessage_t *ret) {
	if(can_buffer_head != can_buffer_tail) {
		HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
		*ret = can_rxBuffer[can_buffer_tail];
		can_buffer_tail = (can_buffer_tail + 1) % CAN_BUFFER_SIZE;
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		return ret;
	}
}

int main(void) {
	MPU_Config();
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_DCACHE1_Init();
	MX_FDCAN1_Init();
	MX_I2C1_Init();
	MX_ICACHE_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_TIM13_Init();

	GPIOC->ODR &= ~(1 << 13); // CAN S
	HAL_TIM_Base_Stop_IT(&htim13);

	HAL_Delay(250);
	GPIOA->ODR |= 1 << 2;

	HAL_Delay(500);
	initial_delay = 1;

	CAN_TxHeader.Identifier = 0x100;
	CAN_TxHeader.IdType = FDCAN_STANDARD_ID;
	CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	CAN_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	CAN_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	CAN_TxHeader.FDFormat = FDCAN_FD_CAN;
	CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CAN_TxHeader.MessageMarker = 0;

	printf("Start\n");

	uint16_t i;
	char rx_buffer_local[RX_BUFFER_SIZE];

	while (1) {

		if(rx_rdy) {
			HAL_NVIC_DisableIRQ(USART1_IRQn);
			for(i = 0; rx_buffer[i] != '\0'; i++) {
				rx_buffer_local[i] = rx_buffer[i];
			}
			rx_rdy = 0;
			HAL_NVIC_EnableIRQ(USART1_IRQn);
			rx_buffer_local[i] = '\0';

			str_removeChar(rx_buffer_local, '\n');
			str_removeChar(rx_buffer_local, '\r');

			if(str_beginsWith(rx_buffer_local, "diags")) {
				diagsMenu();
			}
		}
//    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_TxHeader, CAN_TxData);
	}
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 6;
	RCC_OscInitStruct.PLL.PLLN = 125;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
															|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
															|RCC_CLOCKTYPE_PCLK3;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_ADC1_Init(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_DCACHE1_Init(void) {
	hdcache1.Instance = DCACHE1;
	hdcache1.Init.ReadBurstType = DCACHE_READ_BURST_WRAP;
	if (HAL_DCACHE_Init(&hdcache1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_FDCAN1_Init(void) {
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 1;
	hfdcan1.Init.NominalSyncJumpWidth = 50;
	hfdcan1.Init.NominalTimeSeg1 = 199;
	hfdcan1.Init.NominalTimeSeg2 = 50;
	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 12;
	hfdcan1.Init.DataTimeSeg1 = 37;
	hfdcan1.Init.DataTimeSeg2 = 12;
	hfdcan1.Init.StdFiltersNbr = 1;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 7, 0) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x200;
	sFilterConfig.FilterID2 = 0x2FF;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_I2C1_Init(void) {
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x60808CD3;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_ICACHE_Init(void) {
	ICACHE_RegionConfigTypeDef pRegionConfig = {0};

	pRegionConfig.BaseAddress = 0x0;
	pRegionConfig.RemapAddress = 0x0;
	pRegionConfig.Size = ICACHE_REGIONSIZE_2MB;
	pRegionConfig.TrafficRoute = ICACHE_MASTER1_PORT;
	pRegionConfig.OutputBurstType = ICACHE_OUTPUT_BURST_WRAP;
	if (HAL_ICACHE_EnableRemapRegion(ICACHE_REGION_0, &pRegionConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_ICACHE_Enable() != HAL_OK) {
		Error_Handler();
	}
}

static void MX_TIM13_Init(void) {
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim13.Instance = TIM13;
	htim13.Init.Prescaler = 25000;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 10000;
	htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC4 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = {0};
	MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

	HAL_MPU_Disable();

	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x24000000;
	MPU_InitStruct.LimitAddress = 0x24000040;
	MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
	MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RW;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	MPU_AttributesInit.Number = MPU_REGION_NUMBER0;
	MPU_AttributesInit.Attributes = MPU_NOT_CACHEABLE;

	HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
	HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
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
