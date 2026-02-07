#include "main.h"
#include "BLDC.h"
#include "PWM.h"
#include "ADC.h"
#include "diags.h"
#include "USART.h"
#include "string_utils.h"
#include "tones.h"
#include "EEPROM.h"

#include <stdio.h>

DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

DCACHE_HandleTypeDef hdcache1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

uint8_t board_id = 0;

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_DCACHE1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM2_Init(void);
static void TIM4_init(float);
static void TIM5_init(float);
static void TIM6_init(float);
static void TIM12_init();

/*
Timers
TIM1 - Motor PWM
TIM2 - Encoder
TIM3 - Brake Motor
TIM4 - FOC
TIM5 - motor angle/RPM PID
TIM6 - Sensorless
TIM7 - PWM Sync
TIM8 -
TIM12 - Tones
TIM13 -
TIM14 -
TIM15 - Servo PWM
TIM16 -
TIM17 -
*/

volatile uint8_t can_rx_rdy = 0;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
		HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		can_rx_rdy = 1;
	}
	can_rx_rdy = 1;
}

int main(void) {
	HAL_Init();
	MPU_Config();
	SystemClock_Config();
	MX_GPIO_Init();

	MX_USART1_UART_Init();

	MX_GPDMA1_Init();
	ADCInit();
	MX_DCACHE1_Init();
	MX_FDCAN1_Init();
	MX_I2C1_Init();
	MX_ICACHE_Init();
	MX_TIM1_Init(48000); // Motor pwm freq
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM15_Init();

	TIM4_init(25000);
	TIM5_init(1000);
	TIM6_init(5000);
	TIM12_init();

	// CAN PHY in normal mode (not SILENT)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	// CAN_S

	ee_init(&eeprom_data, sizeof(eeprom_data_t));

//	ee_format();

	eeprom_data.zero_offset = 33.5;
//	ee_write();
//	eeprom_data.zero_offset = 0;
//	ee_read();

	setvbuf(stdout, NULL, _IONBF, 0); // Disable printf buffering

	HAL_Delay(250);

	GPIOC->ODR |= 1 << 14; // Enable encoder vcc

	TIM3->CCR1 = 1000;// Enable servo vcc
	TIM3->CCR2 = 0;

	uint16_t a, b, c, d, e, f;

	while (1) {
		// printf("%f, %d\n", eeprom_data.zero_offset, ee_capacity());

		a = adc_buffer[0] >> 16;
		b = adc_buffer[0] & 0xFFFF;
		c = adc_buffer[1] >> 16;
		d = adc_buffer[1] & 0xFFFF;
		e = adc_buffer[2] >> 16;
		f = adc_buffer[2] & 0xFFFF;

		if(can_rx_rdy) {
			can_rx_rdy = 0;
			printf("%d, %d, %d, %d, %d, %d, %d, %d\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
			for(int i = 0; i < 8; i++) {
				RxData[i] = 0;
			}
		}

		if(rx_rdy) {
			rx_rdy = 0;
			if(str_beginsWith(rx_buffer, "diags")) {
				diagsMenu();
			}
		}

		HAL_Delay(1);
	}
}

void TIM4_init(float f) {
	RCC->APB1LENR |= 1 << 2;

	TIM4->CR1 = 0;
	TIM4->CR2 = 0;

	TIM4->PSC = 25; // 10 MHz after prescaler
	TIM4->ARR = (float)(10000000.0/f);

	TIM4->CNT = 0;

	TIM4->EGR |= 1;

	TIM4->DIER |= 1;

	NVIC_SetPriority(TIM4_IRQn, 1);
	TIM4->SR &= ~(0x1);
	NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM5_init(float f) {
	RCC->APB1LENR |= 1 << 3;

	TIM5->CR1 = 0;
	TIM5->CR2 = 0;

	TIM5->PSC = 25; // 10 MHz after prescaler
	TIM5->ARR = (float)(10000000.0/f);

	TIM5->CNT = 0;

	TIM5->EGR |= 1;

	TIM5->DIER |= 1;

	NVIC_SetPriority(TIM5_IRQn, 2);
	TIM5->SR &= ~(0x1);
	NVIC_EnableIRQ(TIM5_IRQn);
}

void TIM6_init(float f) {
	RCC->APB1LENR |= 1 << 4;

	TIM6->CR1 = 0;
	TIM6->CR2 = 0;

	TIM6->PSC = 25; // 10 MHz after prescaler
	TIM6->ARR = (float)(10000000.0/f);

	TIM6->CNT = 0;

	TIM6->EGR |= 1;

	TIM6->DIER |= 1;

	NVIC_SetPriority(TIM6_IRQn, 0);
	TIM6->SR &= ~(0x1);
	NVIC_EnableIRQ(TIM6_IRQn);
}

void TIM12_init() {
	RCC->APB1LENR |= 1 << 6;

	TIM12->CR1 = 0;
	TIM12->CR2 = 0;

	TIM12->PSC = 5-1; // 50 MHz after prescaler
	TIM12->ARR = 833;

	TIM12->CNT = 0;

	TIM12->EGR |= 1;

	TIM12->DIER |= 1;

	NVIC_SetPriority(TIM12_IRQn, 5);
	TIM12->SR &= ~(0x1);
	NVIC_EnableIRQ(TIM12_IRQn);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
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

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = {0};
	MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	*/
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x20000000;
	MPU_InitStruct.LimitAddress = 0x20000040;
	MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
	MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RW;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_OUTER_SHAREABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	MPU_AttributesInit.Number = MPU_REGION_NUMBER0;
	MPU_AttributesInit.Attributes = MPU_NOT_CACHEABLE;

	HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);
}

static void MX_FDCAN1_Init(void) {
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 5;
	hfdcan1.Init.NominalSyncJumpWidth = 10;
	hfdcan1.Init.NominalTimeSeg1 = 39;
	hfdcan1.Init.NominalTimeSeg2 = 10;
	hfdcan1.Init.DataPrescaler = 2;
	hfdcan1.Init.DataSyncJumpWidth = 5;
	hfdcan1.Init.DataTimeSeg1 = 19;
	hfdcan1.Init.DataTimeSeg2 = 5;
	hfdcan1.Init.StdFiltersNbr = 0;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0;
	sFilterConfig.FilterID2 = 0;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

//	if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 7, 0) != HAL_OK) {
//		Error_Handler();
//	}
//	if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
//		Error_Handler();
//	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_GPDMA1_Init(void) {
	__HAL_RCC_GPDMA1_CLK_ENABLE();
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

static void MX_TIM2_Init(void) {
	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIMEx_EncoderIndexConfigTypeDef sEncoderIndexConfig = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4.294967295E9;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sEncoderIndexConfig.Polarity = TIM_ENCODERINDEX_POLARITY_NONINVERTED;
	sEncoderIndexConfig.Prescaler = TIM_ENCODERINDEX_PRESCALER_DIV1;
	sEncoderIndexConfig.Filter = 0;
	sEncoderIndexConfig.FirstIndexEnable = DISABLE;
	sEncoderIndexConfig.Position = TIM_ENCODERINDEX_POSITION_00;
	sEncoderIndexConfig.Direction = TIM_ENCODERINDEX_DIRECTION_UP_DOWN;
	if (HAL_TIMEx_ConfigEncoderIndex(&htim2, &sEncoderIndexConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC10 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PC6 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // For SPI
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PE0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void Error_Handler(void) {
//	__disable_irq();
	while (1)
	{
		LED0_ON();
		LED1_OFF();
		HAL_Delay(100);
		LED0_OFF();
		LED1_ON();
		HAL_Delay(100);
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
	/* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif
