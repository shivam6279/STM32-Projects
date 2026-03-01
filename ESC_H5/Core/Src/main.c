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

CORDIC_HandleTypeDef hcordic;

DCACHE_HandleTypeDef hdcache1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];
uint16_t can_id;

SPI_HandleTypeDef hspi1;
DMA_NodeTypeDef Node_GPDMA1_Channel2;
DMA_QListTypeDef List_GPDMA1_Channel2;
DMA_HandleTypeDef handle_GPDMA1_Channel2;
DMA_NodeTypeDef Node_GPDMA1_Channel1;
DMA_QListTypeDef List_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

uint8_t board_id = 0;

void SystemClock_Config(void);
static void MX_CORDIC_Init(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_DCACHE1_Init(void);
static void MX_SPI1_Init(void);
static void MX_FDCAN1_Init(uint8_t, uint8_t);
static void MX_I2C1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM2_Init(uint8_t);
static void TIM4_init(float);
static void TIM5_init(float);
static void TIM6_init(float);
static void TIM12_init();
static void TIM16_init();
void delay_us(uint16_t);

/*
Timers
TIM1 - Motor PWM
TIM2 - Encoder
TIM3 - Brake Motor
TIM4 -
TIM5 - motor angle/RPM PID
TIM6 - Sensorless
TIM7 - PWM Sync
TIM8 -
TIM12 - Tones
TIM13 -
TIM14 -
TIM15 - Servo PWM
TIM16 - us Delay
TIM17 -
*/

volatile uint8_t spi_rx_buf[4];
volatile uint8_t spi_tx_cmd[4] = {0, 0, 0, 0};

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_5) {
		LED1_ON();
		// Manually trigger SPI DMA start
		// This takes ~1us to execute; data arrives ~1.3us later
		HAL_SPI_Receive_DMA(&hspi1, spi_rx_buf, 4);
		HAL_SPI_Transmit_DMA(&hspi1, spi_tx_cmd, 4);
		LED1_OFF();
	}
}

float save_data[1500][10];

volatile uint8_t can_rx_rdy = 0;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

		if(RxHeader.Identifier == can_id) {
			can_rx_rdy = 1;
		} else if(RxHeader.Identifier == (can_id+1)) {
			rx_rdy = 2;
			set_serial_mode(SER_MODE_BOTH);
		}

		HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	}
}

void change_motor_mode(char new_mode) {
	pid_focIq.setpoint = 0;
	pid_focId.setpoint = 0;
	pid_rpm.setpoint = 0;
	pid_angle.setpoint = 0;
	PID_reset(&pid_focIq);
	PID_reset(&pid_focId);
	PID_reset(&pid_rpm);
	PID_reset(&pid_angle);
	if(new_mode == 'P') {
		mode = MODE_POWER;
	} else if(new_mode == 'R') {
		pid_focIq.setpoint = 0;
		pid_focId.setpoint = 0;
		mode = MODE_RPM;
	} else if(new_mode == 'A') {
		reset_motion_observer();
		mode = MODE_POS;
	} else if(new_mode == 'X') {
		mode = MODE_OFF;
		MotorOff();
		SetPower(0);
	}
}

int main(void) {
	uint16_t i;

	HAL_Init();
	MPU_Config();
	SystemClock_Config();
	MX_GPIO_Init();

	MX_USART1_UART_Init();

	MX_GPDMA1_Init();
	ADCInit();
	MX_DCACHE1_Init();
	MX_I2C1_Init();
	MX_ICACHE_Init();
	MX_TIM1_Init(50000); // Motor pwm freq
	MX_TIM3_Init();
	MX_TIM15_Init();

//	TIM4_init(25000);
	TIM5_init(10000);
	TIM6_init(5000);
	TIM12_init();
	TIM16_init();

	MX_CORDIC_Init();

	set_serial_mode(SER_MODE_UART);

	// CAN PHY in normal mode (not SILENT)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	// CAN_S

	ee_read();
	board_id = eeprom_data.board_id;
	enc_direction = eeprom_data.enc_direction;
	motor_zero_angle = eeprom_data.zero_offset;
	motor_pole_pairs = eeprom_data.pole_pairs;
	motor_direction = eeprom_data.motor_direction;
	diags_power = eeprom_data.diags_power;
	tone_power = eeprom_data.tone_power;
	tone_amplitude = eeprom_data.tone_amplitude;

	can_id = 0x300 + 0xF*(board_id-1);
	MX_FDCAN1_Init(can_id, 0xF);	

	MX_TIM2_Init(enc_direction); // Encoder timer
//	MX_SPI1_Init();

	MotorPIDInit();

	setvbuf(stdout, NULL, _IONBF, 0); // Disable printf buffering

	HAL_Delay(250);

	GPIOC->ODR |= 1 << 14; // Enable encoder vcc

	PlayWav();
	HAL_Delay(500);

	TIM3->CCR1 = 1000;// Enable servo vcc
	TIM3->CCR2 = 0;

	SetPower(0);
	mode = MODE_OFF;
	waveform_mode = MOTOR_FOC;

	char rx_buffer_local[RX_BUFFER_SIZE];

	pid_focId.setpoint = 0;
	uint8_t can_motor_mode = 'X';
	float can_float;
	while (1) {

		if(can_rx_rdy) {
			can_rx_rdy = 0;
			if(RxData[0] != can_motor_mode) {
				can_motor_mode = RxData[0];
				change_motor_mode(can_motor_mode);
			}
			uint32_t can_float_temp = RxData[1] << 24 | RxData[2] << 16 | RxData[3] << 8 | RxData[4];
			can_float = *(float*)((uint32_t*)&can_float_temp);
		}

		if(rx_rdy) {
			if(rx_rdy == 1) {
				HAL_NVIC_DisableIRQ(USART1_IRQn);
				for(i = 0; rx_buffer[i] != '\0'; i++) {
					rx_buffer_local[i] = rx_buffer[i];
				}
				HAL_NVIC_EnableIRQ(USART1_IRQn);
				rx_buffer_local[i] = '\0';
			} else if(rx_rdy == 2) {
				rx_rdy = 0;
				for(i = 0; RxData[i] != '\0'; i++) {
					rx_buffer_local[i] = RxData[i];
				}
				rx_buffer_local[i] = '\0';
			} else {
				continue;
			}

			str_removeChar(rx_buffer_local, '\n');
			str_removeChar(rx_buffer_local, '\r');

			if(str_beginsWith(rx_buffer_local, "diags")) {
				diagsMenu();
			} else if(str_beginsWith(rx_buffer_local, "capture")) {
				break;
			} else if(rx_buffer_local[1] == '\0' && (rx_buffer_local[0] >= 'A' && rx_buffer_local[0] <= 'Z')) {
				change_motor_mode(rx_buffer_local[0]);

			} else if(str_isFloat(rx_buffer_local)) {
				float input;
				input = str_toFloat(rx_buffer_local);
				if(mode == MODE_POWER) {
					SetPower(input / 2000.0f);
					pid_focIq.setpoint = input;
				} else if(mode == MODE_RPM) {
					pid_rpm.setpoint = input;
				} else if(mode == MODE_POS) {
					pid_angle.setpoint = input;
				}
			}
		}

		// serial_buffer_len = snprintf(serial_buffer, sizeof(serial_buffer),"%f\n", GetPosition());
		// serial_buffer_len = snprintf(serial_buffer, sizeof(serial_buffer),"%f\t%f\t%f\n", GetPosition(), GetRPM(), GetAcc());
		// serial_buffer_len = snprintf(serial_buffer, sizeof(serial_buffer),"%.2f, %.3f\t%.3f\n", thermal_energy, foc_iq, foc_id);
		// serial_buffer_len = snprintf(serial_buffer, sizeof(serial_buffer),"%.2f\t%.3f\t%.3f\t%.3f\n", foc_id, foc_iq, pid_focIq.ki*pid_focIq.integral, pid_focIq.output);
		snprintf(serial_buffer, sizeof(serial_buffer),"%.2f\t%.3f\t%.3f\t%.3f\t%.3f\n", GetRPM(), foc_iq, foc_id, pid_focIq.output, pid_focId.output);
		// serial_buffer_len = snprintf(serial_buffer, sizeof(serial_buffer),"%.3f\t%.3f\t%.3f\t%.3f\n", angle_el/180.0, isns_u, isns_v, isns_w);
		send_serial(serial_buffer);

		// HAL_Delay(1);
	}

	for(i = 0; i < 1500; i++) {
		save_data[i][0] = angle_el/180.0f;
		save_data[i][1] = vsns_u;
		save_data[i][2] = vsns_v;
		save_data[i][3] = vsns_w;
		save_data[i][4] = vsns_x;
		save_data[i][5] = isns_u;
		save_data[i][6] = isns_v;
		save_data[i][7] = isns_w;
		save_data[i][8] = foc_iq;
		save_data[i][9] = foc_id;
		delay_us(20);
//		delay_us(200);
	}
	mode = MODE_OFF;
	MotorOff();
	for(i = 0; i < 1500; i++) {
		printf("%.6f, ", save_data[i][0]);
		// printf("%.2f, ", save_data[i][1]);
		// printf("%.2f, ", save_data[i][2]);
		// printf("%.2f, ", save_data[i][3]);
		// printf("%.6f, ", save_data[i][4]);
		printf("%.6f, ", save_data[i][5]);
		printf("%.6f, ", save_data[i][6]);
		printf("%.6f, ", save_data[i][7]);
		printf("%.6f, ", save_data[i][8]);
		printf("%.6f\n", save_data[i][9]);
		HAL_Delay(10);
	}
	while(1);

}

void TIM4_init(float f) {
	RCC->APB1LENR |= 1 << 2;

	TIM4->CR1 = 0;
	TIM4->CR2 = 0;

	TIM4->PSC = 25; // 10 MHz after prescaler
	TIM4->ARR = (float)(10000000.0f/f);

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
	TIM5->ARR = (float)(10000000.0f/f);

	TIM5->CNT = 0;

	TIM5->EGR |= 1;

	TIM5->DIER |= 1;

	NVIC_SetPriority(TIM5_IRQn, 2);
	TIM5->SR &= ~(0x1);
	NVIC_EnableIRQ(TIM5_IRQn);

	TIM5->CR1 |= 1;
}

void TIM6_init(float f) {
	RCC->APB1LENR |= 1 << 4;

	TIM6->CR1 = 0;
	TIM6->CR2 = 0;

	TIM6->PSC = 25; // 10 MHz after prescaler
	TIM6->ARR = (float)(10000000.0f/f);

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

void TIM16_init() {
	RCC->APB2ENR |= 1 << 17;

	TIM16->CR1 = 0;
	TIM16->CR2 = 0;

	TIM16->PSC = 250-1; // 1 MHz after prescaler
	TIM16->ARR = -1;

	TIM16->CNT = 0;

	TIM16->EGR |= 1;

	TIM16->DIER |= 1;
}

void delay_us(uint16_t d) {
	TIM16->CNT = 0;
	TIM16->CR1 |= 1;
	while(TIM16->CNT < d);
	TIM16->CR1 &= ~1;
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

static void MX_CORDIC_Init(void) {
	hcordic.Instance = CORDIC;
	if (HAL_CORDIC_Init(&hcordic) != HAL_OK)	{
		Error_Handler();
	}

	CORDIC_ConfigTypeDef sConfig = {0};

	// Initial setup
	sConfig.Function   = CORDIC_FUNCTION_SINE;     /* Sine + Cosine */
	sConfig.Precision  = CORDIC_PRECISION_6CYCLES; /* 6 cycles for FOC is plenty */
	sConfig.Scale      = CORDIC_SCALE_0;           /* Range [-1, 1] */
	sConfig.NbWrite    = CORDIC_NBWRITE_1;         /* 1 input: Angle */
	sConfig.NbRead     = CORDIC_NBREAD_2;          /* 2 outputs: Sin then Cos */
	sConfig.InSize     = CORDIC_INSIZE_32BITS;     /* q31 input */
	sConfig.OutSize    = CORDIC_OUTSIZE_32BITS;    /* q31 output */

	if (HAL_CORDIC_Configure(&hcordic, &sConfig) != HAL_OK) {
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

static void MX_SPI1_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7; // MISO MOSI
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_15; // CS_L
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3; // SCK
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 0x7;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
	hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_FDCAN1_Init(uint8_t id, uint8_t range) {
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
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = id;
	sFilterConfig.FilterID2 = sFilterConfig.FilterID1 + range;

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

static void MX_TIM2_Init(uint8_t dir) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_5; // A
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3; // B
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_15; // I
	GPIO_InitStruct.Alternate = GPIO_AF14_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIMEx_EncoderIndexConfigTypeDef sEncoderIndexConfig = {0};

	__HAL_RCC_TIM2_CLK_ENABLE();

	TIM2->SMCR &= ~TIM_SMCR_SMS;
	TIM2->SMCR |= (3U << TIM_SMCR_SMS_Pos); // TI1 + TI2 mode

    // I signal
    TIM2->SMCR &= ~TIM_SMCR_TS;
	TIM2->SMCR |= (7U << TIM_SMCR_TS_Pos);

	// TIM2->ECR |= 0b10 << 6;
	TIM2->ECR |= TIM_ECR_IE; // I pulse reset in both directions

    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM2->CCMR1 |= (1U << TIM_CCMR1_CC1S_Pos) | (1U << TIM_CCMR1_CC2S_Pos); // Set to inputs

    TIM2->CCMR1 |= (3U << TIM_CCMR1_IC1F_Pos) | (3U << TIM_CCMR1_IC2F_Pos); // Filter

    // Reset Counter and set Max Range
    TIM2->CNT = 0;
    TIM2->ARR = 0xFFFFFFFF; // TIM2 is 32-bit

    if(dir) {
    	TIM2->CCER |= TIM_CCER_CC1P;
    }

    /* 4. Start Timer */
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC10 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC6 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
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

	// Encoder and SPI pins to HI-Z
	// Set them up as application specific later
	// Configure GPIO pins : PA5 PA6 PA7 PA15 PB3
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
