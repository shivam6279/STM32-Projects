#include "USART.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

UART_HandleTypeDef huart1;
char serial_buffer[64];
uint8_t serial_buffer_len;
uint8_t serial_mode = 0;

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

void send_serial(char str[]) {
	if(serial_mode != SER_MODE_UART) {
		CAN_send_serial(str);
	}
	if(serial_mode != SER_MODE_CAN) {
		HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	}
}

void set_serial_mode(uint8_t mode) {
	serial_mode = mode;
}

void CAN_send_serial(char str[]) {
	static uint8_t CAN_TxData[64];
	static uint8_t i;

	CAN_TxHeader.Identifier = can_id + 1 - 0x100;
	CAN_TxHeader.IdType = FDCAN_STANDARD_ID;
	CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	CAN_TxHeader.DataLength = FDCAN_DLC_BYTES_64;
	CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	CAN_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	CAN_TxHeader.FDFormat = FDCAN_FD_CAN;
	CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CAN_TxHeader.MessageMarker = 0;

	for(i = 0; str[i] != '\0' && i < 64; i++) {
		CAN_TxData[i] = str[i];
	}
	for(; i < 64; i++) {
		CAN_TxData[i] = '\0';
	}
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_TxHeader, CAN_TxData);
}

volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
static volatile unsigned int rx_buffer_index = 0;
volatile unsigned char rx_rdy = 0;
volatile unsigned char play_tone = 0;
volatile unsigned char auto_stop = 1;

void USART1_IRQHandler(void) {
	static unsigned int r;
	static uint8_t overflow = 0;
	while(USART1->ISR & USART_ISR_RXNE) {
		r = USART1->RDR & 0xFF;
		if(r == '\r') {
			rx_buffer[rx_buffer_index] = '\0';
			rx_buffer_index = 0;

			if(!overflow) {
				rx_rdy = 1;
			}

			overflow = 0;
		} else if(!overflow) {
			rx_buffer[rx_buffer_index++] = r;
			if(rx_buffer_index >= (RX_BUFFER_SIZE - 1)) {
				overflow = 1;
				rx_buffer_index = 0;
			}
		}
	}
}

void MX_USART1_UART_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	USART1->CR1 |= USART_CR1_RXNEIE;
	HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}
