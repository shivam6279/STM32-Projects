#include "USART.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

UART_HandleTypeDef huart1;

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
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
