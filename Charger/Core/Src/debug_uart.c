// Blocking USART1 debug console (PA0/PA1) + printf retarget.

#include "debug_uart.h"
#include "main.h"          // HAL GPIO + CMSIS device header
#include <stdio.h>

#define DBG_USART          USART1
#define DBG_BAUD           115200u
// USART1 kernel clock = PCLK (CCIPR USART1SEL = 00, the reset default) = 48 MHz.
#define DBG_USART_CLK_HZ   48000000u

void DebugUART_Init(void) {
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* PA0 = USART1_TX, PA1 = USART1_RX (AF4). RX pulled up so an open line
	 * idles high rather than framing-erroring. */
	gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
	gpio.Mode      = GPIO_MODE_AF_PP;
	gpio.Pull      = GPIO_PULLUP;
	gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF4_USART1;
	HAL_GPIO_Init(GPIOA, &gpio);

	RCC->APBENR2 |= RCC_APBENR2_USART1EN;
	(void)RCC->APBENR2;                 // ensure the clock is up before use

	DBG_USART->CR1 = 0u;               // disable while configuring (8N1)
	DBG_USART->BRR = (DBG_USART_CLK_HZ + DBG_BAUD / 2u) / DBG_BAUD; // OVER16
	DBG_USART->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

	// Unbuffered stdout so each printf() flushes to the wire immediately.
	setvbuf(stdout, NULL, _IONBF, 0);
}

int DebugUART_PutChar(int ch) {
	while ((DBG_USART->ISR & USART_ISR_TXE_TXFNF) == 0u) {
		// wait for TX data register / FIFO to accept a byte
	}
	DBG_USART->TDR = (uint8_t)ch;
	return ch;
}

/* newlib calls this (via _write in syscalls.c) for printf/puts/etc.
 * Expand '\n' to CRLF so dumb terminals render lines correctly. */
int __io_putchar(int ch) {
	if (ch == '\n') {
		(void)DebugUART_PutChar('\r');
	}
	return DebugUART_PutChar(ch);
}
