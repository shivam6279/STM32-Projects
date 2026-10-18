#include "uart.h"
#include "clock.h"
#include "irq_prio.h"
#include "stm32h7xx.h"

#define TX_LEN 2048	// power of 2
#define RX_LEN 256	// power of 2
#define WRITE_TIMEOUT_MS 50

static char tx_buf[TX_LEN];
static volatile uint32_t tx_head, tx_tail;
static char rx_buf[RX_LEN];
static volatile uint32_t rx_head, rx_tail;

void UartInit() {
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	(void)RCC->APB2ENR;

	// Kernel clock = PCLK2 (USART16SEL reset value), 8N1, oversampling 16, no FIFO
	USART1->CR1 = 0;
	USART1->PRESC = 0;
	USART1->BRR = (CLOCK_PCLK2_HZ + UART_BAUD / 2) / UART_BAUD;
	USART1->CR2 = 0;
	USART1->CR3 = 0;
	USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;

	NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_UART);
	NVIC_EnableIRQ(USART1_IRQn);
}

uint32_t UartWrite(const char *buf, uint32_t len) {
	uint32_t i, t = Millis();

	for (i = 0; i < len; i++) {
		// Wait for room, give up if the UART is stuck
		while (tx_head - tx_tail >= TX_LEN) {
			if (Millis() - t > WRITE_TIMEOUT_MS) return i;
		}
		tx_buf[tx_head & (TX_LEN - 1)] = buf[i];
		tx_head++;
		USART1->CR1 |= USART_CR1_TXEIE_TXFNFIE;
	}
	return len;
}

int UartReadChar() {
	char c;
	if (rx_head == rx_tail) return -1;
	c = rx_buf[rx_tail & (RX_LEN - 1)];
	rx_tail++;
	return c;
}

void USART1_IRQHandler() {
	uint32_t isr = USART1->ISR;

	if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
		USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
	}

	if (isr & USART_ISR_RXNE_RXFNE) {
		char c = USART1->RDR;
		if (rx_head - rx_tail < RX_LEN) { // drop if full
			rx_buf[rx_head & (RX_LEN - 1)] = c;
			rx_head++;
		}
	}

	if ((USART1->CR1 & USART_CR1_TXEIE_TXFNFIE) && (isr & USART_ISR_TXE_TXFNF)) {
		if (tx_head != tx_tail) {
			USART1->TDR = tx_buf[tx_tail & (TX_LEN - 1)];
			tx_tail++;
		} else {
			USART1->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
		}
	}
}
