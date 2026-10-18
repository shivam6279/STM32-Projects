#include "esp.h"
#include "clock.h"
#include "irq_prio.h"
#include "stm32h7xx.h"

#define TX_LEN 4096	// power of 2
#define RX_LEN 4096	// power of 2, ~44ms at 921600

static char tx_buf[TX_LEN];
static volatile uint32_t tx_head, tx_tail;
static char rx_buf[RX_LEN];
static volatile uint32_t rx_head, rx_tail;
static uint32_t baud;
static volatile EspStats stats;

void EspUartInit(uint32_t b) {
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	(void)RCC->APB2ENR;

	// Kernel clock = PCLK2 (USART16910SEL reset value), 8N1, oversampling 16
	USART6->CR1 = 0;
	USART6->PRESC = 0;
	USART6->CR2 = 0;
	USART6->CR3 = 0;
	baud = 0;
	EspBaud(b);

	NVIC_SetPriority(USART6_IRQn, IRQ_PRIO_UART);
	NVIC_EnableIRQ(USART6_IRQn);
}

void EspBaud(uint32_t b) {
	uint32_t t = Millis();

	// BRR is 16 bits, 130MHz / 16 = 8.1M max
	if (b < CLOCK_PCLK2_HZ / 0xFFFF + 1) b = CLOCK_PCLK2_HZ / 0xFFFF + 1;
	if (b > CLOCK_PCLK2_HZ / 16) b = CLOCK_PCLK2_HZ / 16;
	if (b == baud) return;

	// Let whatever's queued go out at the old rate
	while (!EspTxIdle() && (USART6->CR1 & USART_CR1_UE)) {
		if (Millis() - t > 20) break;
	}

	USART6->CR1 = 0;
	USART6->BRR = (CLOCK_PCLK2_HZ + b / 2) / b;
	USART6->CR1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;
	if (tx_head != tx_tail) USART6->CR1 |= USART_CR1_TXEIE_TXFNFIE;
	baud = b;
}

uint32_t EspGetBaud() {
	return baud;
}

uint32_t EspTxRoom() {
	return TX_LEN - (tx_head - tx_tail);
}

bool EspTxIdle() {
	return tx_head == tx_tail && (USART6->ISR & USART_ISR_TC);
}

uint32_t EspWrite(const char *buf, uint32_t len) {
	uint32_t i;

	for (i = 0; i < len && tx_head - tx_tail < TX_LEN; i++) {
		tx_buf[tx_head & (TX_LEN - 1)] = buf[i];
		tx_head++;
	}
	if (i) USART6->CR1 |= USART_CR1_TXEIE_TXFNFIE;
	return i;
}

int EspReadChar() {
	char c;
	if (rx_head == rx_tail) return -1;
	c = rx_buf[rx_tail & (RX_LEN - 1)];
	rx_tail++;
	return c;
}

void EspFlushRx() {
	rx_tail = rx_head;
}

void EspGetStats(EspStats *s) {
	*s = stats;
}

void USART6_IRQHandler() {
	uint32_t isr = USART6->ISR;

	if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
		if (isr & USART_ISR_ORE) stats.overrun++;
		if (isr & USART_ISR_FE) stats.frame++;
		USART6->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
	}

	while (USART6->ISR & USART_ISR_RXNE_RXFNE) {
		char c = USART6->RDR;
		stats.rx++;
		if (rx_head - rx_tail < RX_LEN) {
			rx_buf[rx_head & (RX_LEN - 1)] = c;
			rx_head++;
		} else {
			stats.rx_dropped++;
		}
	}

	if (USART6->CR1 & USART_CR1_TXEIE_TXFNFIE) {
		while (tx_head != tx_tail && (USART6->ISR & USART_ISR_TXE_TXFNF)) {
			USART6->TDR = tx_buf[tx_tail & (TX_LEN - 1)];
			tx_tail++;
			stats.tx++;
		}
		if (tx_head == tx_tail) USART6->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
	}
}
