#ifndef _UART_H_
#define _UART_H_

#include <inttypes.h>

// Debug UART: USART1, PB14 TX / PB15 RX (to the programmer header).
// TX and RX both go through ring buffers + interrupts, nothing blocks in the ISR.

#define UART_BAUD 115200

void UartInit();
uint32_t UartWrite(const char *buf, uint32_t len);
int UartReadChar(); // -1 if nothing

#endif
