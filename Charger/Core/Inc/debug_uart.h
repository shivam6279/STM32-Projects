// USART1 debug console on PA0(TX)/PA1(RX), 115200 8N1; printf -> UART.
#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void DebugUART_Init(void);        // USART1 + PA0/PA1 + printf retarget
int  DebugUART_PutChar(int ch);   // blocking single-byte transmit

#ifdef __cplusplus
}
#endif
#endif // DEBUG_UART_H
