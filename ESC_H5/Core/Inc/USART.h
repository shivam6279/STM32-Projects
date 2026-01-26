#ifndef _USART_H_
#define _USART_H_

#include <main.h>

#define RX_BUFFER_SIZE 128

extern UART_HandleTypeDef huart1;

extern volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
extern volatile unsigned char rx_rdy;
extern volatile unsigned char play_tone;
extern volatile unsigned char auto_stop;

extern void MX_USART1_UART_Init(void);

#endif
