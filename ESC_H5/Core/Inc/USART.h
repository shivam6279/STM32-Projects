#ifndef _USART_H_
#define _USART_H_

#include <main.h>

#define RX_BUFFER_SIZE 64

extern UART_HandleTypeDef huart1;

extern char serial_buffer[64];
extern uint8_t serial_buffer_len;

extern volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
extern volatile unsigned char rx_rdy;
extern volatile unsigned char play_tone;
extern volatile unsigned char auto_stop;

extern void set_serial_mode(uint8_t mode);
extern void send_serial(char [], uint8_t);
extern void CAN_send_serial(char []) 
extern void MX_USART1_UART_Init(void);

#endif
