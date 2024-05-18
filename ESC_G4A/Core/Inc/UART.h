#ifndef __UART_H
#define __UART_H

#include "main.h"

extern void UART_init(float);
extern void UART_send(uint8_t);
extern void UART_send_str(char[]);
extern void UART_write_int(int64_t);
extern void UART_write_float(double, uint8_t);
extern uint8_t UART_receive(void);

#endif
