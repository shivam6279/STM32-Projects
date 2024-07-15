#include "UART.h"
#include "main.h"

void UART_init(float freq) {
    RCC->APB1ENR1 |= 1 << 17;

    GPIOB->AFR[0] |= 0b0111 << 12;
    GPIOB->MODER &= ~(1<<6);
    GPIOB->MODER |= 1<<7;

    GPIOB->AFR[0] |= 0b0111 << 16;
    GPIOB->MODER &= ~(1<<8);
    GPIOB->MODER |= 1<<9;

    USART2->CR1 &= ~(1);
    USART2->CR1 = 0x00000000;
    USART2->CR1 |= 1;
    USART2->CR2 = 0x00000000;
    USART2->CR3 = 0x00000000;

    float f = 170000000.0 / freq;
	USART2->BRR = (uint32_t)f;
    USART2->PRESC &= ~(0b1111);

    //Enable FIFO
    USART2->CR3 |= 1<<12;
//	USART2->CR1 |= 1<<29;

    //Enable UART
    USART2->CR1 |= 1<<2;
    USART2->CR1 |= 1<<3;
}

void UART_send(uint8_t ch) {
    while((USART2->ISR & (1 << 7)) == 0);
	USART2->TDR = ch;
}

void UART_send_str(char str[]) {
    int i;
    for(i = 0; str[i] != '\0'; i++) {
        UART_send(str[i]);
    }
}

void UART_write_int(int64_t a) {
    long int tens;

    if(a < 0) {
        a *= (-1);
        UART_send('-');
    }
    //else UART_send('+');

    for(tens = 1; tens <= a; tens *= 10);
    if(a != 0) tens /= 10;
    for(; tens > 0; tens /= 10) {
        UART_send(((a / tens) % 10) + 48);
    }
}

void UART_write_float(double a, unsigned char right) {
    unsigned char i;
    long int tens;

    if(a < 0) {
        a *= (-1);
        UART_send('-');
    } 
    //else UART_send('+');
    
    if(a >= 1.0) {
        for(tens = 1; tens <= a; tens *= 10);
        tens /= 10;
        for(; tens > 0; tens /= 10)
            UART_send(((long int)(a / tens) % 10) + 48);
    } else {
        UART_send('0');
    }

    UART_send('.');
    for(i = 0, tens = 10; i < right; i++, tens *= 10) {
        UART_send(((long int)(a * tens) % 10) + 48);
    }
}

uint8_t UART_rx_ready() {
	return (USART2->ISR & (1 << 5)) == (1 << 5);
}

uint8_t UART_receive() {
	uint8_t r;
	r = USART2->RDR;
	return r;
}
