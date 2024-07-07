#include "main.h"
#include "stm32.h"
#include "UART.h"
#include "I2C.h"
#include "SPI.h"
#include <math.h>

int main(void) {
	HAL_Init();

	SystemClock_Config();

	GPIO_init();

	GPIOF->MODER &= ~(1 << 1);
	GPIOF->MODER &= ~1;

	GPIOF->MODER &= ~(1 << 3);
	GPIOF->MODER |= 1 << 2;

//	GPIOF->BSRR |= 1 << 17;
	GPIOF->BSRR |= 1 << 1;

	TIM6_init();

	UART_init(115200);
	I2C_init(0);
	SPI_init();

	GPIOF->BSRR |= 1 << 17;
	delay_ms(25);
	GPIOF->BSRR |= 1 << 1;

	delay_ms(100);

	PMW3901_init();

	delay_ms(100);

	int16_t delta_x, delta_y;
	uint8_t squal, shutter_u;
	while(1) {
		PMW3901_read_deltas(&delta_x, &delta_y, &squal, &shutter_u);
		UART_send_str("x: ");
		UART_write_int(delta_x);
		UART_send_str("\ty: ");
		UART_write_int(delta_y);
		UART_send_str("\tQ: ");
		UART_write_int(squal);
		UART_send_str("\tS: ");
		UART_write_int(shutter_u);
		UART_send_str("\n");

//		UART_write_int(read_reg(0x00));
//		UART_send_str("\n");

		delay_ms(100);
	}
}
