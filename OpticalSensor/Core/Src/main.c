#include "main.h"
#include "stm32.h"
#include "UART.h"
#include <math.h>

int main(void) {
	HAL_Init();

	SystemClock_Config();

	GPIO_init();

	TIM1_init();	//PWM timer - 24/48/96 khz
	TIM6_init();	//Delay timer - 1 khz
	TIM3_init();	//FOC timer - 25 khz
	TIM4_init();	//Audio timer
	TIM2_init();

	TIM8_init();
//	UART_init(115200);
}
