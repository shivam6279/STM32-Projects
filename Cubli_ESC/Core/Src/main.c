#include "main.h"
#include "stm32.h"
#include "UART.h"
#include "tones.h"
#include "BLDC.h"
#include <math.h>

//void COMP1_2_3_IRQHandler(void) {
//	if((EXTI->PR1 >> 21) & 1) {
//		EXTI->PR1 |= 1 << 21;
//	} else if((EXTI->PR1 >> 22) & 1) {
//		EXTI->PR1 |= 1 << 22;
//	}
//}

int main(void) {
	HAL_Init();

	SystemClock_Config();

	GPIO_init();

	TIM1_init();	//PWM timer - 24/48/96 khz
	TIM6_init();	//Delay timer - 1 khz
	TIM3_init();	//FOC timer - 25 khz
	TIM4_init();	//Audio timer
	TIM2_init();

	COMP_init();
	MX_ADC1_Init();
	MX_ADC2_Init();

	TIM8_init();	//Encoder
	UART_init(115200);

//	TIM3_on();

	play_music();
	delay_ms(500);

	TIM2_on();
	motor_mode = 0;

	TIM3_on();
	motor_on();
	power = 0.2;
	motor_mode = 1;
//	setPhaseVoltage(0.075, 0);

	/*motor_off();
	UART_send_str("Test\n");
	while(1) {
		if(UART_rx_ready()) {
			UART_send(UART_receive());
		}
		UART_write_int(USART2->ISR);
		UART_send_str("\n");
		delay_ms(100);
	}*/
	while(1) {
		UART_write_float(rpm, 2);
		UART_send('\n');
		delay_ms(50);
		if(power < 0.97) {
			power += 0.003;
		} else {
			break;
		}
	}
	delay_ms(1000);
	motor_off();
	while(1);
	delay_ms(500);
	while(1) {
		UART_write_float(rpm, 2);
		UART_send('\n');
		delay_ms(50);
		if(power > 0.2) {
			power -= 0.003;
		}
	}
}
