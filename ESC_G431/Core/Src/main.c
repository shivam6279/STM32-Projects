#include "main.h"
#include "stm32.h"
#include "UART.h"
#include "BLDC.h"
#include <math.h>

/*void COMP1_2_3_IRQHandler(void) {

	if((EXTI->PR1 >> 21) & 1) {
		EXTI->PR1 |= 1 << 21;

		GPIOB->ODR ^= 1<<3;
	}

	GPIOB->ODR ^= 1<<3;

//	HAL_COMP_IRQHandler(&hcomp1);
//	HAL_COMP_IRQHandler(&hcomp2);
} */

void bemf_rising_U() {

}

int main(void) {
	HAL_Init();

	SystemClock_Config();

	GPIO_init();

//	MX_COMP1_Init();
//	MX_COMP2_Init();

	GPIOA->MODER |= 0b11111111111111;
//	COMP1->CSR = 0;
	COMP1->CSR &= ~(0b11111111 << 16);
//	COMP1->CSR |= (0b1 << 16);
//	COMP1->CSR |= (0b111 << 15);
	COMP1->CSR &= ~(1 << 8);
	COMP1->CSR |= (0b111 << 4);
	COMP1->CSR |= 0b1;

	TIM1_init();
	TIM2_init();
	TIM3_init();

	MX_ADC1_Init();
	MX_ADC2_Init();

	TIM8_init();
	UART_init(115200);

	TIM3_on();

//	setPhaseVoltage(0.05, 0.0);

	uint8_t i = 0, j = 0;

	delay_ms(5000);
	power = 0.1;
	motor_mode = 1;

	while(1) {
//		UART_write_float(position, 2);
//		UART_send_str(", ");
		UART_write_float(position, 2);
		UART_send_str("\n");
		delay_ms(10);
		if(power < 0.3){
			power  += 0.001;
		}
	}

//	uint8_t i = 0, j = 0;
//	while(1) {
//		setPhaseVoltage(0.1, i*60);
//
//		delay_ms(200);
//
//		UART_write_float(position, 2);
//		UART_send_str(", ");
//		UART_write_float(fmod(position * 7, 360.0), 2);
//		UART_send_str("\n");
//
//		delay_ms(200);
//
//		setPhaseVoltage(0.05, i*60);
//		if(++i > 5) {
//			i = 0;
//		}
//	}

	uint32_t flag;

//	int i;
//	float p = 100;
//	while(1) {
//		for(i = 1; i <= 6; i++) {
//			BLDC_phase(i, p);
//			__HAL_TIM_SET_COUNTER(&htim2,0);
//			while (__HAL_TIM_GET_COUNTER(&htim2) < 6000) {
////				flag = (COMP1->CSR >> 30) & 1;
//				flag = COMP1->CSR & (1 << 30);
//				if(flag) {
//					flag = 1;
//					GPIOB->BSRR |= 1 << 3;
//				} else {
//					flag = 0;
//					GPIOB->BSRR |= 1 << 19;
//				}
//			}
//		}
//	}

	TIM1->CCER |= TIM_CCER_CC1NP;
	TIM1->CCER |= TIM_CCER_CC2NP;
	TIM1->CCER |= TIM_CCER_CC3NP;

	while (1) {
		flag = COMP1->CSR & (1 << 30);
		if(flag) {
			flag = 1;
			GPIOB->BSRR |= 1 << 3;
		} else {
			flag = 0;
			GPIOB->BSRR |= 1 << 19;
		}
	}

}
