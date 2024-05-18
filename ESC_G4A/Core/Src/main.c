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
//	UART_init(115200);

//	TIM3_on();

	play_music();
	delay_ms(500);

	TIM2_on();

	motor_on();

//	float ramp_seconds = 4, total_seconds = 7, interval_ms = 20;
//	float power_start = 0.05, power_final = 0.5;
//
//	power = power_start;
//	motor_mode = 1;
//	i = 0;
//	j = 0;
//	while(1) {
//		UART_write_float(rpm, 2);
//		UART_send_str("\n");
//
//		if(power < power_final){
//			power  += (power_final - power_start) / ramp_seconds * interval_ms / 1000;
//		}
//
//		if(j > (total_seconds/interval_ms*1000)) {
//			motor_mode = 0;
//			power = 0;
//			motor_off();
//			break;
//		}
//
//		delay_ms(interval_ms);
//		i++;
//		j++;
//	}
//
//	delay_ms(1000);
//	power_start *= -1;
//	power_final *= -1;
//	power = power_start;
//	motor_on();
//	motor_mode = 1;
//	i = 0;
//	j = 0;
//	while(1) {
//		UART_write_float(rpm, 2);
//		UART_send_str("\n");
//
//		if(power > power_final){
//			power += (power_final - power_start) / ramp_seconds * interval_ms / 1000;
//		}
//
//		if(j > (total_seconds/interval_ms*1000)) {
//			motor_mode = 0;
//			power = 0;
//			motor_off();
//			while(1);
//		}
//
//		delay_ms(interval_ms);
//		i++;
//		j++;
//	}
	TIM1->CCR4 = 0;
	TIM1->DIER |= 1 << 4;
	float p = 0.1;

	uint16_t k = 0;

	while(1) {
//		setPhaseVoltage(0.05, (current_phase - 1)*60);
		BLDC_phase(current_phase, p);
		BEMF_phase(current_phase);

		GPIOB->BSRR |= 1 << 19;

//		TIM2->CR1 |= 1;
		TIM2->CNT = 0;
//		TIM2->DIER &= ~(1 << 4);
//
		TIM1->DIER |= 1 << 4;

		delay_us(2000);

		current_phase++;
		current_phase = (current_phase - 1) % 6 + 1;

		if(k++ > 3000) {
			break;
		}
	}

	while(1) {
		BLDC_phase(current_phase, p);
		BEMF_phase(current_phase);

		GPIOB->BSRR |= 1 << 19;
//		TIM2->CR1 |= 1;
		TIM2->CNT = 0;
//		TIM2->DIER &= ~(1 << 4);

		zero_crossing_flag = 0;
		TIM1->DIER |= 1 << 4;

		while(zero_crossing_flag == 0);
		while(TIM2->CNT < phase_timing);

		current_phase++;
		current_phase = (current_phase - 1) % 6 + 1;

		if(p < 0.5) {
			p += 0.00001;
		}
	}

//	uint32_t flag;
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
}
