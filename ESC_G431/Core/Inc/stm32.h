#ifndef __STM32_H
#define __STM32_H

#include "stm32g4xx_hal.h"

extern void GPIO_init();

extern void TIM1_init();

extern void TIM2_init();
extern void TIM3_init();
extern void delay_ms(uint16_t);
extern void TIM8_init();

extern void TIM1_on();
extern void TIM2_on();
extern void TIM3_on();
extern void TIM4_on();
extern void TIM6_on();
extern void TIM7_on();
extern void TIM8_on();

extern void TIM1_off();
extern void TIM2_off();
extern void TIM3_off();
extern void TIM4_off();
extern void TIM6_off();
extern void TIM7_off();
extern void TIM8_off();

void SystemClock_Config(void);
extern void MX_COMP1_Init(void);
extern void MX_COMP2_Init(void);
extern void MX_ADC1_Init(void);
extern void MX_ADC2_Init(void);

extern void Error_Handler();
extern void assert_failed(uint8_t, uint32_t);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

#endif
