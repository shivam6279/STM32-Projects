#include "jog_timer.h"
#include "clock.h"
#include "irq_prio.h"
#include "jog.h"
#include "stm32h7xx.h"

// TIM7: 10kHz count, update every JOG_PERIOD us
#define JOG_PSC	(CLOCK_TIM_APB1_HZ / 10000 - 1)
#define JOG_ARR	(JOG_PERIOD / 100 - 1)

void JogTimerInit() {
	RCC->APB1LENR |= RCC_APB1LENR_TIM7EN;
	(void)RCC->APB1LENR;

	TIM7->CR1 = 0;
	TIM7->PSC = JOG_PSC;
	TIM7->ARR = JOG_ARR;
	TIM7->EGR = TIM_EGR_UG; // load PSC
	TIM7->SR = 0;
	TIM7->DIER = TIM_DIER_UIE;
	TIM7->CR1 = TIM_CR1_CEN;

	NVIC_SetPriority(TIM7_IRQn, IRQ_PRIO_JOG);
	NVIC_EnableIRQ(TIM7_IRQn);
}

void TIM7_IRQHandler() {
	TIM7->SR = 0;
	JogTick();
}
