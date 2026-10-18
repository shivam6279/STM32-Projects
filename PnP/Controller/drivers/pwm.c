#include "pwm.h"
#include "board.h"
#include "clock.h"
#include "stm32h7xx.h"

#define PWM_ARR (CLOCK_TIM_APB1_HZ / PWM_FREQ - 1)	// 10399 at 260MHz / 25kHz

_Static_assert(CLOCK_TIM_APB1_HZ % PWM_FREQ == 0, "PWM_FREQ doesn't divide the timer clock");
_Static_assert(PWM_ARR <= 0xFFFF, "TIM3 is 16 bit");

typedef struct {
	TIM_TypeDef *tim;
	uint8_t ch;				// 1-4
	GPIO_TypeDef *port;
	uint8_t pin;
	uint8_t af;
} pwm_channel;

// Channel n = net PWM(n+1). Timer, channel and AF all come from the pin sheet (pins.h).
#define CH(n) { PIN_PWM##n##_TIM, PIN_PWM##n##_TIMCH, PIN_PWM##n##_PORT, PIN_PWM##n##_NUM, PIN_PWM##n##_AF }
static const pwm_channel pwm[PWM_CHANNELS] = { CH(1), CH(2), CH(3), CH(4), CH(5), CH(6), CH(7), CH(8) };
#undef CH

static volatile uint32_t *Ccr(const pwm_channel *p) {
	return &p->tim->CCR1 + (p->ch - 1);
}

static void TimerInit(TIM_TypeDef *tim) {
	tim->CR1 = 0;
	tim->PSC = 0;
	tim->ARR = PWM_ARR;
	tim->CR1 = TIM_CR1_ARPE;
}

void PwmInit() {
	uint8_t i;
	const pwm_channel *p;

	RCC->APB1LENR |= RCC_APB1LENR_TIM2EN | RCC_APB1LENR_TIM3EN | RCC_APB1LENR_TIM5EN;
	(void)RCC->APB1LENR;

	// Only TIM2/3/5 are clocked and in the resource table. If the sheet moves a PWM
	// to another timer, stop here instead of driving a dead pin.
	for (i = 0; i < PWM_CHANNELS; i++) {
		if (pwm[i].tim != TIM2 && pwm[i].tim != TIM3 && pwm[i].tim != TIM5) while (1);
	}

	TimerInit(TIM2);
	TimerInit(TIM3);
	TimerInit(TIM5);

	for (i = 0; i < PWM_CHANNELS; i++) {
		p = &pwm[i];
		*Ccr(p) = 0; // starts off

		// PWM mode 1 (OCxM = 110), preload on, active high
		if (p->ch <= 2) {
			uint32_t sh = (p->ch - 1) * 8;
			p->tim->CCMR1 = (p->tim->CCMR1 & ~(0xFF << sh)) | ((TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE) << sh);
		} else {
			uint32_t sh = (p->ch - 3) * 8;
			p->tim->CCMR2 = (p->tim->CCMR2 & ~(0xFF << sh)) | ((TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE) << sh);
		}
		p->tim->CCER |= TIM_CCER_CC1E << ((p->ch - 1) * 4);
	}

	TIM2->EGR = TIM_EGR_UG;
	TIM3->EGR = TIM_EGR_UG;
	TIM5->EGR = TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM5->CR1 |= TIM_CR1_CEN;

	// Timers are running with 0% duty, now hand the pins over from GPIO low to the timer
	for (i = 0; i < PWM_CHANNELS; i++) {
		p = &pwm[i];
		p->port->AFR[p->pin >> 3] = (p->port->AFR[p->pin >> 3] & ~(0xF << ((p->pin & 7) * 4))) | (p->af << ((p->pin & 7) * 4));
		p->port->MODER = (p->port->MODER & ~(3 << (p->pin * 2))) | (2 << (p->pin * 2));
	}
}

void PwmSet(uint8_t ch, float duty) {
	if (ch >= PWM_CHANNELS) return;
	if (duty < 0) duty = 0;
	if (duty > 1) duty = 1;
	// CCR = ARR + 1 is 100% in PWM mode 1
	*Ccr(&pwm[ch]) = (uint32_t)(duty * (PWM_ARR + 1) + 0.5f);
}

float PwmGet(uint8_t ch) {
	if (ch >= PWM_CHANNELS) return 0;
	return (float)*Ccr(&pwm[ch]) / (PWM_ARR + 1);
}

void PwmAllOff() {
	uint8_t i;
	for (i = 0; i < PWM_CHANNELS; i++) *Ccr(&pwm[i]) = 0;
	// Load the 0s now instead of at the end of the period (CCR preload)
	TIM2->EGR = TIM_EGR_UG;
	TIM3->EGR = TIM_EGR_UG;
	TIM5->EGR = TIM_EGR_UG;
}

float PwmMeasure(uint8_t ch) {
	const pwm_channel *p;
	uint32_t i, high = 0, n = 400000;

	if (ch >= PWM_CHANNELS) return -1;
	p = &pwm[ch];
	// Sample the pin itself, ~250 periods at 25kHz
	for (i = 0; i < n; i++) {
		if (p->port->IDR & (1 << p->pin)) high++;
	}
	return (float)high / n;
}
