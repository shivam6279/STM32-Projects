// Software (TIM14-driven) PWM for an active-low RGB LED + HSV helpers.

#include "rgb_pwm.h"
#include "main.h"          // -> stm32c0xx_hal.h -> CMSIS device + HAL

// ---- Pin mapping (all on GPIOA) -------------------------------------------
#define RGB_PORT      GPIOA
#define RGB_R_BIT     (1u << 12)   // PA12
#define RGB_G_BIT     (1u << 13)   // PA13 / SWDIO
#define RGB_B_BIT     (1u << 14)   // PA14 / SWCLK/BOOT0

// BSRR: low half SETs a pin (drives high), high half RESETs it (drives low).
#if RGB_ACTIVE_LOW
#define RGB_ON(bit)   ((uint32_t)(bit) << 16)   // lit  -> drive low
#define RGB_OFF(bit)  ((uint32_t)(bit))         // dark -> drive high
#else
#define RGB_ON(bit)   ((uint32_t)(bit))         // lit  -> drive high
#define RGB_OFF(bit)  ((uint32_t)(bit) << 16)   // dark -> drive low
#endif

static volatile uint8_t s_duty[3];   // R, G, B target duty (0..255)
static volatile uint8_t s_tick;      // free-running PWM step (0..255)

/* ---------------------------------------------------------------------------
 * Software-PWM tick. Fires at RGB_PWM_STEPS * RGB_PWM_REFRESH_HZ (= 32 kHz).
 * Kept short: one compare + one atomic BSRR write per channel.
 *
 * NOTE: TIM14 is configured here, not in CubeMX. If you later enable TIM14 in
 * the .ioc, delete one of the two TIM14_IRQHandler definitions.
 * ------------------------------------------------------------------------- */
void TIM14_IRQHandler(void) {
	TIM14->SR = (uint16_t)~TIM_SR_UIF;          // clear update flag

	uint8_t  c    = s_tick;
	uint32_t bsrr = 0u;

	bsrr |= (c < s_duty[0]) ? RGB_ON(RGB_R_BIT) : RGB_OFF(RGB_R_BIT);
	bsrr |= (c < s_duty[1]) ? RGB_ON(RGB_G_BIT) : RGB_OFF(RGB_G_BIT);
	bsrr |= (c < s_duty[2]) ? RGB_ON(RGB_B_BIT) : RGB_OFF(RGB_B_BIT);
	RGB_PORT->BSRR = bsrr;

	s_tick = (uint8_t)(c + 1u);                 // wraps 255 -> 0
}

void RGB_PWM_Init(void) {
	GPIO_InitTypeDef gpio = {0};

	/* PA12/PA13/PA14 as push-pull outputs. MX_GPIO_Init() already does this;
	 * repeated so the driver is self-contained. */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
	gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio.Pull  = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RGB_PORT, &gpio);

	/* Drive all channels to the OFF level at once. The pins default to low when
	 * switched to output, which for active-low LEDs is ON -- this avoids a brief
	 * all-on flash before the first PWM tick. */
	RGB_PORT->BSRR = RGB_OFF(RGB_R_BIT) | RGB_OFF(RGB_G_BIT) | RGB_OFF(RGB_B_BIT);

	RGB_SetRGB(0, 0, 0);                         // start dark

	// TIM14 update event at RGB_PWM_STEPS * RGB_PWM_REFRESH_HZ.
	__HAL_RCC_TIM14_CLK_ENABLE();
	TIM14->PSC  = 0u;
	TIM14->ARR  = (RGB_TIMCLK_HZ / (RGB_PWM_STEPS * RGB_PWM_REFRESH_HZ)) - 1u;
	TIM14->CNT  = 0u;
	TIM14->EGR  = TIM_EGR_UG;                    // latch PSC/ARR
	TIM14->SR   = (uint16_t)~TIM_SR_UIF;         // drop the flag UG just set
	TIM14->DIER = TIM_DIER_UIE;
	TIM14->CR1 |= TIM_CR1_CEN;

	NVIC_SetPriority(TIM14_IRQn, 1);
	NVIC_EnableIRQ(TIM14_IRQn);
}

void RGB_SetRGB(uint8_t r, uint8_t g, uint8_t b) {
	// 8-bit stores are atomic on Cortex-M0+; the ISR reads each independently.
	s_duty[0] = r;
	s_duty[1] = g;
	s_duty[2] = b;
}

// Integer HSV -> RGB. h: 0..359, s/v: 0..255.
void RGB_SetHSV(uint16_t h, uint8_t s, uint8_t v) {
	if (h >= 360u) {
		h %= 360u;
	}
	if (s == 0u) {                              // achromatic
		RGB_SetRGB(v, v, v);
		return;
	}

	uint8_t  region = (uint8_t)(h / 60u);                   // 0..5
	uint16_t frac   = (uint16_t)((h % 60u) * 255u / 60u);   // 0..255 within sector

	uint8_t p = (uint8_t)((uint16_t)v * (255u - s) / 255u);
	uint8_t q = (uint8_t)((uint16_t)v * (255u - (uint16_t)s * frac / 255u) / 255u);
	uint8_t t = (uint8_t)((uint16_t)v * (255u - (uint16_t)s * (255u - frac) / 255u) / 255u);

	switch (region) {
		case 0:  RGB_SetRGB(v, t, p); break;
		case 1:  RGB_SetRGB(q, v, p); break;
		case 2:  RGB_SetRGB(p, v, t); break;
		case 3:  RGB_SetRGB(p, q, v); break;
		case 4:  RGB_SetRGB(t, p, v); break;
		default: RGB_SetRGB(v, p, q); break;    // region 5
	}
}

/* Non-blocking: call freely from the main loop. Advances the hue by 1 degree
 * every step_ms at full saturation and full brightness. */
void RGB_HueCycle(uint16_t step_ms) {
	static uint32_t last = 0u;
	static uint16_t hue  = 0u;

	uint32_t now = HAL_GetTick();
	if ((uint32_t)(now - last) >= step_ms) {
		last = now;
		RGB_SetHSV(hue, 255u, 255u);
		if (++hue >= 360u) {
			hue = 0u;
		}
	}
}
