// Software PWM RGB LED driver (PA12=R, PA13/14=G/B, shared with SWD).
// Active-low by default (RGB_ACTIVE_LOW).
#ifndef RGB_PWM_H
#define RGB_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ---- Configuration --------------------------------------------------------
/* 0 = common-cathode LED (drive pin HIGH to light).
 * 1 = common-anode  LED (drive pin LOW  to light).
 * If your colours come out inverted, flip this. */
#ifndef RGB_ACTIVE_LOW
#define RGB_ACTIVE_LOW        1u
#endif

#define RGB_PWM_STEPS         256u        // 8-bit duty resolution per channel
#define RGB_PWM_REFRESH_HZ    125u        // full-frame rate (flicker-free)
#define RGB_TIMCLK_HZ         48000000u   // TIM14 kernel clock (APB timer clk)

// ---- API ------------------------------------------------------------------
void RGB_PWM_Init(void);                            // GPIO + TIM14 + NVIC
void RGB_SetRGB(uint8_t r, uint8_t g, uint8_t b);   // per-channel duty 0..255
void RGB_SetHSV(uint16_t h, uint8_t s, uint8_t v);  // h 0..359, s/v 0..255
void RGB_HueCycle(uint16_t step_ms);                // non-blocking rainbow

#ifdef __cplusplus
}
#endif
#endif // RGB_PWM_H
