/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rgb_pwm.h
  * @brief   Software (timer-driven) PWM for an RGB LED + hue-cycling helper.
  *
  *  Channel mapping (STM32C011F6Ux, UFQFPN20):
  *      R = PA12   (pad defaults to PA12 -- no SYSCFG remap required)
  *      G = PA13   (NOTE: shared with SWDIO -- see README note below)
  *      B = PA14   (NOTE: shared with SWCLK / BOOT0)
  *
  *  PWM is generated in software from the TIM14 update interrupt:
  *  RGB_PWM_STEPS levels refreshed RGB_PWM_REFRESH_HZ times per second.
  *
  *  WARNING: configuring PA13/PA14 as GPIO disables the SWD debug port.
  *  After flashing, attach with "connect under reset" (STM32CubeProgrammer)
  *  or use the system bootloader to reprogram.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef RGB_PWM_H
#define RGB_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- Configuration -------------------------------------------------------- */
/* 0 = common-cathode LED (drive pin HIGH to light).
 * 1 = common-anode  LED (drive pin LOW  to light).
 * If your colours come out inverted, flip this. */
#ifndef RGB_ACTIVE_LOW
#define RGB_ACTIVE_LOW        0u
#endif

#define RGB_PWM_STEPS         256u        /* 8-bit duty resolution per channel */
#define RGB_PWM_REFRESH_HZ    125u        /* full-frame rate (flicker-free)    */
#define RGB_TIMCLK_HZ         48000000u   /* TIM14 kernel clock (APB timer clk)*/

/* ---- API ------------------------------------------------------------------ */
void RGB_PWM_Init(void);                            /* GPIO + TIM14 + NVIC     */
void RGB_SetRGB(uint8_t r, uint8_t g, uint8_t b);   /* per-channel duty 0..255 */
void RGB_SetHSV(uint16_t h, uint8_t s, uint8_t v);  /* h 0..359, s/v 0..255    */
void RGB_HueCycle(uint16_t step_ms);                /* non-blocking rainbow    */

#ifdef __cplusplus
}
#endif
#endif /* RGB_PWM_H */
