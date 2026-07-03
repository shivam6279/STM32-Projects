// Interrupt-driven charger-status LED engine. All motion (breath, morph-to-full,
// pre-sleep fade, fault blink, static display) is computed once per PWM frame
// from the TIM14 frame callback, so it keeps advancing even while the main loop
// is blocked (e.g. the balancer's pause-measure HAL_Delays). The main loop only
// sets the target via the setters below; it never computes brightness itself.
#ifndef LED_ANIM_H
#define LED_ANIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ---- Animation tuning -----------------------------------------------------
#define BATT_BREATH_MS       2400u   // breath period
#define BATT_BREATH_MIN_V    24u     // breath trough brightness
#define BATT_BREATH_HUE_DROP 25u     // hue shifted toward red at the breath trough
#define BATT_FULL_MORPH_MS   800u    // morph from current colour -> solid green on full
#define BATT_FADE_MS         700u    // breath-out duration on unplug

// ---- API ------------------------------------------------------------------
// Register the per-frame callback with the PWM driver and start dark. Call once
// after RGB_PWM_Init().
void    LED_Init(void);

void    LED_Off(void);                          // dark
void    LED_Breathe(uint16_t hue);              // charging breath at this base hue
void    LED_Full(void);                         // morph to solid green, then hold
void    LED_Fault(void);                        // red blink
void    LED_ShowStatic(uint16_t hue, uint8_t val); // fixed colour (touch-wake SoC)
void    LED_FadeOut(void);                      // begin pre-sleep fade to dark

// 1 while a one-shot transition (morph or fade) is still running. The main loop
// polls this before standby so the fade is allowed to finish.
uint8_t LED_Busy(void);

#ifdef __cplusplus
}
#endif
#endif // LED_ANIM_H
