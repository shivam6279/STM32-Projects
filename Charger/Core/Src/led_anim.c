// Interrupt-driven charger-status LED engine. See led_anim.h.
//
// The main loop only posts a target (mode + hue/val) via the setters; all state
// mutation and brightness computation happens in LED_Frame(), fired once per PWM
// frame from the TIM14 ISR. Because that ISR keeps running while the main loop is
// blocked (e.g. the balancer's HAL_Delay settle), the breath never stutters.
//
// Single-writer discipline: only LED_Frame() touches the s_cur_*/s_shown_*/
// s_start_* state. Setters only post the volatile s_req_* fields (data first,
// mode last so the ISR sees a consistent request).

#include "led_anim.h"
#include "rgb_pwm.h"
#include "main.h"          // HAL_GetTick

typedef enum {
	LED_M_OFF = 0,
	LED_M_BREATHE,
	LED_M_FULL,
	LED_M_FAULT,
	LED_M_STATIC,
	LED_M_FADEOUT,
} led_mode_t;

// ---- Posted by the main loop ----------------------------------------------
static volatile uint8_t  s_req_mode = LED_M_OFF;
static volatile uint16_t s_req_hue;
static volatile uint8_t  s_req_val;
static volatile uint8_t  s_busy;        // 1 while a morph/fade is running

// ---- Owned exclusively by LED_Frame() (ISR context) -----------------------
static led_mode_t s_cur_mode = LED_M_OFF;
static uint32_t   s_breath_base;        // breath phase anchor
static uint32_t   s_xstart_tick;        // morph/fade start tick
static uint16_t   s_start_hue;          // morph/fade start colour...
static uint8_t    s_start_val;          // ...captured from the live LED
static uint16_t   s_shown_hue;          // last colour pushed to the LED
static uint8_t    s_shown_val;

// Linear interpolate a -> b as p advances 0..dur (p clamped by the caller).
static int32_t Lerp(int32_t a, int32_t b, uint32_t p, uint32_t dur) {
	return a + (b - a) * (int32_t)p / (int32_t)dur;
}

// Fired once per PWM frame (~125 Hz) from the TIM14 ISR.
static void LED_Frame(void) {
	uint32_t now = HAL_GetTick();
	uint8_t  req = s_req_mode;

	// Transition entry: capture per-mode start state once when the mode changes.
	if (req != (uint8_t)s_cur_mode) {
		s_busy = 0u;
		switch (req) {
			case LED_M_BREATHE:
				s_breath_base = now;            // restart at the trough
				break;
			case LED_M_FULL:
			case LED_M_FADEOUT:
				s_start_hue   = s_shown_hue;    // ease from whatever is lit now
				s_start_val   = s_shown_val;
				s_xstart_tick = now;
				s_busy        = 1u;
				break;
			default:
				break;
		}
		s_cur_mode = (led_mode_t)req;
	}

	switch (s_cur_mode) {
		case LED_M_OFF:
			s_shown_hue = 0u;
			s_shown_val = 0u;
			break;

		case LED_M_STATIC:
			s_shown_hue = s_req_hue;
			s_shown_val = s_req_val;
			break;

		case LED_M_BREATHE: {
			uint32_t half  = BATT_BREATH_MS / 2u;
			uint32_t phase = (uint32_t)(now - s_breath_base) % BATT_BREATH_MS;
			uint32_t tri   = (phase < half) ? phase : (BATT_BREATH_MS - phase);
			uint16_t hue   = s_req_hue;         // base hue tracked live
			uint16_t drop  = (uint16_t)(BATT_BREATH_HUE_DROP * (half - tri) / half);
			s_shown_val = (uint8_t)(BATT_BREATH_MIN_V
								  + (255u - BATT_BREATH_MIN_V) * tri / half);
			s_shown_hue = (hue > drop) ? (uint16_t)(hue - drop) : 0u;
			break;
		}

		case LED_M_FULL: {
			uint32_t p = now - s_xstart_tick;
			if (p >= BATT_FULL_MORPH_MS) {
				s_shown_hue = 120u;             // solid green, hold
				s_shown_val = 255u;
				s_busy      = 0u;
			} else {
				s_shown_hue = (uint16_t)Lerp(s_start_hue, 120, p, BATT_FULL_MORPH_MS);
				s_shown_val = (uint8_t) Lerp(s_start_val, 255, p, BATT_FULL_MORPH_MS);
			}
			break;
		}

		case LED_M_FADEOUT: {
			uint32_t p = now - s_xstart_tick;
			if (p >= BATT_FADE_MS) {
				s_shown_val = 0u;               // dark, hold (main sleeps next)
				s_busy      = 0u;
			} else {
				s_shown_hue = s_start_hue;
				s_shown_val = (uint8_t)Lerp(s_start_val, 0, p, BATT_FADE_MS);
			}
			break;
		}

		case LED_M_FAULT: {
			uint8_t on = (uint8_t)((now / 250u) & 1u);
			s_shown_hue = 0u;                   // red blink
			s_shown_val = on ? 255u : 0u;
			break;
		}
	}

	if (s_shown_val == 0u) {
		RGB_SetRGB(0u, 0u, 0u);
	} else {
		RGB_SetHSV(s_shown_hue, 255u, s_shown_val);
	}
}

void LED_Init(void) {
	s_req_mode = LED_M_OFF;
	s_cur_mode = LED_M_OFF;
	RGB_SetFrameCallback(LED_Frame);
}

void LED_Off(void) {
	s_req_mode = LED_M_OFF;
}

void LED_Breathe(uint16_t hue) {
	s_req_hue  = hue;
	s_req_mode = LED_M_BREATHE;
}

void LED_Full(void) {
	s_busy     = 1u;            // assert now so LED_Busy() is true before the
	s_req_mode = LED_M_FULL;    // first frame picks up the transition
}

void LED_Fault(void) {
	s_req_mode = LED_M_FAULT;
}

void LED_ShowStatic(uint16_t hue, uint8_t val) {
	s_req_hue  = hue;
	s_req_val  = val;
	s_req_mode = LED_M_STATIC;
}

void LED_FadeOut(void) {
	s_busy     = 1u;            // assert now so the caller's LED_Busy() wait
	s_req_mode = LED_M_FADEOUT; // doesn't fall through before the fade starts
}

uint8_t LED_Busy(void) {
	return s_busy;
}
