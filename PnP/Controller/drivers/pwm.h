#ifndef _PWM_H_
#define _PWM_H_

#include <stdbool.h>
#include <inttypes.h>

// Coil driver outputs PWM1-8: each drives a logic level N-FET (SSM3K376R) low side switch
// with a flyback diode. Solenoids, relays, small motors.
// PWM1-8 nets, channel n = PWM(n+1). Which timer/channel each one is comes from the pin sheet.
// Frequency is per timer, so per group, not per channel.
//
// Strike and hold (pwm_profile.c): PwmOn runs the channel's profile, strike duty for strike_ms
// then hold duty (a solenoid needs more to pull in than to stay in). A channel without a profile
// (strike_ms 0, hold 0) goes to 100% on PwmOn. PwmSet is a plain duty that ends any profile.

#define PWM_CHANNELS	8
#define PWM_FREQ		25000	// above audible, small switching loss with direct gate drive

typedef struct {
	float strike;		// duty 0-1 for the first strike_ms after PwmOn
	uint16_t strike_ms;	// 0 = straight to hold
	float hold;			// duty 0-1 after the strike
} PwmProfile;

// Profile state of a channel
#define PWM_OFF		0
#define PWM_DUTY	1	// plain duty, PwmSet
#define PWM_STRIKE	2
#define PWM_HOLD	3

extern PwmProfile pwm_profiles[PWM_CHANNELS];	// saved with the settings

void PwmInit();
void PwmSet(uint8_t ch, float duty);	// 0.0 - 1.0, no profile
float PwmGet(uint8_t ch);				// duty right now
void PwmOn(uint8_t ch);					// strike, then hold
void PwmOff(uint8_t ch);
void PwmAllOff();						// takes effect immediately
void PwmTask();							// main loop: strike -> hold when strike_ms is up
bool PwmProfiled(uint8_t ch);			// has a strike / hold profile
uint8_t PwmState(uint8_t ch);			// PWM_OFF ...
float PwmMeasure(uint8_t ch);			// duty read back off the pin
uint32_t PwmLastChange();				// Millis() of the last change to any output (on, off, duty)

#endif
