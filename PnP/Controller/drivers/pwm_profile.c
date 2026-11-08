#include "pwm.h"
#include "pwm_hw.h"
#include "clock.h"

// Channels 1 + 2 are the solenoid valves (pad.h PAD_PWM_VALVE0/1): 50% for 100ms to pull in,
// 25% to hold. The rest have no profile until one is set (M790, pwm profile)
PwmProfile pwm_profiles[PWM_CHANNELS] = {
	[1] = { 0.5f, 100, 0.25f },
	[2] = { 0.5f, 100, 0.25f },
};

static uint8_t state[PWM_CHANNELS];
static uint32_t strike_start[PWM_CHANNELS];
static uint32_t changed_ms;	// last time any output changed (vacuum probe blanking)

static float Clamp(float d) {
	if (d < 0) return 0;
	if (d > 1) return 1;
	return d;
}

void PwmSet(uint8_t ch, float duty) {
	if (ch >= PWM_CHANNELS) return;
	duty = Clamp(duty);
	if (duty != PwmHwGet(ch)) changed_ms = Millis();
	state[ch] = duty > 0 ? PWM_DUTY : PWM_OFF;
	PwmHwSet(ch, duty);
}

float PwmGet(uint8_t ch) {
	return PwmHwGet(ch);
}

bool PwmProfiled(uint8_t ch) {
	if (ch >= PWM_CHANNELS) return false;
	return pwm_profiles[ch].strike_ms > 0 || pwm_profiles[ch].hold > 0;
}

void PwmOn(uint8_t ch) {
	PwmProfile *p;

	if (ch >= PWM_CHANNELS) return;
	p = &pwm_profiles[ch];
	if (!PwmProfiled(ch)) {
		PwmSet(ch, 1);
		return;
	}
	// Already pulled in: a second on doesn't strike again
	if (state[ch] == PWM_STRIKE || state[ch] == PWM_HOLD) return;
	changed_ms = Millis();
	if (p->strike_ms == 0) {
		state[ch] = PWM_HOLD;
		PwmHwSet(ch, Clamp(p->hold));
		return;
	}
	state[ch] = PWM_STRIKE;
	strike_start[ch] = Millis();
	PwmHwSet(ch, Clamp(p->strike));
}

void PwmOff(uint8_t ch) {
	PwmSet(ch, 0);
}

void PwmAllOff() {
	uint8_t i;
	for (i = 0; i < PWM_CHANNELS; i++) state[i] = PWM_OFF;
	PwmHwAllOff();
	changed_ms = Millis();
}

uint32_t PwmLastChange() {
	return changed_ms;
}

void PwmTask() {
	uint32_t now = Millis();
	uint8_t i;

	for (i = 0; i < PWM_CHANNELS; i++) {
		if (state[i] != PWM_STRIKE || now - strike_start[i] < pwm_profiles[i].strike_ms) continue;
		state[i] = PWM_HOLD;
		PwmHwSet(i, Clamp(pwm_profiles[i].hold));
	}
}

uint8_t PwmState(uint8_t ch) {
	if (ch >= PWM_CHANNELS) return PWM_OFF;
	return state[ch];
}
