// Host test for pwm_profile.c: strike / hold on the coil outputs. The timer side (pwm_hw.h) and
// Millis are stand-ins, PwmTask is called every simulated ms like the main loop.
//
// Build/run: make -C test/host pwm
#include <math.h>
#include <stdio.h>

#include "pwm.h"
#include "pwm_hw.h"

static int failures;

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		if (failures++ < 20) { \
			printf("FAIL %s:%d: ", __FILE__, __LINE__); \
			printf(__VA_ARGS__); \
			printf("\n"); \
		} \
	} \
} while (0)

// ---- stand-ins for the hardware ----
static float duty[PWM_CHANNELS];
static uint32_t now_ms = 1000;

void PwmHwSet(uint8_t ch, float d) { duty[ch] = d; }
float PwmHwGet(uint8_t ch) { return duty[ch]; }
void PwmHwAllOff() { for (int i = 0; i < PWM_CHANNELS; i++) duty[i] = 0; }
uint32_t Millis() { return now_ms; }

static void Run(uint32_t ms) {
	while (ms--) {
		now_ms++;
		PwmTask();
	}
}

static bool Near(float a, float b) {
	return fabsf(a - b) < 1e-4f;
}

static void Reset() {
	PwmAllOff();
	pwm_profiles[1] = (PwmProfile){ 0.5f, 100, 0.25f };
	pwm_profiles[2] = (PwmProfile){ 0.5f, 100, 0.25f };
	pwm_profiles[3] = (PwmProfile){ 0, 0, 0 };
}

// Valves: 50% for 100ms, then 25%
static void TestStrikeHold() {
	Reset();
	PwmOn(1);
	CHECK(Near(duty[1], 0.5f) && PwmState(1) == PWM_STRIKE, "on: strike 50%%, got %.3f state %u", duty[1], PwmState(1));
	Run(99);
	CHECK(Near(duty[1], 0.5f) && PwmState(1) == PWM_STRIKE, "99ms: still strike, got %.3f", duty[1]);
	Run(1);
	CHECK(Near(duty[1], 0.25f) && PwmState(1) == PWM_HOLD, "100ms: hold 25%%, got %.3f state %u", duty[1], PwmState(1));
	Run(5000);
	CHECK(Near(duty[1], 0.25f) && PwmState(1) == PWM_HOLD, "5s on: still hold, got %.3f", duty[1]);
	PwmOff(1);
	CHECK(duty[1] == 0 && PwmState(1) == PWM_OFF, "off, got %.3f", duty[1]);
}

// The pad calls PwmOn on every button change: a valve already on doesn't strike again
static void TestNoRestrike() {
	Reset();
	PwmOn(1);
	Run(50);
	PwmOn(1);
	Run(50);
	CHECK(PwmState(1) == PWM_HOLD, "on again mid strike: hold still at 100ms, state %u", PwmState(1));
	PwmOn(1);
	CHECK(Near(duty[1], 0.25f) && PwmState(1) == PWM_HOLD, "on again while holding: stays at hold, got %.3f", duty[1]);
}

// Off then on again: a fresh strike
static void TestRestrikeAfterOff() {
	Reset();
	PwmOn(2);
	Run(200);
	PwmOff(2);
	Run(10);
	PwmOn(2);
	CHECK(Near(duty[2], 0.5f) && PwmState(2) == PWM_STRIKE, "on after off: strike, got %.3f", duty[2]);
	Run(100);
	CHECK(Near(duty[2], 0.25f), "then hold, got %.3f", duty[2]);
}

// Off during the strike: off for good, the task doesn't bring it back at hold
static void TestOffMidStrike() {
	Reset();
	PwmOn(1);
	Run(30);
	PwmOff(1);
	Run(200);
	CHECK(duty[1] == 0 && PwmState(1) == PWM_OFF, "off mid strike stays off, got %.3f", duty[1]);
}

// E-stop (M112) during the strike
static void TestAllOff() {
	Reset();
	PwmOn(1);
	PwmOn(2);
	Run(30);
	PwmAllOff();
	Run(200);
	CHECK(duty[1] == 0 && duty[2] == 0, "all off mid strike stays off, got %.3f %.3f", duty[1], duty[2]);
}

// A plain duty (diags pwm <ch> <duty>) ends the profile
static void TestSetOverrides() {
	Reset();
	PwmOn(1);
	Run(30);
	PwmSet(1, 0.8f);
	Run(200);
	CHECK(Near(duty[1], 0.8f) && PwmState(1) == PWM_DUTY, "PwmSet mid strike wins, got %.3f state %u", duty[1], PwmState(1));
}

// No profile: on = 100%. Hold only (strike_ms 0): straight to hold
static void TestNoProfile() {
	Reset();
	CHECK(!PwmProfiled(3) && PwmProfiled(1), "profiled flags");
	PwmOn(3);
	CHECK(Near(duty[3], 1) && PwmState(3) == PWM_DUTY, "no profile: 100%%, got %.3f", duty[3]);
	pwm_profiles[3] = (PwmProfile){ 0, 0, 0.4f };
	PwmOff(3);
	PwmOn(3);
	CHECK(Near(duty[3], 0.4f) && PwmState(3) == PWM_HOLD, "hold only: 40%% at once, got %.3f", duty[3]);
}

// Millis wraps after 49 days: the strike still ends after strike_ms
static void TestWrap() {
	Reset();
	now_ms = 0xFFFFFFFF - 40;
	PwmOn(1);
	Run(99);
	CHECK(PwmState(1) == PWM_STRIKE, "across the wrap: still strike at 99ms");
	Run(1);
	CHECK(Near(duty[1], 0.25f), "across the wrap: hold at 100ms, got %.3f", duty[1]);
}

int main() {
	TestStrikeHold();
	TestNoRestrike();
	TestRestrikeAfterOff();
	TestOffMidStrike();
	TestAllOff();
	TestSetOverrides();
	TestNoProfile();
	TestWrap();
	if (failures) {
		printf("%d failures\n", failures);
		return 1;
	}
	printf("pwm: all passed\n");
	return 0;
}
