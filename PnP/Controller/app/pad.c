#include "pad.h"
#include "board.h"
#include "esp.h"
#include "jog.h"
#include "stepgen.h"
#include "tmc.h"

#include <math.h>
#include <stdio.h>

#define STICK_CENTRE	32767.5f
#define STICK_DEAD		0.12f	// of full stick, they rest a few % off centre
#define TRIG_MAX		1023.0f
#define TRIG_DEAD		0.05f

// TODO check on the machine: stick up reads low (HID convention), so up = Y- = away from you
#define STICK_Y_SIGN	-1.0f

float pad_speed_xy = 75;
float pad_speed_z = 20;

static uint8_t mode = PAD_MODE_OFF;
static bool armed;
static int8_t ax_x = -1, ax_y = -1, ax_z = -1;

static int8_t AxisNamed(char c) {
	uint8_t i;
	for (i = 0; i < MOTION_AXES; i++) {
		if (motion_axes[i].name == c) return i;
	}
	return -1;
}

// Trigger 0-1023 -> 0..1 past the dead zone
static float Trigger(uint16_t raw) {
	float t = raw / TRIG_MAX;
	if (t <= TRIG_DEAD) return 0;
	if (t > 1) t = 1;
	return (t - TRIG_DEAD) / (1 - TRIG_DEAD);
}

// Left stick -> X / Y in -1..1. Round dead zone, then squared so small moves are fine
static void Stick(const EspPad *p, float *x, float *y) {
	float sx = (p->lx - STICK_CENTRE) / STICK_CENTRE;
	float sy = (p->ly - STICK_CENTRE) / STICK_CENTRE * STICK_Y_SIGN;
	float r = sqrtf(sx * sx + sy * sy), k;

	*x = *y = 0;
	if (r <= STICK_DEAD) return;
	k = (r > 1 ? 1 : r);
	k = (k - STICK_DEAD) / (1 - STICK_DEAD);
	k = k * k / r;
	*x = sx * k;
	*y = sy * k;
}

static void Jog() {
	float v[MOTION_AXES] = { 0 }, x, y, z;
	EspPad p;

	if (!EspPadGet(&p)) {
		armed = false; // gone: centre the sticks again before anything moves
		JogSet(v);
		return;
	}
	Stick(&p, &x, &y);
	z = Trigger(p.rt) - Trigger(p.lt);
	if (!armed) {
		armed = x == 0 && y == 0 && z == 0;
		JogSet(v);
		return;
	}
	if (ax_x >= 0) v[ax_x] = x * pad_speed_xy;
	if (ax_y >= 0) v[ax_y] = y * pad_speed_xy;
	if (ax_z >= 0) v[ax_z] = z * pad_speed_z;
	JogSet(v);
}

void PadTask() {
	JogTask();
	if (mode != PAD_MODE_JOG) return;
	if (!JogActive()) {
		mode = PAD_MODE_OFF; // stopped under us (fault / E-stop)
		return;
	}
	Jog();
}

uint8_t PadMode() {
	return mode;
}

const char *PadModeName(uint8_t m) {
	static const char *names[PAD_MODES] = { "off", "jog" };
	return m < PAD_MODES ? names[m] : "?";
}

bool PadArmed() {
	return armed;
}

bool PadSetMode(uint8_t m) {
	uint8_t res;

	if (m == mode) return true;
	if (mode == PAD_MODE_JOG) JogStop(); // ramps down, JogTask hands the axes back
	mode = PAD_MODE_OFF;
	if (m == PAD_MODE_OFF) return true;

	// Jog
	ax_x = AxisNamed('X');
	ax_y = AxisNamed('Y');
	ax_z = AxisNamed('Z');
	if (JogActive()) {
		printf("Still stopping from the last jog, try again\n");
		return false;
	}
	// Dry run: nothing reaches the pins, so it works without the drivers (no motor supply)
	if (!BoardStepperEnabled() && !StepDry()) {
		if (!TmcReady()) {
			printf("Drivers not ready, staying off\n");
			return false;
		}
		BoardStepperEnable(true);
		printf("Drivers on\n");
	}
	res = JogStart();
	if (res == JOG_BUSY) {
		printf("Motion still queued, wait for it to finish\n");
		return false;
	}
	if (res == JOG_FAULT) {
		printf("Stopped, step recover first\n");
		return false;
	}
	armed = false;
	mode = PAD_MODE_JOG;
	return true;
}
