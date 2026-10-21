#include "jog.h"
#include "clock.h"
#include "stepgen.h"

#include <math.h>
#include <string.h>

// Each tick, every axis gets constant acceleration segments JOG_PERIOD long until JOG_AHEAD is
// queued. The acceleration takes the speed straight to the target, or as far as a_max allows.
// A segment's speed can't change sign (the fill wants v >= 0 with DIR fixed), so one that would
// goes only as far as v = 0, ending on a whole tick, and the next starts a new run the other way.
// Positions: the fill counts real steps, MotionSync picks them up when the jog ends.

#define JOG_AHEAD		(2 * JOG_PERIOD)	// queued ahead of the step generator
#define LEAD_TICKS		(2 * STEP_HALF)		// same as motion.c: never start in the past
#define MAX_STEP_RATE	(MOTION_MAX_STEP_HZ / (double)STEP_TICK_HZ)	// steps/tick per driver
#define STEP_BUDGET		0.5		// all drivers together, steps/tick, as motion.c

typedef struct {
	double v;		// steps/tick, signed, + = the axis' + direction
	double p;		// steps into the current run (one direction), fractional
	uint64_t next;	// tick the next segment starts at
	bool moving;
} jog_axis;

static jog_axis ja[MOTION_AXES];
static volatile float target[MOTION_AXES];	// mm/s
static volatile uint32_t target_ms;
static volatile bool running;	// JogTick plans
static volatile bool stopping;	// targets forced to 0, running ends once everything stopped
static volatile bool faulted;
static bool held;				// main loop side: the axes are locked for jog
static volatile uint32_t late;

static uint8_t Bits(uint8_t m) {
	uint8_t n = 0;
	for (; m; m &= m - 1) n++;
	return n;
}

// One segment of d ticks at acceleration acc (steps/tick^2, signed) from the axis' state
static bool Queue(uint8_t i, jog_axis *a, uint32_t d, double acc) {
	StepSegment s;
	bool pos = a->v > 0 || (a->v == 0 && acc > 0);
	uint8_t drv;

	s.start = a->next;
	s.duration = d;
	s.p0 = a->p;
	s.v0 = fabs(a->v);
	s.a0 = pos ? acc : -acc;
	s.j = 0;
	for (drv = 0; drv < MOTION_DRIVERS; drv++) {
		if (!((motion_axes[i].drivers >> drv) & 1)) continue;
		if ((StepHalted() >> drv) & 1) continue; // stalled while homing, the others carry on
		s.dir = pos != ((motion_invert >> drv) & 1);
		// Refused because the DIAG interrupt halted it just now is fine too
		if (!StepQueue(drv, &s) && !((StepHalted() >> drv) & 1)) return false;
	}
	a->p += s.v0 * d + s.a0 * d * (double)d / 2;
	return true;
}

static bool Room(uint8_t i) {
	uint8_t drv;
	for (drv = 0; drv < MOTION_DRIVERS; drv++) {
		if ((motion_axes[i].drivers >> drv) & 1 && !((StepHalted() >> drv) & 1) && StepQueueFree(drv) < 2) return false;
	}
	return true;
}

// Plan axis i up to the horizon towards vt (steps/tick, signed). False if the queue refused.
static bool Axis(uint8_t i, double vt) {
	jog_axis *a = &ja[i];
	uint64_t now = StepNow();
	double amax = motion_axes[i].a_max * motion_axes[i].steps_per_mm / ((double)STEP_TICK_HZ * STEP_TICK_HZ);

	if (motion_axes[i].drivers == 0) return true;
	if (!a->moving) {
		if (vt == 0) return true;
		a->v = 0;
		a->p = 0;
		a->next = now + LEAD_TICKS;
		a->moving = true;
	} else if (a->next < now) {
		// Ran dry: the motor already stopped dead. Start over from rest
		late++;
		a->v = 0;
		a->p = 0;
		a->next = now + LEAD_TICKS;
	}

	while (a->next < now + JOG_AHEAD && Room(i)) {
		uint32_t d = JOG_PERIOD;
		double acc = (vt - a->v) / d, v1;

		if (acc > amax) acc = amax;
		else if (acc < -amax) acc = -amax;
		if (a->v == 0 && acc == 0) {
			a->moving = false; // at rest, target 0
			return true;
		}
		v1 = a->v + acc * d;
		if ((a->v > 0 && v1 < 0) || (a->v < 0 && v1 > 0)) {
			// Through zero: stop on a whole tick, the next run goes the other way
			d = (uint32_t)ceil(-a->v / acc);
			if (d == 0) d = 1;
			acc = -a->v / d;
			v1 = 0;
		}
		if (fabs(v1) < 1e-12) v1 = 0;
		if (!Queue(i, a, d, acc)) return false;
		a->v = v1;
		a->next += d;
		if (v1 == 0) a->p = 0;
	}
	return true;
}

static void Fault() {
	running = false;
	faulted = true;
	memset(ja, 0, sizeof(ja));
}

void JogTick() {
	double vt[MOTION_AXES], total = 0, scale = 1, lim;
	bool zero, any = false;
	uint8_t i;

	if (!running) return;
	if (!StepRunning()) {
		Fault();
		return;
	}
	zero = stopping || Millis() - target_ms > JOG_TIMEOUT_MS;

	// mm/s -> steps/tick, capped at the axis' v_max and the per driver step rate
	for (i = 0; i < MOTION_AXES; i++) {
		vt[i] = zero ? 0 : target[i] * motion_axes[i].steps_per_mm / STEP_TICK_HZ;
		lim = motion_axes[i].v_max * motion_axes[i].steps_per_mm / STEP_TICK_HZ;
		if (lim > MAX_STEP_RATE) lim = MAX_STEP_RATE;
		if (vt[i] > lim) vt[i] = lim;
		else if (vt[i] < -lim) vt[i] = -lim;
		total += fabs(vt[i]) * Bits(motion_axes[i].drivers);
	}
	// All drivers together stay inside the step budget, same as a MotionMove
	if (total > STEP_BUDGET) scale = STEP_BUDGET / total;

	for (i = 0; i < MOTION_AXES; i++) {
		if (!Axis(i, vt[i] * scale)) {
			Fault();
			return;
		}
		if (ja[i].moving) any = true;
	}
	if (stopping && !any) running = false;
}

uint8_t JogStart() {
	uint8_t i;

	if (held) return JOG_OK;
	if (!StepRunning()) return JOG_FAULT;
	if (!MotionIdle()) return JOG_BUSY;

	MotionLock(true);
	memset(ja, 0, sizeof(ja));
	for (i = 0; i < MOTION_AXES; i++) target[i] = 0;
	target_ms = Millis();
	stopping = false;
	faulted = false;
	held = true;
	running = true;
	return JOG_OK;
}

void JogSet(const float *v) {
	uint8_t i;
	for (i = 0; i < MOTION_AXES; i++) target[i] = v[i];
	target_ms = Millis();
}

void JogStop() {
	stopping = true;
}

void JogTask() {
	// Stopped (or faulted) and played out: the planner takes over from where the steps are
	if (held && !running && MotionIdle()) {
		MotionSync();
		MotionLock(false);
		held = false;
	}
}

bool JogActive() {
	return held;
}

bool JogFaulted() {
	return faulted;
}

uint32_t JogLate() {
	return late;
}
