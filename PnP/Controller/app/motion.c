#include "motion.h"
#include "scurve.h"
#include "stepgen.h"

#include <math.h>
#include <stdio.h>

// Moves are scheduled this far ahead of what the step generator is filling, so a segment is
// never already in the past when it lands in the queue (2 half buffers = ~1ms).
#define LEAD_TICKS		(2 * STEP_HALF)
// Step rate limits, steps/tick. The pins could do 0.5 per driver; the real limit is the refill
// ISR, which costs ~1.1us per S-curve step on the M7 ('step bench': 8 drivers at 700k steps/s
// total took 407 of the 512us budget). 500k keeps the worst case around 60% so comms still get
// CPU. Moves run one at a time, so per move total = machine total. Every driver counts, so a
// ganged axis costs twice.
#define MAX_STEP_RATE	(MOTION_MAX_STEP_HZ / (double)STEP_TICK_HZ)	// steps/tick per driver
#define STEP_BUDGET		0.5		// all drivers together, 500k steps/s

_Static_assert(MOTION_DRIVERS == STEP_AXES, "one motion driver per step generator channel");

// Driver 0 / 1 spare (U, C), 2 X, 3 Z, 4 toolhead 1 (A, rotation), 5 toolhead 2 (B), 6 + 7 Y gantry
// X Y Z: 1.8deg steppers, GT2 belt, 20 tooth pulley = 40mm/rev. 200 * 16 microsteps (tmc_config) /
// 40 = 80 steps/mm. A B C U steps/mm and every speed / accel / jerk are still placeholders:
// TODO 300mm/s, 3000mm/s^2, 100000mm/s^3
static const MotionAxis defaults[MOTION_AXES] = {
	{ 'X', 1 << 2,			80, 300, 3000, 100000 },
	{ 'Y', 1 << 6 | 1 << 7,	80, 300, 3000, 100000 },
	{ 'Z', 1 << 3,			80, 300, 3000, 100000 },
	{ 'A', 1 << 4,			80, 300, 3000, 100000 },
	{ 'B', 1 << 5,			80, 300, 3000, 100000 },
	{ 'C', 1 << 1,			80, 300, 3000, 100000 },
	{ 'U', 1 << 0,			80, 300, 3000, 100000 },
};
// Bit per driver. Driver 6 faces driver 7 across the gantry, so it turns the other way.
// TODO the rest from the real directions once each motor has been checked
#define INVERT_DEFAULT	(1 << 6)

MotionAxis motion_axes[MOTION_AXES];
uint8_t motion_invert = INVERT_DEFAULT;
float motion_accel = 0;

static int64_t pos_steps[MOTION_AXES];	// planned position, steps
static int64_t pos_offset[MOTION_AXES];	// G92 shift between planned steps and the step generator
static uint64_t timeline_end;			// tick the last planned move finishes at
static uint64_t last_start;
static SCurve last_plan;
static bool locked;						// jog mode has the axes

static uint8_t Bits(uint8_t m) {
	uint8_t n = 0;
	for (; m; m &= m - 1) n++;
	return n;
}

// First driver of an axis: that's where its position is read from
static uint8_t Primary(uint8_t axis) {
	uint8_t d;
	for (d = 0; d < MOTION_DRIVERS; d++) {
		if ((motion_axes[axis].drivers >> d) & 1) return d;
	}
	return 0;
}

// Driver's step count in the axis' direction
static int64_t DriverPos(uint8_t drv) {
	int64_t p = StepPosition(drv);
	return (motion_invert >> drv) & 1 ? -p : p;
}

int8_t MotionAxisOf(uint8_t drv) {
	uint8_t i;
	for (i = 0; i < MOTION_AXES; i++) {
		if ((motion_axes[i].drivers >> drv) & 1) return i;
	}
	return -1;
}

const char *MotionDriverName(uint8_t drv) {
	static char names[MOTION_DRIVERS][6];	// letter + up to 3 digits
	int8_t i = drv < MOTION_DRIVERS ? MotionAxisOf(drv) : -1;
	uint8_t d, k = 1;

	if (i < 0) return "-";
	if (Bits(motion_axes[i].drivers) == 1) {
		snprintf(names[drv], sizeof(names[drv]), "%c", motion_axes[i].name);
	} else {
		for (d = 0; d < drv; d++) k += (motion_axes[i].drivers >> d) & 1;
		snprintf(names[drv], sizeof(names[drv]), "%c%d", motion_axes[i].name, k);
	}
	return names[drv];
}

void MotionDefaults() {
	uint8_t i;
	for (i = 0; i < MOTION_AXES; i++) motion_axes[i] = defaults[i];
	motion_invert = INVERT_DEFAULT;
	motion_accel = 0;
}

void MotionSync() {
	uint8_t i;
	for (i = 0; i < MOTION_AXES; i++) pos_steps[i] = DriverPos(Primary(i)) + pos_offset[i];
	timeline_end = 0;
}

void MotionInit() {
	uint8_t i;
	MotionDefaults();
	for (i = 0; i < MOTION_AXES; i++) {
		pos_steps[i] = 0;
		pos_offset[i] = 0;
	}
	timeline_end = 0;
}

uint8_t MotionMove(const float *target, uint8_t mask, float feed) {
	int64_t d[MOTION_AXES];
	uint32_t steps[MOTION_AXES];
	double len = 0, v, a, j, lim, mm;
	uint8_t i, drv, moving = 0;
	uint64_t start;
	SCurve sc;
	StepSegment seg[SCURVE_PHASES];
	uint32_t n, k;

	if (feed <= 0) return MOTION_BAD;
	if (!StepRunning()) return MOTION_FAULT;
	if (locked) return MOTION_JOG;

	// Steps each axis has to go, and the path length in mm
	for (i = 0; i < MOTION_AXES; i++) {
		d[i] = 0;
		if (!((mask >> i) & 1) || motion_axes[i].drivers == 0) continue;
		d[i] = llround((double)target[i] * motion_axes[i].steps_per_mm) - pos_steps[i];
		if (d[i] == 0) continue;
		mm = d[i] / (double)motion_axes[i].steps_per_mm;
		len += mm * mm;
		moving |= 1 << i;
	}
	if (!moving) return MOTION_NOMOVE;
	len = sqrt(len);

	// Path limits on the normalized path (u = 0..1, per tick). Start from the feed, then every
	// moving axis can only lower them: an axis moving |d| steps goes |d| times as fast as u.
	v = feed / len / STEP_TICK_HZ;
	a = 1e30;
	j = 1e30;
	for (i = 0; i < MOTION_AXES; i++) {
		if (!((moving >> i) & 1)) continue;
		steps[i] = (uint32_t)(d[i] < 0 ? -d[i] : d[i]);
		double spt = motion_axes[i].steps_per_mm / (double)steps[i];	// (steps/mm) / steps
		lim = motion_axes[i].v_max * spt / STEP_TICK_HZ;
		if (lim < v) v = lim;
		lim = MAX_STEP_RATE / steps[i];
		if (lim < v) v = lim;
		lim = motion_axes[i].a_max * spt / ((double)STEP_TICK_HZ * STEP_TICK_HZ);
		if (lim < a) a = lim;
		lim = motion_axes[i].j_max * spt / ((double)STEP_TICK_HZ * STEP_TICK_HZ * STEP_TICK_HZ);
		if (lim < j) j = lim;
	}
	// Path acceleration (M204) on top of the per axis limits
	if (motion_accel > 0) {
		lim = motion_accel / len / ((double)STEP_TICK_HZ * STEP_TICK_HZ);
		if (lim < a) a = lim;
	}

	// Total step rate: every driver runs at steps[i] * u', so the sum is total_steps * u'
	{
		double total = 0;
		for (i = 0; i < MOTION_AXES; i++) {
			if ((moving >> i) & 1) total += (double)steps[i] * Bits(motion_axes[i].drivers);
		}
		lim = STEP_BUDGET / total;
		if (lim < v) v = lim;
	}
	if (!SCurvePlan(&sc, v, a, j)) return MOTION_BAD;

	// Room for all segments on every driver, or nothing gets queued
	for (i = 0; i < MOTION_AXES; i++) {
		if (!((moving >> i) & 1)) continue;
		for (drv = 0; drv < MOTION_DRIVERS; drv++) {
			if ((motion_axes[i].drivers >> drv) & 1 && StepQueueFree(drv) < SCURVE_PHASES) return MOTION_BUSY;
		}
	}

	// Start after the last move, and never in the past
	start = StepNow() + LEAD_TICKS;
	if (timeline_end > start) start = timeline_end;

	for (i = 0; i < MOTION_AXES; i++) {
		if (!((moving >> i) & 1)) continue;
		for (drv = 0; drv < MOTION_DRIVERS; drv++) {
			if (!((motion_axes[i].drivers >> drv) & 1)) continue;
			bool dir = (d[i] > 0) != ((motion_invert >> drv) & 1);
			n = SCurveSegments(&sc, steps[i], dir, start, seg);
			for (k = 0; k < n; k++) StepQueue(drv, &seg[k]);
		}
		pos_steps[i] += d[i];
	}
	timeline_end = start + sc.total;
	last_start = start;
	last_plan = sc;
	return MOTION_OK;
}

// Test stepping on one driver: one constant rate segment, no S-curve, queued after whatever is
// planned so it shares the timeline with normal moves. Raw: + = DIR high, the invert setting
// isn't used. If it's the driver an axis reads its position from, the axis follows.
// On a ganged axis this moves one side only (Y1 without Y2 racks the gantry).
uint8_t MotionSteps(uint8_t drv, int32_t steps, float rate) {
	StepSegment s;
	uint32_t n, period;
	uint64_t start;
	int8_t i;

	if (drv >= MOTION_DRIVERS || rate <= 0) return MOTION_BAD;
	if (!StepRunning()) return MOTION_FAULT;
	if (locked) return MOTION_JOG;
	if (steps == 0) return MOTION_NOMOVE;
	period = (uint32_t)lroundf(STEP_TICK_HZ / rate);
	if (period < 1 / MAX_STEP_RATE) return MOTION_BAD;
	if (StepQueueFree(drv) < 1) return MOTION_BUSY;

	start = StepNow() + LEAD_TICKS;
	if (timeline_end > start) start = timeline_end;
	n = steps < 0 ? -steps : steps;
	if ((uint64_t)n * period > 0xFFFFFFFF) return MOTION_BAD; // duration is 32 bit ticks (~71 min)
	StepSegConst(&s, start, n, period, steps > 0);
	StepQueue(drv, &s);
	i = MotionAxisOf(drv);
	if (i >= 0 && Primary(i) == drv) pos_steps[i] += (motion_invert >> drv) & 1 ? -steps : steps;
	timeline_end = start + s.duration;
	return MOTION_OK;
}

void MotionLock(bool lock) {
	locked = lock;
}

bool MotionIdle() {
	uint8_t d;
	for (d = 0; d < MOTION_DRIVERS; d++) {
		if (!StepAxisIdle(d)) return false;
	}
	return true;
}

float MotionPosition(uint8_t axis) {
	if (axis >= MOTION_AXES) return 0;
	return pos_steps[axis] / motion_axes[axis].steps_per_mm;
}

int64_t MotionCount(uint8_t axis) {
	return axis < MOTION_AXES ? DriverPos(Primary(axis)) : 0;
}

float MotionActual(uint8_t axis) {
	if (axis >= MOTION_AXES) return 0;
	return (DriverPos(Primary(axis)) + pos_offset[axis]) / motion_axes[axis].steps_per_mm;
}

void MotionSetPosition(uint8_t axis, float mm) {
	int64_t s;
	if (axis >= MOTION_AXES) return;
	s = llround((double)mm * motion_axes[axis].steps_per_mm);
	// Planned position becomes s; the generator's count keeps going, the offset lines them up
	pos_offset[axis] += s - pos_steps[axis];
	pos_steps[axis] = s;
}

uint32_t MotionLastTime() {
	return last_plan.total;
}

uint64_t MotionLastStart() {
	return last_start;
}

void MotionLastPlan(SCurve *s) {
	*s = last_plan;
}
