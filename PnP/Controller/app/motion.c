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

// Driver 0 + 1 Y gantry, 2 X, 3 Z, 4 toolhead 1 (A, rotation), 5 toolhead 2 (B), 6 / 7 spare (U, C)
// X Y Z: 1.8deg steppers, GT2 belt, 20 tooth pulley = 40mm/rev. 200 * 16 microsteps (tmc_config) /
// 40 = 80 steps/mm. A B: tool rotation, units are degrees. 1.8deg steppers, direct drive:
// 200 * 16 / 360 = 8.889 steps/deg. X Y Z accel / jerk cut to 65% (2026-11-04). A B: NEMA8, only
// a nozzle with an SMT part on it, so faster. TODO tune on the machine (missed steps / part slip).
// C U steps/mm and the X Y Z speed are still placeholders
static const MotionAxis defaults[MOTION_AXES] = {
	{ 'X', 1 << 2,			80, 300, 1950, 65000 },
	{ 'Y', 1 << 0 | 1 << 1,	80, 300, 1950, 65000 },
	{ 'Z', 1 << 3,			80, 300, 1950, 65000 },
	{ 'A', 1 << 4,			200 * 16 / 360.0f, 1000, 20000, 1000000 },
	{ 'B', 1 << 5,			200 * 16 / 360.0f, 1000, 20000, 1000000 },
	{ 'C', 1 << 7,			80, 300, 3000, 100000 },
	{ 'U', 1 << 6,			80, 300, 3000, 100000 },
};
// Bit per driver. Driver 0 faces driver 1 across the gantry, so it turns the other way.
// TODO the rest from the real directions once each motor has been checked
#define INVERT_DEFAULT	(1 << 0)

MotionAxis motion_axes[MOTION_AXES];
uint8_t motion_invert = INVERT_DEFAULT;
float motion_accel = 0;

static int64_t pos_steps[MOTION_AXES];	// planned position, steps
static int64_t pos_offset[MOTION_AXES];	// G92 shift between planned steps and the step generator
static uint64_t timeline_end;			// tick the last planned move finishes at
static uint64_t last_start;
static SCurve last_plan;
static bool locked;						// jog mode has the axes

// ---- junctions (G1 .. E): a move that runs into the next one at speed ----
// Every move is queued ending at rest, so whatever is in the step queues always stops by itself.
// When the next G1 comes and carries straight on, the part of the last move that hasn't played
// yet (from a point where its accel is 0: its start, cruise or decel) is taken back and queued
// again ending at the junction speed, and the new move starts at that speed and ends at rest.
// If it comes too late (that point has played), turns, or there isn't room, the last move just
// stops like any other.
#define JUNCTION_DIFF	0.0175	// largest change in any axis' share of the direction (~1 deg), more = a corner
#define JUNCTION_JUMP	1.0		// mm/s, largest step in any one axis' speed at the junction
#define JUNCTION_MIN	0.01	// mm/s, slower than this is a stop
#define REPLAN_TICKS	(LEAD_TICKS + 2 * STEP_HALF)	// a queued piece starting this far ahead can be swapped

typedef struct {
	bool open;				// last thing on the timeline, can still be re-planned
	float exit;				// exit speed asked for, mm/s, 0 = none
	uint8_t moving;			// axes
	int64_t d[MOTION_AXES];	// steps, signed
	double len;				// path length (mm, degrees for a rotary only move)
	double v, a, j;			// its normalized limits
	SCurve sc;
	uint64_t start;
} Planned;

typedef struct {
	uint32_t phase;			// re-planned from the start of this phase (0, 3 or 4)
	uint64_t tick;			// which starts here
	double u;				// and at this u
	double vj;				// junction speed, mm/s
	SCurve rem;				// the rest of the last move, from there to the junction
} Join;

static Planned last;
static MotionJunctions junc;

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

// Nothing can join the last move any more. 'cut': it asked for an exit speed and something
// other than a G1 came next, that counts as late.
static void Close(bool cut) {
	if (cut && last.open && last.exit > 0) {
		junc.asked++;
		junc.late++;
		junc.last_asked = last.exit;
		junc.last_used = 0;
	}
	last.open = false;
}

void MotionSync() {
	uint8_t i;
	Close(false);
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
	last.open = false;
	MotionJunctionsReset();
}

void MotionJunctionsReset() {
	MotionJunctions z = { 0 };
	junc = z;
}

void MotionJunctionStats(MotionJunctions *s) {
	*s = junc;
}

// Rest of the last move from vb (at u) to vj, and the new move from vj, both inside their limits
static bool JoinOk(Join *jn, double vb, double vj, double len, double v, double a, double j) {
	SCurve nx;
	double dr = 1 - jn->u;

	jn->vj = vj;
	return SCurvePlanV(&jn->rem, vb / dr, vj / last.len / STEP_TICK_HZ / dr, last.v / dr, last.a / dr, last.j / dr)
		&& SCurvePlanV(&nx, vj / len / STEP_TICK_HZ, 0, v, a, j);
}

// Junction with the last move: speed and where to re-plan it from. False if it has to stop,
// with the counter for why already bumped.
static bool Junction(Join *jn, const int64_t *d, uint8_t moving, double len, double v, double a, double j) {
	double diff = 0, cap, lim, u0, u1, vb = 0, lo, hi, mid;
	uint32_t ph, t = 0, k;
	uint8_t i;

	junc.asked++;
	junc.last_asked = last.exit;
	junc.last_used = 0;

	// Direction: each axis' share of the path before and after. Straight on = the same shares.
	for (i = 0; i < MOTION_AXES; i++) {
		if (!(((moving | last.moving) >> i) & 1)) continue;
		u0 = last.d[i] / (double)motion_axes[i].steps_per_mm / last.len;
		u1 = d[i] / (double)motion_axes[i].steps_per_mm / len;
		if (fabs(u0 - u1) > diff) diff = fabs(u0 - u1);
	}
	if (diff > JUNCTION_DIFF) {
		junc.corner++;
		return false;
	}

	// As asked, but inside both moves' speed limits and the speed step the turn leaves
	cap = last.exit;
	lim = last.v * last.len * STEP_TICK_HZ;
	if (lim < cap) cap = lim;
	lim = v * len * STEP_TICK_HZ;
	if (lim < cap) cap = lim;
	if (diff > 0) {
		lim = JUNCTION_JUMP / diff;
		if (lim < cap) cap = lim;
	}

	// Earliest accel 0 point of the last move that is far enough ahead to swap what follows it
	for (ph = 0; ph <= 4; ph++) {
		if (ph == 1 || ph == 2) continue;
		SCurveAt(&last.sc, ph, &jn->u, &vb, &t);
		if (last.start + t >= StepNow() + REPLAN_TICKS) break;
	}
	if (ph > 4 || jn->u >= 1) {
		junc.late++;
		return false;
	}
	jn->phase = ph;
	jn->tick = last.start + t;

	// Fastest both can do: the last move still has to get there from vb, and the new one is
	// queued ending at rest so it has to be able to stop from vj
	if (!JoinOk(jn, vb, cap, len, v, a, j)) {
		lo = 0;
		hi = cap;
		for (k = 0; k < 16; k++) { // to 1/65536 of cap: plenty, and cheap enough to stay ahead of REPLAN_TICKS
			mid = (lo + hi) / 2;
			if (JoinOk(jn, vb, mid, len, v, a, j)) lo = mid;
			else hi = mid;
		}
		if (lo < JUNCTION_MIN || !JoinOk(jn, vb, lo, len, v, a, j)) {
			junc.limited++;
			return false;
		}
	}
	return true;
}

// 'need' free queue slots on every driver of these axes
static bool Room(uint8_t axes, uint32_t need) {
	uint8_t i, drv;
	for (i = 0; i < MOTION_AXES; i++) {
		if (!((axes >> i) & 1)) continue;
		for (drv = 0; drv < MOTION_DRIVERS; drv++) {
			if ((motion_axes[i].drivers >> drv) & 1 && StepQueueFree(drv) < need) return false;
		}
	}
	return true;
}

uint8_t MotionMove(const float *target, uint8_t mask, float feed) {
	return MotionMoveExit(target, mask, feed, -1);
}

uint8_t MotionMoveExit(const float *target, uint8_t mask, float feed, float exit) {
	int64_t d[MOTION_AXES];
	uint32_t steps[MOTION_AXES];
	double len = 0, rot = 0, v, a, j, lim, mm;
	uint8_t i, drv, moving = 0;
	uint64_t start;
	SCurve sc;
	StepSegment seg[SCURVE_PHASES];
	uint32_t n, k;
	Join jn;
	bool want, join = false;

	if (feed <= 0) return MOTION_BAD;
	if (!StepRunning()) return MOTION_FAULT;
	if (locked) return MOTION_JOG;

	// Steps each axis has to go, and the path length. The length is X Y Z only, so F is mm/s and
	// rotary axes just finish with them (still held to their own limits). A rotary only move
	// uses the degrees instead, F is deg/s then.
	for (i = 0; i < MOTION_AXES; i++) {
		d[i] = 0;
		if (!((mask >> i) & 1) || motion_axes[i].drivers == 0) continue;
		d[i] = llround((double)target[i] * motion_axes[i].steps_per_mm) - pos_steps[i];
		if (d[i] == 0) continue;
		mm = d[i] / (double)motion_axes[i].steps_per_mm;
		if (i < MOTION_LINEAR) len += mm * mm;
		else rot += mm * mm;
		moving |= 1 << i;
	}
	if (!moving) return MOTION_NOMOVE;
	len = sqrt(len > 0 ? len : rot);

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

	// Room for all segments on every driver, or nothing gets queued. A junction re-queues the
	// rest of the last move too: room for both, or wait while it can still join
	want = last.open && last.exit > 0 && exit >= 0;
	if (want && !Room(moving | last.moving, 2 * SCURVE_PHASES)) {
		if (last.start + last.sc.total >= StepNow() + REPLAN_TICKS) return MOTION_BUSY;
		want = false; // too late to join anyway
	}
	if (!want && !Room(moving, SCURVE_PHASES)) return MOTION_BUSY;

	// Run into it from the last move?
	if (last.open && last.exit > 0) {
		if (want) join = Junction(&jn, d, moving, len, v, a, j);
		else Close(true);
	}
	if (join) {
		if (!SCurvePlanV(&sc, jn.vj / len / STEP_TICK_HZ, 0, v, a, j)) return MOTION_BAD;
		// The planning took time: is the piece to swap still far enough ahead?
		if (jn.tick < StepNow() + LEAD_TICKS) {
			junc.late++;
			join = false;
			if (!SCurvePlan(&sc, v, a, j)) return MOTION_BAD;
		}
	}
	if (join) {
		for (i = 0; i < MOTION_AXES; i++) {
			if (!((last.moving >> i) & 1)) continue;
			uint32_t ls = (uint32_t)(last.d[i] < 0 ? -last.d[i] : last.d[i]);
			for (drv = 0; drv < MOTION_DRIVERS; drv++) {
				if (!((motion_axes[i].drivers >> drv) & 1)) continue;
				bool dir = (last.d[i] > 0) != ((motion_invert >> drv) & 1);
				StepUnqueue(drv, jn.tick);
				n = SCurveSegmentsPart(&jn.rem, jn.u * ls, (1 - jn.u) * ls, dir, jn.tick, seg);
				for (k = 0; k < n; k++) StepQueue(drv, &seg[k]);
			}
		}
		timeline_end = jn.tick + jn.rem.total;
		junc.last_used = (float)jn.vj;
		if (jn.vj >= last.exit * 0.999) junc.full++;
		else junc.limited++;
		junc.from_phase[jn.phase == 0 ? 0 : jn.phase == 3 ? 1 : 2]++;
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

	// This one can be run into by the next G1 (and asks for exit speed 'exit')
	last.open = exit >= 0;
	last.exit = exit > 0 ? exit : 0;
	last.moving = moving;
	for (i = 0; i < MOTION_AXES; i++) last.d[i] = d[i];
	last.len = len;
	last.v = v;
	last.a = a;
	last.j = j;
	last.sc = sc;
	last.start = start;
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
	Close(true);

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
	if (lock) Close(true);
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
	Close(true);
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
