// Host test for G1 E junctions: scurve.c v0 -> v1 profiles, and motion.c running one move
// into the next (taking back the unplayed end of the last move and queueing it again).
//
// - SCurvePlanV against an independent evaluation of its segments (StepSegPos, not SCurveAt)
// - a bed scan row (fast, slow through the photo spot, fast ...) never stops at a junction
// - late / corner / cut junctions fall back to a stop and land in the right counter
// - fuzz: after every planner call the step queues are checked: no gaps at speed, no speed
//   jumps beyond JUNCTION_JUMP, and the newest segment of every driver ends (about) at rest
//
// Build/run: make -C test/host junction
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion.h"
#include "scurve.h"
#include "stepgen.h"
#include "stepgen_fill.h"

#define HALF		512
#define DIR_SETUP	2
#define HZ			1e6
#define JUMP		1.0		// mm/s, = JUNCTION_JUMP in motion.c

static int failures;

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		if (failures++ < 30) { \
			printf("FAIL %s:%d: ", __FILE__, __LINE__); \
			printf(__VA_ARGS__); \
			printf("\n"); \
		} \
	} \
} while (0)

// ---- stand-ins for the hardware half of stepgen.c ----
static StepGen g;
bool StepQueue(uint32_t axis, const StepSegment *seg) { return StepFillQueue(&g, axis, seg); }
uint32_t StepUnqueue(uint32_t axis, uint64_t from) { return StepFillUnqueue(&g, axis, from); }
uint32_t StepQueueFree(uint32_t axis) { return StepFillQueueFree(&g, axis); }
bool StepAxisIdle(uint32_t axis) { return StepFillAxisIdle(&g, axis); }
uint64_t StepNow() { return g.tick; }
int64_t StepPosition(uint32_t axis) { return g.axis[axis].position; }
bool StepRunning() { return true; }

// ---- simulated port: rising STEP edges per driver ----
typedef struct {
	int64_t *rise;
	uint32_t n, cap;
	int level;
} Log;

static Log logs[STEP_AXES];
static uint32_t odr;
static int64_t now;

static void LogReset() {
	uint32_t i;
	for (i = 0; i < STEP_AXES; i++) {
		free(logs[i].rise);
		memset(&logs[i], 0, sizeof(logs[i]));
	}
}

static void Tick() {
	uint32_t buf[HALF], k, i;
	StepFillWords(&g, buf, HALF);
	for (k = 0; k < HALF; k++, now++) {
		odr |= buf[k] & 0xFFFF;
		odr &= ~(buf[k] >> 16);
		for (i = 0; i < STEP_AXES; i++) {
			int s = (odr >> i) & 1;
			if (s && !logs[i].level) {
				if (logs[i].n == logs[i].cap) {
					logs[i].cap = logs[i].cap ? logs[i].cap * 2 : 4096;
					logs[i].rise = realloc(logs[i].rise, logs[i].cap * sizeof(int64_t));
				}
				logs[i].rise[logs[i].n++] = now;
			}
			logs[i].level = s;
		}
	}
}

static bool AllIdle() {
	uint32_t i;
	for (i = 0; i < STEP_AXES; i++) {
		if (!StepFillAxisIdle(&g, i)) return false;
	}
	return true;
}

static void RunUntilIdle() {
	while (!AllIdle()) Tick();
	Tick();
	Tick();
}

static void Setup() {
	uint32_t i, step_mask[STEP_AXES], dir_mask[STEP_AXES];
	for (i = 0; i < STEP_AXES; i++) {
		step_mask[i] = 1u << i;
		dir_mask[i] = 1u << (i + 8);
	}
	StepFillInit(&g, step_mask, dir_mask, DIR_SETUP);
	MotionInit();
	for (i = 0; i < MOTION_AXES; i++) motion_axes[i].drivers = 1 << i;
	motion_invert = 0;
	odr = 0;
	now = 0;
	LogReset();
}

// ---- segment helpers (stepgen's own p(t), nothing from scurve.c) ----
static double SegV(const StepSegment *s, double t) { return s->v0 + t * (s->a0 + t * s->j / 2); }
static double SegA(const StepSegment *s, double t) { return s->a0 + t * s->j; }

// Smallest speed inside a segment (speed is quadratic in t)
static double SegVMin(const StepSegment *s) {
	double m = SegV(s, 0), e = SegV(s, s->duration), t;
	if (e < m) m = e;
	if (s->j != 0) {
		t = -s->a0 / s->j;
		if (t > 0 && t < s->duration && SegV(s, t) < m) m = SegV(s, t);
	}
	return m;
}

// ---- 1: SCurvePlanV maths ----

static double Side(double dv, double a, double j) {
	if (dv <= 0) return 0;
	if (dv * j >= a * a) return a / j + dv / a;	// 2*tj + ta = 2a/j + dv/a - a/j
	return 2 * sqrt(dv / j);
}

static double Rnd(uint32_t *s) {
	*s ^= *s << 13;
	*s ^= *s >> 17;
	*s ^= *s << 5;
	return (*s & 0xFFFFFF) / (double)0x1000000;
}

static void TestPlanV() {
	uint32_t r = 777, n, i, k, ok = 0, no = 0, tight = 0;
	int start_fail = failures;
	SCurve s, s2;
	StepSegment seg[SCURVE_PHASES];

	// Rest to rest is exactly the old planner
	for (i = 0; i < 2000; i++) {
		double len = 0.05 + Rnd(&r) * 300, v = (5 + Rnd(&r) * 300) / len / HZ;
		double a = (500 + Rnd(&r) * 3000) / len / (HZ * HZ), j = (1e4 + Rnd(&r) * 1e6) / len / (HZ * HZ * HZ);
		memset(&s, 0, sizeof(s)); // padding too, for the memcmp
		memset(&s2, 0, sizeof(s2));
		SCurvePlan(&s, v, a, j);
		SCurvePlanV(&s2, 0, 0, v, a, j);
		CHECK(memcmp(&s, &s2, sizeof(s)) == 0, "rest to rest plan differs (case %u)", i);
	}

	// v0 -> v1, in mm so the tolerances mean something. Steps = 80/mm like X.
	for (i = 0; i < 20000; i++) {
		double len = 0.05 + Rnd(&r) * (Rnd(&r) < 0.3 ? 2 : 300);
		double vm = 5 + Rnd(&r) * 300, am = 500 + Rnd(&r) * 3000, jm = 1e4 + Rnd(&r) * 1e6;
		double v0m = Rnd(&r) < 0.2 ? 0 : Rnd(&r) * vm, v1m = Rnd(&r) < 0.2 ? 0 : Rnd(&r) * vm;
		if (Rnd(&r) < 0.1) v1m = v0m;
		if (v0m == 0 && v1m == 0) v0m = 1;
		double v = vm / len / HZ, a = am / len / (HZ * HZ), j = jm / len / (HZ * HZ * HZ);
		double v0 = v0m / len / HZ, v1 = v1m / len / HZ;
		double steps = len * 80;

		if (!SCurvePlanV(&s, v0, v1, v, a, j)) {
			// Only when it really can't get from v0 to v1 in the distance, or it's tight: a short
			// move (under 1.5 mm here) or one within 1% of the edge, or at a speed where one tick of
			// rounding is already a speed change over the accel / jerk limits. The junction just
			// comes down to a speed that does fit.
			double dmin = (v0 + v1) / 2 * Side(fabs(v1 - v0), a, j);
			if (dmin <= 1 - 1e-9) {
				CHECK(len < 1.5 || dmin > 0.99, "refused a feasible plan: len %.3f v0 %.2f v1 %.2f vmax %.1f a %.0f dmin %.4f", len, v0m, v1m, vm, am, dmin);
				tight++;
			}
			no++;
			continue;
		}
		ok++;
		n = SCurveSegmentsPart(&s, 0, steps, true, 0, seg);
		uint32_t tsum = 0;
		double vmin = 1e30, vmax = 0, amax = 0;
		for (k = 0; k < n; k++) {
			tsum += seg[k].duration;
			if (k > 0) {
				// pieces join up: position, speed, accel
				double t = seg[k - 1].duration;
				CHECK(fabs(StepSegPos(&seg[k - 1], t) - seg[k].p0) < 1e-6, "plan %u: position jump at piece %u", i, k);
				CHECK(fabs(SegV(&seg[k - 1], t) - seg[k].v0) < 1e-12, "plan %u: speed jump at piece %u", i, k);
				CHECK(fabs(SegA(&seg[k - 1], t) - seg[k].a0) < 1e-15, "plan %u: accel jump at piece %u", i, k);
			}
			double m = SegVMin(&seg[k]);
			if (m < vmin) vmin = m;
			if (SegV(&seg[k], 0) > vmax) vmax = SegV(&seg[k], 0);
			if (fabs(SegA(&seg[k], 0)) > amax) amax = fabs(SegA(&seg[k], 0));
			if (fabs(SegA(&seg[k], seg[k].duration)) > amax) amax = fabs(SegA(&seg[k], seg[k].duration));
		}
		const StepSegment *e = &seg[n - 1];
		double vend = SegV(e, e->duration);
		CHECK(tsum == s.total, "plan %u: segments last %u, plan says %u", i, tsum, s.total);
		CHECK(fabs(StepSegPos(e, e->duration) - steps) < 1e-6, "plan %u: ends at %.9f of %.9f steps", i, StepSegPos(e, e->duration), steps);
		CHECK(fabs(seg[0].v0 - v0 * steps) < 1e-15, "plan %u: starts at the wrong speed", i);
		CHECK(fabs(vend - v1 * steps) < 1e-12, "plan %u: ends at %.6f mm/s, want %.6f", i, vend / 80 * HZ, v1m);
		CHECK(vmin > -1e-12, "plan %u: goes backwards (%.3g)", i, vmin);
		// in mm/s, mm/s^2: the limits hold (the rounding only lowers them)
		CHECK(vmax / 80 * HZ <= vm * (1 + 1e-6), "plan %u: %.4f mm/s over %.4f", i, vmax / 80 * HZ, vm);
		CHECK(amax / 80 * HZ * HZ <= am * (1 + 1e-6) + 1e-3, "plan %u: %.3f mm/s^2 over %.3f", i, amax / 80 * HZ * HZ, am);
	}
	CHECK(tight * 200 < ok + no, "too many tight refusals: %u of %u", tight, ok + no);
	printf("%-52s %s (%u planned, %u refused, %u of them tight)\n", "v0 -> v1 S-curves: ends, limits, continuity", failures == start_fail ? "PASS" : "FAIL", ok, no, tight);
}

// ---- queue checker: run after every planner call ----
// Per driver: what's playing + what's queued, in order. Pieces of one move must join up; where
// a new move starts (p0 = 0) right after another the speed may only step by JUMP; with a gap
// in between both sides must be at rest. And the newest piece ends at rest (or within JUMP).
static uint32_t checked_joins;

static void CheckQueues(const char *name) {
	uint32_t d, t;
	for (d = 0; d < STEP_AXES; d++) {
		StepAxis *a = &g.axis[d];
		StepSegment list[STEP_QUEUE_LEN + 1];
		uint32_t n = 0, k;
		double tol = JUMP * motion_axes[d < MOTION_AXES ? d : 0].steps_per_mm / HZ * (1 + 1e-6) + 1e-12;
		if (a->active) list[n++] = a->seg;
		for (t = a->tail; t != a->head; t++) list[n++] = a->queue[t & (STEP_QUEUE_LEN - 1)];
		for (k = 1; k < n; k++) {
			const StepSegment *p = &list[k - 1], *q = &list[k];
			double ve = SegV(p, p->duration);
			bool joined = q->start == p->start + p->duration;
			CHECK(q->start >= p->start + p->duration, "%s drv %u: pieces overlap", name, d);
			CHECK(SegVMin(q) > -1e-12, "%s drv %u: piece goes backwards", name, d);
			if (q->p0 != 0) {
				CHECK(joined, "%s drv %u: gap inside a move", name, d);
				CHECK(fabs(StepSegPos(p, p->duration) - q->p0) < 1e-6, "%s drv %u: position jump inside a move (%.9f -> %.9f)", name, d, StepSegPos(p, p->duration), q->p0);
				CHECK(fabs(ve - q->v0) < 1e-10, "%s drv %u: speed jump inside a move", name, d);
				CHECK(fabs(SegA(p, p->duration) - q->a0) < 1e-13, "%s drv %u: accel jump inside a move", name, d);
			} else if (joined) {
				CHECK(fabs(ve - q->v0) <= tol, "%s drv %u: speed steps %.4f mm/s at a junction", name, d, fabs(ve - q->v0) / 80 * HZ);
				if (q->v0 > 0) checked_joins++;
			} else {
				// A gap: rest either side, or an axis with a tiny share of the path (under JUNCTION_DIFF)
				// that starts / stops inside the JUMP allowance
				CHECK(ve <= tol && q->v0 <= tol, "%s drv %u: gap but not at rest (%.4f -> %.4f mm/s)", name, d, ve / 80 * HZ, q->v0 / 80 * HZ);
			}
		}
		if (n > 0) {
			const StepSegment *e = &list[n - 1];
			CHECK(SegV(e, e->duration) <= tol, "%s drv %u: the queue ends at %.4f mm/s, not at rest", name, d, SegV(e, e->duration) / 80 * HZ);
		}
	}
}

static uint8_t Queue(const float *target, uint8_t mask, float feed, float exit, const char *name) {
	uint8_t res;
	while ((res = MotionMoveExit(target, mask, feed, exit)) == MOTION_BUSY) Tick();
	CheckQueues(name);
	return res;
}

// Longest gap between two steps of driver d between positions [from, to) mm on an 80/mm axis
static int64_t Longest(uint32_t d, double from, double to) {
	uint32_t k, a = (uint32_t)lround(from * 80), b = (uint32_t)lround(to * 80);
	int64_t m = 0;
	for (k = a + 1; k < b && k < logs[d].n; k++) {
		if (logs[d].rise[k] - logs[d].rise[k - 1] > m) m = logs[d].rise[k] - logs[d].rise[k - 1];
	}
	return m;
}

// ---- 2: a bed scan row ----
static void TestRow() {
	float t[MOTION_AXES] = { 0 };
	MotionJunctions s;
	int start_fail = failures;
	uint32_t k;

	Setup();
	// fast to just before a spot, 2 mm slow through it, fast to the next ...
	const float xs[] = { 20, 22, 40, 42, 60, 62, 80 };
	const float fs[] = { 150, 10, 150, 10, 150, 10, 150 };
	for (k = 0; k < 7; k++) {
		t[0] = xs[k];
		CHECK(Queue(t, 1, fs[k], k < 6 ? 10 : 0, "row") == MOTION_OK, "row move %u", k);
	}
	RunUntilIdle();
	MotionJunctionStats(&s);
	CHECK(s.asked == 6 && s.full == 6, "row: %u asked, %u full (limited %u late %u corner %u)", s.asked, s.full, s.limited, s.late, s.corner);
	CHECK(StepPosition(0) == 80 * 80, "row ends at %lld steps", (long long)StepPosition(0));
	CHECK(g.axis[0].late == 0, "row: %u late segments", g.axis[0].late);
	// 10 mm/s = 800 steps/s = 1250 ticks a step: through every slow spot, and no stop either side
	for (k = 0; k < 3; k++) {
		int64_t m = Longest(0, xs[2 * k] - 1, xs[2 * k + 1] + 1);
		CHECK(m <= 1252, "row spot %u: a %lld tick gap between steps (10 mm/s = 1250)", k, (long long)m);
	}
	// and the slow part really is slow: step gaps 1250 +-1 from just after the spot starts
	for (k = 0; k < 3; k++) {
		uint32_t a = (uint32_t)(xs[2 * k] * 80) + 2, b = (uint32_t)(xs[2 * k + 1] * 80) - 1, i;
		for (i = a + 1; i < b; i++) {
			int64_t gap = logs[0].rise[i] - logs[0].rise[i - 1];
			if (gap < 1249 || gap > 1251) {
				CHECK(0, "row spot %u: step %u gap %lld, want 1250", k, i, (long long)gap);
				break;
			}
		}
	}
	printf("%-52s %s\n", "bed scan row: fast / 10mm/s spot / fast, no stops", failures == start_fail ? "PASS" : "FAIL");
}

// Same along a diagonal, on X + Y with Y ganged on two drivers like the machine
static void TestDiagonalRow() {
	float t[MOTION_AXES] = { 0 };
	MotionJunctions s;
	int start_fail = failures;
	uint32_t k;

	Setup();
	motion_axes[1].drivers = 1 << 1 | 1 << 7;
	motion_invert = 1 << 7;
	for (k = 1; k <= 5; k++) {
		t[0] = 3 * 4.0f * k;
		t[1] = 4 * 4.0f * k;
		CHECK(Queue(t, 3, (k & 1) ? 120 : 15, k < 5 ? 15 : 0, "diagonal") == MOTION_OK, "diagonal move %u", k);
	}
	RunUntilIdle();
	MotionJunctionStats(&s);
	CHECK(s.full == 4, "diagonal: %u full of %u (limited %u late %u corner %u)", s.full, s.asked, s.limited, s.late, s.corner);
	CHECK(StepPosition(0) == 60 * 80 && StepPosition(1) == 80 * 80 && StepPosition(7) == -80 * 80,
		"diagonal ends at %lld %lld %lld", (long long)StepPosition(0), (long long)StepPosition(1), (long long)StepPosition(7));
	CHECK(logs[1].n == logs[7].n, "diagonal: ganged drivers %u vs %u steps", logs[1].n, logs[7].n);
	for (k = 0; k < logs[1].n && k < logs[7].n; k++) {
		if (logs[1].rise[k] != logs[7].rise[k]) {
			CHECK(0, "diagonal: ganged step %u at %lld vs %lld", k, (long long)logs[1].rise[k], (long long)logs[7].rise[k]);
			break;
		}
	}
	printf("%-52s %s\n", "diagonal row on X + ganged Y, no stops", failures == start_fail ? "PASS" : "FAIL");
}

// ---- 3: the ones that have to stop ----
static void TestFallbacks() {
	float t[MOTION_AXES] = { 0 };
	MotionJunctions s;
	int start_fail = failures;

	// Late: the move after comes once the first has played out
	Setup();
	t[0] = 10;
	Queue(t, 1, 100, 10, "late");
	RunUntilIdle();
	t[0] = 20;
	Queue(t, 1, 100, 0, "late");
	RunUntilIdle();
	MotionJunctionStats(&s);
	CHECK(s.asked == 1 && s.late == 1, "late: asked %u late %u full %u", s.asked, s.late, s.full);
	CHECK(StepPosition(0) == 1600, "late ends at %lld", (long long)StepPosition(0));

	// Late, just: the first move is already decelerating when the next one comes
	Setup();
	t[0] = 10;
	Queue(t, 1, 100, 10, "decel");
	while (now < (int64_t)(MotionLastStart() + MotionLastTime() - 1000)) Tick();
	t[0] = 20;
	Queue(t, 1, 100, 0, "decel");
	RunUntilIdle();
	MotionJunctionStats(&s);
	CHECK(s.late == 1, "decel: late %u full %u", s.late, s.full);
	CHECK(StepPosition(0) == 1600 && g.axis[0].late == 0, "decel ends at %lld, %u late segments", (long long)StepPosition(0), g.axis[0].late);

	// Corner: X then Y
	Setup();
	t[0] = 10;
	t[1] = 0;
	Queue(t, 3, 100, 10, "corner");
	t[1] = 10;
	Queue(t, 3, 100, 0, "corner");
	RunUntilIdle();
	MotionJunctionStats(&s);
	CHECK(s.corner == 1 && s.full == 0, "corner: corner %u full %u", s.corner, s.full);

	// Cut: something other than a G1 comes next (homing / diags moves use MotionMove)
	Setup();
	t[0] = 10;
	t[1] = 0;
	Queue(t, 1, 100, 10, "cut");
	t[0] = 20;
	CHECK(MotionMove(t, 1, 100) == MOTION_OK, "cut move");
	CheckQueues("cut");
	RunUntilIdle();
	MotionJunctionStats(&s);
	CHECK(s.late == 1 && s.full == 0, "cut: late %u full %u", s.late, s.full);

	// No E, no junction: exactly the old rest to rest timing
	Setup();
	t[0] = 10;
	Queue(t, 1, 100, 0, "plain");
	SCurve a1, b1;
	MotionLastPlan(&a1);
	t[0] = 20;
	Queue(t, 1, 100, 0, "plain");
	MotionLastPlan(&b1);
	MotionJunctionStats(&s);
	CHECK(s.asked == 0 && a1.v0 == 0 && b1.v0 == 0, "plain: asked %u, v0 %g %g", s.asked, a1.v0, b1.v0);

	// E bigger than the moves allow: runs at what they can, counted as limited
	Setup();
	t[0] = 10;
	Queue(t, 1, 20, 500, "limited");
	t[0] = 20;
	Queue(t, 1, 20, 0, "limited");
	RunUntilIdle();
	MotionJunctionStats(&s);
	CHECK(s.limited == 1 && s.last_used > 19.9f && s.last_used <= 20.0001f, "limited: limited %u, used %.3f", s.limited, (double)s.last_used);

	// Short: a 2 step move can't stop from 100 mm/s, so the junction comes down to what it can
	Setup();
	t[0] = 10;
	Queue(t, 1, 100, 100, "short");
	t[0] = 10.025f;
	Queue(t, 1, 100, 0, "short");
	RunUntilIdle();
	MotionJunctionStats(&s);
	CHECK(s.limited == 1 && s.last_used < 100, "short: limited %u, used %.3f", s.limited, (double)s.last_used);
	CHECK(StepPosition(0) == 802 && g.axis[0].late == 0, "short ends at %lld", (long long)StepPosition(0));
	printf("%-52s %s\n", "late / decelerating / corner / cut / plain / limited", failures == start_fail ? "PASS" : "FAIL");
}

// ---- 4: the next move comes while the last one is already playing ----
// The last move is swapped from its cruise (next comes during the speed up) or from its decel
// (next comes during the cruise): still no stop, and the counter says where it was swapped from.
static void TestLateSwap() {
	float t[MOTION_AXES] = { 0 };
	MotionJunctions s;
	SCurve sc;
	double u, v;
	uint32_t t3, t4, which;
	uint64_t st;
	int start_fail = failures;

	for (which = 1; which <= 2; which++) {
		Setup();
		t[0] = 40;
		Queue(t, 1, 20, 20, "swap");	// 40 mm at 20 mm/s: a 2 s cruise
		MotionLastPlan(&sc);
		st = MotionLastStart();
		SCurveAt(&sc, 3, &u, &v, &t3);
		SCurveAt(&sc, 4, &u, &v, &t4);
		// 1: just after it started (cruise still ahead), 2: in the middle of the cruise
		int64_t until = which == 1 ? (int64_t)st + 1000 : (int64_t)(st + t3 + (t4 - t3) / 2);
		while (now < until) Tick();
		t[0] = 50;
		Queue(t, 1, 20, 0, "swap");
		RunUntilIdle();
		MotionJunctionStats(&s);
		CHECK(s.full == 1 && s.from_phase[which] == 1, "swap %u: full %u, from %u/%u/%u", which, s.full, s.from_phase[0], s.from_phase[1], s.from_phase[2]);
		CHECK(Longest(0, 30, 45) <= 3752, "swap %u: a %lld tick gap at the junction (20 mm/s = 625)", which, (long long)Longest(0, 30, 45));
		CHECK(StepPosition(0) == 4000 && g.axis[0].late == 0, "swap %u ends at %lld, %u late", which, (long long)StepPosition(0), g.axis[0].late);
	}
	printf("%-52s %s\n", "next move during the speed up / cruise: swapped", failures == start_fail ? "PASS" : "FAIL");
}

// ---- 5: fuzz ----
static void TestFuzz() {
	float t[MOTION_AXES] = { 0 };
	int64_t want[MOTION_AXES] = { 0 };
	uint32_t r = 4242, m, i, k;
	MotionJunctions s;
	int start_fail = failures;
	double dir[3] = { 1, 0, 0 }, pos[3] = { 0, 0, 0 };

	Setup();
	motion_axes[1].drivers = 1 << 1 | 1 << 7;
	motion_invert = 1 << 7;
	checked_joins = 0;
	for (m = 0; m < 3000; m++) {
		// Mostly straight on, sometimes a new direction (some of them almost the same)
		if (Rnd(&r) < 0.12) {
			double th = Rnd(&r) * 6.283, z = Rnd(&r) < 0.3 ? Rnd(&r) - 0.5 : 0;
			if (Rnd(&r) < 0.3) th = atan2(dir[1], dir[0]) + (Rnd(&r) - 0.5) * 0.03;
			dir[0] = cos(th);
			dir[1] = sin(th);
			dir[2] = z;
		}
		double l = Rnd(&r) < 0.3 ? Rnd(&r) * 2 : Rnd(&r) * 40, n2 = sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
		for (i = 0; i < 3; i++) {
			pos[i] += dir[i] / n2 * l;
			if (pos[i] > 200 || pos[i] < -200) {
				dir[i] = -dir[i];
				pos[i] += 2 * dir[i] / n2 * l;
			}
			t[i] = (float)pos[i];
		}
		float feed = 5 + Rnd(&r) * 300, exit = Rnd(&r) < 0.7 ? Rnd(&r) * 200 : 0;
		uint8_t res = Queue(t, 7, feed, exit, "fuzz");
		CHECK(res == MOTION_OK || res == MOTION_NOMOVE, "fuzz move %u: %u", m, res);
		for (i = 0; i < 3; i++) want[i] = llround((double)t[i] * motion_axes[i].steps_per_mm);
		// The generator runs in between: nothing, a bit, or long enough to make the next late
		double w = Rnd(&r);
		k = w < 0.6 ? 0 : w < 0.95 ? (uint32_t)(Rnd(&r) * 40) : 4000;
		while (k--) Tick();
		if (m % 50 == 49) {
			// G1s mixed with other moves now and then
			if (MotionMove(t, 7, 50) == MOTION_OK) CheckQueues("fuzz cut");
		}
	}
	RunUntilIdle();
	MotionJunctionStats(&s);
	for (i = 0; i < 3; i++) CHECK(StepPosition(i) == want[i], "fuzz axis %u at %lld, want %lld", i, (long long)StepPosition(i), (long long)want[i]);
	CHECK(StepPosition(7) == -want[1], "fuzz ganged driver at %lld", (long long)StepPosition(7));
	for (i = 0; i < STEP_AXES; i++) CHECK(g.axis[i].late == 0, "fuzz driver %u: %u late segments", i, g.axis[i].late);
	CHECK(s.asked == s.full + s.limited + s.late + s.corner, "fuzz: counters don't add up");
	CHECK(s.from_phase[0] > 0 && s.from_phase[1] > 0, "fuzz: re-plans from %u / %u / %u", s.from_phase[0], s.from_phase[1], s.from_phase[2]);
	printf("%-52s %s\n", "3000 random G1 E moves, queues checked every move", failures == start_fail ? "PASS" : "FAIL");
	printf("  asked %u: full %u limited %u late %u corner %u, re-planned from start %u cruise %u decel %u, %u speed joins checked\n",
		s.asked, s.full, s.limited, s.late, s.corner, s.from_phase[0], s.from_phase[1], s.from_phase[2], checked_joins);
}

int main(void) {
	setvbuf(stdout, NULL, _IONBF, 0);
	TestPlanV();
	TestRow();
	TestDiagonalRow();
	TestFallbacks();
	TestLateSwap();
	TestFuzz();
	printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
	return failures ? 1 : 0;
}
