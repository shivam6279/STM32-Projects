// Host test for the motion stack: motion.c (planner) -> scurve.c -> stepgen_fill.c (edges).
// The real files are compiled for the PC; the hardware side of stepgen is replaced by a few
// stand-ins below. Every BSRR word is replayed onto a simulated port and each step edge is
// checked against an independent evaluation of the S-curve (bisection, no shared code).
//
// Build/run: make -C test/host run
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

// ---- stand-ins for the hardware half of stepgen.c ----
static StepGen g;
bool StepQueue(uint32_t axis, const StepSegment *seg) { return StepFillQueue(&g, axis, seg); }
uint32_t StepQueueFree(uint32_t axis) { return StepFillQueueFree(&g, axis); }
bool StepAxisIdle(uint32_t axis) { return StepFillAxisIdle(&g, axis); }
uint64_t StepNow() { return g.tick; }
int64_t StepPosition(uint32_t axis) { return g.axis[axis].position; }
bool StepRunning() { return true; }

// ---- simulated port + recorded edges ----
typedef struct {
	int64_t *rise;		// tick of every rising edge
	int *dir;			// DIR level at that edge
	uint32_t n, cap;
	int64_t last_dir_change;
	int step_level, dir_level;
	int64_t last_rise;
} AxisLog;

static AxisLog logs[STEP_AXES];
static uint32_t odr;
static int64_t now;
static uint32_t step_mask[STEP_AXES], dir_mask[STEP_AXES];

static void LogReset() {
	uint32_t i;
	for (i = 0; i < STEP_AXES; i++) {
		free(logs[i].rise);
		free(logs[i].dir);
		memset(&logs[i], 0, sizeof(logs[i]));
		logs[i].last_dir_change = -1000;
		logs[i].last_rise = -1000;
	}
}

static void Replay(const uint32_t *buf, uint32_t n) {
	uint32_t i, ax;
	for (i = 0; i < n; i++, now++) {
		odr = (odr & ~(buf[i] >> 16)) | (buf[i] & 0xFFFF); // BSRR: set wins
		for (ax = 0; ax < STEP_AXES; ax++) {
			AxisLog *L = &logs[ax];
			int step = (odr & step_mask[ax]) != 0, dir = (odr & dir_mask[ax]) != 0;
			if (dir != L->dir_level) {
				CHECK(now - L->last_rise >= 2, "axis %u: DIR changed %lld ticks after a step", ax, (long long)(now - L->last_rise));
				L->dir_level = dir;
				L->last_dir_change = now;
			}
			if (step && !L->step_level) {
				CHECK(now - L->last_rise >= 2, "axis %u: steps %lld ticks apart", ax, (long long)(now - L->last_rise));
				CHECK(now - L->last_dir_change >= DIR_SETUP, "axis %u: DIR setup %lld", ax, (long long)(now - L->last_dir_change));
				if (L->n == L->cap) {
					L->cap = L->cap ? L->cap * 2 : 1024;
					L->rise = realloc(L->rise, L->cap * sizeof(int64_t));
					L->dir = realloc(L->dir, L->cap * sizeof(int));
				}
				L->rise[L->n] = now;
				L->dir[L->n] = dir;
				L->n++;
				L->last_rise = now;
			}
			if (!step && L->step_level) CHECK(now - L->last_rise == 1, "axis %u: STEP high %lld ticks", ax, (long long)(now - L->last_rise));
			L->step_level = step;
		}
	}
}

static void Tick() {
	uint32_t buf[HALF];
	StepFillWords(&g, buf, HALF);
	Replay(buf, HALF);
}

static void RunUntilIdle() {
	uint32_t i, idle = 0;
	while (idle < 3) {
		Tick();
		idle = 1;
		for (i = 0; i < STEP_AXES; i++) if (!StepFillAxisIdle(&g, i)) idle = 0;
		idle = idle ? idle + 1 : 0;
		if (idle) idle = 3;
	}
	Tick();
	Tick();
}

static void Setup() {
	uint32_t i;
	for (i = 0; i < STEP_AXES; i++) {
		step_mask[i] = 1u << i;
		dir_mask[i] = 1u << (i + 8);
	}
	StepFillInit(&g, step_mask, dir_mask, DIR_SETUP);
	MotionInit();
	// Tests drive axis n on driver n, nothing ganged, no inverts (TestGanged sets its own)
	for (i = 0; i < MOTION_AXES; i++) motion_axes[i].drivers = 1 << i;
	motion_invert = 0;
	odr = 0;
	now = 0;
	LogReset();
}

// ---- independent S-curve: u(t) by walking the phases, then bisection for each step ----
static double RefU(const SCurve *s, double t) {
	const int js[7] = { 1, 0, -1, 0, -1, 0, 1 };
	double dur[7] = { s->tj, s->ta, s->tj, s->tv, s->tj, s->ta, s->tj };
	double p = 0, v = 0, a = 0, d;
	int i;
	for (i = 0; i < 7; i++) {
		double jj = js[i] * s->j;
		d = t < dur[i] ? t : dur[i];
		if (d > 0) {
			double np = p + d * v + d * d * a / 2 + d * d * d * jj / 6;
			if (t <= dur[i]) return np;
			p = np;
			v += d * a + d * d * jj / 2;
			a += d * jj;
		}
		t -= dur[i];
	}
	return p;
}

static double RefStepTime(const SCurve *s, double target) {
	double lo = 0, hi = s->total;
	int i;
	for (i = 0; i < 80; i++) {
		double m = (lo + hi) / 2;
		if (RefU(s, m) < target) lo = m;
		else hi = m;
	}
	return hi;
}

// Check the edges logged for one move on one axis against the reference
static void CheckMove(const char *name, uint32_t ax, uint32_t from, uint32_t steps, uint64_t start, const SCurve *s, int dir) {
	uint32_t k, worst = 0;
	CHECK(logs[ax].n - from == steps, "%s axis %u: %u steps, want %u", name, ax, logs[ax].n - from, steps);
	for (k = 1; k <= steps && from + k - 1 < logs[ax].n; k++) {
		double t = RefStepTime(s, (double)k / steps);
		int64_t want = (int64_t)start + (int64_t)floor(t + 0.5);
		int64_t got = logs[ax].rise[from + k - 1];
		uint32_t err = (uint32_t)llabs(got - want);
		if (err > worst) worst = err;
		CHECK(logs[ax].dir[from + k - 1] == dir, "%s axis %u step %u: DIR %d", name, ax, k, logs[ax].dir[from + k - 1]);
	}
	CHECK(worst <= 1, "%s axis %u: step time off by %u ticks", name, ax, worst);
}

static void TestConstant() {
	StepSegment s;
	uint32_t i, k, period;
	int64_t first;

	for (period = 2; period <= 7; period += 5) {
		Setup();
		// 1e6 steps as 100 back to back constant rate segments, no gaps
		for (i = 0; i < 100; i++) {
			StepSegConst(&s, 1000 + (uint64_t)i * 10000 * period, 10000, period, true);
			while (!StepFillQueue(&g, 0, &s)) Tick();
		}
		RunUntilIdle();
		CHECK(logs[0].n == 1000000, "constant period %u: %u steps", period, logs[0].n);
		first = logs[0].rise[0];
		for (k = 1; k < logs[0].n; k++) {
			if (logs[0].rise[k] - logs[0].rise[k - 1] != period) {
				CHECK(0, "constant period %u: gap %lld at step %u", period, (long long)(logs[0].rise[k] - logs[0].rise[k - 1]), k);
				break;
			}
		}
		CHECK(first == 1000 + period, "constant: first step at %lld", (long long)first);
		printf("%-48s %s\n", period == 2 ? "1e6 steps, period 2, 100 segments" : "1e6 steps, period 7, 100 segments", failures ? "FAIL" : "PASS");
	}
}

static void TestSingle() {
	const uint32_t sizes[] = { 1, 2, 3, 10, 100, 1000, 10000, 123457 };
	float target[MOTION_AXES] = { 0 };
	uint32_t i, from;
	SCurve sc;
	int start_fail = failures;

	for (i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
		Setup();
		target[0] = sizes[i] / motion_axes[0].steps_per_mm;
		from = logs[0].n;
		CHECK(MotionMove(target, 1, 200) == MOTION_OK, "move %u", sizes[i]);
		MotionLastPlan(&sc);
		uint64_t st = MotionLastStart();
		RunUntilIdle();
		CheckMove("single", 0, from, sizes[i], st, &sc, 1);
	}
	printf("%-48s %s\n", "single axis S-curves, 1..123457 steps", failures == start_fail ? "PASS" : "FAIL");
}

static void TestDiagonal() {
	float target[MOTION_AXES] = { 125, 41.6625f, 0.0875f, 0, 0, 0, 0 };	// 10000, 3333, 7 steps
	uint32_t want[3] = { 10000, 3333, 7 }, i;
	SCurve sc;
	int start_fail = failures;

	Setup();
	CHECK(MotionMove(target, 0x07, 250) == MOTION_OK, "diagonal move");
	MotionLastPlan(&sc);
	uint64_t st = MotionLastStart();
	RunUntilIdle();
	for (i = 0; i < 3; i++) CheckMove("diagonal", i, 0, want[i], st, &sc, 1);
	// Same profile, so every axis finishes inside the last tick of the move
	for (i = 0; i < 3; i++) {
		int64_t last = logs[i].rise[logs[i].n - 1];
		CHECK(last <= (int64_t)(st + sc.total) + 1 && last >= (int64_t)(st + sc.total) - 1, "diagonal axis %u ends at %lld, move ends %llu", i, (long long)last, (unsigned long long)(st + sc.total));
	}
	printf("%-48s %s\n", "3 axis diagonal, all on one S-curve", failures == start_fail ? "PASS" : "FAIL");
}

static void TestBackAndForth() {
	float target[MOTION_AXES] = { 0 };
	SCurve sc;
	uint32_t i, from;
	int start_fail = failures;

	Setup();
	for (i = 0; i < 6; i++) {
		target[0] = (i & 1) ? 0 : 50;	// 4000 steps out, 4000 back
		from = logs[0].n;
		CHECK(MotionMove(target, 1, 300) == MOTION_OK, "move %u", i);
		MotionLastPlan(&sc);
		uint64_t st = MotionLastStart();
		RunUntilIdle();
		CheckMove("back and forth", 0, from, 4000, st, &sc, !(i & 1));
	}
	CHECK(StepPosition(0) == 0, "back and forth ends at %lld", (long long)StepPosition(0));
	printf("%-48s %s\n", "reversals, DIR setup/hold, back to 0", failures == start_fail ? "PASS" : "FAIL");
}

// Full speed one way, then straight back with no pause: DIR hold/setup has to push steps out
static void TestInstantReversal() {
	StepSegment s;
	uint64_t t = 1000;
	uint32_t i;
	int start_fail = failures;

	Setup();
	for (i = 0; i < 20; i++) {
		StepSegConst(&s, t, 50, 2, (i & 1) == 0);	// 50 steps at period 2, alternating DIR
		while (!StepFillQueue(&g, 0, &s)) Tick();
		t += s.duration;
	}
	RunUntilIdle();
	CHECK(logs[0].n == 1000, "instant reversal: %u steps", logs[0].n);
	CHECK(StepPosition(0) == 0, "instant reversal ends at %lld", (long long)StepPosition(0));
	printf("%-48s %s\n", "instant reversals at full speed (DIR timing)", failures == start_fail ? "PASS" : "FAIL");
}

static uint32_t Rand(uint32_t *s) {
	*s ^= *s << 13;
	*s ^= *s >> 17;
	*s ^= *s << 5;
	return *s;
}

static void TestFuzz() {
	float target[MOTION_AXES];
	int64_t want[MOTION_AXES] = { 0 };
	uint32_t r = 12345, m, i, mask;
	uint8_t res;
	int start_fail = failures;

	Setup();
	for (m = 0; m < 300; m++) {
		mask = Rand(&r) & ((1 << MOTION_AXES) - 1);
		if (!mask) mask = 1;
		for (i = 0; i < MOTION_AXES; i++) {
			int32_t st = (int32_t)(Rand(&r) % 8001) - 4000;	// -4000..4000 steps
			target[i] = st / motion_axes[i].steps_per_mm;
			if ((mask >> i) & 1) want[i] = st;
		}
		// Queue as fast as the planner allows, the generator runs in between
		while ((res = MotionMove(target, mask, 50 + Rand(&r) % 400)) == MOTION_BUSY) Tick();
		CHECK(res == MOTION_OK || res == MOTION_NOMOVE, "fuzz move %u: %u", m, res);
	}
	RunUntilIdle();
	for (i = 0; i < MOTION_AXES; i++) {
		CHECK(StepPosition(i) == want[i], "fuzz axis %u at %lld, want %lld", i, (long long)StepPosition(i), (long long)want[i]);
		CHECK(g.axis[i].late == 0, "fuzz axis %u: %u late segments", i, g.axis[i].late);
	}
	printf("%-48s %s\n", "300 random moves on 7 axes, exact positions", failures == start_fail ? "PASS" : "FAIL");
}

// One axis on two drivers, the second one reversed (Y1 + Y2 facing each other): same step times,
// opposite DIR, opposite counts
static void TestGanged() {
	float target[MOTION_AXES] = { 0 };
	uint32_t k, n;
	int start_fail = failures;

	Setup();
	motion_axes[1].drivers = 1 << 1 | 1 << 7;
	motion_invert = 1 << 7;
	target[1] = 5000 / motion_axes[1].steps_per_mm;
	CHECK(MotionMove(target, 1 << 1, 200) == MOTION_OK, "ganged move");
	RunUntilIdle();
	target[1] = 1234 / motion_axes[1].steps_per_mm;
	CHECK(MotionMove(target, 1 << 1, 200) == MOTION_OK, "ganged move back");
	RunUntilIdle();
	CHECK(StepPosition(1) == 1234, "ganged: driver 1 at %lld", (long long)StepPosition(1));
	CHECK(StepPosition(7) == -1234, "ganged: driver 7 at %lld", (long long)StepPosition(7));
	CHECK(MotionCount(1) == 1234, "ganged: axis count %lld", (long long)MotionCount(1));
	n = logs[1].n;
	CHECK(n == logs[7].n && n == 5000 + 3766, "ganged: %u and %u steps", logs[1].n, logs[7].n);
	for (k = 0; k < n && k < logs[7].n; k++) {
		if (logs[1].rise[k] != logs[7].rise[k]) {
			CHECK(0, "ganged: step %u at %lld vs %lld", k, (long long)logs[1].rise[k], (long long)logs[7].rise[k]);
			break;
		}
	}
	printf("%-48s %s\n", "ganged axis, second driver reversed, lockstep", failures == start_fail ? "PASS" : "FAIL");
}

int main(void) {
	TestConstant();
	TestSingle();
	TestDiagonal();
	TestBackAndForth();
	TestInstantReversal();
	TestFuzz();
	TestGanged();
	printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
	return failures ? 1 : 0;
}
