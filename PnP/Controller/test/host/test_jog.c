// Host test for jog.c: JogTick every JOG_PERIOD ticks, the fill run in STEP_HALF chunks like the
// DMA interrupt, every BSRR word replayed onto a simulated port. Checks speed, ramps, reversals,
// the target timeout, ganged drivers and that the queue never runs dry.
//
// Build/run: make -C test/host jog
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "jog.h"
#include "motion.h"
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

// ---- stand-ins for the hardware ----
static StepGen g;
bool StepQueue(uint32_t axis, const StepSegment *seg) { return StepFillQueue(&g, axis, seg); }
uint32_t StepQueueFree(uint32_t axis) { return StepFillQueueFree(&g, axis); }
bool StepAxisIdle(uint32_t axis) { return StepFillAxisIdle(&g, axis); }
uint64_t StepNow() { return g.tick; }
int64_t StepPosition(uint32_t axis) { return g.axis[axis].position; }
bool StepRunning() { return true; }
uint32_t Millis() { return (uint32_t)(g.tick / 1000); }

// ---- simulated port ----
typedef struct {
	int dir;
	int64_t pos;			// from the replayed edges
	int64_t last_rise;
	int64_t last_dir_change;
	uint32_t steps;			// edges in the current window
	uint32_t reversals;
} Pin;

static Pin pins[STEP_AXES];
static int64_t tick;
static uint32_t step_mask[STEP_AXES], dir_mask[STEP_AXES];
static uint64_t next_jog;

static void Replay(const uint32_t *buf, uint32_t n) {
	uint32_t i, a;
	for (i = 0; i < n; i++, tick++) {
		for (a = 0; a < STEP_AXES; a++) {
			Pin *p = &pins[a];
			if (buf[i] & dir_mask[a] && !p->dir) {
				p->dir = 1;
				p->last_dir_change = tick;
				p->reversals++;
			}
			if (buf[i] & (dir_mask[a] << 16) && p->dir) {
				p->dir = 0;
				p->last_dir_change = tick;
				p->reversals++;
			}
			if (buf[i] & step_mask[a]) {
				CHECK(tick - p->last_rise >= 2, "axis %u: steps %lld ticks apart", a, (long long)(tick - p->last_rise));
				CHECK(tick - p->last_dir_change >= DIR_SETUP, "axis %u: step %lld ticks after DIR", a, (long long)(tick - p->last_dir_change));
				p->last_rise = tick;
				p->pos += p->dir ? 1 : -1;
				p->steps++;
			}
		}
	}
}

// Run for ms, JogTick on the 10ms grid like TIM7, the fill in half buffers like the DMA
static void Run(uint32_t ms) {
	uint32_t buf[HALF];
	uint64_t end = g.tick + (uint64_t)ms * 1000;
	while (g.tick < end) {
		if (g.tick >= next_jog) {
			JogTick();
			next_jog += JOG_PERIOD;
		}
		StepFillWords(&g, buf, HALF);
		Replay(buf, HALF);
	}
}

// Keep setting the target (like PadTask does from the main loop) while running
static void RunTarget(const float *v, uint32_t ms) {
	uint32_t t;
	for (t = 0; t < ms; t += 10) {
		JogSet(v);
		Run(10);
	}
}

static uint64_t window_start;

// Y2's raw step count given Y1's: opposite if its DIR is inverted (motion_invert)
static int64_t Y2(int64_t y1) {
	return (motion_invert >> 1) & 1 ? -y1 : y1;
}

static void WindowReset() {
	uint32_t i;
	for (i = 0; i < STEP_AXES; i++) pins[i].steps = 0;
	window_start = g.tick;
}

// Steps mm_s should give over the window (Run goes in half buffers, so not exactly the ms asked)
static double WindowWant(float mm_s) {
	return fabs(mm_s) * 80 * (g.tick - window_start) / 1e6;
}

static void Setup() {
	uint32_t i;
	memset(pins, 0, sizeof(pins));
	for (i = 0; i < STEP_AXES; i++) {
		step_mask[i] = 1u << i;
		dir_mask[i] = 1u << (i + 8);
		pins[i].last_rise = -100;
		pins[i].last_dir_change = -100;
	}
	StepFillInit(&g, step_mask, dir_mask, DIR_SETUP);
	MotionInit(); // X driver 2, Y drivers 0 + 1, Z driver 3, 80 steps/mm, 300mm/s, 3000mm/s^2
	tick = 0;
	next_jog = 0;
}

// Steady speed, then stop with JogStop: speed right, stops, axes handed back, positions line up
static void TestSteady() {
	float v[MOTION_AXES] = { 50, 0, 0 };	// X 50mm/s = 4000 steps/s
	uint64_t t0, t1;
	Setup();
	t0 = g.tick;
	CHECK(JogStart() == JOG_OK, "start");
	CHECK(MotionMove(v, 1, 10) == MOTION_JOG, "MotionMove while jogging");
	RunTarget(v, 500);
	WindowReset();
	RunTarget(v, 100);
	CHECK(fabs(pins[2].steps - WindowWant(50)) <= 2, "X 50mm/s: %u steps, want %.1f", pins[2].steps, WindowWant(50));
	CHECK(pins[0].steps == 0 && pins[3].steps == 0, "only X moves");
	t1 = g.tick;
	JogStop();
	Run(200);
	JogTask();
	CHECK(!JogActive(), "handed back after the stop");
	CHECK(MotionIdle(), "idle after the stop");
	CHECK(pins[2].pos == StepPosition(2), "replayed %lld, fill %lld", (long long)pins[2].pos, (long long)StepPosition(2));
	CHECK(fabsf(MotionPosition(0) - pins[2].pos / 80.0f) < 1e-3f, "M114 X %.3f after sync, steps say %.3f",
		(double)MotionPosition(0), pins[2].pos / 80.0);
	// The ramps cancel out (0.42mm short on the way up, 0.42 over on the way down), the stop lands
	// up to JOG_AHEAD + one period after JogStop: 50mm/s * (t1 - t0 + 0..30ms)
	{
		double run = (t1 - t0) / 1e6, mm = pins[2].pos / 80.0;
		CHECK(mm >= 50 * run - 0.05 && mm <= 50 * (run + 0.03) + 0.05, "X went %.2f mm in %.3f s at 50mm/s", mm, run);
	}
	CHECK(JogLate() == 0, "queue ran dry %u times", JogLate());
}

// Full reversal mid run: direction flips once, speed the other way is right
static void TestReverse() {
	float fwd[MOTION_AXES] = { 0, 0, 20 }, back[MOTION_AXES] = { 0, 0, -20 };
	Setup();
	JogStart();
	RunTarget(fwd, 300);
	RunTarget(back, 300);
	WindowReset();
	RunTarget(back, 100);
	CHECK(fabs(pins[3].steps - WindowWant(20)) <= 2, "Z -20mm/s: %u steps, want %.1f", pins[3].steps, WindowWant(20));
	CHECK(pins[3].dir == 0, "Z going down");
	CHECK(pins[3].reversals == 2, "Z DIR changed %u times, want 2 (up, then down)", pins[3].reversals);
	JogStop();
	Run(200);
	JogTask();
	CHECK(!JogActive() && pins[3].pos == StepPosition(3), "Z back to rest");
	CHECK(JogLate() == 0, "queue ran dry %u times", JogLate());
}

// Nobody sets the target any more (main loop stuck, pad gone): ramps down and stops by itself
static void TestTimeout() {
	float v[MOTION_AXES] = { 0, 100, 0 };	// Y, ganged on drivers 0 + 1
	int64_t p;
	Setup();
	JogStart();
	RunTarget(v, 300);
	Run(JOG_TIMEOUT_MS + 100); // ramp is 33ms from 100mm/s
	p = pins[0].pos;
	WindowReset();
	Run(100);
	CHECK(pins[0].steps == 0, "Y still moving %u steps/100ms after the timeout", pins[0].steps);
	CHECK(pins[1].pos == Y2(pins[0].pos) && p == pins[0].pos, "Y1 %lld, Y2 %lld", (long long)pins[0].pos, (long long)pins[1].pos);
	CHECK(JogActive(), "timeout only stops, jog stays on");
	// Target back: moves again
	WindowReset();
	RunTarget(v, 100);
	CHECK(pins[0].steps > 500, "moving again after the timeout: %u", pins[0].steps);
	JogStop();
	Run(200);
	JogTask();
	CHECK(!JogActive(), "stopped");
}

// Stick wiggling around: random targets every 10ms, everything stays legal and lines up at the end
static void TestFuzz() {
	float v[MOTION_AXES] = { 0 };
	uint32_t s = 12345, i, k;
	Setup();
	JogStart();
	for (i = 0; i < 300; i++) {
		for (k = 0; k < 3; k++) {
			s = s * 1103515245 + 12345;
			v[k] = ((int)((s >> 16) % 401) - 200) * (k == 2 ? 0.1f : 1.0f);
			if ((s >> 8) % 5 == 0) v[k] = 0;
		}
		RunTarget(v, 10 + (s >> 4) % 40);
	}
	JogStop();
	Run(300);
	JogTask();
	CHECK(!JogActive() && MotionIdle(), "fuzz: stopped and handed back");
	for (k = 0; k < 4; k++) CHECK(pins[k].pos == StepPosition(k), "fuzz driver %u: replayed %lld, fill %lld", k, (long long)pins[k].pos, (long long)StepPosition(k));
	CHECK(pins[1].pos == Y2(pins[0].pos), "fuzz: Y1 %lld, Y2 %lld", (long long)pins[0].pos, (long long)pins[1].pos);
	CHECK(JogLate() == 0, "fuzz: queue ran dry %u times", JogLate());
	// Not checked: the fill's own late count. A reversal right after a step at the end of a half
	// buffer puts DIR (and so the segment) into the next one. The run starts from rest, first step
	// ~3ms in, so it costs nothing.
}

int main(void) {
	TestSteady();
	TestReverse();
	TestTimeout();
	TestFuzz();
	if (failures) {
		printf("%d failures\n", failures);
		return 1;
	}
	printf("jog: all passed\n");
	return 0;
}
