#include "home.h"
#include "board.h"
#include "clock.h"
#include "jog.h"
#include "stepgen.h"
#include "tmc.h"

#include <math.h>
#include <stdio.h>

#define ST_IDLE		0
#define ST_START	1	// next axis: currents, StallGuard threshold, move off the stop first
#define ST_PRE		2	// moving pre mm away, so it gets up to speed before the stop
#define ST_SEEK		3	// jogging into the stop
#define ST_STOP		4	// jog ramping down / played out
#define ST_BACKOFF	5	// position set, moving off the stop

// X / Y / Z tuned on the machine (2026-10-19), travel = axis length + margin. The rest don't home
HomeAxis home_axes[MOTION_AXES] = {
	{ -1, 25, 75, 0, 220, 75, 0, 5 },	// X
	{ -1, 25, 50, 0, 220, 75, 0, 5 },	// Y
	{ 1, 25, 50, 0, 120, 29.6f, 0, 5 },	// Z
	{ 0, 25, 0, 0, 0, 3, 0, 5 },	// A
	{ 0, 25, 0, 0, 0, 3, 0, 5 },	// B
	{ 0, 25, 0, 0, 0, 3, 0, 5 },	// C
	{ 0, 25, 0, 0, 0, 3, 0, 5 },	// U
};

static uint8_t state = ST_IDLE;
static uint8_t result = HOME_OK;
static uint8_t todo;			// axes still to home
static uint8_t ax;				// axis being homed
static uint8_t drv;				// its drivers, ignored ones left out
static bool armed;
static uint8_t fail;			// HOME_OK while it's going right
static uint32_t t0, arm_ms;
static int64_t start[MOTION_DRIVERS];
static uint8_t homed;

static uint8_t Drivers(uint8_t a) {
	return motion_axes[a].drivers & ~TmcIgnored();
}

static float Travel(uint8_t d) {
	int64_t s = StepPosition(d) - start[d];
	return (s < 0 ? -s : s) / motion_axes[ax].steps_per_mm;
}

// HOME_ORDER first (Z up before anything moves under it), then the rest in table order
static uint8_t Next() {
	const char *c;
	uint8_t i;
	for (c = HOME_ORDER; *c; c++) {
		for (i = 0; i < MOTION_AXES; i++) {
			if ((todo >> i) & 1 && motion_axes[i].name == *c) return i;
		}
	}
	for (i = 0; i < MOTION_AXES; i++) {
		if ((todo >> i) & 1) return i;
	}
	return 0;
}

// StallGuard off, currents back. Safe to call more than once
static void Restore() {
	uint8_t d;
	TmcStallDisarm();
	for (d = 0; d < MOTION_DRIVERS; d++) {
		if (!((drv >> d) & 1)) continue;
		TmcStallSetup(d, 0, 0);
		TmcRunCurrent(d, 0);
	}
}

static void Finish(uint8_t res) {
	Restore();
	StepRelease(motion_axes[ax].drivers);
	if (res != HOME_OK) printf("Error:%c homing: %s\n", motion_axes[ax].name, HomeError(res));
	todo = 0;
	result = res;
	state = ST_IDLE;
}

// Jog into the stop, travel counted from here
static void SeekStart() {
	HomeAxis *h = &home_axes[ax];
	uint8_t d;

	for (d = 0; d < MOTION_DRIVERS; d++) {
		if ((drv >> d) & 1) start[d] = StepPosition(d);
	}
	if (JogStart() != JOG_OK) {
		Finish(HOME_MOTION);
		return;
	}
	t0 = Millis();
	arm_ms = (uint32_t)(h->speed / motion_axes[ax].a_max * 1000) + HOME_SETTLE_MS;
	state = ST_SEEK;
}

static void Begin() {
	HomeAxis *h;
	float target[MOTION_AXES];
	uint8_t d, i, res;

	ax = Next();
	h = &home_axes[ax];
	drv = Drivers(ax);
	armed = false;
	fail = HOME_OK;
	if (drv == 0) {
		printf("Error:%c: all its drivers are ignored\n", motion_axes[ax].name);
		Finish(HOME_DRIVERS);
		return;
	}
	// Threshold in, but TCOOLTHRS 0 (off) until the axis is at speed: accelerating is load too
	for (d = 0; d < MOTION_DRIVERS; d++) {
		if (!((drv >> d) & 1)) continue;
		if (TmcRunCurrent(d, h->current) != TMC_OK || TmcStallSetup(d, h->sgthrs, 0) != TMC_OK) {
			printf("Error:Driver %d didn't take the homing settings\n", d);
			Finish(HOME_DRIVERS);
			return;
		}
	}
	printf("echo:Homing %c, %s end, %.1f mm/s\n", motion_axes[ax].name, h->dir < 0 ? "-" : "+", (double)h->speed);
	if (h->pre <= 0) {
		SeekStart();
		return;
	}
	// Already on (or right next to) the stop: it would hit it before StallGuard is on
	for (i = 0; i < MOTION_AXES; i++) target[i] = MotionPosition(i);
	target[ax] -= h->dir * h->pre;
	res = MotionMove(target, 1 << ax, h->speed);
	if (res != MOTION_OK && res != MOTION_NOMOVE) {
		Finish(res == MOTION_FAULT ? HOME_FAULT : HOME_MOTION);
		return;
	}
	state = ST_PRE;
}

static void Seek() {
	HomeAxis *h = &home_axes[ax];
	float v[MOTION_AXES] = { 0 }, hz;
	uint8_t d, st;

	if (!StepRunning()) {
		fail = HOME_FAULT;
		state = ST_STOP;
		return;
	}
	v[ax] = h->dir * h->speed;
	JogSet(v);

	if (!armed && Millis() - t0 >= arm_ms) {
		// Arm the interrupt first, so the first stall DIAG already counts as a stall
		TmcStallArm(drv);
		hz = h->speed * motion_axes[ax].steps_per_mm * HOME_ARM_FRAC;
		for (d = 0; d < MOTION_DRIVERS; d++) {
			if ((drv >> d) & 1) TmcStallSetup(d, h->sgthrs, TmcTcoolFor(d, hz));
		}
		armed = true;
	}

	st = TmcStalled() & drv;
	if (armed && st == drv) {
		JogStop(); // every driver is halted already, this just ends the jog
		state = ST_STOP;
		return;
	}
	for (d = 0; d < MOTION_DRIVERS; d++) {
		if ((drv >> d) & 1 && !((st >> d) & 1) && Travel(d) > h->travel) {
			fail = HOME_NO_STALL;
			JogStop(); // the ones still moving ramp down
			state = ST_STOP;
			return;
		}
	}
}

static void Stop() {
	HomeAxis *h = &home_axes[ax];
	float target[MOTION_AXES], lo = 1e9f, hi = 0, t, err = 0;
	uint8_t d, i, res;
	bool again = (homed >> ax) & 1;

	if (!StepRunning()) {
		// Queues stay as they are until step recover, so the jog can't play out. It lets go then
		Finish(fail == HOME_ABORTED ? HOME_ABORTED : HOME_FAULT);
		return;
	}
	JogTask();
	if (JogActive()) return; // still ramping down / playing out, then MotionSync

	Restore();
	StepRelease(motion_axes[ax].drivers);
	if (TmcStallCheck(drv)) {
		Finish(HOME_FAULT);
		return;
	}
	if (fail != HOME_OK) {
		Finish(fail);
		return;
	}

	// Ganged drivers: how far apart they stopped = how far out of square it was
	for (d = 0; d < MOTION_DRIVERS; d++) {
		if (!((drv >> d) & 1)) continue;
		t = Travel(d);
		if (t < lo) lo = t;
		if (t > hi) hi = t;
	}
	if (again) err = MotionActual(ax) - h->pos; // where it thought it was vs where the stop is
	MotionSetPosition(ax, h->pos);
	homed |= 1 << ax;

	printf("echo:%c on the stop after %.2f mm", motion_axes[ax].name, (double)hi);
	if (drv & (drv - 1)) printf(", drivers %.3f mm apart", (double)(hi - lo));
	if (again) printf(", %+.3f mm off the last home", (double)err);
	printf("\n");

	for (i = 0; i < MOTION_AXES; i++) target[i] = MotionPosition(i);
	target[ax] = h->pos - h->dir * h->backoff;
	res = MotionMove(target, 1 << ax, h->speed);
	if (res != MOTION_OK && res != MOTION_NOMOVE) {
		Finish(HOME_FAULT);
		return;
	}
	state = ST_BACKOFF;
}

void HomeTask() {
	switch (state) {
		case ST_START:
			Begin();
			break;
		case ST_PRE:
			if (!StepRunning()) {
				Finish(HOME_FAULT);
			} else if (MotionIdle()) {
				if (fail != HOME_OK) Finish(fail); // aborted on the way
				else SeekStart();
			}
			break;
		case ST_SEEK:
			Seek();
			break;
		case ST_STOP:
			Stop();
			break;
		case ST_BACKOFF:
			if (!StepRunning()) {
				Finish(HOME_FAULT);
			} else if (MotionIdle()) {
				todo &= ~(1 << ax);
				if (todo) {
					state = ST_START;
				} else {
					result = fail; // HOME_ABORTED if it was stopped during the backoff
					state = ST_IDLE;
				}
			}
			break;
	}
}

uint8_t HomeStart(uint8_t mask) {
	uint8_t i, d;

	if (state != ST_IDLE) return HOME_BUSY;
	mask &= (1 << MOTION_AXES) - 1;
	if (mask == 0) return HOME_OK;
	if (!StepRunning()) return HOME_FAULT;
	if (JogActive() || !MotionIdle()) return HOME_MOTION;
	for (i = 0; i < MOTION_AXES; i++) {
		if (!((mask >> i) & 1)) continue;
		if (!HomeReady(i)) {
			printf("Error:%c isn't set up for homing (dir, sgthrs, travel: home set)\n", motion_axes[i].name);
			return HOME_NOT_SET;
		}
		for (d = 0; d < MOTION_DRIVERS; d++) {
			if ((Drivers(i) >> d) & 1 && tmc_config[d].spread) {
				printf("Error:Driver %d is in SpreadCycle, StallGuard needs StealthChop (tmc set %d mode stealth)\n", d, d);
				return HOME_DRIVERS;
			}
		}
	}
	// Dry run: nothing moves, so nothing stalls either. It runs out the travel (a dry test)
	if (!BoardStepperEnabled() && !StepDry()) {
		if (!TmcReady()) return HOME_DRIVERS;
		BoardStepperEnable(true);
	}
	todo = mask;
	result = HOME_BUSY;
	state = ST_START;
	return HOME_OK;
}

void HomeAbort() {
	if (state == ST_IDLE) return;
	if (state == ST_START) {
		todo = 0;
		result = HOME_ABORTED;
		state = ST_IDLE;
		return;
	}
	if (state == ST_SEEK) {
		JogStop();
		state = ST_STOP;
	}
	fail = HOME_ABORTED;
	todo = 1 << ax; // a backoff already going still finishes, nothing after it
}

bool HomeBusy() {
	return state != ST_IDLE;
}

uint8_t HomeResult() {
	return result;
}

bool HomeReady(uint8_t a) {
	const HomeAxis *h;
	if (a >= MOTION_AXES) return false;
	h = &home_axes[a];
	return h->dir != 0 && h->sgthrs > 0 && h->travel > 0 && h->speed > 0 && Drivers(a);
}

uint8_t HomeReadyMask() {
	uint8_t i, m = 0;
	for (i = 0; i < MOTION_AXES; i++) {
		if (HomeReady(i)) m |= 1 << i;
	}
	return m;
}

bool HomeHomed(uint8_t a) {
	return (homed >> a) & 1;
}

const char *HomeError(uint8_t res) {
	switch (res) {
		case HOME_OK:		return "ok";
		case HOME_BUSY:		return "already homing";
		case HOME_NOT_SET:	return "not set up (home set)";
		case HOME_MOTION:	return "jog mode on or motion queued";
		case HOME_DRIVERS:	return "drivers not ready";
		case HOME_NO_STALL:	return "no stall within the travel (sgthrs too low, or nothing in the way)";
		case HOME_FAULT:	return "stopped by a fault";
		case HOME_ABORTED:	return "aborted";
	}
	return "?";
}
