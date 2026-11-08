#include "probe.h"
#include "probe_detect.h"
#include "adc.h"
#include "clock.h"
#include "home.h"
#include "jog.h"
#include "motion.h"
#include "mpu.h"
#include "pad.h"
#include "pwm.h"
#include "stepgen.h"

#include <math.h>
#include <stdio.h>

#define ST_IDLE		0
#define ST_BLANK	1	// waiting out the blanking time
#define ST_REF		2	// taking the reference
#define ST_MOVE		3	// moving, watching
#define ST_HALT		4	// stopped dead, waiting for the queue to drop

#define HIST_LEN		4096	// rolling reference history, ~17 s at 239 Hz (power of 2)
#define ROLL_MAX_MS		5000
#define STALL_MS		100		// no fresh sample for this long while it should be sampling = error

ProbeConfig probe_config = {
	.feed = 2,			// F120
	.ch = 0,
	.drop = 5,			// N1 / N2 measured: -31 / -8 kPa free, below -45 / -42 sealed
	.need = 3,
	.avg = 4,
	.blank_ms = 1000,
	.ref_ms = 200,
	.roll_ms = 1000,	// free air drifts ~0.4 kPa over 10 s after the pump comes on
	.min_vac = 2,
	.rise = 0,
	.timeout_ms = 0,
	.z_lo = 0,
	.z_hi = 0,
};

static DetSample hist[HIST_LEN] RAM_D1;
static Detector det;
static ProbeConfig cfg;
static uint8_t state = ST_IDLE;
static uint8_t zax;				// Z's index in motion_axes
static uint8_t drv;				// its drivers
static float target;
static bool target_error;		// G38.2
static uint32_t t_start, t_ref, t_move, t_end, timeout, pwm_at_start;
static uint32_t seq_next, total, moving_n, first_ms, last_ms, last_new;
static double ref_sum;
static uint32_t ref_n;
static float ref, ref_lo, ref_hi, last_x;
static float z_hit;
static bool have_z, have_ref, moved;
static char result;				// '1' trigger, '0' target, 'E' error
static const char *why = "";
static char report[200];

static void Done(char res, const char *w) {
	result = res;
	why = w;
	t_end = Millis();
	state = ST_IDLE;
}

// Stop Z now (it drops its queue at the next refill), finish once it has
static void Halt(char res, const char *w) {
	result = res;
	why = w;
	t_end = Millis();
	StepHalt(drv);
	state = ST_HALT;
}

static bool Fail(const char *w) {
	why = w;
	return false;
}

static uint8_t Refuse(const char *w, uint8_t code) {
	why = w;
	return code;
}

bool ProbeCheck(const ProbeConfig *c) {
	if (c->feed <= 0 || c->feed > PROBE_FEED_MAX) return Fail("F out of range (0-1500 mm/min)");
	if (c->ch >= ADC_CHANNELS) return Fail("P is 0 or 1");
	if (c->drop <= 0) return Fail("S must be > 0");
	if (c->need < 1 || c->need > 100) return Fail("N is 1-100");
	if (c->avg < 1 || c->avg > DET_AVG_MAX) return Fail("A is 1-32");
	if (c->ref_ms < 10) return Fail("R must be >= 10 ms");
	if (c->roll_ms > ROLL_MAX_MS || (c->roll_ms > 0 && c->roll_ms <= DET_GAP_MS)) return Fail("W is 0 or 51-5000 ms");
	if (c->min_vac < 0 || c->rise < 0) return Fail("V and U can't be negative");
	return true;
}

uint8_t ProbeStart(float tgt, const ProbeConfig *c, bool tgt_error) {
	uint8_t i;
	float dist;

	if (state != ST_IDLE) return Refuse("already probing", PROBE_BUSY);
	if (!ProbeCheck(c)) return PROBE_BAD;
	if (c->z_lo < c->z_hi && (tgt < c->z_lo || tgt > c->z_hi)) return Refuse("target outside L..H", PROBE_BAD);
	if (!StepRunning()) return Refuse("stopped (M999)", PROBE_MOTION);
	if (JogActive() || HomeBusy() || PadMode() != PAD_MODE_OFF) return Refuse("jog, pad mode or homing on", PROBE_MOTION);
	if (!MotionIdle()) return Refuse("motion still queued", PROBE_MOTION);
	zax = MOTION_AXES;
	for (i = 0; i < MOTION_AXES; i++) {
		if (motion_axes[i].name == 'Z') zax = i;
	}
	if (zax == MOTION_AXES || motion_axes[zax].drivers == 0) return Refuse("no Z axis", PROBE_BAD);
	dist = fabsf(tgt - MotionPosition(zax));
	if (dist * motion_axes[zax].steps_per_mm < 1) return Refuse("already at the target", PROBE_BAD);

	cfg = *c;
	drv = motion_axes[zax].drivers;
	target = tgt;
	target_error = tgt_error;
	t_start = Millis();
	pwm_at_start = PwmLastChange();
	timeout = cfg.timeout_ms ? cfg.timeout_ms : (uint32_t)(dist / cfg.feed * 1500) + cfg.blank_ms + cfg.ref_ms + 2000;
	total = 0;
	moving_n = 0;
	have_z = false;
	have_ref = false;
	moved = false;
	ref = 0;
	last_x = 0;
	result = 'E';
	why = "";
	AdcStart(); // and left running
	state = ST_BLANK;
	return PROBE_OK;
}

// Next fresh sample in kPa, false if none yet. A gap (the main loop stalled for seconds) is skipped
static bool Next(float *x, uint32_t *ms) {
	uint16_t raw;
	uint32_t now = AdcSeq();

	if (seq_next >= now) return false;
	if (now - seq_next >= ADC_RING - 2) seq_next = now - 1;
	if (!AdcSample(seq_next, cfg.ch, &raw, ms)) {
		seq_next = AdcSeq() - 1;
		return false;
	}
	seq_next++;
	*x = PressKpaRaw(raw);
	if (total == 0) first_ms = *ms;
	last_ms = *ms;
	last_new = Millis();
	total++;
	last_x = *x;
	return true;
}

static void Blank() {
	uint32_t now = Millis();

	if (now - t_start < cfg.blank_ms || now - PwmLastChange() < cfg.blank_ms) return;
	DetInit(&det, cfg.avg, cfg.need, cfg.drop, cfg.rise, cfg.roll_ms, hist, HIST_LEN);
	seq_next = AdcSeq();
	ref_sum = 0;
	ref_n = 0;
	ref_lo = 1e9f;
	ref_hi = -1e9f;
	t_ref = now;
	last_new = now;
	pwm_at_start = PwmLastChange();
	state = ST_REF;
}

static void Ref() {
	float x, f, tgt[MOTION_AXES];
	uint32_t ms;
	uint8_t i, res;

	if (PwmLastChange() != pwm_at_start) {
		state = ST_BLANK; // pump / valve changed: blank again from that change
		return;
	}
	while (Next(&x, &ms)) {
		DetFeed(&det, x, ms);
		ref_sum += x;
		ref_n++;
		if (ref_n >= cfg.avg) {
			f = DetFiltered(&det);
			if (f < ref_lo) ref_lo = f;
			if (f > ref_hi) ref_hi = f;
		}
	}
	if (Millis() - last_new > STALL_MS) {
		Done('E', "no samples from the ADC");
		return;
	}
	if (Millis() - t_ref < cfg.ref_ms || ref_n < cfg.avg || ref_n < 3) return;

	ref = (float)(ref_sum / ref_n);
	have_ref = true;
	if (cfg.min_vac > 0 && ref > -cfg.min_vac) {
		Done('E', "no vacuum (pump or valve off?)");
		return;
	}
	if (ref_hi - ref_lo > cfg.drop) {
		Done('E', "unsteady: moved by more than S while still");
		return;
	}
	DetArm(&det, ref);
	for (i = 0; i < MOTION_AXES; i++) tgt[i] = MotionPosition(i);
	tgt[zax] = target;
	res = MotionMove(tgt, 1 << zax, cfg.feed);
	if (res != MOTION_OK) {
		Done('E', res == MOTION_FAULT ? "stopped (M999)" : "move refused");
		return;
	}
	t_move = Millis();
	moved = true;
	state = ST_MOVE;
}

static void Move() {
	float x;
	uint32_t ms;
	uint8_t r;

	if (!StepRunning()) {
		Done('E', "stopped by a fault / M112");
		return;
	}
	while (Next(&x, &ms)) {
		moving_n++;
		r = DetFeed(&det, x, ms);
		if (r == DET_TRIGGER) {
			z_hit = MotionActual(zax); // step count when the N-th sample over came
			have_z = true;
			Halt('1', "trigger");
			return;
		}
		if (r == DET_LOST) {
			z_hit = MotionActual(zax);
			have_z = true;
			Halt('E', "vacuum lost (rose by more than U)");
			return;
		}
	}
	if (PwmLastChange() != pwm_at_start) {
		Halt('E', "pump / valve changed while probing");
		return;
	}
	if (Millis() - last_new > STALL_MS) {
		Halt('E', "no samples from the ADC");
		return;
	}
	if (Millis() - t_start > timeout) {
		Halt('E', "timeout");
		return;
	}
	if (MotionIdle()) {
		z_hit = MotionActual(zax);
		have_z = true;
		if (target_error) Done('E', "reached the target without a trigger");
		else Done('0', "target");
	}
}

static void Report() {
	int n = 0;
	float hz = last_ms > first_ms && total > 1 ? (total - 1) * 1000.0f / (last_ms - first_ms) : 0;

	n += snprintf(report + n, sizeof(report) - n, "[PRB:");
	if (have_z) n += snprintf(report + n, sizeof(report) - n, "Z%.3f:", (double)z_hit);
	n += snprintf(report + n, sizeof(report) - n, "%c", result);
	if (have_ref) {
		n += snprintf(report + n, sizeof(report) - n, " ref:%.2f at:%.2f filt:%.2f min:%.2f", (double)(det.armed ? det.ref : ref), (double)last_x,
			(double)DetFiltered(&det), (double)(det.armed ? det.min : DetFiltered(&det)));
	}
	n += snprintf(report + n, sizeof(report) - n, " n:%" PRIu32 " t:%" PRIu32 " hz:%.0f stop:%.3f samples:%" PRIu32 " why:%s]",
		moving_n, t_end - (moved ? t_move : t_start), (double)hz,
		zax < MOTION_AXES ? (double)MotionPosition(zax) : 0.0, total, why);
}

void ProbeTask() {
	uint8_t was = state;

	if ((state == ST_BLANK || state == ST_REF) && Millis() - t_start > timeout) {
		Done('E', "timeout (before moving: pump / valve kept changing?)");
	}
	switch (state) {
		case ST_BLANK:
			Blank();
			break;
		case ST_REF:
			Ref();
			break;
		case ST_MOVE:
			Move();
			break;
		case ST_HALT:
			if (!StepRunning()) {
				StepRelease(drv);
				state = ST_IDLE;
			} else if (MotionIdle()) {
				StepRelease(drv);
				MotionSync(); // the planner takes the real count, it stays here
				state = ST_IDLE;
			}
			break;
	}
	if (was != ST_IDLE && state == ST_IDLE) Report();
}

void ProbeAbort() {
	if (state == ST_IDLE) return;
	if (state == ST_MOVE && StepRunning()) {
		Halt('E', "aborted");
		return;
	}
	if (state == ST_HALT) {
		why = "aborted";
		result = 'E';
		return;
	}
	StepRelease(drv);
	Done('E', "aborted");
	Report();
}

bool ProbeBusy() {
	return state != ST_IDLE;
}

const char *ProbeWhy() {
	return why;
}

char ProbeResult() {
	return result;
}

const char *ProbeReport() {
	return report;
}
