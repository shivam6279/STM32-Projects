#include "scurve.h"
#include <math.h>

// For a profile from rest to rest with phase times tj, ta, tv and jerk j:
//   peak accel     A = j*tj
//   cruise speed   V = A*(tj + ta)
//   distance       D = V*(2*tj + ta + tv)   (accel and decel each cover V*(2tj+ta)/2)
// With the times fixed, D is just j times a number, so after rounding the times up to whole
// ticks, j is recomputed to make D exactly 1. Rounding times up only ever lowers j, A and V,
// so the limits still hold.

static uint32_t CeilTicks(double t) {
	double c = ceil(t - 1e-9);
	return c < 0 ? 0 : (uint32_t)c;
}

bool SCurvePlan(SCurve *s, double v, double a, double j) {
	double tj, ta, tv, k;

	if (v <= 0 || a <= 0 || j <= 0) return false;

	// Ideal (non whole) times. First as if the move is long enough to reach full speed.
	if (v * j >= a * a) {
		tj = a / j;			// accel reaches the limit
		ta = v / a - tj;
	} else {
		tj = sqrt(v / j);	// speed limit comes first, accel never reaches a
		ta = 0;
	}
	tv = 1 / v - (2 * tj + ta);	// time left at cruise speed

	if (tv < 0) {
		// Too short to reach v: find the lower peak speed that uses the whole distance, no cruise.
		tv = 0;
		tj = a / j;
		// With full accel: V*(tj + V/a) = 1 -> V^2/a + V*tj - 1 = 0
		double vp = (-tj + sqrt(tj * tj + 4 / a)) * a / 2;
		ta = vp / a - tj;
		if (ta < 0) {
			// Doesn't even reach full accel: V = j*tj^2 and 2*V*tj = 1 -> tj = (1/(2j))^(1/3)
			ta = 0;
			tj = cbrt(1 / (2 * j));
		}
	}

	s->tj = CeilTicks(tj);
	s->ta = CeilTicks(ta);
	s->tv = CeilTicks(tv);
	if (s->tj == 0) s->tj = 1;

	k = (double)s->tj * (s->tj + s->ta) * (2.0 * s->tj + s->ta + s->tv);
	s->j = 1 / k;
	s->a = s->j * s->tj;
	s->v = s->a * (s->tj + s->ta);
	s->total = 4 * s->tj + 2 * s->ta + s->tv;
	return true;
}

uint32_t SCurveSegments(const SCurve *s, uint32_t steps, bool dir, uint64_t start, StepSegment *out) {
	static const int8_t jerk_sign[SCURVE_PHASES] = { 1, 0, -1, 0, -1, 0, 1 };
	uint32_t dur[SCURVE_PHASES] = { s->tj, s->ta, s->tj, s->tv, s->tj, s->ta, s->tj };
	double p = 0, v = 0, a = 0, jj, t;
	uint32_t i, n = 0;

	// Walk the 7 phases on the normalized path, scale each by the step count.
	for (i = 0; i < SCURVE_PHASES; i++) {
		jj = jerk_sign[i] * s->j;
		if (dur[i] > 0) {
			out[n].start = start;
			out[n].duration = dur[i];
			out[n].dir = dir;
			out[n].p0 = p * steps;
			out[n].v0 = v * steps;
			out[n].a0 = a * steps;
			out[n].j = jj * steps;
			n++;
		}
		t = dur[i];
		p += t * (v + t * (a / 2 + t * jj / 6));
		v += t * (a + t * jj / 2);
		a += t * jj;
		start += dur[i];
	}
	return n;
}
