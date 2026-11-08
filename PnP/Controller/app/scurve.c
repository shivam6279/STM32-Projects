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
	s->tj2 = s->tj;
	s->ta2 = s->ta;
	s->j2 = s->j;
	s->v0 = 0;
	s->v1 = 0;
	return true;
}

// ---- v0 -> v1 ----
// One side (accel or decel) is a jerk limited speed change dv: times tj, ta like above, and since
// the speed curve is symmetric about its middle it covers (v_from + v_to) / 2 * (2*tj + ta).

static double Side(double dv, double a, double j, double *tj, double *ta) {
	if (dv <= 0) {
		*tj = 0;
		*ta = 0;
	} else if (dv * j >= a * a) {
		*tj = a / j;
		*ta = dv / a - *tj;
	} else {
		*tj = sqrt(dv / j);
		*ta = 0;
	}
	return 2 * *tj + *ta;
}

// A side with jerk time *tj, accel time ta changing the speed by dv: inside a and j? If not, *tj
// becomes the shortest that is (at least one more) and true comes back.
static bool Longer(uint32_t *tj, uint32_t ta, double dv, double a, double j) {
	double t = (double)*tj, need;
	if (dv / (t * (t + ta)) <= j * (1 + 1e-6) && dv / (t + ta) <= a * (1 + 1e-6)) return false;
	need = dv / a - ta;												// tj + ta >= dv / a
	t = (-(double)ta + sqrt((double)ta * ta + 4 * dv / j)) / 2;	// tj * (tj + ta) >= dv / j
	if (t > need) need = t;
	t = ceil(need);
	*tj = t > *tj + 1.0 ? (uint32_t)t : *tj + 1;
	return true;
}

// Distance with peak speed V (>= v0, v1) and no cruise
static double Dist(double v0, double v1, double V, double a, double j) {
	double tj, ta;
	return (v0 + V) / 2 * Side(V - v0, a, j, &tj, &ta) + (V + v1) / 2 * Side(V - v1, a, j, &tj, &ta);
}

bool SCurvePlanV(SCurve *s, double v0, double v1, double v, double a, double j) {
	double V, Vr, lo, vf, hi, d, tj1, ta1, tj2, ta2, tv, T1, T2;
	uint32_t i, min1 = 1, min2 = 1;	// tj >= 1 on both sides so each side's jerk can take up the rounding

	if (v0 == 0 && v1 == 0) return SCurvePlan(s, v, a, j);
	if (v <= 0 || a <= 0 || j <= 0 || v0 < 0 || v1 < 0) return false;
	if (v0 > v * (1 + 1e-9) || v1 > v * (1 + 1e-9)) return false;
	lo = v0 > v1 ? v0 : v1;
	if (lo > v) lo = v;
	vf = lo; // the peak never goes under this
	if (Dist(v0, v1, lo, a, j) > 1) return false; // can't even get from v0 to v1 in the distance

	// Peak speed: v if there's room to cruise, else the one that uses the whole distance
	d = Dist(v0, v1, v, a, j);
	if (d <= 1) {
		V = v;
	} else {
		hi = v;
		for (i = 0; i < 60; i++) {
			V = (lo + hi) / 2;
			if (Dist(v0, v1, V, a, j) > 1) hi = V;
			else lo = V;
		}
		V = lo;
	}
	s->v0 = v0;
	s->v1 = v1;

	// Whole ticks. v0 and v1 stay exact (the moves either side are planned with them), so the
	// rounding has to come out of the peak speed: 1 = v0*T1/2 + v1*T2/2 + Vr*(T1/2 + tv + T2/2).
	// The cruise is picked so Vr stays within a tick's worth of V, but with v0 / v1 well above 0
	// even that can be a dip under them, and a short side makes the dip a big accel. Such a side
	// gets a minimum length (out of the cruise), and with no cruise to take it from, V comes down
	// towards max(v0, v1) to make one. A move too short for its speed never fits: can't be done.
	for (i = 0; i < 80; i++) {
		Side(V - v0, a, j, &tj1, &ta1);
		Side(V - v1, a, j, &tj2, &ta2);
		s->tj = CeilTicks(tj1);
		s->ta = CeilTicks(ta1);
		s->tj2 = CeilTicks(tj2);
		s->ta2 = CeilTicks(ta2);
		if (s->tj < min1) s->tj = min1;
		if (s->tj2 < min2) s->tj2 = min2;
		T1 = 2.0 * s->tj + s->ta;
		T2 = 2.0 * s->tj2 + s->ta2;

		// Cruise that keeps the peak at V
		tv = (1 - v0 * T1 / 2 - v1 * T2 / 2) / V - T1 / 2 - T2 / 2;
		if (tv < 0) {
			if (V - vf <= 1e-9 * V) return false; // the sides alone are too long already
			V = vf + (V - vf) / 2;
			continue;
		}
		s->tv = (uint32_t)(tv + 0.5);
		Vr = (1 - v0 * T1 / 2 - v1 * T2 / 2) / (T1 / 2 + s->tv + T2 / 2);
		if (Vr > v * (1 + 1e-9)) {
			s->tv++;
			Vr = (1 - v0 * T1 / 2 - v1 * T2 / 2) / (T1 / 2 + s->tv + T2 / 2);
		}
		if (Vr < 0) return false;
		s->v = Vr;
		s->j = (Vr - v0) / ((double)s->tj * (s->tj + s->ta));
		s->j2 = (Vr - v1) / ((double)s->tj2 * (s->tj2 + s->ta2));
		s->a = s->j * s->tj;
		s->total = 2 * s->tj + s->ta + s->tv + 2 * s->tj2 + s->ta2;
		bool long1 = Longer(&s->tj, s->ta, fabs(Vr - v0), a, j);
		bool long2 = Longer(&s->tj2, s->ta2, fabs(Vr - v1), a, j);
		if (!long1 && !long2) return true;
		if (long1) min1 = s->tj;
		if (long2) min2 = s->tj2;
	}
	return false;
}

// ---- segments ----

static const int8_t jerk_sign[SCURVE_PHASES] = { 1, 0, -1, 0, -1, 0, 1 };

static void Phases(const SCurve *s, uint32_t *dur, double *jerk) {
	uint32_t i;

	dur[0] = s->tj;
	dur[1] = s->ta;
	dur[2] = s->tj;
	dur[3] = s->tv;
	dur[4] = s->tj2;
	dur[5] = s->ta2;
	dur[6] = s->tj2;
	for (i = 0; i < SCURVE_PHASES; i++) jerk[i] = jerk_sign[i] * (i < 4 ? s->j : s->j2);
}

uint32_t SCurveSegmentsPart(const SCurve *s, double p0, double steps, bool dir, uint64_t start, StepSegment *out) {
	uint32_t dur[SCURVE_PHASES];
	double jerk[SCURVE_PHASES], p = 0, v = s->v0, a = 0, jj, t;
	uint32_t i, n = 0;

	// Walk the 7 phases on the normalized path, scale each by the step count.
	Phases(s, dur, jerk);
	for (i = 0; i < SCURVE_PHASES; i++) {
		jj = jerk[i];
		if (dur[i] > 0) {
			out[n].start = start;
			out[n].duration = dur[i];
			out[n].dir = dir;
			out[n].p0 = p0 + p * steps;
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

uint32_t SCurveSegments(const SCurve *s, uint32_t steps, bool dir, uint64_t start, StepSegment *out) {
	return SCurveSegmentsPart(s, 0, steps, dir, start, out);
}

void SCurveAt(const SCurve *s, uint32_t phase, double *u, double *v, uint32_t *t) {
	uint32_t dur[SCURVE_PHASES], i;
	double jerk[SCURVE_PHASES], p = 0, vv = s->v0, a = 0, d;

	Phases(s, dur, jerk);
	*t = 0;
	for (i = 0; i < phase && i < SCURVE_PHASES; i++) {
		d = dur[i];
		p += d * (vv + d * (a / 2 + d * jerk[i] / 6));
		vv += d * (a + d * jerk[i] / 2);
		a += d * jerk[i];
		*t += dur[i];
	}
	*u = p;
	*v = vv;
}

uint32_t SCurvePhasesFrom(const SCurve *s, uint32_t phase) {
	uint32_t dur[SCURVE_PHASES], i, n = 0;
	double jerk[SCURVE_PHASES];

	Phases(s, dur, jerk);
	for (i = phase; i < SCURVE_PHASES; i++) {
		if (dur[i] > 0) n++;
	}
	return n;
}
