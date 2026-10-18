#ifndef _SCURVE_H_
#define _SCURVE_H_

#include <stdbool.h>
#include <inttypes.h>
#include "stepgen_fill.h"

// Jerk limited (S-curve) profile for one move, from rest to rest.
//
// The move is planned on a normalized path u: u = 0 at the start, u = 1 at the end. Every
// axis in the move follows the same u(t), scaled by its own step count, so all axes start
// and stop together and stay on the straight line between the two points.
//
// 7 phases, constant jerk in each:
//   0 jerk up    (+J)   accel rises 0 -> A
//   1 accel      ( 0)   accel stays A
//   2 jerk down  (-J)   accel falls A -> 0, now at cruise speed V
//   3 cruise     ( 0)
//   4 jerk down  (-J)   decel starts
//   5 decel      ( 0)
//   6 jerk up    (+J)   decel ends, stopped at u = 1
// Phases 0/2/4/6 last tj, 1/5 last ta, 3 lasts tv. Short moves get ta = 0 and/or tv = 0.
// All times are whole ticks, so the profile is exact on the tick grid.

#define SCURVE_PHASES 7

typedef struct {
	uint32_t tj, ta, tv;	// phase durations, ticks
	double j;				// jerk actually used, normalized (u/tick^3), <= the limit
	double a, v;			// resulting peak accel / velocity, normalized
	uint32_t total;			// whole move, ticks
} SCurve;

// Limits in normalized units: v in u/tick, a in u/tick^2, j in u/tick^3 (distance is always 1).
bool SCurvePlan(SCurve *s, double v, double a, double j);

// Segments for one axis that moves 'steps' (> 0) steps in this move, starting at tick 'start'.
// Returns how many were written to out[SCURVE_PHASES] (phases that last 0 ticks are skipped).
uint32_t SCurveSegments(const SCurve *s, uint32_t steps, bool dir, uint64_t start, StepSegment *out);

#endif
