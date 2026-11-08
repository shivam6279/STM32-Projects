#ifndef _SCURVE_H_
#define _SCURVE_H_

#include <stdbool.h>
#include <inttypes.h>
#include "stepgen_fill.h"

// Jerk limited (S-curve) profile for one move: from rest to rest, or from a start speed v0 to
// an end speed v1 (moves that run into each other, G1 .. E).
//
// The move is planned on a normalized path u: u = 0 at the start, u = 1 at the end. Every
// axis in the move follows the same u(t), scaled by its own step count, so all axes start
// and stop together and stay on the straight line between the two points.
//
// 7 phases, constant jerk in each:
//   0 jerk up    (+J)   accel rises 0 -> A, speed from v0
//   1 accel      ( 0)   accel stays A
//   2 jerk down  (-J)   accel falls A -> 0, now at cruise speed V
//   3 cruise     ( 0)
//   4 jerk down  (-J2)  decel starts
//   5 decel      ( 0)
//   6 jerk up    (+J2)  decel ends at v1 (0 = stopped) at u = 1
// Phases 0/2 last tj, 1 ta, 3 tv, 4/6 tj2, 5 ta2. Rest to rest: tj2 = tj, ta2 = ta, J2 = J.
// Accel is 0 at the start of phases 0, 3 and 4 (and at the end): a move can be re-planned from
// any of those. All times are whole ticks, so the profile is exact on the tick grid.

#define SCURVE_PHASES 7

typedef struct {
	uint32_t tj, ta, tv;	// phase durations, ticks
	uint32_t tj2, ta2;		// decel side
	double j, j2;			// jerk actually used, normalized (u/tick^3), <= the limit
	double a, v;			// resulting peak accel / cruise velocity, normalized
	double v0, v1;			// start / end velocity, normalized (0 for rest to rest)
	uint32_t total;			// whole move, ticks
} SCurve;

// Limits in normalized units: v in u/tick, a in u/tick^2, j in u/tick^3 (distance is always 1).
bool SCurvePlan(SCurve *s, double v, double a, double j);

// From v0 to v1 (both <= v). False if it can't: v0 or v1 over v, or the move is too short to
// get from v0 to v1 inside a and j. v0 = v1 = 0 is exactly SCurvePlan.
bool SCurvePlanV(SCurve *s, double v0, double v1, double v, double a, double j);

// Segments for one axis that moves 'steps' (> 0) steps in this move, starting at tick 'start'.
// Returns how many were written to out[SCURVE_PHASES] (phases that last 0 ticks are skipped).
uint32_t SCurveSegments(const SCurve *s, uint32_t steps, bool dir, uint64_t start, StepSegment *out);

// Same for a plan that covers part of a move: positions p0 + u * steps (steps need not be whole).
uint32_t SCurveSegmentsPart(const SCurve *s, double p0, double steps, bool dir, uint64_t start, StepSegment *out);

// State at the start of phase 'phase' (0..SCURVE_PHASES): u, velocity, ticks from the start
void SCurveAt(const SCurve *s, uint32_t phase, double *u, double *v, uint32_t *t);

// Segments with a non zero duration from phase 'phase' on
uint32_t SCurvePhasesFrom(const SCurve *s, uint32_t phase);

#endif
