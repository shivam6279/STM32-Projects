#ifndef _MOTION_H_
#define _MOTION_H_

#include <stdbool.h>
#include <inttypes.h>
#include "scurve.h"

// Motion planner: point to point moves in mm, broken into S-curve segments for the step
// generator. Moves run one after another on a shared timeline (like G-code lines). All axes in
// a move start and stop together and move in a straight line between the two points.
//
// Axes are what G-code talks about (X, Y, ...). Drivers are the 8 STEP/DIR channels. An axis can
// drive more than one driver (Y = Y1 + Y2 gantry): every driver of an axis gets the same
// segments on the same ticks, so they step in lockstep.
//
// Units are Marlin's, so M92 / M203 / M201 map straight onto the config:
//   steps/mm, mm/s, mm/s^2, mm/s^3 (rotary axes: read mm as degrees).
//
// Positive directions, looking at the front of the machine: X right, Y towards you, Z up.

#define MOTION_AXES		7
#define MOTION_DRIVERS	8
#define MOTION_LINEAR	3	// X Y Z, the first in the table. The rest are rotary (degrees)

typedef struct {
	char name;			// G-code letter
	uint8_t drivers;	// bit per driver this axis moves
	float steps_per_mm;
	float v_max;		// mm/s
	float a_max;		// mm/s^2
	float j_max;		// mm/s^3
} MotionAxis;

extern MotionAxis motion_axes[MOTION_AXES];
extern uint8_t motion_invert;		// bit per driver: flip its DIR pin so + is the axis' + direction
extern float motion_accel;			// path acceleration limit, mm/s^2 (M204), 0 = axis limits only

#define MOTION_OK		0
#define MOTION_BUSY		1	// step queues too full for another move, try again later
#define MOTION_NOMOVE	2	// nothing to do (already there)
#define MOTION_BAD		3	// bad target / feed / config
#define MOTION_FAULT	4	// step generator stopped (E-stop / fault), needs MotionRecover
#define MOTION_JOG		5	// jog mode has the axes (jog.c)

#define MOTION_MAX_STEP_HZ	200000	// per driver, the refill ISR is the limit (pins could do 500k)

// Junction counters (G1 .. E). Every move that asked for an exit speed lands in exactly one of
// full / limited / late / corner once the move after it (or something else) comes.
typedef struct {
	uint32_t asked;			// moves with E > 0 that got decided
	uint32_t full;			// ran into the next move at the E asked for
	uint32_t limited;		// at speed, but slower than E (speed limits, too short), or stopped for that
	uint32_t late;			// stopped: the next G1 came after the last move's decel had started, or no G1 came next
	uint32_t corner;		// stopped: the next move turns (more than ~1 deg)
	uint32_t from_phase[3];	// joins re-planned from the last move's start / cruise / decel
	float last_asked, last_used;	// mm/s, the latest decided junction (used 0 = stopped)
} MotionJunctions;

void MotionInit();
void MotionDefaults();				// axis table + motion_accel back to the built in values (M502)
void MotionSync();					// planned position = where the steps really are, timeline reset
uint8_t MotionMove(const float *target, uint8_t mask, float feed);	// absolute mm, axes in mask
// G1: same, and the next MotionMoveExit can run into it. exit > 0: try to end at that speed (mm/s
// along the path) when the next move carries straight on. It's queued ending at rest either way,
// the end is only swapped once the next move is here. exit < 0 = MotionMove
uint8_t MotionMoveExit(const float *target, uint8_t mask, float feed, float exit);
void MotionJunctionStats(MotionJunctions *s);
void MotionJunctionsReset();
bool MotionIdle();					// all queued motion played out
void MotionLock(bool lock);			// jog.c: MotionMove / MotionSteps return MOTION_JOG while locked
float MotionPosition(uint8_t axis);	// where the planner has sent the axis (mm)
float MotionActual(uint8_t axis);	// where the steps put out so far have it (mm)
int64_t MotionCount(uint8_t axis);	// steps put out, in the axis' direction (its first driver)
void MotionSetPosition(uint8_t axis, float mm);	// G92: say "we're at mm" without moving
uint8_t MotionSteps(uint8_t drv, int32_t steps, float rate);	// raw STEP pulses on one driver, constant rate (steps/s), + = DIR high
int8_t MotionAxisOf(uint8_t drv);	// axis that moves this driver, -1 = none
const char *MotionDriverName(uint8_t drv);	// "Y1", "Y2", "X" ... ("-" if no axis)
uint32_t MotionLastTime();			// ticks the last planned move takes
uint64_t MotionLastStart();			// tick it starts at
void MotionLastPlan(SCurve *s);		// its S-curve (for checking / diags)

#endif
