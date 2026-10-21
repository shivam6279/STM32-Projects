#ifndef _HOME_H_
#define _HOME_H_

#include <stdbool.h>
#include <inttypes.h>
#include "motion.h"

// Sensorless homing: each axis runs into its hard stop and the TMC2209s' StallGuard says when
// (tmc.h). First pre mm away from the stop (it may start right on it), then a velocity jog
// towards the stop; once at speed, StallGuard goes on and the DIAG
// interrupt halts each driver the moment it stalls. A gantry (Y1 + Y2) keeps going until both
// sides are on the stop, so it squares itself. Then off the stop by backoff and the position
// is set. Axes go one at a time in HOME_ORDER: Z first so the head is up before X / Y move.
// Not as repeatable as a switch, and it moves with speed, current and friction: fine with the
// camera doing the real positioning.
//
// An axis only homes once dir, sgthrs and travel are set (home set, X / Y / Z have defaults). Tuning: home sg.

typedef struct {
	int8_t dir;			// end it homes to: -1 = the - end, +1 = the + end, 0 = doesn't home
	float speed;		// mm/s into the stop. StallGuard needs speed: too slow and it can't tell
	uint8_t sgthrs;		// StallGuard threshold 1-255, stall when SG_RESULT <= 2 * sgthrs. 0 = not tuned
	float current;		// A RMS while homing, 0 = the normal run current
	float travel;		// mm, furthest it goes looking for the stop. 0 = not set
	float backoff;		// mm off the stop afterwards
	float pos;			// mm, position of the stop itself
	float pre;			// mm away from the stop before homing, room to get up to speed when it starts close
} HomeAxis;

extern HomeAxis home_axes[MOTION_AXES];

#define HOME_OK			0
#define HOME_BUSY		1	// homing already running
#define HOME_NOT_SET	2	// an axis has no dir / sgthrs / travel
#define HOME_MOTION		3	// jog mode on, or motion still queued
#define HOME_DRIVERS	4	// drivers not ready, or a driver is in SpreadCycle (StallGuard needs StealthChop)
#define HOME_NO_STALL	5	// the whole travel without a stall
#define HOME_FAULT		6	// step generator stopped (fault / E-stop) or a driver fault
#define HOME_ABORTED	7

#define HOME_ORDER		"ZYX"	// axes not in it go after, in table order
#define HOME_ARM_FRAC	0.5f	// StallGuard on above this fraction of the homing speed
#define HOME_SETTLE_MS	100		// after the speed ramp, before StallGuard goes on

uint8_t HomeStart(uint8_t mask);	// axes in mask. HOME_OK = started, HomeTask does the rest
void HomeTask();					// main loop
void HomeAbort();					// ramps down, nothing gets homed
bool HomeBusy();
uint8_t HomeResult();				// of the last run, HOME_BUSY while running
bool HomeReady(uint8_t axis);		// dir, sgthrs and travel set
uint8_t HomeReadyMask();
bool HomeHomed(uint8_t axis);		// homed since boot
const char *HomeError(uint8_t res);

#endif
