#ifndef _PROBE_H_
#define _PROBE_H_

#include <stdbool.h>
#include <inttypes.h>

// Vacuum probe (G38.2 / G38.3): Z toward a target with the nozzle's pump + valve on, stop when
// the tip seals on a surface, seen as a vacuum drop on a pressure sensor.
//
// 1. waits for queued motion to finish (G-code side)
// 2. blanking: D ms after the start and after the last pump / valve change, nothing measured
// 3. reference: R ms of samples, their mean. Has to be V kPa of vacuum at least (pump + valve
//    on), and steady: moving by more than S inside it is an error
// 4. one plain Z move to the target at F. Every fresh pair from the ADC (adc.c ring, 239 Hz)
//    goes through the detector (probe_detect.h): filtered (A), against the fixed or rolling (W)
//    reference, N in a row over S = trigger. Z halts dead (StepHalt), the planner takes the real
//    step count, and it stays there
// 5. the target reached without a trigger: G38.3 result 0, G38.2 error. Timeout T, M112, a step
//    fault, a pump / valve change or the vacuum going (U) end it with an error
// The ADC is started and left running (M800 is instant after).

typedef struct {
	float feed;			// mm/s (G-code F is mm/min)
	uint8_t ch;			// pressure channel 0 / 1 (PRESS0 / PRESS1)
	float drop;			// S: kPa below the reference to count
	uint8_t need;		// N: fresh samples in a row over it
	uint8_t avg;		// A: moving average length, 1 = none
	uint16_t blank_ms;	// D
	uint16_t ref_ms;	// R
	uint16_t roll_ms;	// W: 0 = fixed reference
	float min_vac;		// V: the reference has to be at least this far below 0 kPa, 0 = no check
	float rise;			// U: kPa above the reference = vacuum lost, 0 = off
	uint32_t timeout_ms;	// T: 0 = |distance| / feed * 1.5 + D + R + 2 s
	float z_lo, z_hi;	// L / H: targets outside are refused. lo >= hi = no check
} ProbeConfig;

extern ProbeConfig probe_config;	// the defaults, M821 / saved with M500

#define PROBE_FEED_MAX	25.0f	// mm/s: it stops dead on a trigger, like homing does

#define PROBE_OK		0
#define PROBE_BUSY		1
#define PROBE_BAD		2		// bad argument / target out of L..H (why says which)
#define PROBE_MOTION	3		// jog mode, homing, not running

uint8_t ProbeStart(float target, const ProbeConfig *c, bool target_is_error);	// G38.2: true
void ProbeTask();					// main loop
void ProbeAbort();					// M112: ends it with an error at once
bool ProbeBusy();
bool ProbeCheck(const ProbeConfig *c);	// settings in range? ProbeWhy says what isn't
char ProbeResult();					// last probe: '1' trigger, '0' target (G38.3), 'E' error
const char *ProbeWhy();				// why the last start was refused
const char *ProbeReport();			// the [PRB:...] line of the last probe

#endif
