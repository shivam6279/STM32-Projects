#ifndef _STEPGEN_FILL_H_
#define _STEPGEN_FILL_H_

#include <stdbool.h>
#include <inttypes.h>

// Turns per-axis motion segments into GPIO BSRR words, one word per tick (1 tick = 1us).
// No register access so it builds on the host too (test/host).
//
// A segment is a piece of motion with constant jerk. Inside it, the position (in steps,
// counted from the start of the current move) is
//     p(t) = p0 + v0*t + a0*t^2/2 + j*t^3/6        t = ticks since 'start'
// A step edge is put down every time p(t) crosses a whole number: step 1 when p = 1,
// step 2 when p = 2, ... The exact crossing time is solved for (Newton) and rounded
// to the nearest tick. So velocity, acceleration and jerk are all just "how p moves".
//
// Rules for whoever builds segments (the planner):
//   - p(t) never goes down inside a move (v >= 0). Direction is the 'dir' flag.
//   - Segments of one move follow each other, p0 = where the previous one ended.
//   - A new move starts at p0 = 0, and should end on a whole number of steps.
//   - 'start' is absolute time. Segments of different axes with the same start stay in sync.
//
// STEP is high for 1 tick, steps are at least 2 ticks apart (500kHz max per axis).
// DIR changes >= 1 tick after STEP falls, and >= dir_setup ticks before the next step.

#define STEP_AXES		8
#define STEP_QUEUE_LEN	32		// segments per axis, power of 2. A move uses up to 7.
#define STEP_EPS		1e-4	// steps of slack when deciding if a segment reaches a whole step

typedef struct {
	uint64_t start;		// absolute tick this segment starts at
	uint32_t duration;	// ticks
	bool dir;			// DIR pin level for the whole segment
	double p0;			// steps already done in this move when the segment starts
	double v0;			// steps/tick at start
	double a0;			// steps/tick^2 at start
	double j;			// steps/tick^3, constant over the segment
} StepSegment;

typedef struct {
	StepSegment queue[STEP_QUEUE_LEN];
	volatile uint32_t head;		// written by StepFillQueue
	volatile uint32_t tail;		// written by StepFillWords

	// Segment being played (fill side only)
	bool active;
	StepSegment seg;
	double c2, c3, d2;			// a0/2, j/6, j/2: no divides when evaluating p(t), v(t)
	double inv_v0;				// 1/v0 for constant rate segments (a0 = j = 0), 0 otherwise
	double p_end;				// p at the end of seg
	double tau;					// time of the last step found in seg, ticks from seg.start
	uint32_t next_k;			// next whole step position to hit
	bool have_edge;
	uint64_t edge_tick;			// next edge, already solved
	uint64_t earliest;			// no edge before this (DIR setup)

	bool dir;
	bool dir_valid;
	bool has_edge;
	uint64_t last_edge;
	uint64_t emitted;			// total steps put out
	int64_t position;			// signed step position
	uint32_t late;				// segments that started after their start tick
} StepAxis;

typedef struct {
	StepAxis axis[STEP_AXES];
	uint32_t step_mask[STEP_AXES];	// BSRR set bit for each STEP pin (bits 0..15)
	uint32_t dir_mask[STEP_AXES];	// BSRR set bit for each DIR pin (bits 0..15)
	uint32_t dir_setup;				// ticks, >= 1
	volatile uint64_t tick;			// tick of the next word to fill
	uint32_t carry;					// STEP resets that spill into the next fill
} StepGen;

void StepFillInit(StepGen *sg, const uint32_t *step_mask, const uint32_t *dir_mask, uint32_t dir_setup);
bool StepFillQueue(StepGen *sg, uint32_t axis, const StepSegment *seg);	// false if full or bad
uint32_t StepFillQueueFree(const StepGen *sg, uint32_t axis);
bool StepFillAxisIdle(const StepGen *sg, uint32_t axis);
void StepFillWords(StepGen *sg, uint32_t *buf, uint32_t n);
void StepFillFlush(StepGen *sg);	// drop everything queued and in progress (only while stopped)

// Helpers
double StepSegPos(const StepSegment *s, double t);	// p(t)
void StepSegConst(StepSegment *s, uint64_t start, uint32_t steps, uint32_t period, bool dir);	// constant rate

#endif
