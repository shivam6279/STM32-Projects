#include "stepgen_fill.h"
#include <stdatomic.h>
#include <string.h>

void StepFillInit(StepGen *sg, const uint32_t *step_mask, const uint32_t *dir_mask, uint32_t dir_setup) {
	uint32_t i;

	memset(sg, 0, sizeof(*sg));
	for (i = 0; i < STEP_AXES; i++) {
		sg->step_mask[i] = step_mask[i];
		sg->dir_mask[i] = dir_mask[i];
	}
	sg->dir_setup = dir_setup < 1 ? 1 : dir_setup;
}

double StepSegPos(const StepSegment *s, double t) {
	return s->p0 + t * (s->v0 + t * (s->a0 / 2 + t * s->j / 6));
}

void StepSegConst(StepSegment *s, uint64_t start, uint32_t steps, uint32_t period, bool dir) {
	s->start = start;
	s->duration = steps * period;
	s->dir = dir;
	s->p0 = 0;
	s->v0 = 1.0 / period;
	s->a0 = 0;
	s->j = 0;
}

bool StepFillQueue(StepGen *sg, uint32_t axis, const StepSegment *seg) {
	StepAxis *a;
	uint32_t head;

	if (axis >= STEP_AXES || seg->duration == 0 || seg->v0 < 0 || seg->v0 > 0.5) return false;
	a = &sg->axis[axis];
	head = a->head;
	if (head - a->tail >= STEP_QUEUE_LEN) return false;

	a->queue[head & (STEP_QUEUE_LEN - 1)] = *seg;
	atomic_thread_fence(memory_order_release); // segment written before head moves
	a->head = head + 1;
	return true;
}

uint32_t StepFillQueueFree(const StepGen *sg, uint32_t axis) {
	const StepAxis *a = &sg->axis[axis];
	return STEP_QUEUE_LEN - (a->head - a->tail);
}

bool StepFillAxisIdle(const StepGen *sg, uint32_t axis) {
	const StepAxis *a = &sg->axis[axis];
	return !a->active && a->head == a->tail;
}

// Time t in [lo, hi] where p(t) = target. p is increasing, p(lo) < target <= p(hi) (or just
// past it by < STEP_EPS). Newton from lo, falls back to halving whenever Newton would leave
// the bracket or the velocity is ~0 (start of a move, where v0 = a0 = 0).
// Done when the time error |f|/v is under 0.001 tick. That's a time test, not a position one:
// near the end of a move the speed goes to 0 and a tiny position error is a big time error.
static double Solve(const StepAxis *a, double target, double lo, double hi) {
	const StepSegment *s = &a->seg;
	double x = lo, f, d, nx;
	int i;

	for (i = 0; i < 60; i++) {
		f = s->p0 + x * (s->v0 + x * (a->c2 + x * a->c3)) - target;
		d = s->v0 + x * (s->a0 + x * a->d2);
		if (f < 0) lo = x;
		else hi = x;
		if (d > 0 && f > -1e-3 * d && f < 1e-3 * d) break;
		if (hi - lo < 1e-3) break;
		nx = d > 1e-15 ? x - f / d : (lo + hi) / 2;
		if (nx < lo || nx > hi) nx = (lo + hi) / 2;
		x = nx;
	}
	return x;
}

// Find the next step edge in the current segment. False when the segment has no more steps.
static bool NextEdge(StepAxis *a) {
	double t;
	uint64_t tick;

	if (a->next_k > a->p_end + STEP_EPS) return false;

	if (a->inv_v0 > 0) {
		// Constant rate: straight division, no iterating
		t = (a->next_k - a->seg.p0) * a->inv_v0;
		if (t > a->seg.duration) t = a->seg.duration;
	} else {
		t = Solve(a, a->next_k, a->tau, a->seg.duration);
	}
	a->tau = t;
	tick = a->seg.start + (uint32_t)(t + 0.5);

	// Hardware limits: 2 ticks between rising edges, and DIR setup after a direction change
	if (a->has_edge && tick < a->last_edge + 2) tick = a->last_edge + 2;
	if (tick < a->earliest) tick = a->earliest;
	a->edge_tick = tick;
	a->have_edge = true;
	return true;
}

// Start the next queued segment if it begins before 'end'. Writes the DIR bit if the direction
// changes. False if nothing to start yet.
static bool LoadNext(StepGen *sg, uint32_t ax, uint32_t *buf, uint64_t start, uint64_t end) {
	StepAxis *a = &sg->axis[ax];
	uint32_t tail = a->tail;
	StepSegment *s;
	uint64_t dir_tick;

	if (tail == a->head) return false;
	atomic_thread_fence(memory_order_acquire);
	s = &a->queue[tail & (STEP_QUEUE_LEN - 1)];
	if (s->start >= end) return false; // not yet

	a->earliest = 0;
	if (!a->dir_valid || s->dir != a->dir) {
		// DIR goes at the segment start, but never before STEP from the last step has fallen
		dir_tick = s->start > start ? s->start : start;
		if (a->has_edge && a->last_edge + 2 > dir_tick) dir_tick = a->last_edge + 2;
		if (dir_tick >= end) return false; // retry next fill
		buf[dir_tick - start] |= s->dir ? sg->dir_mask[ax] : (sg->dir_mask[ax] << 16);
		a->dir = s->dir;
		a->dir_valid = true;
		a->earliest = dir_tick + sg->dir_setup;
	}
	if (s->start < start) a->late++; // planner was too slow, this segment plays behind time

	a->seg = *s;
	a->c2 = s->a0 / 2;
	a->c3 = s->j / 6;
	a->d2 = s->j / 2;
	a->inv_v0 = (s->a0 == 0 && s->j == 0 && s->v0 > 0) ? 1 / s->v0 : 0;
	a->p_end = StepSegPos(s, s->duration);
	a->tau = 0;
	a->next_k = (uint32_t)(s->p0 + STEP_EPS) + 1;
	a->have_edge = false;
	a->active = true;
	a->tail = tail + 1;
	return true;
}

void StepFillFlush(StepGen *sg) {
	uint32_t i;
	for (i = 0; i < STEP_AXES; i++) {
		StepAxis *a = &sg->axis[i];
		a->tail = a->head;
		a->active = false;
		a->have_edge = false;
		a->earliest = 0;
	}
	sg->carry = 0;
}

void StepFillWords(StepGen *sg, uint32_t *buf, uint32_t n) {
	uint64_t start = sg->tick;
	uint64_t end = start + n;
	uint32_t ax;

	memset(buf, 0, n * sizeof(buf[0]));
	buf[0] = sg->carry;
	sg->carry = 0;

	for (ax = 0; ax < STEP_AXES; ax++) {
		StepAxis *a = &sg->axis[ax];
		uint32_t set = sg->step_mask[ax];
		uint32_t reset = set << 16;

		while (1) {
			if (!a->active && !LoadNext(sg, ax, buf, start, end)) break;
			if (!a->have_edge && !NextEdge(a)) {
				a->active = false; // segment done, maybe the next one starts in this window
				continue;
			}
			if (a->edge_tick >= end) break;
			if (a->edge_tick < start) a->edge_tick = start; // late segment, catch up

			uint32_t i = (uint32_t)(a->edge_tick - start);
			buf[i] |= set;
			if (i + 1 < n) buf[i + 1] |= reset;
			else sg->carry |= reset;

			a->last_edge = a->edge_tick;
			a->has_edge = true;
			a->have_edge = false;
			a->emitted++;
			a->position += a->dir ? 1 : -1;
			a->next_k++;
		}
	}
	sg->tick = end;
}
