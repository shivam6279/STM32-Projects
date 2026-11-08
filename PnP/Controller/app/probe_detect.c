#include "probe_detect.h"

void DetInit(Detector *d, uint8_t avg, uint8_t need, float drop, float rise, uint32_t roll_ms, DetSample *hist, uint32_t cap) {
	d->avg = avg < 1 ? 1 : (avg > DET_AVG_MAX ? DET_AVG_MAX : avg);
	d->need = need < 1 ? 1 : need;
	d->drop = drop;
	d->rise = rise;
	d->roll_ms = roll_ms;
	d->hist = hist;
	d->cap = cap;
	d->head = 0;
	d->win_lo = 0;
	d->win_hi = 0;
	d->win_sum = 0;
	d->fn = 0;
	d->fsum = 0;
	d->armed = false;
	d->ref_fixed = 0;
	d->ref = 0;
	d->filt = 0;
	d->min = 0;
	d->over = 0;
}

float DetFiltered(const Detector *d) {
	uint32_t n = d->fn < d->avg ? d->fn : d->avg;
	return n ? d->fsum / n : 0;
}

void DetArm(Detector *d, float ref) {
	d->ref_fixed = ref;
	d->ref = ref;
	d->min = DetFiltered(d);
	d->over = 0;
	d->armed = true;
}

// Rolling window over [ms - roll_ms, ms - DET_GAP_MS]: samples come in at the old end of the
// gap, leave past roll_ms. The buffer only has to hold roll_ms worth (older ones are gone anyway).
static void Window(Detector *d, uint32_t ms) {
	uint32_t m = d->cap - 1;

	while (d->win_hi < d->head && ms - d->hist[d->win_hi & m].ms >= DET_GAP_MS) {
		d->win_sum += d->hist[d->win_hi & m].x;
		d->win_hi++;
	}
	while (d->win_lo < d->win_hi && ms - d->hist[d->win_lo & m].ms > d->roll_ms) {
		d->win_sum -= d->hist[d->win_lo & m].x;
		d->win_lo++;
	}
	if (d->win_lo == d->win_hi) d->win_sum = 0; // no rounding left over when it empties
}

// The next sample goes over the oldest one: take that out of the window first, while it's still there
static void Evict(Detector *d) {
	uint32_t m = d->cap - 1;

	// One in, one out: head - win_lo never gets past cap
	if (d->head - d->win_lo >= d->cap) {
		if (d->win_lo < d->win_hi) d->win_sum -= d->hist[d->win_lo & m].x;
		else d->win_hi++; // window empty: it's the next one to come in that goes (buffer shorter than the gap)
		d->win_lo++;
	}
}

uint8_t DetFeed(Detector *d, float x, uint32_t ms) {
	uint32_t i = d->fn % d->avg;

	// Filter
	if (d->fn >= d->avg) d->fsum -= d->fbuf[i];
	d->fbuf[i] = x;
	d->fsum += x;
	d->fn++;
	if (d->fn % 256 == 0) {
		// Float drift from adding and taking away: sum it up again now and then
		uint32_t k;
		d->fsum = 0;
		for (k = 0; k < d->avg; k++) d->fsum += d->fbuf[k];
	}
	d->filt = DetFiltered(d);

	// History, then the rolling reference
	if (d->cap && d->roll_ms > DET_GAP_MS) {
		Evict(d);
		d->hist[d->head & (d->cap - 1)].x = x;
		d->hist[d->head & (d->cap - 1)].ms = ms;
		d->head++;
		Window(d, ms);
	}
	d->ref = d->win_hi > d->win_lo ? (float)(d->win_sum / (d->win_hi - d->win_lo)) : d->ref_fixed;

	if (!d->armed) return DET_NONE;
	if (d->filt < d->min) d->min = d->filt;
	if (d->rise > 0 && d->filt - d->ref > d->rise) return DET_LOST;
	if (d->ref - d->filt > d->drop) {
		if (d->over < 255) d->over++;
	} else {
		d->over = 0;
	}
	return d->over >= d->need ? DET_TRIGGER : DET_NONE;
}
