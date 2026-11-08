#ifndef _PROBE_DETECT_H_
#define _PROBE_DETECT_H_

#include <stdbool.h>
#include <inttypes.h>

// Vacuum probe trigger logic, no hardware (test/host/test_probe.c). Fed one fresh pressure
// sample at a time (kPa gauge, more negative = more vacuum) with the time it came.
//
// - filter: mean of the last 'avg' samples (1 = none)
// - reference: fixed (the mean given to DetArm), or rolling: the mean of the samples that came
//   between roll_ms and gap_ms ago, so slow drift moves it along but a seal doesn't (it's over
//   in a few samples). Until that window has samples, the fixed one
// - trigger: reference - filtered > drop on 'need' fresh samples in a row
// - lost: filtered - reference > rise (rise > 0): vacuum went away (hose off, valve shut)
// Samples before DetArm only fill the filter and the history (the reference phase).

#define DET_AVG_MAX		32
#define DET_GAP_MS		50		// rolling reference: leave out the newest samples

#define DET_NONE		0
#define DET_TRIGGER		1
#define DET_LOST		2

typedef struct {
	float x;
	uint32_t ms;
} DetSample;

typedef struct {
	// settings
	uint8_t avg, need;
	float drop, rise;
	uint32_t roll_ms;
	// history for the rolling reference, the caller's buffer (power of 2 long)
	DetSample *hist;
	uint32_t cap;
	uint32_t head;			// samples ever put in
	uint32_t win_lo, win_hi;	// window = hist[win_lo .. win_hi)
	double win_sum;
	// filter
	float fbuf[DET_AVG_MAX];
	uint32_t fn;
	float fsum;
	// state
	bool armed;
	float ref_fixed;
	float ref, filt, min;	// after the last sample
	uint8_t over;			// samples in a row over the threshold
} Detector;

void DetInit(Detector *d, uint8_t avg, uint8_t need, float drop, float rise, uint32_t roll_ms, DetSample *hist, uint32_t cap);
uint8_t DetFeed(Detector *d, float x, uint32_t ms);	// DET_NONE / DET_TRIGGER / DET_LOST
void DetArm(Detector *d, float ref);				// from now on it can trigger
float DetFiltered(const Detector *d);				// mean of the last avg samples

#endif
