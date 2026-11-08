// Host test for the vacuum probe trigger logic (app/probe_detect.c): filter, fixed and rolling
// reference, N fresh samples in a row, vacuum lost. Samples at 475 Hz like the ADC ring.
//
// Build/run: make -C test/host probe
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "probe_detect.h"

#define RATE	475.0

static int failures;

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		if (failures++ < 30) { \
			printf("FAIL %s:%d: ", __FILE__, __LINE__); \
			printf(__VA_ARGS__); \
			printf("\n"); \
		} \
	} \
} while (0)

static DetSample hist[4096];
static uint32_t rng = 99;

static double Rnd() {
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	return (rng & 0xFFFFFF) / (double)0x1000000;
}

static float Noise(float amp) {
	return (float)((Rnd() * 2 - 1) * amp);
}

static uint32_t Ms(uint32_t k) {
	return (uint32_t)(k * 1000.0 / RATE);
}

// Feed 'pre' samples unarmed at level, arm with their mean, return the detector
static float Prime(Detector *d, uint32_t *k, uint32_t pre, float level, float noise) {
	double sum = 0;
	uint32_t i;
	for (i = 0; i < pre; i++, (*k)++) {
		float x = level + Noise(noise);
		sum += x;
		CHECK(DetFeed(d, x, Ms(*k)) == DET_NONE, "triggered before it was armed");
	}
	DetArm(d, (float)(sum / pre));
	return (float)(sum / pre);
}

// A clean seal: -31 -> -45 in one step. With A = 4, S = 5 the filtered value is past ref - 5
// on the 2nd new sample, N = 3 in a row -> the 4th
static void TestSeal() {
	Detector d;
	uint32_t k = 0, i, at = 0;
	uint8_t r = 0;
	int start_fail = failures;

	DetInit(&d, 4, 3, 5, 0, 0, hist, 4096);
	Prime(&d, &k, 95, -31, 0.05f);
	for (i = 0; i < 500 && r == DET_NONE; i++, k++) r = DetFeed(&d, i < 200 ? -31 + Noise(0.05f) : -45 + Noise(0.05f), Ms(k)), at = i;
	CHECK(r == DET_TRIGGER, "seal: no trigger");
	CHECK(at == 203, "seal: triggered on sample %u after the step, want the 4th (203)", at);
	CHECK(fabsf(d.ref - -31) < 0.05f, "seal: reference %.3f", (double)d.ref);
	CHECK(d.min < -38, "seal: min %.2f", (double)d.min);

	// N = 1, A = 1: the first sample past it
	DetInit(&d, 1, 1, 5, 0, 0, hist, 4096);
	k = 0;
	Prime(&d, &k, 95, -8, 0.05f);
	r = DET_NONE;
	for (i = 0; i < 500 && r == DET_NONE; i++, k++) r = DetFeed(&d, i < 100 ? -8 + Noise(0.05f) : -42, Ms(k)), at = i;
	CHECK(r == DET_TRIGGER && at == 100, "seal N1 A1: at %u", at);
	printf("%-52s %s\n", "seal: triggers on the N-th filtered sample", failures == start_fail ? "PASS" : "FAIL");
}

// Glitches and noise must not trigger
static void TestNoise() {
	Detector d;
	uint32_t k = 0, i;
	uint8_t r = DET_NONE;
	int start_fail = failures;

	// single -6 kPa spikes every 50 samples: A = 4 makes them 1.5, never over S = 5
	DetInit(&d, 4, 3, 5, 0, 1000, hist, 4096);
	Prime(&d, &k, 95, -31, 0.05f);
	for (i = 0; i < 20000 && r == DET_NONE; i++, k++) r = DetFeed(&d, -31 + Noise(0.05f) + (i % 50 == 0 ? -6 : 0), Ms(k));
	CHECK(r == DET_NONE, "spikes A4: triggered at %u", i);

	// no filter, N = 3: spikes 2 samples long never make 3 in a row
	DetInit(&d, 1, 3, 5, 0, 0, hist, 4096);
	k = 0;
	Prime(&d, &k, 95, -31, 0.05f);
	r = DET_NONE;
	for (i = 0; i < 20000 && r == DET_NONE; i++, k++) r = DetFeed(&d, -31 + Noise(0.05f) + (i % 50 < 2 ? -8 : 0), Ms(k));
	CHECK(r == DET_NONE, "2 sample spikes N3: triggered at %u", i);

	// over / under / over ...: the count starts again every time
	DetInit(&d, 1, 2, 5, 0, 0, hist, 4096);
	k = 0;
	Prime(&d, &k, 95, -31, 0);
	r = DET_NONE;
	for (i = 0; i < 1000 && r == DET_NONE; i++, k++) r = DetFeed(&d, (i & 1) ? -40 : -31, Ms(k));
	CHECK(r == DET_NONE, "alternating: triggered at %u", i);
	printf("%-52s %s\n", "noise, spikes, alternating: no trigger", failures == start_fail ? "PASS" : "FAIL");
}

// Drift: free air going -7.8 -> -8.2 over 10 s. With S = 0.3 a fixed reference triggers on the
// drift alone, the rolling one doesn't, and still catches a real seal
static void TestDrift() {
	Detector d;
	uint32_t k, i, n = (uint32_t)(20 * RATE);
	uint8_t r;
	int start_fail = failures;

	for (int roll = 0; roll <= 1; roll++) {
		DetInit(&d, 4, 3, 0.3f, 0, roll ? 1000 : 0, hist, 4096);
		k = 0;
		Prime(&d, &k, 95, -7.8f, 0.05f);
		r = DET_NONE;
		for (i = 0; i < n && r == DET_NONE; i++, k++) r = DetFeed(&d, -7.8f - 0.04f * (float)(i / RATE) + Noise(0.05f), Ms(k));
		if (roll) {
			CHECK(r == DET_NONE, "rolling reference triggered on drift at %.2f s", i / RATE);
			float base = -7.8f - 0.04f * (float)(i / RATE);
			for (i = 0; i < 100 && r == DET_NONE; i++, k++) r = DetFeed(&d, base - 3 + Noise(0.05f), Ms(k));
			CHECK(r == DET_TRIGGER && i <= 6, "rolling reference: seal after drift, %u samples", i);
		} else {
			CHECK(r == DET_TRIGGER, "fixed reference should have triggered on the drift");
		}
	}
	printf("%-52s %s\n", "drift: rolling reference follows it, still triggers", failures == start_fail ? "PASS" : "FAIL");
}

// The rolling reference is the mean over [t - W, t - 50 ms], against brute force
static void TestWindow() {
	Detector d;
	static float xs[20000];
	uint32_t k, j, cnt, worst_k = 0;
	double worst = 0;
	int start_fail = failures;

	for (uint32_t cap = 4096; cap >= 256; cap /= 16) {
		DetInit(&d, 1, 1, 1e9f, 0, 700, hist, cap);
		for (k = 0; k < 20000; k++) {
			xs[k] = (float)(Rnd() * 100 - 50);
			DetFeed(&d, xs[k], Ms(k));
			double sum = 0;
			cnt = 0;
			for (j = 0; j <= k; j++) {
				uint32_t age = Ms(k) - Ms(j);
				if (age >= DET_GAP_MS && age <= 700 && k - j < cap) {
					sum += xs[j];
					cnt++;
				}
			}
			if (cnt) {
				double e = fabs(sum / cnt - d.ref);
				if (e > worst) {
					worst = e;
					worst_k = k;
				}
			}
		}
		CHECK(worst < 1e-3, "window (cap %u): off by %.6f at sample %u", cap, worst, worst_k);
		worst = 0;
	}
	printf("%-52s %s\n", "rolling window = brute force mean, small buffers too", failures == start_fail ? "PASS" : "FAIL");
}

// Vacuum lost: U = 3, the reading rises by 5
static void TestLost() {
	Detector d;
	uint32_t k = 0, i;
	uint8_t r = DET_NONE;
	int start_fail = failures;

	DetInit(&d, 4, 3, 5, 3, 0, hist, 4096);
	Prime(&d, &k, 95, -31, 0.05f);
	for (i = 0; i < 200 && r == DET_NONE; i++, k++) r = DetFeed(&d, i < 50 ? -31 : -26, Ms(k));
	CHECK(r == DET_LOST, "lost: got %u", r);
	DetInit(&d, 4, 3, 5, 0, 0, hist, 4096);
	k = 0;
	Prime(&d, &k, 95, -31, 0.05f);
	r = DET_NONE;
	for (i = 0; i < 200 && r == DET_NONE; i++, k++) r = DetFeed(&d, i < 50 ? -31 : -26, Ms(k));
	CHECK(r == DET_NONE, "U = 0 is off: got %u", r);
	printf("%-52s %s\n", "vacuum lost (U), and U0 = off", failures == start_fail ? "PASS" : "FAIL");
}

int main(void) {
	TestSeal();
	TestNoise();
	TestDrift();
	TestWindow();
	TestLost();
	printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
	return failures ? 1 : 0;
}
