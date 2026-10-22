// 3S cell taps on PA6/7/8 (tab1..3 cumulative; cells via differencing).
// PA11 = analog-enable (HIGH to read, LOW otherwise so dividers don't drain).
#ifndef BATTERY_ADC_H
#define BATTERY_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* tap_mV = raw * TABn_SCALER -- one independent scaler per channel, since each
 * tap has its own divider ratio and its own source impedance.
 *
 * Calibrated 2026-09-26 against the baseline raw counts, which have been
 * consistent (~2570 / 2510 / 1990 at ~4.17 V/cell) across many boots and builds:
 *   metered cells : 4175 / 4170 / 4185 mV
 *   tab targets   : 4175 / 8345 / 12530 mV   (cumulative)
 *   raw counts    : 2579 / 2520 / 1994
 * Within 0.4% of the original factory values (1.6215 / 3.3202 / 6.3087), which
 * were correct all along. A short run of boots once read ~8-26% high on raw; do
 * NOT calibrate against a reading unless it matches this baseline.
 * To redo: meter each cell, take RAW from the boot line, scaler = tab_mV / raw. */
#define TAB1_SCALER   1.6188445f
#define TAB2_SCALER   3.3115079f
#define TAB3_SCALER   6.2838516f

// Settling time (ms) after raising the analog-enable before sampling. The
// divider low-pass caps must charge through the taps first (slowest node
// tau ~2 ms); the scalers above assume FULLY settled nodes.
#define BATT_ADC_SETTLE_MS   20u

// Samples averaged per tap, taken back-to-back once settled.
#define BATT_ADC_AVG_SAMPLES 8u

typedef struct {
	uint16_t tab_mV[3];    // tab1, tab2, tab3 (cumulative)
	uint16_t cell_mV[3];   // cell1, cell2, cell3 (differences)
	uint16_t raw[3];       // raw 12-bit counts, for debug/calibration
} batt_cells_t;

void BatteryADC_Init(void);             // analog pins + enable pin + ADC calibrate
void BatteryADC_Read(batt_cells_t *out);// pulse enable, sample, scale, difference
void BatteryADC_Sleep(void);            // force analog-enable LOW (before standby)

#ifdef __cplusplus
}
#endif
#endif // BATTERY_ADC_H
