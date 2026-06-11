// 3S cell taps on PA6/7/8 (tab1..3 cumulative; cells via differencing).
// PA11 = analog-enable (HIGH to read, LOW otherwise so dividers don't drain).
#ifndef BATTERY_ADC_H
#define BATTERY_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// tap_mV = raw * TABn_SCALER (calibrate per board)
#define TAB1_SCALER   1.6214971967605f
#define TAB2_SCALER   3.3202432870043f
#define TAB3_SCALER   6.3087186081308f

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
