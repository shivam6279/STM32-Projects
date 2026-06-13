// Per-cell passive balancing: PA3/PC14/PC15 gate a FET + ~50R bleed across
// cell 1/2/3 (HIGH = bleed on, ~85 mA at 4.2 V).
//
// Decisions are made on CLEAN readings only: charging and all bleeds are
// paused, the cells relax (the I*R inflation dies out in milliseconds), the
// taps are sampled, then bleeds/charging are re-applied. Cells more than
// BAL_START_DELTA_MV above the lowest cell bleed until they come back within
// BAL_STOP_DELTA_MV (per-cell hysteresis). Balancing only happens above
// BAL_MIN_CELL_MV -- below that the OCV curve is too flat for millivolt
// deltas to mean anything.
//
// Safety: if any cell exceeds BAL_CELL_OV_MV, charging is held off (EN_CHG)
// until the highest cell bleeds back below BAL_CELL_OV_CLEAR_MV.
//
// Top-balance loop: checks keep running after charge termination (the board
// stays awake while USB is present), so bleeding continues after DONE. Once
// the spread closes, one EN_CHG toggle restarts a charge cycle to top up the
// charge that was bled away; this repeats until the pack is full AND balanced
// in the same session. Balancer_Balanced() gates the "full" LED so the user
// isn't told to unplug before balancing has finished.
#ifndef BALANCER_H
#define BALANCER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bq25798.h"
#include "battery_adc.h"

#define BAL_CHECK_PERIOD_MS   30000u  // pause-measure-decide interval
#define BAL_SETTLE_MS         300u    // relaxation before sampling
#define BAL_MIN_CELL_MV       3900u   // never bleed below this (flat OCV region)
#define BAL_START_DELTA_MV    12u     // start bleeding this far above the lowest
#define BAL_STOP_DELTA_MV     8u      // stop once back within this of the lowest
#define BAL_CELL_OV_MV        4250u   // suspend charging when any cell above
#define BAL_CELL_OV_CLEAR_MV  4180u   // resume once the highest cell is below

// Per-cell CV clamp: the pack VREG is trimmed down so the highest cell never
// charges above this; restored toward BAL_PACK_VREG_MV as the balance closes.
#define BAL_CELL_VMAX_MV      4200u   // ceiling for the highest cell
#define BAL_PACK_VREG_MV     12600u   // nominal VREG (keep = cfg.charge_mV)
#define BAL_PACK_VREG_MIN_MV 12000u   // never trim VREG below this

void    Balancer_Init(void);          // bleed pins as outputs, all off
void    Balancer_AllOff(void);        // kill all bleeds (call before standby)
uint8_t Balancer_BleedMask(void);     // bit n = cell n+1 currently bleeding
uint8_t Balancer_OvSuspended(void);   // charging held off due to cell OV
uint8_t Balancer_Balanced(void);      // last clean check: spread <= target

// Last clean measurement (taken with charging and bleeds paused); NULL until
// the first check has run. Use this for display while charging -- a live tap
// read is inflated by I*R.
const batt_cells_t *Balancer_CleanCells(void);

// Periodic pause-measure-decide cycle; call from the charge loop (~1 Hz is
// fine, it self-times to BAL_CHECK_PERIOD_MS). Pass the charger phase so the
// charge pause is skipped when no current is flowing anyway.
void    Balancer_Task(bq_chg_state_t chg_state);

#ifdef __cplusplus
}
#endif
#endif // BALANCER_H
