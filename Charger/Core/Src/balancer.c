// Passive cell balancing with pause-measure-decide. See balancer.h.

#include "balancer.h"
#include "battery_adc.h"
#include "main.h"
#include <stdio.h>

// Bleed-enable pins: HIGH = FET on = ~50R across that cell.
static GPIO_TypeDef *const s_bal_port[3] = { GPIOA, GPIOC, GPIOC };
static const uint16_t      s_bal_pin[3]  = { GPIO_PIN_3, GPIO_PIN_14, GPIO_PIN_15 };

static uint8_t  s_bleed_mask;     // bit n = cell n+1 bleeding
static uint8_t  s_ov_suspended;   // charging held off: a cell went over BAL_CELL_OV_MV
static uint8_t  s_balanced;       // last clean check: no bleed active or wanted
static uint8_t  s_bled_at_full;   // bled after termination -> pack needs a top-up
static uint32_t s_last_check_ms;
static batt_cells_t s_clean_cells;  // last clean measurement (charge+bleeds paused)
static uint8_t  s_have_clean;       // s_clean_cells holds a real measurement
static uint16_t s_vreg_mV;          // pack VREG currently programmed into the BQ

static void ApplyBleeds(uint8_t mask) {
	for (int i = 0; i < 3; i++) {
		HAL_GPIO_WritePin(s_bal_port[i], s_bal_pin[i],
						  (mask & (1u << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

void Balancer_Init(void) {
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	for (int i = 0; i < 3; i++) {
		HAL_GPIO_WritePin(s_bal_port[i], s_bal_pin[i], GPIO_PIN_RESET);
		gpio.Pin   = s_bal_pin[i];
		gpio.Mode  = GPIO_MODE_OUTPUT_PP;
		gpio.Pull  = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(s_bal_port[i], &gpio);
	}
	s_bleed_mask    = 0u;
	s_ov_suspended  = 0u;
	s_balanced      = 0u;
	s_bled_at_full  = 0u;
	s_have_clean    = 0u;
	s_vreg_mV       = BAL_PACK_VREG_MV;   // what BQ25798_Configure programs
	// backdate so the first Balancer_Task call measures immediately
	s_last_check_ms = HAL_GetTick() - BAL_CHECK_PERIOD_MS;
}

void Balancer_AllOff(void) {
	s_bleed_mask = 0u;
	ApplyBleeds(0u);
}

uint8_t Balancer_BleedMask(void)   { return s_bleed_mask; }
uint8_t Balancer_OvSuspended(void) { return s_ov_suspended; }
uint8_t Balancer_Balanced(void)    { return s_balanced; }

const batt_cells_t *Balancer_CleanCells(void) {
	return s_have_clean ? &s_clean_cells : (const batt_cells_t *)0;
}

void Balancer_Task(bq_chg_state_t chg_state) {
	if ((uint32_t)(HAL_GetTick() - s_last_check_ms) < BAL_CHECK_PERIOD_MS) {
		return;
	}
	s_last_check_ms = HAL_GetTick();

	/* Charge current inflates the tap readings by I*R, and an active bleed
	 * skews its own cell's reading through the tap network -- so pause both
	 * and let the cells relax before sampling. Skip the charge pause when the
	 * charger isn't pushing current anyway (idle/done): readings are already
	 * clean, and toggling EN_CHG at full could needlessly restart a cycle. */
	uint8_t charge_active = (chg_state != BQ_CHG_NOT_CHARGING &&
							 chg_state != BQ_CHG_DONE);
	if (charge_active && BQ25798_EnableCharging(0u) != BQ_OK) {
		printf("BAL: charge pause failed, skipping check\n");
		BQ25798_EnableCharging(1u);   // best effort: don't leave it off
		return;
	}
	ApplyBleeds(0u);
	HAL_Delay(BAL_SETTLE_MS);

	BatteryADC_Read(&s_clean_cells);
	s_have_clean = 1u;

	uint16_t min_mV = s_clean_cells.cell_mV[0];
	uint16_t max_mV = s_clean_cells.cell_mV[0];
	for (int i = 1; i < 3; i++) {
		if (s_clean_cells.cell_mV[i] < min_mV) { min_mV = s_clean_cells.cell_mV[i]; }
		if (s_clean_cells.cell_mV[i] > max_mV) { max_mV = s_clean_cells.cell_mV[i]; }
	}

	// Per-cell hysteresis: start at lowest+START_DELTA, stop at lowest+STOP_DELTA.
	uint8_t mask = s_bleed_mask;
	for (int i = 0; i < 3; i++) {
		uint16_t v   = s_clean_cells.cell_mV[i];
		uint8_t  bit = (uint8_t)(1u << i);
		if (mask & bit) {
			if (v < BAL_MIN_CELL_MV ||
				v <= (uint16_t)(min_mV + BAL_STOP_DELTA_MV)) {
				mask &= (uint8_t)~bit;
			}
		} else if (v >= BAL_MIN_CELL_MV &&
				   v >= (uint16_t)(min_mV + BAL_START_DELTA_MV)) {
			mask |= bit;
		}
	}

	// Cell overvoltage: hold charging off until the high cell bleeds back down.
	if (max_mV >= BAL_CELL_OV_MV) {
		s_ov_suspended = 1u;
	} else if (s_ov_suspended && max_mV <= BAL_CELL_OV_CLEAR_MV) {
		s_ov_suspended = 0u;
	}

	// balanced = nothing left to bleed: no bleed active and no cell far enough
	// above the lowest to start one
	s_balanced = (uint8_t)(mask == 0u &&
						   (uint16_t)(max_mV - min_mV) < BAL_START_DELTA_MV);

	/* Top-balance loop: bleeding after termination drains the high cells, so
	 * once the spread has closed, restart one charge cycle to refill what was
	 * bled. Repeats until the pack is full AND balanced in the same session. */
	uint8_t topup = 0u;
	if (chg_state == BQ_CHG_DONE) {
		if (mask != 0u) {
			s_bled_at_full = 1u;
		} else if (s_bled_at_full && s_balanced && !s_ov_suspended) {
			s_bled_at_full = 0u;
			topup = 1u;
		}
	}

	/* Per-cell CV clamp: trim the pack setpoint so the highest cell sits at
	 * BAL_CELL_VMAX_MV. The clean pack sum minus the excess is the pack
	 * voltage at which the high cell lands on its ceiling; current keeps
	 * flowing into the low cells while the bleed works the high one down.
	 * Restored to nominal as the balance closes. */
	uint16_t vreg = BAL_PACK_VREG_MV;
	if (max_mV > BAL_CELL_VMAX_MV) {
		uint32_t v = (uint32_t)s_clean_cells.tab_mV[2]
				   - (uint32_t)(max_mV - BAL_CELL_VMAX_MV);
		if (v < BAL_PACK_VREG_MIN_MV) { v = BAL_PACK_VREG_MIN_MV; }
		if (v > BAL_PACK_VREG_MV)     { v = BAL_PACK_VREG_MV; }
		vreg = (uint16_t)v;
	}
	if (vreg != s_vreg_mV) {
		if (BQ25798_SetChargeVoltage_mV(vreg) == BQ_OK) {
			printf("BAL: VREG -> %u mV (max cell %u)\n", vreg, max_mV);
			s_vreg_mV = vreg;
		}
	}

	s_bleed_mask = mask;
	ApplyBleeds(mask);

	/* Re-assert the charge enable every check (idempotent write): this both
	 * resumes after the measurement pause and recovers the enable once an OV
	 * suspend clears, regardless of which path turned it off. An EN_CHG 0->1
	 * toggle (top-up) arms a fresh charge cycle after termination. */
	if (topup) {
		printf("BAL: balanced after post-full bleed -> top-up charge\n");
		BQ25798_EnableCharging(0u);
	}
	BQ25798_EnableCharging(s_ov_suspended ? 0u : 1u);

	printf("BAL: cells=%u/%u/%u mV spread=%u bleed=%u%u%u ov=%u bal=%u\n",
		   s_clean_cells.cell_mV[0], s_clean_cells.cell_mV[1], s_clean_cells.cell_mV[2],
		   (unsigned)(max_mV - min_mV),
		   (unsigned)(mask & 1u), (unsigned)((mask >> 1) & 1u),
		   (unsigned)((mask >> 2) & 1u), (unsigned)s_ov_suspended,
		   (unsigned)s_balanced);
}
