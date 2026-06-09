// BQ25798 charger driver, bare-metal I2C1 (no HAL I2C). See bq25798.h.

#include "bq25798.h"
#include "main.h"

#define BQ_I2C_TIMINGR     0xB0421318u   // ~90 kHz @ 48 MHz PCLK
#define BQ_I2C_TIMEOUT_MS  10u

// ---- Bare-metal I2C1 transport --------------------------------------------

// PE=0 clears a latched BUSY and aborts a stuck transfer; reprogram and re-enable
static void bq_i2c_periph_reset(void) {
	I2C1->CR1    &= ~I2C_CR1_PE;
	(void)I2C1->CR1;
	I2C1->TIMINGR = BQ_I2C_TIMINGR;
	I2C1->CR1    |= I2C_CR1_PE;
}

// clock SCL up to 9 times to free a slave holding SDA low, then restore AF
static void bq_i2c_bus_clear(void) {
	GPIO_InitTypeDef g = {0};

	g.Pull  = GPIO_PULLUP;
	g.Speed = GPIO_SPEED_FREQ_LOW;

	g.Pin  = GPIO_PIN_6;                 // SCL: open-drain GPIO output
	g.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(GPIOB, &g);
	g.Pin  = GPIO_PIN_7;                 // SDA: input
	g.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB, &g);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	for (int i = 0; i < 9; i++) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
			break;                       // SDA released
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(1);
	}

	// Restore PB6/PB7 to I2C1 alternate function (external pull-ups).
	g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
	g.Mode      = GPIO_MODE_AF_OD;
	g.Pull      = GPIO_NOPULL;
	g.Speed     = GPIO_SPEED_FREQ_HIGH;
	g.Alternate = GPIO_AF6_I2C1;
	HAL_GPIO_Init(GPIOB, &g);
}

static bq_status_t bq_wait_not_busy(void) {
	uint32_t t0 = HAL_GetTick();
	while (I2C1->ISR & I2C_ISR_BUSY) {
		if ((uint32_t)(HAL_GetTick() - t0) >= BQ_I2C_TIMEOUT_MS) {
			bq_i2c_periph_reset();   // clear latched BUSY, retry once
			if (I2C1->ISR & I2C_ISR_BUSY) {
				return BQ_BUSY;
			}
			return BQ_OK;
		}
	}
	return BQ_OK;
}

// Wait until ISR has `flag` set, returning early on a device NACK or timeout.
static bq_status_t bq_wait(uint32_t flag) {
	uint32_t t0 = HAL_GetTick();
	for (;;) {
		uint32_t isr = I2C1->ISR;
		if (isr & flag) {
			return BQ_OK;
		}
		if (isr & I2C_ISR_NACKF) {
			I2C1->ICR = I2C_ICR_NACKCF;
			return BQ_NACK;
		}
		if ((uint32_t)(HAL_GetTick() - t0) >= BQ_I2C_TIMEOUT_MS) {
			return BQ_TIMEOUT;
		}
	}
}

/* Write `reg` followed by `len` payload bytes in a single transaction.
 * The BQ25798 auto-increments its register pointer for multi-byte writes. */
static bq_status_t bq_i2c_write(uint8_t reg, const uint8_t *data, uint8_t len) {
	bq_status_t st = bq_wait_not_busy();
	if (st != BQ_OK) {
		return st;
	}
	I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

	I2C1->CR2 = ((uint32_t)BQ25798_I2C_ADDR << 1)
			  | (((uint32_t)len + 1u) << I2C_CR2_NBYTES_Pos)
			  | I2C_CR2_AUTOEND
			  | I2C_CR2_START;

	st = bq_wait(I2C_ISR_TXIS);
	if (st != BQ_OK) {
		return st;
	}
	I2C1->TXDR = reg;

	for (uint8_t i = 0u; i < len; i++) {
		st = bq_wait(I2C_ISR_TXIS);
		if (st != BQ_OK) {
			return st;
		}
		I2C1->TXDR = data[i];
	}

	st = bq_wait(I2C_ISR_STOPF);
	if (st != BQ_OK) {
		return st;
	}
	I2C1->ICR = I2C_ICR_STOPCF;
	return BQ_OK;
}

// Set the register pointer, then repeated-start and read `len` bytes.
static bq_status_t bq_i2c_read(uint8_t reg, uint8_t *data, uint8_t len) {
	bq_status_t st = bq_wait_not_busy();
	if (st != BQ_OK) {
		return st;
	}
	I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

	/* Phase 1: write the 1-byte register pointer with a software stop (TC),
	 * so we can issue a repeated start instead of releasing the bus. */
	I2C1->CR2 = ((uint32_t)BQ25798_I2C_ADDR << 1)
			  | (1u << I2C_CR2_NBYTES_Pos)
			  | I2C_CR2_START;

	st = bq_wait(I2C_ISR_TXIS);
	if (st != BQ_OK) {
		return st;
	}
	I2C1->TXDR = reg;

	st = bq_wait(I2C_ISR_TC);
	if (st != BQ_OK) {
		return st;
	}

	// Phase 2: repeated start in read direction, auto stop after `len` bytes.
	I2C1->CR2 = ((uint32_t)BQ25798_I2C_ADDR << 1)
			  | I2C_CR2_RD_WRN
			  | ((uint32_t)len << I2C_CR2_NBYTES_Pos)
			  | I2C_CR2_AUTOEND
			  | I2C_CR2_START;

	for (uint8_t i = 0u; i < len; i++) {
		st = bq_wait(I2C_ISR_RXNE);
		if (st != BQ_OK) {
			return st;
		}
		data[i] = (uint8_t)I2C1->RXDR;
	}

	st = bq_wait(I2C_ISR_STOPF);
	if (st != BQ_OK) {
		return st;
	}
	I2C1->ICR = I2C_ICR_STOPCF;
	return BQ_OK;
}

// Generic write at any 7-bit address on this bus (e.g. STUSB4500 @ 0x28).
bq_status_t BQ_BusWrite(uint8_t addr7, uint8_t reg, const uint8_t *data, uint8_t len) {
	bq_status_t st = bq_wait_not_busy();
	if (st != BQ_OK) { return st; }
	I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
	I2C1->CR2 = ((uint32_t)addr7 << 1) | (((uint32_t)len + 1u) << I2C_CR2_NBYTES_Pos)
			  | I2C_CR2_AUTOEND | I2C_CR2_START;
	st = bq_wait(I2C_ISR_TXIS);
	if (st != BQ_OK) { return st; }
	I2C1->TXDR = reg;
	for (uint8_t i = 0u; i < len; i++) {
		st = bq_wait(I2C_ISR_TXIS);
		if (st != BQ_OK) { return st; }
		I2C1->TXDR = data[i];
	}
	st = bq_wait(I2C_ISR_STOPF);
	if (st != BQ_OK) { return st; }
	I2C1->ICR = I2C_ICR_STOPCF;
	return BQ_OK;
}

// Generic read at any 7-bit address on this bus (e.g. STUSB4500 @ 0x28).
bq_status_t BQ_BusRead(uint8_t addr7, uint8_t reg, uint8_t *data, uint8_t len) {
	bq_status_t st = bq_wait_not_busy();
	if (st != BQ_OK) { return st; }
	I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;

	I2C1->CR2 = ((uint32_t)addr7 << 1) | (1u << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	st = bq_wait(I2C_ISR_TXIS);
	if (st != BQ_OK) { return st; }
	I2C1->TXDR = reg;
	st = bq_wait(I2C_ISR_TC);
	if (st != BQ_OK) { return st; }

	I2C1->CR2 = ((uint32_t)addr7 << 1) | I2C_CR2_RD_WRN
			  | ((uint32_t)len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
	for (uint8_t i = 0u; i < len; i++) {
		st = bq_wait(I2C_ISR_RXNE);
		if (st != BQ_OK) { return st; }
		data[i] = (uint8_t)I2C1->RXDR;
	}
	st = bq_wait(I2C_ISR_STOPF);
	if (st != BQ_OK) { return st; }
	I2C1->ICR = I2C_ICR_STOPCF;
	return BQ_OK;
}

/* ===========================================================================
 * Register access
 * ===========================================================================*/

bq_status_t BQ25798_ReadReg8(uint8_t reg, uint8_t *val) {
	return bq_i2c_read(reg, val, 1u);
}

bq_status_t BQ25798_WriteReg8(uint8_t reg, uint8_t val) {
	return bq_i2c_write(reg, &val, 1u);
}

bq_status_t BQ25798_ReadReg16(uint8_t reg, uint16_t *val) {
	uint8_t b[2];
	bq_status_t st = bq_i2c_read(reg, b, 2u);   // b[0]=MSB, b[1]=LSB
	if (st == BQ_OK) {
		*val = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
	}
	return st;
}

bq_status_t BQ25798_WriteReg16(uint8_t reg, uint16_t val) {
	uint8_t b[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFFu) };
	return bq_i2c_write(reg, b, 2u);
}

bq_status_t BQ25798_UpdateBits(uint8_t reg, uint8_t mask, uint8_t val) {
	uint8_t cur;
	bq_status_t st = BQ25798_ReadReg8(reg, &cur);
	if (st != BQ_OK) {
		return st;
	}
	uint8_t next = (uint8_t)((cur & ~mask) | (val & mask));
	if (next == cur) {
		return BQ_OK;
	}
	return BQ25798_WriteReg8(reg, next);
}

/* ===========================================================================
 * Init
 * ===========================================================================*/

bq_status_t BQ25798_Init(void) {
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PB6 = SCL, PB7 = SDA: AF6 (I2C1), open-drain. Bus pull-ups are external
	 * (on the board's 3V3 rail), so no internal pull. */
	gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Mode      = GPIO_MODE_AF_OD;
	gpio.Pull      = GPIO_NOPULL;
	gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF6_I2C1;
	HAL_GPIO_Init(GPIOB, &gpio);

	// I2C1 kernel clock defaults to PCLK (CCIPR I2C1SEL = 00) = 48 MHz.
	__HAL_RCC_I2C1_CLK_ENABLE();

	/* Free the bus if a slave is mid-transaction holding SDA low, then bring
	 * the peripheral up (bq_i2c_periph_reset programs TIMINGR and sets PE). */
	bq_i2c_bus_clear();
	bq_i2c_periph_reset();

	// Confirm a BQ25798 is actually responding before reporting success.
	uint8_t info;
	bq_status_t st = BQ25798_ReadReg8(BQ_REG_PART_INFO, &info);
	if (st != BQ_OK) {
		return st;
	}
	if (((info >> 3) & 0x7u) != BQ25798_PART_NUMBER) {
		return BQ_BADID;
	}
	return BQ_OK;
}

/* ===========================================================================
 * Configuration helpers
 * ===========================================================================*/

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
	if (v < lo) { return lo; }
	if (v > hi) { return hi; }
	return v;
}

bq_status_t BQ25798_Configure(const bq_config_t *cfg) {
	// Per-cell default minimal system voltage (REG00), index by cell_count-1.
	static const uint16_t vsysmin_mV[4] = { 3500u, 7000u, 9000u, 12000u };

	uint8_t cells = cfg->cell_count;
	if (cells < 1u) { cells = 1u; }
	if (cells > 4u) { cells = 4u; }

	bq_status_t st;

	/* Watchdog first: disabling (or widening) it stops the charge-related
	 * registers from snapping back to POR defaults on a timeout. */
	st = BQ25798_SetWatchdog(cfg->watchdog);
	if (st != BQ_OK) { return st; }

	// Cell count (REG0A[7:6]) and minimal system voltage (REG00[5:0]).
	st = BQ25798_UpdateBits(BQ_REG_RECHG_CTRL, 0xC0u,
							(uint8_t)((cells - 1u) << 6));
	if (st != BQ_OK) { return st; }

	uint16_t vsys = vsysmin_mV[cells - 1u];
	st = BQ25798_WriteReg8(BQ_REG_MIN_SYS_V, (uint8_t)((vsys - 2500u) / 250u));
	if (st != BQ_OK) { return st; }

	// Voltage / current setpoints (each range-clamped in its helper).
	st = BQ25798_SetChargeVoltage_mV(cfg->charge_mV);        if (st != BQ_OK) { return st; }
	st = BQ25798_SetChargeCurrent_mA(cfg->charge_mA);        if (st != BQ_OK) { return st; }
	st = BQ25798_SetInputCurrentLimit_mA(cfg->input_ilim_mA);if (st != BQ_OK) { return st; }
	st = BQ25798_SetInputVoltageLimit_mV(cfg->input_vlim_mV);if (st != BQ_OK) { return st; }

	// Pre-charge (REG08[5:0]) and termination (REG09[4:0]); both 40 mA/step.
	uint16_t ip = cfg->iprechg_mA;
	if (ip < 40u)   { ip = 40u; }
	if (ip > 2000u) { ip = 2000u; }
	st = BQ25798_UpdateBits(BQ_REG_PRECHG_CTRL, 0x3Fu, (uint8_t)(ip / 40u));
	if (st != BQ_OK) { return st; }

	uint16_t it = cfg->iterm_mA;
	if (it < 40u)   { it = 40u; }
	if (it > 1000u) { it = 1000u; }
	st = BQ25798_UpdateBits(BQ_REG_TERM_CTRL, 0x1Fu, (uint8_t)(it / 40u));
	if (st != BQ_OK) { return st; }

	// Make sure the input is live and charging is enabled.
	st = BQ25798_EnableHIZ(0);      if (st != BQ_OK) { return st; }
	st = BQ25798_EnableCharging(1); if (st != BQ_OK) { return st; }
	return BQ_OK;
}

bq_status_t BQ25798_Reset(void) {
	// REG_RST self-clears once the reset completes.
	return BQ25798_UpdateBits(BQ_REG_TERM_CTRL, BQ_TERM_CTRL_REG_RST,
							  BQ_TERM_CTRL_REG_RST);
}

bq_status_t BQ25798_SetChargeVoltage_mV(uint16_t mv) {
	mv = clamp_u16(mv, 3000u, 18800u);
	return BQ25798_WriteReg16(BQ_REG_CHG_V_LIM, (uint16_t)(mv / 10u)); // 10 mV LSB
}

bq_status_t BQ25798_SetChargeCurrent_mA(uint16_t ma) {
	ma = clamp_u16(ma, 50u, 5000u);
	return BQ25798_WriteReg16(BQ_REG_CHG_I_LIM, (uint16_t)(ma / 10u)); // 10 mA LSB
}

bq_status_t BQ25798_SetInputCurrentLimit_mA(uint16_t ma) {
	ma = clamp_u16(ma, 100u, 3300u);
	return BQ25798_WriteReg16(BQ_REG_IN_I_LIM, (uint16_t)(ma / 10u)); // 10 mA LSB
}

bq_status_t BQ25798_SetInputVoltageLimit_mV(uint16_t mv) {
	mv = clamp_u16(mv, 3600u, 22000u);
	return BQ25798_WriteReg8(BQ_REG_IN_V_LIM, (uint8_t)(mv / 100u));  // 100 mV LSB
}

bq_status_t BQ25798_EnableCharging(uint8_t enable) {
	return BQ25798_UpdateBits(BQ_REG_CHG_CTRL0, BQ_CTRL0_EN_CHG,
							  enable ? BQ_CTRL0_EN_CHG : 0u);
}

bq_status_t BQ25798_EnableHIZ(uint8_t enable) {
	return BQ25798_UpdateBits(BQ_REG_CHG_CTRL0, BQ_CTRL0_EN_HIZ,
							  enable ? BQ_CTRL0_EN_HIZ : 0u);
}

bq_status_t BQ25798_SetWatchdog(bq_watchdog_t sel) {
	return BQ25798_UpdateBits(BQ_REG_CHG_CTRL1, BQ_CTRL1_WATCHDOG_Msk,
							  (uint8_t)sel);
}

bq_status_t BQ25798_KickWatchdog(void) {
	// WD_RST self-clears after the timer is reloaded.
	return BQ25798_UpdateBits(BQ_REG_CHG_CTRL1, BQ_CTRL1_WD_RST,
							  BQ_CTRL1_WD_RST);
}

bq_status_t BQ25798_EnableADC(uint8_t enable, uint8_t one_shot) {
	uint8_t mask = BQ_ADC_CTRL_EN | BQ_ADC_CTRL_ONESHOT;
	uint8_t val  = (uint8_t)((enable   ? BQ_ADC_CTRL_EN      : 0u) |
							 (one_shot ? BQ_ADC_CTRL_ONESHOT : 0u));
	return BQ25798_UpdateBits(BQ_REG_ADC_CTRL, mask, val);
}

/* ===========================================================================
 * Telemetry / status
 * ===========================================================================*/

bq_status_t BQ25798_ReadADC(bq_adc_t *out) {
	uint16_t raw;
	bq_status_t st;

	st = BQ25798_ReadReg16(BQ_REG_VBUS_ADC, &raw); if (st != BQ_OK) return st;
	out->vbus_mV = raw;                                   // 1 mV LSB
	st = BQ25798_ReadReg16(BQ_REG_VBAT_ADC, &raw); if (st != BQ_OK) return st;
	out->vbat_mV = raw;
	st = BQ25798_ReadReg16(BQ_REG_VSYS_ADC, &raw); if (st != BQ_OK) return st;
	out->vsys_mV = raw;
	st = BQ25798_ReadReg16(BQ_REG_IBUS_ADC, &raw); if (st != BQ_OK) return st;
	out->ibus_mA = (int16_t)raw;                          // signed, 1 mA LSB
	st = BQ25798_ReadReg16(BQ_REG_IBAT_ADC, &raw); if (st != BQ_OK) return st;
	out->ibat_mA = (int16_t)raw;
	st = BQ25798_ReadReg16(BQ_REG_TDIE_ADC, &raw); if (st != BQ_OK) return st;
	out->tdie_C_x10 = (int16_t)((int16_t)raw * 5);        // 0.5 C LSB -> 0.1 C
	return BQ_OK;
}

bq_status_t BQ25798_GetChargeState(bq_chg_state_t *state) {
	uint8_t v;
	bq_status_t st = BQ25798_ReadReg8(BQ_REG_CHG_STAT1, &v);
	if (st == BQ_OK) {
		*state = (bq_chg_state_t)((v >> 5) & 0x7u);
	}
	return st;
}

bq_status_t BQ25798_ReadStatus(uint8_t status[5]) {
	return bq_i2c_read(BQ_REG_CHG_STAT0, status, 5u);   // REG1B..REG1F
}

bq_status_t BQ25798_ReadFaults(uint8_t faults[2]) {
	return bq_i2c_read(BQ_REG_FAULT_STAT0, faults, 2u); // REG20..REG21
}
