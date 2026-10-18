#include "i2c.h"
#include "clock.h"
#include "stm32h7xx.h"

#define TIMEOUT_MS 5

// Kernel clock = PCLK1 130MHz (I2C123SEL reset value). PRESC 12 -> 100ns ticks.
// SCLL 5.5us, SCLH 4.5us, SDADEL 0.5us, SCLDEL 1.3us -> ~95kHz with the sync delays
#define TIMING ((12u << I2C_TIMINGR_PRESC_Pos) | (12u << I2C_TIMINGR_SCLDEL_Pos) | (5u << I2C_TIMINGR_SDADEL_Pos) | \
	(44u << I2C_TIMINGR_SCLH_Pos) | (54u << I2C_TIMINGR_SCLL_Pos))

void I2cInit() {
	RCC->APB1LENR |= RCC_APB1LENR_I2C3EN;
	(void)RCC->APB1LENR;

	I2C3->CR1 = 0;
	I2C3->TIMINGR = TIMING;
	I2C3->CR1 = I2C_CR1_PE; // analog filter on, no interrupts
}

// PE off clears the state machine and flags
static bool Fail() {
	I2C3->CR1 &= ~I2C_CR1_PE;
	while (I2C3->CR1 & I2C_CR1_PE);
	I2C3->CR1 |= I2C_CR1_PE;
	return false;
}

// Wait for a flag, false on NACK or timeout
static bool WaitFlag(uint32_t flag) {
	uint32_t t = Millis();
	while (!(I2C3->ISR & flag)) {
		if (I2C3->ISR & I2C_ISR_NACKF) return false;
		if (Millis() - t > TIMEOUT_MS) return false;
	}
	return true;
}

static bool Stop() {
	if (!WaitFlag(I2C_ISR_STOPF)) return false;
	I2C3->ICR = I2C_ICR_STOPCF;
	return true;
}

static bool Idle() {
	uint32_t t = Millis();
	while (I2C3->ISR & I2C_ISR_BUSY) {
		if (Millis() - t > TIMEOUT_MS) return false;
	}
	I2C3->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
	return true;
}

bool I2cWrite(uint8_t addr, const uint8_t *buf, uint8_t len) {
	uint8_t i;

	if (!Idle()) return Fail();
	I2C3->CR2 = (addr << 1) | (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
	for (i = 0; i < len; i++) {
		if (!WaitFlag(I2C_ISR_TXIS)) return Fail();
		I2C3->TXDR = buf[i];
	}
	if (!Stop()) return Fail();
	return true;
}

bool I2cReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
	uint8_t i;

	if (!Idle()) return Fail();
	I2C3->CR2 = (addr << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	if (!WaitFlag(I2C_ISR_TXIS)) return Fail();
	I2C3->TXDR = reg;
	if (!WaitFlag(I2C_ISR_TC)) return Fail();

	I2C3->CR2 = (addr << 1) | I2C_CR2_RD_WRN | (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
	for (i = 0; i < len; i++) {
		if (!WaitFlag(I2C_ISR_RXNE)) return Fail();
		buf[i] = I2C3->RXDR;
	}
	if (!Stop()) return Fail();
	return true;
}
