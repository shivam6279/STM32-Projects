#include "ioexp.h"
#include "i2c.h"

// Registers, one per 8 pin port (DS40821)
#define REG_INPUT0		0x00
#define REG_INPUT1		0x01
#define REG_OUTPUT0		0x02
#define REG_OUTPUT1		0x03
#define REG_CONFIG0		0x06	// 1 = input
#define REG_CONFIG1		0x07

static uint16_t config = 0xFFFF; // copy of the config registers

static bool WriteReg(uint8_t reg, uint8_t val) {
	uint8_t buf[2] = { reg, val };
	return I2cWrite(IOEXP_ADDR, buf, 2);
}

bool IoExpInit() {
	I2cInit();

	// Outputs low first so a pin can only ever pull low
	config = 0xFFFF;
	return WriteReg(REG_OUTPUT0, 0) && WriteReg(REG_OUTPUT1, 0) &&
		WriteReg(REG_CONFIG0, 0xFF) && WriteReg(REG_CONFIG1, 0xFF);
}

bool IoExpLow(uint8_t pin, bool low) {
	if (pin >= IOEXP_PINS) return false;
	if (low) config &= ~(1 << pin);
	else config |= 1 << pin;

	if (pin < 8) return WriteReg(REG_CONFIG0, config & 0xFF);
	return WriteReg(REG_CONFIG1, config >> 8);
}

bool IoExpIsLow(uint8_t pin) {
	return pin < IOEXP_PINS && !((config >> pin) & 1);
}

bool IoExpRead(uint16_t *levels) {
	uint8_t p0, p1;

	if (!I2cReadReg(IOEXP_ADDR, REG_INPUT0, &p0, 1)) return false;
	if (!I2cReadReg(IOEXP_ADDR, REG_INPUT1, &p1, 1)) return false;
	*levels = p0 | (p1 << 8);
	return true;
}
