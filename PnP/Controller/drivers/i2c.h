#ifndef _I2C_H_
#define _I2C_H_

#include <stdbool.h>
#include <inttypes.h>

// I2C_MAIN = I2C3, PA8 SCL / PC9 SDA, ~95kHz, blocking.
// 7 bit addresses, false on NACK / timeout

void I2cInit();
bool I2cWrite(uint8_t addr, const uint8_t *buf, uint8_t len);
bool I2cReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);	// write reg, restart, read

#endif
