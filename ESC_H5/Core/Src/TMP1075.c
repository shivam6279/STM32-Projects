#include "TMP1075.h"
#include "bitbang_I2C.h"
#include "pic32.h"
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>

void TMP1075Init() {	
	I2C_WriteRegisters(TMP1075_ADDR, (unsigned char[2]){0x01, 0b01000100}, 2);
	I2C_WriteRegisters(TMP1075_ADDR, (unsigned char[2]){0x02, 0b01000100}, 2);

	I2C_WriteRegisters(TMP1075_ADDR + 1, (unsigned char[2]){0x01, 0b01000100}, 2);
	I2C_WriteRegisters(TMP1075_ADDR + 1, (unsigned char[2]){0x02, 0b01000100}, 2);
}

bool TMP1075_getRawTemp(unsigned char addr_offset, int16_t *temp) {
	unsigned char t[2];
	if(!I2C_ReadRegisters(TMP1075_ADDR + addr_offset, 0x00, t, 2)) {
		return false;
	}

	*temp = (int16_t)(t[0] << 8 | (t[1] & 0xF0));
	
	return true;
}

bool TMP1075_getTemp(unsigned char addr_offset, float *temp) {
	int16_t t;
	if(!TMP1075_getRawTemp(addr_offset, &t)) {
		return false;
	}

	*temp = (float)t * 0.0625f / 16.0;
	
	return true;
}
