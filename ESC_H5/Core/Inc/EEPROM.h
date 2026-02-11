#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "main.h"
#include <assert.h>

#define EEPROM_STRUCT_SIZE 3 + (1 + 4*3 + 32 + 3) * 4

typedef struct __attribute__((aligned(16))) {
	uint8_t board_id;
	uint8_t pole_pairs;
	uint8_t motor_direction;
	float zero_offset;
	
	float pid_foc_iq[3];
	float pid_foc_id[3];
	float pid_angle[3];
	float pid_rpm[3];

	float enc_calib[32];

	float diags_power;
	float tone_power;
	float tone_amplitude;

	uint8_t _auto_pad[ (16 - (EEPROM_STRUCT_SIZE % 16)) % 16];
} eeprom_data_t;

_Static_assert((sizeof(eeprom_data_t) % 16) == 0, "eeprom_data_t size must be a multiple of 16 bytes for STM32H5 Flash");

#define EEPROM_START_ADDR    0x081FE000
#define EEPROM_FLASH_BANK    FLASH_BANK_2
#define EEPROM_FLASH_SECTOR  127

extern eeprom_data_t eeprom_data;

extern void ee_read();
extern void ee_write();


#endif
