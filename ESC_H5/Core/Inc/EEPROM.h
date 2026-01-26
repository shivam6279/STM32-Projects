#ifndef _EE_H_
#define _EE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "main.h"

typedef struct __attribute__((packed)) {
	uint8_t board_id;
	float zero_offset;
	uint8_t pole_pairs;
	uint8_t motor_direction;
	
	float pid_foc_iq[3];
	float pid_foc_id[3];
	float pid_angle[3];
	float pid_rpm[3];

	float enc_calib[32];

} eeprom_data_t;

extern eeprom_data_t eeprom_data;

/* Handle struct of EEPROM */
typedef struct {
	uint8_t                *data;
	uint32_t               size;
	uint32_t               page_sector_size;
	uint32_t               address;
	uint8_t                page_sector_number;
#if (defined FLASH_BANK_1) || (defined FLASH_BANK_2)
	uint8_t                bank_number;
#endif

} ee_t;

/* Initializes the EEPROM emulation module */
bool      ee_init(void *data, uint32_t size);

/* Retrieves the capacity of the EEPROM emulation area */
uint32_t  ee_capacity(void);

/* Formats the EEPROM emulation area */
bool      ee_format(void);

/* Reads data from the EEPROM emulation area */
void      ee_read(void);

/* Writes data to the EEPROM emulation area */
bool      ee_write(void);

#ifdef __cplusplus
}
#endif
#endif
