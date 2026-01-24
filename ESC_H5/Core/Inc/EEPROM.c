#include "EEPROM.h"
#include <xc.h>
#include <inttypes.h>
#include "pic32.h"

#define EEKEY1 0xEDB7
#define EEKEY2 0x1248

#define EEPROM_BOARD_ID_ADDR 0x04
#define EEPROM_MOTOR_OFFSET_ADDR 0x14
#define EEPROM_MOTOR_POLEPAIRS_ADDR 0x18
#define EEPROM_MOTOR_DIRECTION_ADDR 0x1C
#define EEPROM_PID_ANGLE_ADDR 0x22
#define EEPROM_PID_RPM_ADDR 0x28
#define EEPROM_PID_FOC_IQ_ADDR 0x34
#define EEPROM_PID_FOC_ID_ADDR 0x40
#define EEPROM_ENCODER_CALIB_ADDR 0xA0

uint8_t eeprom_board_id;
float eeprom_zero_offset;
uint8_t eeprom_pole_pairs;
uint8_t eeprom_motor_direction;
float eeprom_encoder_calib_data[32];
float eeprom_pid_angle[3];
float eeprom_pid_rpm[3];
float eeprom_pid_foc_iq[3];
float eeprom_pid_foc_id[3];

unsigned char board_id = 0;

void EEPROM_init() {
	
	CFGCON2bits.EEWS = 5;
	
	EECONbits.ON = 1;

	while (EECONbits.RDY == 0); // Wait until EEPROM is ready (~125 us)

	EECONbits.WREN = 1; // Enable writing to the EEPROM
	EECONbits.CMD = 0b100; // Set the command to Configuration Write

	EEADDR = 0x00; // Addr 0x00 = DEVEE1;
	EEDATA = DEVEE0;
	EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
	EEKEY = EEKEY2;
	EECONSET = _EECON_RW_MASK;
	while (EECONbits.RW); // desired

	EEADDR = 0x04; // Addr 0x04 = DEVEE2;
	EEDATA = DEVEE1;
	EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
	EEKEY = EEKEY2;
	EECONSET = _EECON_RW_MASK;
	while (EECONbits.RW); // desired

	EEADDR = 0x08;// Addr 0x08 = DEVEE3;
	EEDATA = DEVEE2;
	EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
	EEKEY = EEKEY2;
	EECONSET = _EECON_RW_MASK;
	while (EECONbits.RW); // desired

	EEADDR = 0x0C; // Addr 0x08 = DEVEE3;
	EEDATA = DEVEE3;
	EEKEY = EEKEY1; // Unlock the EEPROM to enable writing
	EEKEY = EEKEY2;
	EECONSET = _EECON_RW_MASK;
	while (EECONbits.RW); // desired

	EECONbits.WREN = 0; // Turn off writes.    
}

uint32_t EEPROM_read(uint32_t ee_addr) {
	uint32_t data;
	
	EEADDR = ee_addr & 0xFFC; // Set address on 32-bit boundary
	EECONbits.CMD = 0; // Load CMD<2:0> with
	// Data EEPROM read command
	EECONbits.WREN = 0; // Access for read

	EEKEY = EEKEY1; // Write unlock sequence
	EEKEY = EEKEY2;
	EECONSET = _EECON_RW_MASK; // Start the operation 

	while (EECONbits.RW==1); // Wait until read is complete
	data = EEDATA; // Read the data
	
	return data;
}

float EEPROM_readFloat(uint32_t ee_addr) {
	uint32_t data = EEPROM_read(ee_addr);
	return *(float*)(char*)&data;
}

void EEPROM_write(uint32_t ee_addr, uint32_t ee_data)
{
	EECONbits.CMD = 1; // Load CMD<2:0> with write command
	EECONbits.WREN = 1; // Access for write

	EEADDR = ee_addr & 0xFFC; // Load address on a 32-bit boundary
	EEDATA = ee_data;

	EEKEY = EEKEY1; // Write unlock sequence
	EEKEY = EEKEY2;
	EECONSET = _EECON_RW_MASK; 

	while (EECONbits.RW == 1);
}

void EEPROM_writeFloat(uint32_t ee_addr, float ee_data) {
	EEPROM_write(ee_addr, *(uint32_t*)(char*)&ee_data);
}

void EEPROM_readAll() {
	uint8_t i;
	
	eeprom_board_id = EEPROM_read(EEPROM_BOARD_ID_ADDR);

	eeprom_zero_offset = EEPROM_readFloat(EEPROM_MOTOR_OFFSET_ADDR);
	eeprom_pole_pairs = EEPROM_read(EEPROM_MOTOR_POLEPAIRS_ADDR);
	
	eeprom_motor_direction = EEPROM_read(EEPROM_MOTOR_DIRECTION_ADDR);

	eeprom_pid_angle[0] = EEPROM_readFloat(EEPROM_PID_ANGLE_ADDR);
	eeprom_pid_angle[1] = EEPROM_readFloat(EEPROM_PID_ANGLE_ADDR + 4);
	eeprom_pid_angle[2] = EEPROM_readFloat(EEPROM_PID_ANGLE_ADDR + 8);

	eeprom_pid_rpm[0] = EEPROM_readFloat(EEPROM_PID_RPM_ADDR);
	eeprom_pid_rpm[1] = EEPROM_readFloat(EEPROM_PID_RPM_ADDR + 4);
	eeprom_pid_rpm[2] = EEPROM_readFloat(EEPROM_PID_RPM_ADDR + 8);

	eeprom_pid_foc_iq[0] = EEPROM_readFloat(EEPROM_PID_FOC_IQ_ADDR);
	eeprom_pid_foc_iq[1] = EEPROM_readFloat(EEPROM_PID_FOC_IQ_ADDR + 4);
	eeprom_pid_foc_iq[2] = EEPROM_readFloat(EEPROM_PID_FOC_IQ_ADDR + 8);

	eeprom_pid_foc_id[0] = EEPROM_readFloat(EEPROM_PID_FOC_ID_ADDR);
	eeprom_pid_foc_id[1] = EEPROM_readFloat(EEPROM_PID_FOC_ID_ADDR + 4);
	eeprom_pid_foc_id[2] = EEPROM_readFloat(EEPROM_PID_FOC_ID_ADDR + 8);

	for(i = 0; i < 32; i++) {
		eeprom_encoder_calib_data[i] = EEPROM_readFloat(EEPROM_ENCODER_CALIB_ADDR + 4*i);
	}
}

void EEPROM_writeAll() {
	uint8_t i;
	
	EEPROM_write(EEPROM_BOARD_ID_ADDR, eeprom_board_id);
	
	EEPROM_writeFloat(EEPROM_MOTOR_OFFSET_ADDR, eeprom_zero_offset);
	EEPROM_write(EEPROM_MOTOR_POLEPAIRS_ADDR, eeprom_pole_pairs);
	EEPROM_write(EEPROM_MOTOR_DIRECTION_ADDR, eeprom_motor_direction);

	EEPROM_writeFloat(EEPROM_PID_ANGLE_ADDR, eeprom_pid_angle[0]);
	EEPROM_writeFloat(EEPROM_PID_ANGLE_ADDR + 4, eeprom_pid_angle[1]);
	EEPROM_writeFloat(EEPROM_PID_ANGLE_ADDR + 8, eeprom_pid_angle[2]);

	EEPROM_writeFloat(EEPROM_PID_RPM_ADDR, eeprom_pid_rpm[0]);
	EEPROM_writeFloat(EEPROM_PID_RPM_ADDR + 4, eeprom_pid_rpm[1]);
	EEPROM_writeFloat(EEPROM_PID_RPM_ADDR + 8, eeprom_pid_rpm[2]);

	EEPROM_writeFloat(EEPROM_PID_FOC_IQ_ADDR, eeprom_pid_foc_iq[0]);
	EEPROM_writeFloat(EEPROM_PID_FOC_IQ_ADDR + 4, eeprom_pid_foc_iq[1]);
	EEPROM_writeFloat(EEPROM_PID_FOC_IQ_ADDR + 8, eeprom_pid_foc_iq[2]);

	EEPROM_writeFloat(EEPROM_PID_FOC_ID_ADDR, eeprom_pid_foc_id[0]);
	EEPROM_writeFloat(EEPROM_PID_FOC_ID_ADDR + 4, eeprom_pid_foc_id[1]);
	EEPROM_writeFloat(EEPROM_PID_FOC_ID_ADDR + 8, eeprom_pid_foc_id[2]);

	for(i = 0; i < 32; i++) {
		EEPROM_writeFloat(EEPROM_ENCODER_CALIB_ADDR + 4*i, eeprom_encoder_calib_data[i]);
	}
}

void write_encoderCalibration(float arr[], unsigned int len) {
	unsigned int i;
	EEPROM_write(EEPROM_ENCODER_CALIB_ADDR, *(uint32_t*)(char*)&len);
	for(i = 0; i < len; i++) {
		EEPROM_write(EEPROM_ENCODER_CALIB_ADDR + (i+1), *(uint32_t*)(char*)&arr[i]);
	}
}
