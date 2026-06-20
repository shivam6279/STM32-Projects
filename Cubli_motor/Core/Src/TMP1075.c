#include "TMP1075.h"
#include "main.h"
#include "BLDC.h"
#include <math.h>

// Single TMP1075 on the FETs (ADDR=0, 0x48). Service() (paced by TIM5 at 10 Hz)
// checks temp + kicks an IT read of the temp register; the Rx callback stores it.

extern I2C_HandleTypeDef hi2c1;

volatile float tmp_temp = 0;
volatile int16_t tmp_raw = 0;

static uint8_t tmp_rx[2];

void TMP1075Init() {
}

static void tmp_thermal_check() {
	if(tmp_temp >= TEMP_LIMIT) {
		temp_fault = 1;
		MotorOff();
		UpdateFaultLED();
	} else if(tmp_temp < (TEMP_LIMIT - TEMP_HYST)) {
		temp_fault = 0;
		UpdateFaultLED();
	}
}

void TMP1075_Service() {
	tmp_thermal_check();

	if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) {
		HAL_I2C_Mem_Read_IT(&hi2c1, TMP1075_ADDR << 1, 0x00,
			I2C_MEMADD_SIZE_8BIT, tmp_rx, 2);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(hi2c->Instance != I2C1) {
		return;
	}

	// 12-bit left-justified, 0.0625 C/LSB
	tmp_raw = (int16_t)(tmp_rx[0] << 8 | (tmp_rx[1] & 0xF0));
	tmp_temp = (float)tmp_raw * 0.0625f / 16.0f;
}

uint8_t TMP1075_getRawTemp(int16_t *temp) {
	*temp = tmp_raw;
	return 1;
}

uint8_t TMP1075_getTemp(float *temp) {
	*temp = tmp_temp;
	return 1;
}
