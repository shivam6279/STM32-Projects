#ifndef _TMP1075_H_
#define _TMP1075_H_

#include "main.h"

#define TMP1075_ADDR 0x48 // FET sensor, ADDR pin = 0

// Over-temperature cutoff (non-latched, auto-recovers below LIMIT - HYST)
#define TEMP_LIMIT 100.0f
#define TEMP_HYST 15.0f

// Poll/check divider on the 10 kHz TIM5 loop -> 10 Hz
#define TEMP_POLL_DIV 1000

extern volatile float tmp_temp;
extern volatile int16_t tmp_raw;

void TMP1075Init();
void TMP1075_Service();
uint8_t TMP1075_getRawTemp(int16_t*);
uint8_t TMP1075_getTemp(float*);

#endif
