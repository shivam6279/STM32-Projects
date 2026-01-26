#ifndef _TMP1075_H_
#define _TMP1075_H_

#include "main.h"

#define TMP1075_ADDR 0x48

void TMP1075Init();
uint8_t TMP1075_getRawTemp(unsigned char, int16_t*);
uint8_t TMP1075_getTemp(unsigned char, float*);

#endif
