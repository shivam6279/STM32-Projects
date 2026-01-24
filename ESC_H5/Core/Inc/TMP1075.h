#ifndef _TMP1075_H_
#define _TMP1075_H_

#include <stdbool.h>
#include <inttypes.h>

#define TMP1075_ADDR 0x48

void TMP1075Init();
bool TMP1075_getRawTemp(unsigned char, int16_t*);
bool TMP1075_getTemp(unsigned char, float*);

#endif
