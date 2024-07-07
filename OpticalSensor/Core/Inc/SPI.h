#ifndef __SPI_H
#define __SPI_H

#include "main.h"

extern void SPI_init();
extern void SPI_write(uint8_t);
extern uint8_t SPI_read();

extern void PMW3901_init();
extern void PMW3901_read_deltas(int16_t*, int16_t*, uint8_t*, uint8_t*);
extern void write_reg(uint8_t, uint8_t);
extern uint8_t read_reg(uint8_t);

#endif
