#ifndef _SPI_H_
#define _SPI_H_

#include "main.h"
#include <inttypes.h>

extern volatile uint16_t spi_angle;

extern void SPI1_init(float);
extern void SPI1_write16(uint16_t);

#endif
