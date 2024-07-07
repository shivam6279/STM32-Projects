#include "I2C.h"
#include "main.h"

void I2C_init(float freq) {
	GPIOA->MODER &= 0b11 << 30;
	GPIOB->MODER &= 0b11 << 14;
}
