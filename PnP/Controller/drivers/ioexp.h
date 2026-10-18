#ifndef _IOEXP_H_
#define _IOEXP_H_

#include <stdbool.h>
#include <inttypes.h>

// PI4IOE5V6416 IO expander (U1) on I2C_MAIN
// Pin 0-7 = P0_0-P0_7, 8-15 = P1_0-P1_7
// Pins only get pulled low or let go (input), never driven high

#define IOEXP_ADDR		0x20	// ADDR = GND, checked on the board
#define IOEXP_PINS		16

#define IOEXP_ESP_EN	0		// ESP_EN, 10k pull-up + 100nF. 0 = ESP32 in reset
#define IOEXP_ESP_BOOT	1		// ESP_BOOT = ESP32-C3 IO9. 0 at reset = ROM download mode

bool IoExpInit();							// false if it doesn't answer
bool IoExpLow(uint8_t pin, bool low);		// pull low / let go
bool IoExpIsLow(uint8_t pin);				// what we asked for, not the pin level
bool IoExpRead(uint16_t *levels);			// pin levels, bit n = pin n

#endif
