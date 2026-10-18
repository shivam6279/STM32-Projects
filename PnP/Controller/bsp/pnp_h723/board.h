#ifndef _BOARD_H_
#define _BOARD_H_

#include <stdbool.h>
#include <inttypes.h>
#include "stm32h7xx.h"
#include "pins.h"

#define STEPPER_EN_ACTIVE_LOW	1	// net STP_EN_L, checked: all 8 TMC2209s read ENN=1 with the pin high
#define DEBUG_LED_ACTIVE_HIGH	1	// TODO check schematic

void BoardStepperSafe();
void BoardIoInit();
void BoardLed(uint8_t led, bool on);
void BoardStepperEnable(bool enable);
bool BoardStepperEnabled();

#endif
