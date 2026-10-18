#ifndef _PWM_H_
#define _PWM_H_

#include <inttypes.h>

// Coil driver outputs PWM1-8: each drives a logic level N-FET (SSM3K376R) low side switch
// with a flyback diode. Solenoids, relays, small motors.
// PWM1-8 nets, channel n = PWM(n+1). Which timer/channel each one is comes from the pin sheet.
// Frequency is per timer, so per group, not per channel.

#define PWM_CHANNELS	8
#define PWM_FREQ		25000	// above audible, small switching loss with direct gate drive

void PwmInit();
void PwmSet(uint8_t ch, float duty);	// 0.0 - 1.0
float PwmGet(uint8_t ch);
void PwmAllOff();			// takes effect immediately
float PwmMeasure(uint8_t ch);	// duty read back off the pin

#endif
