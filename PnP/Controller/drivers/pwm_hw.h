#ifndef _PWM_HW_H_
#define _PWM_HW_H_

#include <inttypes.h>

// The timer side of pwm.h: pwm.c on the board, a stub in the host tests. Only pwm_profile.c uses
// these, everything else goes through pwm.h so a strike in progress isn't left running

void PwmHwSet(uint8_t ch, float duty);	// 0.0 - 1.0, from the next period
float PwmHwGet(uint8_t ch);
void PwmHwAllOff();						// takes effect immediately

#endif
