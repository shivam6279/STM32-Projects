#ifndef _PAD_H_
#define _PAD_H_

#include <stdbool.h>
#include <inttypes.h>

// What the Xbox controller does. Off at every boot, picked with "pad mode" in diags.
//   off   nothing, the state is only shown (pad, pad -s)
//   jog   left stick = X / Y, RT / LT = Z up / down, speed in mm/s (pad speed). Sticks have to
//         be centred once before anything moves (also after the pad drops out and comes back)
//         LS toggles fast XY (pad speed fast). Pneumatics, off or PAD_PUMP_DUTY / PAD_VALVE_DUTY:
//         Y = pump (PWM0), LB = valve 0 (PWM1), RB = valve 1 (PWM2), all toggles.
//         They work before the sticks are centred, and go off when jog mode ends

#define PAD_MODE_OFF	0
#define PAD_MODE_JOG	1
#define PAD_MODES		2

#define PAD_PWM_PUMP	0
#define PAD_PWM_VALVE0	1
#define PAD_PWM_VALVE1	2
#define PAD_PUMP_DUTY	0.5f
#define PAD_VALVE_DUTY	0.5f

extern float pad_speed_xy;		// mm/s, full stick
extern float pad_speed_xy_fast;	// mm/s, full stick with LS fast on
extern float pad_speed_z;	// mm/s, full trigger

void PadTask();						// main loop
uint8_t PadMode();
const char *PadModeName(uint8_t mode);
bool PadSetMode(uint8_t mode);		// prints why not
bool PadArmed();					// jog: sticks were centred, moving on input
bool PadFast();						// jog: LS fast XY on
bool PadPump();
bool PadValve(uint8_t n);			// 0 / 1

#endif
