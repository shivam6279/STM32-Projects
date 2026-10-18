#ifndef _PAD_H_
#define _PAD_H_

#include <stdbool.h>
#include <inttypes.h>

// What the Xbox controller does. Off at every boot, picked with "pad mode" in diags.
//   off   nothing, the state is only shown (pad, pad -s)
//   jog   left stick = X / Y, RT / LT = Z up / down, speed in mm/s (pad speed). Sticks have to
//         be centred once before anything moves (also after the pad drops out and comes back)

#define PAD_MODE_OFF	0
#define PAD_MODE_JOG	1
#define PAD_MODES		2

extern float pad_speed_xy;	// mm/s, full stick
extern float pad_speed_z;	// mm/s, full trigger

void PadTask();						// main loop
uint8_t PadMode();
const char *PadModeName(uint8_t mode);
bool PadSetMode(uint8_t mode);		// prints why not
bool PadArmed();					// jog: sticks were centred, moving on input

#endif
