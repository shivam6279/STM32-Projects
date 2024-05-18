#ifndef __TONES_H
#define __TONES_H

#include "main.h"

#define TONE_POWER 0.075f

extern void play_tone(float);
extern void tone_off();
extern void TIM4_update_freq(float);
extern void play_music();

#endif
