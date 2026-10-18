#ifndef _BUTTONS_H_
#define _BUTTONS_H_

#include <stdbool.h>
#include <inttypes.h>

// BTN1-8 on PD0-PD7 (order from the pin sheet), EXTI lines 0-7, interrupt on both edges.
// Button n = net BTN(n+1); bit n in the masks below.
// The ISR only records edges. ButtonsTask() (main loop) debounces: a new level is taken
// once the pin has been stable for BUTTON_DEBOUNCE_MS.
// Levels are 0/1 pin levels, polarity (pressed = 0 or 1) isn't known yet.

#define BUTTONS			8
#define BUTTON_DEBOUNCE_MS	10

void ButtonsInit();
void ButtonsTask();				// call from the main loop
void ButtonsIsr();				// EXTI lines 0-7
uint8_t ButtonsState();			// debounced levels, bit n = BTN(n+1)
uint8_t ButtonsRaw();			// pin levels right now
uint8_t ButtonsChanged();		// debounced changes since the last call (read and clear)
uint32_t ButtonsEdges(uint8_t n);	// raw edge count, shows bounce

#endif
