// Auto-standby; wakes on PA2 (touch, WKUP4) or PA4 (USB 3V3, WKUP2).
#ifndef LOWPOWER_H
#define LOWPOWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Both wake lines must read low continuously for this long before standby.
#ifndef LOWPOWER_IDLE_MS
#define LOWPOWER_IDLE_MS   5000u
#endif

void    LowPower_Init(void);              // configure PA2/PA4 + arm wake pins
uint8_t LowPower_WokeFromStandby(void);   // 1 if this reset was a standby wake
void    LowPower_Task(void);              // call each loop; standby when idle
void    LowPower_NoteActivity(void);      // restart the idle timer (stay awake)
void    LowPower_EnterStandby(void);      // standby now (wake = reset)
uint8_t LowPower_UsbPresent(void);        // 1 if PA4 (REGN 3V3 rail) is high

#ifdef __cplusplus
}
#endif
#endif // LOWPOWER_H
