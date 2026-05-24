/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lowpower.h
  * @brief   Auto-standby with wake on PA2/PA4.
  *
  *  Wake sources (active HIGH):
  *      PA2 -> PWR_WKUP4
  *      PA4 -> PWR_WKUP2
  *
  *  When both lines have read low for LOWPOWER_IDLE_MS, the MCU enters Standby.
  *  A high level on either line then wakes it -- which on STM32 Standby is a
  *  full system reset, so main() re-runs and the hue cycle resumes.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef LOWPOWER_H
#define LOWPOWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Both wake lines must read low continuously for this long before standby. */
#ifndef LOWPOWER_IDLE_MS
#define LOWPOWER_IDLE_MS   5000u
#endif

void    LowPower_Init(void);              /* configure PA2/PA4 + arm wake pins   */
uint8_t LowPower_WokeFromStandby(void);   /* 1 if this reset was a standby wake  */
void    LowPower_Task(void);              /* call each loop; standby when idle   */

#ifdef __cplusplus
}
#endif
#endif /* LOWPOWER_H */
