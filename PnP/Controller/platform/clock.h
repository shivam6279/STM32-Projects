#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <inttypes.h>

// 25MHz HSE -> PLL1 -> 520MHz SYSCLK, VOS0
// 550MHz needs the CPU_FREQ_BOOST option byte, not worth it for now

#define CLOCK_SYSCLK_HZ		520000000
#define CLOCK_HCLK_HZ		260000000
#define CLOCK_PCLK1_HZ		130000000
#define CLOCK_PCLK2_HZ		130000000
#define CLOCK_TIM_APB1_HZ	260000000	// 2x PCLK1 since APB1 prescaler != 1, TODO RM0468 ref
#define CLOCK_FDCAN_HZ		80000000	// PLL2Q

void ClockInit();
void ClockUsbInit();	// PLL3Q = 48MHz -> USB
void ClockCanInit();	// PLL2Q = 80MHz -> FDCAN
uint32_t Millis();

#endif
