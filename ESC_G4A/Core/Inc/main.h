#ifndef __MAIN_H
#define __MAIN_H

#include "stm32g4xx_hal.h"

#define SYSCLK_FREQ 170000000UL

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);

#endif
