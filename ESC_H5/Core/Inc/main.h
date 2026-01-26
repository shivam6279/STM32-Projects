#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include "stm32h5xx_hal.h"

#define MOTOR_TIM TIM1

#define CCR_U CCR1
#define CCR_V CCR2
#define CCR_W CCR3

#define CCNP_U TIM_CCER_CC1NP
#define CCNP_V TIM_CCER_CC2NP
#define CCNP_W TIM_CCER_CC3NP

#define CCE_U TIM_CCER_CC1E
#define CCE_V TIM_CCER_CC2E
#define CCE_W TIM_CCER_CC3E

#define ENC_TIM TIM2

#define LED0_ON() (GPIOC->ODR |= 1<<10)
#define LED0_OFF() (GPIOC->ODR &= ~(1<<10))
#define LED0_TOG() (GPIOC->ODR ^= 1<<10)

#define LED1_ON() (GPIOC->ODR |= 1<<15)
#define LED1_OFF() (GPIOC->ODR &= ~(1<<15))
#define LED1_TOG() (GPIOC->ODR ^= 1<<15)

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

extern uint8_t board_id;

#ifdef __cplusplus
}
#endif

#endif
