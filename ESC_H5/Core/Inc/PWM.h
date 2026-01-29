#ifndef _PWM_H_
#define _PWM_H_

#include "main.h"
#include <inttypes.h>

extern void MX_TIM1_Init(float freq);
extern void MX_TIM3_Init(void);
extern void MX_TIM15_Init(void);
extern void setServo_us(float);
extern void servoOff();

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern uint16_t PWM_MAX;

#endif
