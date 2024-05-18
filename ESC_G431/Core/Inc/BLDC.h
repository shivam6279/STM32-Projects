#ifndef __BLDC_H
#define __BLDC_H

#include "main.h"

#define ENCODER_RES 8192

#define MAX_PWM 3500.0
#define SVPWM_SIZE 360
#define SVPWM_INCREMENT (SVPWM_SIZE / 3)
#define POLE_PAIRS 7.0f
#define DEG_PER_POLE_PAIR  (float)(360.0 / POLE_PAIRS)

extern const float SVPWM_table[SVPWM_SIZE];
extern const float SPWM_table[SVPWM_SIZE];

extern volatile float pre_pos, position, rpm, power;
extern volatile uint8_t motor_mode;

#define MOTOR_ZERO_ANGLE 22.0f
#define PHASE_DIFF 90.0f
#define RPM_LPF 0.99f

extern void setPhaseVoltage(float, float);
extern void set_motor_pwm(uint16_t, uint16_t, uint16_t); 
extern void motor_on();
extern void motor_off();

#endif
