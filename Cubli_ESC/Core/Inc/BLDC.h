#ifndef __BLDC_H
#define __BLDC_H

#include "main.h"

#define ENCODER_RES 4096

#define SVPWM_SIZE 720
#define SVPWM_INCREMENT (SVPWM_SIZE / 3)
#define POLE_PAIRS 7.0f
#define DEG_PER_POLE_PAIR  (float)(360.0 / POLE_PAIRS)

extern float SVPWM_table[SVPWM_SIZE];

extern volatile float pre_pos, position, rpm, power;
extern volatile uint8_t motor_mode;

#define PHASE_DIFF 90.0f
#define RPM_LPF 0.99f

extern void setPhaseVoltage(float, float);
extern void set_motor_pwm(uint16_t, uint16_t, uint16_t); 
extern void BLDC_phase(unsigned char phase, float p);
extern void BEMF_phase(uint8_t);
extern uint8_t get_COMP_value(uint8_t);
extern void motor_on();
extern void motor_off();

extern int32_t motor_zero_offset;
extern volatile uint8_t current_phase;
extern volatile float phase_timing;
extern volatile uint8_t zero_crossing_flag;
#endif
