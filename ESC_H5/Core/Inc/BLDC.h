#ifndef _BLDC_H
#define _BLDC_H

#include "main.h"
#include <math.h>
#include "PID.h"

#define ENCODER_RES 2048.0f//4096.0f
#define ENCODER_RES_MASK 0x7FF//0xFFF

#define ENC_LATENCY 0.00001f

#define LUT_SIZE 720 // For one quadrant of a time period
#define LUT_120_SHIFT (LUT_SIZE / 3)

// #define FOC_TIMER_ON() (TIM3->CR1 |= 1)
// #define FOC_TIMER_OFF() (TIM3->CR1 &= ~1)

typedef enum motor_waveform_type {
	MOTOR_FOC,
	MOTOR_SVPWM,
	MOTOR_SIN,
	MOTOR_SADDLE,
	MOTOR_TRAPEZOID,
	MODE_SENSORLESS
} motor_waveform_type;

typedef enum motor_mode {
	MODE_OFF,
	MODE_POWER,
	MODE_RPM,
	MODE_POS
} motor_mode;

extern volatile enum motor_waveform_type waveform_mode;
extern volatile enum motor_mode mode;

extern PID pid_angle, pid_rpm, pid_focIq, pid_focId;

extern volatile float phase_delay;
#define LPF_PHASE 0.5f

extern float motor_zero_angle;
extern float encoder_LUT[];
extern float encoder_calib_data[32];

extern volatile unsigned char comparator, comp_u, comp_v, comp_w;

extern float motor_pole_pairs;
extern uint8_t motor_direction, enc_direction;

extern volatile float foc_id, foc_iq;
extern volatile float isns_u, isns_v, isns_w;
extern volatile float angle_el;
extern volatile float sin_el, cos_el;

extern volatile float thermal_energy;

extern void MotorPhase(int8_t, float);
extern void MotorPhasePWM(float, float, float);
extern void MotorOff();
extern void MotorShort(float);

extern void init_encoder_lut();
extern void interpolate_encoder_lut(float[], unsigned int);
extern void MotorPIDInit();

extern bool bemf_phase(unsigned char);
extern void SensorlessStart(float);

extern void setPhaseVoltage(float, float, float);
extern float normalizeAngle(float);
extern void ResetMotorPID();

extern void reset_motion_observer();
extern void SetRPM(float);
extern void SetPower(float);
extern void SetPosition(float);
extern float GetPositionRaw();
extern float GetPosition();
extern float GetPower();
extern void ResetPosition();
extern float GetRPM();
extern float GetAcc();
//extern float wave_lut(uint16_t*, float);

#endif
