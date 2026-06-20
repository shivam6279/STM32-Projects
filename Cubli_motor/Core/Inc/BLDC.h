#ifndef _BLDC_H
#define _BLDC_H

#include "main.h"
#include <math.h>
#include "PID.h"

#define ENCODER_RES 2048.0f // 4096.0f
#define ENCODER_RES_MASK 0x7FF // 0xFFF

#define ENC_LATENCY 0.0f // 0.00001f

#define LUT_SIZE 720 // For one quadrant of a time period
#define LUT_120_SHIFT (LUT_SIZE / 3)

// #define FOC_TIMER_ON() (TIM3->CR1 |= 1)
// #define FOC_TIMER_OFF() (TIM3->CR1 &= ~1)

// Wrap an angle in degrees to [0, 360). The integer-multiple subtraction
// handles any finite magnitude in one shot (no fmodf); the conditionals then
// fold the remaining (-360, 360) range into [0, 360).
static inline float WrapAngle360(float angle) {
	angle -= 360.0f * (int)(angle * 0.002777778f);
	if(angle < 0.0f) {
		angle += 360.0f;
	} else if(angle > 360.0f) {
		angle -= 360.0f;
	}
	return angle;
}

typedef enum motor_waveform_type {
	MOTOR_FOC_TORQUE,	// Iq/Id closed loop current control, with torque input and friction compensation
	MOTOR_FOC_IQ_ID,	// Default Iq/Id closed loop current control
	MOTOR_FOC_VQ_ID,	// Id closed loop (0A setpoint) and Vq open loop
	MOTOR_SVPWM,		// Vq open loop. Vd = 0 open loop
	MOTOR_TRAPEZOID,	// 6 step sensored trapezoid
	MODE_SENSORLESS		// 6 step sensorless trapezoid
} motor_waveform_type;

typedef enum motor_mode {
	MODE_OFF,
	MODE_POWER,
	MODE_RPM,
	MODE_POS
} motor_mode;

typedef struct motor_t {
	char name[20];
	uint8_t polepairs;
	float kv; // Mechanical rpm/volt
	float r_p2p; // Phase to phase resistance
	float l_p2p; // Phase to phase inductance
	float stiction;
	float coulomb;
	float viscous;
} motor_t;

// Motor preset table — single source of truth for the built-in motors.
// To add a motor, add ONE X() row below: the index enum (MOTOR_LIST_*),
// MOTOR_LIST_SIZE, and the motor_list[] array entry are all generated from it.
//
//       id            name       pp   kv       r_p2p     l_p2p     stic   coul   visc
#define MOTOR_TABLE \
	X(MAD3506,     "mad3506",      7,  400.0f,  240E-3f,  180E-6f,  0.25f, 0.0f,  0.0f)     \
	X(MAD4006,     "mad4006",     12,  740.0f,   51E-3f,   25E-6f,  0.5f,  0.26f, 0.0f)     \
	X(FLYSKY,      "flysky",       7,  750.0f,   92E-3f,   35E-6f,  0.0f,  0.0f,  0.0f)     \
	X(TMOTOR_2806, "tmotor_2806",  7,  400.0f,   1.8f,    190E-6f,  0.0f,  0.0f,  0.0f)     \
	X(TMOTOR_4004, "tmotor_4004", 12,  400.0f,   0.4f,    190E-6f,  0.0f,  0.0f,  0.0f)     \
	X(MT2204,      "mt2204",       7, 2300.0f,  200E-3f, 18.5E-6f,  0.2f,  0.05f, 0.00005f)

typedef enum motor_index {
#define X(id, ...) MOTOR_LIST_##id,
	MOTOR_TABLE
#undef X
	MOTOR_LIST_SIZE
} motor_index;

extern motor_waveform_type waveform_mode;
extern motor_mode mode;

extern PID pid_angle, pid_rpm, pid_focIq, pid_focId;

extern float phase_delay;
#define LPF_PHASE 0.5f

extern float motor_zero_angle;
extern float encoder_LUT[];
extern float encoder_calib_data[32];

extern unsigned char comparator, comp_u, comp_v, comp_w;

extern motor_t *motor_active;
extern motor_t motor_list[MOTOR_LIST_SIZE];
extern float motor_pole_pairs;
extern uint8_t motor_direction, enc_direction;

extern float torque_setpoint;
extern float Vq_setpoint;
extern volatile float foc_id, foc_iq;
extern volatile float angle_el;
extern float sin_el, cos_el;

extern float thermal_energy;

extern volatile uint8_t fault_latched, temp_fault;
void ClearFault();
void UpdateFaultLED();

void MotorPhase(int8_t, float);
void MotorPhasePWM(float, float, float);
void MotorOff();
void MotorShort(float);

void init_encoder_lut();
void interpolate_encoder_lut(float[], uint16_t);
uint8_t MotorPIDInit(motor_t*);

bool bemf_phase(unsigned char);
void SensorlessStart(float);

void setPhaseVoltage(float, float, float);
float normalizeAngle(float);
void ResetMotorPID();

void reset_motion_observer();
void SetRPM(float);
void SetPosition(float);
float GetPositionRaw();
float GetPosition();
void ResetPosition();
float GetRPM();
float GetAcc();
//extern float wave_lut(uint16_t*, float);

#endif
