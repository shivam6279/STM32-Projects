#ifndef _PID_H_
#define _PID_H_

#include <inttypes.h>
#include <stdbool.h>

typedef struct {
	//Gains
	float kp, ki, kd;
	float input, p_input;
	float error, setpoint;
	float integral, derivative;
	float output;
	float lpf_der;

	float error_min, error_max;
	float integral_max, integral_min;
	float output_max, output_min;
	
	bool constrain_integral;
	bool constrain_error;
	bool constrain_output;
	
	bool compute_integral;
	bool compute_derivative;
} PID;

void PID_init(PID*);
void PID_reset(PID*);
void PID_setGain(PID*, float, float, float);

// Hot-path control math: defined inline here so the 50 kHz current loop and
// 10 kHz outer loops inline them at the call site (no cross-TU call / no LTO).
static inline float PID_clamp(float val, float min, float max) {
	return val > max ? max : val < min ? min : val;
}

static inline void PID_integrate(PID *pid, float deltat) {
	float error = pid->error;
	if(pid->constrain_error) {
		error = PID_clamp(pid->error, pid->error_min, pid->error_max);
	}
	pid->integral += pid->ki * error * deltat;
	if(pid->constrain_integral) {
		pid->integral = PID_clamp(pid->integral, pid->integral_min, pid->integral_max);
	}
}

static inline void PID_differentiate(PID *pid, float deltat) {
	pid->derivative += pid->lpf_der * ((pid->input - pid->p_input) / deltat - pid->derivative);
	pid->p_input = pid->input;
}

static inline float PID_compute(PID *pid, float input, float deltat) {
	float temp_output;
	uint8_t is_saturated;

	pid->input = input;
	pid->error = pid->setpoint - pid->input;

	is_saturated = pid->constrain_output && (pid->output <= pid->output_min || pid->output >= pid->output_max);

	// P
	temp_output = pid->kp * pid->error;

	// I
	if(pid->ki) {
		if(pid->compute_integral) {
			if(!is_saturated || (pid->error * pid->output) < 0) {
				PID_integrate(pid, deltat);
			}
		}
		temp_output += pid->integral;
	}
	// D
	if(pid->kd) {
		if(pid->compute_derivative) {
			PID_differentiate(pid, deltat);
		}
		temp_output += pid->kd * pid->derivative;
	}

	pid->output = temp_output;

	if(pid->constrain_output) {
		pid->output = PID_clamp(pid->output, pid->output_min, pid->output_max);
	}
	return pid->output;
}

void PID_enableComputeIntegral(PID*);
void PID_disableComputeIntegral(PID*);
void PID_enableComputeDerivative(PID*);
void PID_disableComputeDerivative(PID*);

void PID_enableErrorConstrain(PID*);
void PID_disableErrorConstrain(PID*);
void PID_setErrorLimits(PID*, float, float);

void PID_enableIntegralConstrain(PID*);
void PID_disableIntegralConstrain(PID*);
void PID_setIntegralLimits(PID*, float, float);

void PID_enableOutputConstrain(PID*);
void PID_disableOutputConstrain(PID*);
void PID_setOutputLimits(PID*, float, float);
void PID_setDerivativeLPF(PID*, float);

#endif
