#include "PID.h"
#include <inttypes.h>

static inline float clamp(float val, float min, float max) {
    return val > max ? max : val < min ? min : val;
}

void PID_init(PID *pid) {
	pid->kp = 0;
	pid->ki = 0;
	pid->kd = 0;
	pid->error = 0;
	pid->setpoint = 0;
	pid->p_input = 0;
	pid->integral = 0;
	pid->derivative = 0;
	pid->output = 0;
	pid->integral_max = 0;
	pid->integral_min = 0;
	pid->output_max = 0;
	pid->output_min = 0;
	
	pid->compute_integral = true;
	pid->compute_derivative = true;
	
	pid->constrain_error = false;
	pid->constrain_integral = false;
	pid->constrain_output = false;

	pid->lpf_der = 1.0f;
}

void PID_reset(PID *pid) {
	pid->error = 0;
	pid->setpoint = 0;
	pid->p_input = 0;
	pid->integral = 0;
	pid->derivative = 0;
	pid->output = 0;
}

void PID_setGain(PID *pid, float kp, float ki, float kd) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

void PID_integrate(PID *pid, float deltat) {
	float error = pid->error;
	if(pid->constrain_error) {
		error = clamp(pid->error, pid->error_min, pid->error_max);
	}
	pid->integral += pid->ki * error * deltat;
	if(pid->constrain_integral) {
		pid->integral = clamp(pid->integral, pid->integral_min, pid->integral_max);
	}
}

void PID_differentiate(PID *pid, float deltat) {
	pid->derivative += pid->lpf_der * ((pid->input - pid->p_input) / deltat - pid->derivative);
	pid->p_input = pid->input;
}

float PID_compute(PID *pid, float input, float deltat) {
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
		pid->output = clamp(pid->output, pid->output_min, pid->output_max);
	}
	return pid->output;
}

// Choose to or not to compute integral/derivative
void PID_enableComputeIntegral(PID *pid) {
	pid->compute_integral = true;
}

void PID_disableComputeIntegral(PID *pid) {
	pid->compute_integral = false;
}

void PID_enableComputeDerivative(PID *pid) {
	pid->compute_derivative = true;
}

void PID_disableComputeDerivative(PID *pid) {
	pid->compute_derivative = false;
}

// Max error to integrate constrain
void PID_enableErrorConstrain(PID *pid) {
	pid->constrain_error = true;
}

void PID_disableErrorConstrain(PID *pid) {
	pid->constrain_error = false;
}

void PID_setErrorLimits(PID *pid, float min, float max) {
	pid->error_min = min;
	pid->error_max = max;
}

// Integral constrain
void PID_enableIntegralConstrain(PID *pid) {
	pid->constrain_integral = true;
}

void PID_disableIntegralConstrain(PID *pid) {
	pid->constrain_integral = false;
}

void PID_setIntegralLimits(PID *pid, float min, float max) {
	pid->integral_min = min;
	pid->integral_max = max;
}

// Output constrain
void PID_enableOutputConstrain(PID *pid) {
	pid->constrain_output = true;
}

void PID_disableOutputConstrain(PID *pid) {
	pid->constrain_output = false;
}

void PID_setOutputLimits(PID *pid, float min, float max) {
	pid->output_min = min;
	pid->output_max = max;
}

void PID_setDerivativeLPF(PID *pid, float lpf) {
	pid->lpf_der = lpf;
}
