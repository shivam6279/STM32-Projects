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
	float lpf;

	float error_min, error_max;
	float integral_max, integral_min;
	float output_max, output_min;
	
	bool constrain_integral;
	bool constrain_error;
	bool constrain_output;
	
	bool compute_integral;
	bool compute_derivative;
} PID;

extern void PID_init(PID*);
extern void PID_reset(PID*);
extern void PID_setGain(PID*, float, float, float);
extern void PID_integrate(PID *pid, float);
extern void PID_differentiate(PID*, float);
extern float PID_compute(PID*, float, float);

extern void PID_enableComputeIntegral(PID*);
extern void PID_disableComputeIntegral(PID*);
extern void PID_enableComputeDerivative(PID*);
extern void PID_disableComputeDerivative(PID*);

extern void PID_enableErrorConstrain(PID*);
extern void PID_disableErrorConstrain(PID*);
extern void PID_setErrorLimits(PID*, float, float);

extern void PID_enableIntegralConstrain(PID*);
extern void PID_disableIntegralConstrain(PID*);
extern void PID_setIntegralLimits(PID*, float, float);

extern void PID_enableOutputConstrain(PID*);
extern void PID_disableOutputConstrain(PID*);
extern void PID_setOutputLimits(PID*, float, float);
extern void PID_setLPF(PID*, float);

#endif
