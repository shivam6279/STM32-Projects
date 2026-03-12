#ifndef _AHRS_H_
#define _AHRS_H_


#define FUSION_BETA	0.1f // 0.05f

#define ROLLOFFSET 0.0
#define PITCHOFFSET 0.0
#define HEADINGOFFSET 0.0

void QuaternionInit(float q[], float ax, float ay, float az, float yaw);
void QuaternionToEuler(float[], float*, float*, float*);

void MadgwickQuaternionUpdate(float*, float, float, float, float, float, float, float, float, float, float);
void MadgwickQuaternionUpdateGyro(float*, float, float, float, float);
void MadgwickQuaternionUpdateAcc(float*, float, float, float, float);
float invSqrt(float);

extern float roll_offset, pitch_offset, heading_offset;

#endif
