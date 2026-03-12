#include "AHRS.h"
#include <math.h>

#define PI 3.1415926535897931f
#define TO_DEG(x) (x * 180.0f / PI)
#define TO_RAD(x) (x * PI / 180.0f)

float roll_offset = ROLLOFFSET;
float pitch_offset = PITCHOFFSET;
float heading_offset = HEADINGOFFSET;

void QuaternionInit(float q[4], float ax, float ay, float az, float yaw) {
	float n = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
	ax *= n; ay *= n; az *= n;

	yaw = TO_RAD(yaw);

	float roll  = atan2f(ay, az);
	float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

	float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
	float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
	float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);

	// ZYX Euler → quaternion
	q[0] = cr*cp*cy + sr*sp*sy;
	q[1] = sr*cp*cy - cr*sp*sy;
	q[2] = cr*sp*cy + sr*cp*sy;
	q[3] = cr*cp*sy - sr*sp*cy;
}

void QuaternionToEuler(float q[], float *roll, float *pitch, float *yaw) {
	float q2_2 = q[2] * q[2];

	//Converting quaternion to Euler angles
	// *roll = TO_DEG(atan2f(q[2]*q[3] + q[0]*q[1], 0.5f - (q[1]*q[1] + q2_2)));
	// *pitch = TO_DEG(asinf(2.0f * (q[1]*q[3] - q[0]*q[2])));
	// *yaw = TO_DEG(atan2f((q[1]*q[2] + q[0]*q[3]), 0.5f - (q2_2 + q[3]*q[3])));

	float w = q[0], x = q[1], y = q[2], z = q[3];

	/* Roll  (phi) — rotation about X */
	*roll  = TO_DEG(atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)));

	/* Pitch (theta) — rotation about Y; clamped for asinf domain */
	float sinp = 2.0f * (w*y - z*x);
	if (sinp >  1.0f) sinp =  1.0f;
	if (sinp < -1.0f) sinp = -1.0f;
	*pitch = TO_DEG(asinf(sinp));

	/* Yaw   (psi) — rotation about Z */
	*yaw = TO_DEG(atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z)));
}

void MultiplyQuaternion(float q1[], float q2[], float qr[]) {
	qr[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	qr[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
	qr[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3];
	qr[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1]; 
}

void MadgwickQuaternionUpdate(float q[], float a_x, float a_y, float a_z, float g_x, float g_y, float g_z, float m_x, float m_y, float m_z, float deltat) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q[1] * g_x - q[2] * g_y - q[3] * g_z);
	qDot2 = 0.5f * (q[0] * g_x + q[2] * g_z - q[3] * g_y);
	qDot3 = 0.5f * (q[0] * g_y - q[1] * g_z + q[3] * g_x);
	qDot4 = 0.5f * (q[0] * g_z + q[1] * g_y - q[2] * g_x);

	// Normalise accelerometer measurement
	recipNorm = invSqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x *= recipNorm;
	a_y *= recipNorm;
	a_z *= recipNorm;

	// Normalise magnetometer measurement
	recipNorm = invSqrt(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x *= recipNorm;
	m_y *= recipNorm;
	m_z *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	_2q0mx = 2.0f * q[0] * m_x;
	_2q0my = 2.0f * q[0] * m_y;
	_2q0mz = 2.0f * q[0] * m_z;
	_2q1mx = 2.0f * q[1] * m_x;
	_2q0 = 2.0f * q[0];
	_2q1 = 2.0f * q[1];
	_2q2 = 2.0f * q[2];
	_2q3 = 2.0f * q[3];
	_2q0q2 = 2.0f * q[0] * q[2];
	_2q2q3 = 2.0f * q[2] * q[3];
	q0q0 = q[0] * q[0];
	q0q1 = q[0] * q[1];
	q0q2 = q[0] * q[2];
	q0q3 = q[0] * q[3];
	q1q1 = q[1] * q[1];
	q1q2 = q[1] * q[2];
	q1q3 = q[1] * q[3];
	q2q2 = q[2] * q[2];
	q2q3 = q[2] * q[3];
	q3q3 = q[3] * q[3];

	// Reference direction of Earth's magnetic field
	hx = m_x * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + m_x * q1q1 + _2q1 * m_y * q[2] + _2q1 * m_z * q[3] - m_x * q2q2 - m_x * q3q3;
	hy = _2q0mx * q[3] + m_y * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - m_y * q1q1 + m_y * q2q2 + _2q2 * m_z * q[3] - m_y * q3q3;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q0mx * q[2] + _2q0my * q[1] + m_z * q0q0 + _2q1mx * q[3] - m_z * q1q1 + _2q2 * m_y * q[3] - m_z * q2q2 + m_z * q3q3;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - a_x) + _2q1 * (2.0f * q0q1 + _2q2q3 - a_y) - _2bz * q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m_x) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m_y) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m_z);
	s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - a_x) + _2q0 * (2.0f * q0q1 + _2q2q3 - a_y) - 4.0f * q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a_z) + _2bz * q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m_x) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m_y) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m_z);
	s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - a_x) + _2q3 * (2.0f * q0q1 + _2q2q3 - a_y) - 4.0f * q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a_z) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m_x) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m_y) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m_z);
	s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - a_x) + _2q2 * (2.0f * q0q1 + _2q2q3 - a_y) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m_x) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m_y) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m_z);
	recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;

	// Apply feedback step
	qDot1 -= FUSION_BETA * s0;
	qDot2 -= FUSION_BETA * s1;
	qDot3 -= FUSION_BETA * s2;
	qDot4 -= FUSION_BETA * s3;

	// Integrate rate of change of quaternion to yield quaternion
	q[0] += qDot1 * deltat;
	q[1] += qDot2 * deltat;
	q[2] += qDot3 * deltat;
	q[3] += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

void MadgwickQuaternionUpdateGyro(float q[], float g_x, float g_y, float g_z, float deltat) {
	float recipNorm;
	float qDot1, qDot2, qDot3, qDot4;
	// Convert gyroscope degrees/sec to radians/sec
	g_x = TO_RAD(g_x);
	g_y = TO_RAD(g_y);
	g_z = TO_RAD(g_z);

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q[1] * g_x - q[2] * g_y - q[3] * g_z);
	qDot2 = 0.5f * (q[0] * g_x + q[2] * g_z - q[3] * g_y);
	qDot3 = 0.5f * (q[0] * g_y - q[1] * g_z + q[3] * g_x);
	qDot4 = 0.5f * (q[0] * g_z + q[1] * g_y - q[2] * g_x);

	// Integrate rate of change of quaternion to yield quaternion
	q[0] += qDot1 * deltat;
	q[1] += qDot2 * deltat;
	q[2] += qDot3 * deltat;
	q[3] += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

void MadgwickQuaternionUpdateAcc(float q[], float a_x, float a_y, float a_z, float deltat) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Normalise accelerometer measurement
	recipNorm = invSqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x *= recipNorm;
	a_y *= recipNorm;
	a_z *= recipNorm;
	
	_2q0 = 2.0f * q[0];
	_2q1 = 2.0f * q[1];
	_2q2 = 2.0f * q[2];
	_2q3 = 2.0f * q[3];
	_4q0 = 4.0f * q[0];
	_4q1 = 4.0f * q[1];
	_4q2 = 4.0f * q[2];
	_8q1 = 8.0f * q[1];
	_8q2 = 8.0f * q[2];
	q0q0 = q[0] * q[0];
	q1q1 = q[1] * q[1];
	q2q2 = q[2] * q[2];
	q3q3 = q[3] * q[3];

	// Gradient decent algorithm corrective step
	s0 = _4q0 * q2q2 + _2q2 * a_x + _4q0 * q1q1 - _2q1 * a_y;
	s1 = _4q1 * q3q3 - _2q3 * a_x + 4.0f * q0q0 * q[1] - _2q0 * a_y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a_z;
	s2 = 4.0f * q0q0 * q[2] + _2q0 * a_x + _4q2 * q3q3 - _2q3 * a_y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a_z;
	s3 = 4.0f * q1q1 * q[3] - _2q1 * a_x + 4.0f * q2q2 * q[3] - _2q2 * a_y;

	recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;

	// Apply feedback step
	qDot1 = -FUSION_BETA * s0;
	qDot2 = -FUSION_BETA * s1;
	qDot3 = -FUSION_BETA * s2;
	qDot4 = -FUSION_BETA * s3;

	// Integrate rate of change of quaternion to yield quaternion
	q[0] += qDot1 * deltat;
	q[1] += qDot2 * deltat;
	q[2] += qDot3 * deltat;
	q[3] += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}
