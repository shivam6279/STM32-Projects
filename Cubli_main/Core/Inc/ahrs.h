#ifndef AHRS_H
#define AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Mahony filter gains ─────────────────────────────────────────────────── *
 *
 * Kp  — proportional gain.  Higher = faster correction, more noise sensitive.
 * Ki  — integral gain.      Corrects gyro bias.  Set to 0 to disable.
 *
 * Typical starting values:
 *   Static / slow motion:  Kp = 0.5f,  Ki = 0.01f
 *   Dynamic motion:        Kp = 2.0f,  Ki = 0.005f
 */
typedef struct {
    float Kp;   /**< Proportional gain */
    float Ki;   /**< Integral gain     */
} Mahony_Gains_t;

/** Mahony integral error accumulator (one per filter instance) */
typedef struct {
    float ix;
    float iy;
    float iz;
} Mahony_State_t;

/* ── Madgwick filter gain ────────────────────────────────────────────────── *
 *
 * beta — gradient descent step size.
 *   Larger = faster convergence, more noise in the output.
 *   Smaller = smoother, slower to correct drift.
 *
 * Rule of thumb: beta ≈ sqrt(3/4) * gyro_noise_rad_s
 *   For MPU6050 at ±500 °/s:  beta ≈ 0.033f  (good starting point)
 *   Aggressive correction:    beta = 0.1f
 */
typedef struct {
    float beta;
} Madgwick_Gains_t;


void Mahony_UpdateMARG(float q[4],
                        Mahony_State_t        *state,
                        const Mahony_Gains_t  *gains,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float dt);

void Mahony_UpdateIMU(float q[4],
                       Mahony_State_t        *state,
                       const Mahony_Gains_t  *gains,
                       float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt);

void Mahony_UpdateGyro(float q[4],
                        float gx, float gy, float gz,
                        float dt);

void Madgwick_UpdateMARG(float q[4],
                          const Madgwick_Gains_t *gains,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt);

void Madgwick_UpdateIMU(float q[4],
                         const Madgwick_Gains_t *gains,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt);

void Madgwick_UpdateGyro(float q[4],
                          float gx, float gy, float gz,
                          float dt);

void AHRS_QuatNorm(float q[4]);
void AHRS_QuatToEuler(const float q[4],
                       float *roll_rad,
                       float *pitch_rad,
                       float *yaw_rad);

void AHRS_InitFromSensors(float q[4], float ax, float ay, float az, float mx, float my, float mz);
void AHRS_InitFromAccelYaw(float q[4], float ax, float ay, float az, float yaw_rad);

Mahony_Gains_t  Mahony_DefaultGains(void);
Madgwick_Gains_t Madgwick_DefaultGains(void);

#ifdef __cplusplus
}
#endif
#endif /* AHRS_H */
