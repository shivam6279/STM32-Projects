/**
 * @file    ahrs.c
 * @brief   Mahony and Madgwick AHRS filter implementations
 *
 * References
 * ──────────
 *  Mahony:   R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary
 *            Filters on the Special Orthogonal Group", IEEE TAC 2008.
 *  Madgwick: S. O. H. Madgwick, "An efficient orientation filter for inertial
 *            and inertial/magnetic sensor arrays", 2010.
 *
 * Implementation notes
 * ────────────────────
 *  • Uses ARM CMSIS arm_sqrt_f32() when __ARM_FEATURE_FPU is defined,
 *    otherwise falls back to standard sqrtf().  The STM32H5 Cortex-M33
 *    has a single-precision FPU so the intrinsic path is taken automatically.
 *  • No dynamic allocation, no static state — all state is in caller structs.
 *  • All input vectors are normalised internally; zero-magnitude vectors are
 *    silently ignored (filter runs on remaining sensors).
 */

#include "ahrs.h"
#include <math.h>
#include <string.h>

/* ── Fast inverse square root ─────────────────────────────────────────────── *
 * On Cortex-M33 with FPU the compiler will use VSQRT + VDIV which is fast
 * enough. The Quake rsqrt trick is kept as a fallback comment only.
 */
static inline float prv_inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

/* ── Normalise a 3-vector; returns false if magnitude is ~zero ───────────── */
static int prv_norm3(float *x, float *y, float *z)
{
    float n = (*x) * (*x) + (*y) * (*y) + (*z) * (*z);
    if (n < 1e-10f) return 0;
    float inv = prv_inv_sqrt(n);
    *x *= inv;  *y *= inv;  *z *= inv;
    return 1;
}

/* ── Normalise a quaternion ──────────────────────────────────────────────── */
void AHRS_QuatNorm(float q[4])
{
    float inv = prv_inv_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= inv;  q[1] *= inv;  q[2] *= inv;  q[3] *= inv;
}

/* ── Quaternion to Euler (ZYX) ───────────────────────────────────────────── */
void AHRS_QuatToEuler(const float q[4],
                       float *roll_rad,
                       float *pitch_rad,
                       float *yaw_rad)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    /* Roll  (phi) — rotation about X */
    *roll_rad  = atan2f(2.0f*(w*x + y*z),
                        1.0f - 2.0f*(x*x + y*y));

    /* Pitch (theta) — rotation about Y; clamped for asinf domain */
    float sinp = 2.0f * (w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch_rad = asinf(sinp);

    /* Yaw   (psi) — rotation about Z */
    *yaw_rad   = atan2f(2.0f*(w*z + x*y),
                        1.0f - 2.0f*(y*y + z*z));
}

/* ── Default gains ───────────────────────────────────────────────────────── */
Mahony_Gains_t Mahony_DefaultGains(void)
{
    return (Mahony_Gains_t){ .Kp = 0.5f, .Ki = 0.01f };
}

Madgwick_Gains_t Madgwick_DefaultGains(void)
{
    return (Madgwick_Gains_t){ .beta = 0.033f };
}


/* ═══════════════════════════════════════════════════════════════════════════
 *  MAHONY
 * ═══════════════════════════════════════════════════════════════════════════ */

void Mahony_UpdateMARG(float q[4],
                        Mahony_State_t        *state,
                        const Mahony_Gains_t  *gains,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float dt)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    /* ── Accel correction ── */
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    if (prv_norm3(&ax, &ay, &az)) {
        /* Estimated gravity direction in body frame from current quaternion */
        float vx = 2.0f*(x*z - w*y);
        float vy = 2.0f*(w*x + y*z);
        float vz = w*w - x*x - y*y + z*z;

        /* Cross product: measured × estimated = error */
        ex += ay*vz - az*vy;
        ey += az*vx - ax*vz;
        ez += ax*vy - ay*vx;
    }

    /* ── Mag correction ── */
    if (prv_norm3(&mx, &my, &mz)) {
        /* Magnetic field reference in Earth frame (from current quaternion) */
        float hx = 2.0f*(mx*(0.5f - y*y - z*z) + my*(x*y - w*z) + mz*(x*z + w*y));
        float hy = 2.0f*(mx*(x*y + w*z) + my*(0.5f - x*x - z*z) + mz*(y*z - w*x));
        float hz = 2.0f*(mx*(x*z - w*y) + my*(y*z + w*x) + mz*(0.5f - x*x - y*y));

        /* Flatten to horizontal plane; only use x and z components */
        float bx = sqrtf(hx*hx + hy*hy);
        float bz = hz;

        /* Estimated magnetic field direction in body frame */
        float wx2 = 2.0f*(bx*(0.5f - y*y - z*z) + bz*(x*z - w*y));
        float wy2 = 2.0f*(bx*(x*y - w*z)         + bz*(w*x + y*z));
        float wz2 = 2.0f*(bx*(w*y + x*z)          + bz*(0.5f - x*x - y*y));

        ex += my*wz2 - mz*wy2;
        ey += mz*wx2 - mx*wz2;
        ez += mx*wy2 - my*wx2;
    }

    /* ── PI controller on error ── */
    if (gains->Ki > 0.0f) {
        state->ix += gains->Ki * ex * dt;
        state->iy += gains->Ki * ey * dt;
        state->iz += gains->Ki * ez * dt;
    }

    gx += gains->Kp * ex + state->ix;
    gy += gains->Kp * ey + state->iy;
    gz += gains->Kp * ez + state->iz;

    /* ── Quaternion integration (1st-order Runge-Kutta) ── */
    float half_dt = 0.5f * dt;
    q[0] = w + half_dt*(-x*gx - y*gy - z*gz);
    q[1] = x + half_dt*( w*gx + y*gz - z*gy);
    q[2] = y + half_dt*( w*gy - x*gz + z*gx);
    q[3] = z + half_dt*( w*gz + x*gy - y*gx);

    AHRS_QuatNorm(q);
}

/* ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── */

void Mahony_UpdateIMU(float q[4],
                       Mahony_State_t        *state,
                       const Mahony_Gains_t  *gains,
                       float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    if (prv_norm3(&ax, &ay, &az)) {
        float vx = 2.0f*(x*z - w*y);
        float vy = 2.0f*(w*x + y*z);
        float vz = w*w - x*x - y*y + z*z;

        ex = ay*vz - az*vy;
        ey = az*vx - ax*vz;
        ez = ax*vy - ay*vx;
    }

    if (gains->Ki > 0.0f) {
        state->ix += gains->Ki * ex * dt;
        state->iy += gains->Ki * ey * dt;
        state->iz += gains->Ki * ez * dt;
    }

    gx += gains->Kp * ex + state->ix;
    gy += gains->Kp * ey + state->iy;
    gz += gains->Kp * ez + state->iz;

    float half_dt = 0.5f * dt;
    q[0] = w + half_dt*(-x*gx - y*gy - z*gz);
    q[1] = x + half_dt*( w*gx + y*gz - z*gy);
    q[2] = y + half_dt*( w*gy - x*gz + z*gx);
    q[3] = z + half_dt*( w*gz + x*gy - y*gx);

    AHRS_QuatNorm(q);
}

/* ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── */

void Mahony_UpdateGyro(float q[4],
                        float gx, float gy, float gz,
                        float dt)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float half_dt = 0.5f * dt;

    q[0] = w + half_dt*(-x*gx - y*gy - z*gz);
    q[1] = x + half_dt*( w*gx + y*gz - z*gy);
    q[2] = y + half_dt*( w*gy - x*gz + z*gx);
    q[3] = z + half_dt*( w*gz + x*gy - y*gx);

    AHRS_QuatNorm(q);
}


/* ═══════════════════════════════════════════════════════════════════════════
 *  MADGWICK
 * ═══════════════════════════════════════════════════════════════════════════ */

void Madgwick_UpdateMARG(float q[4],
                          const Madgwick_Gains_t *gains,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    /* Gradient step terms */
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;

    /* ── Accel gradient ── */
    if (prv_norm3(&ax, &ay, &az)) {
        /* Objective function f_g and its Jacobian */
        float f1 = 2.0f*(x*z - w*y) - ax;
        float f2 = 2.0f*(w*x + y*z) - ay;
        float f3 = 1.0f - 2.0f*(x*x + y*y) - az;

        /* J_g^T * f_g */
        s0 += -2.0f*y*f1 + 2.0f*x*f2;
        s1 +=  2.0f*z*f1 + 2.0f*w*f2 - 4.0f*x*f3;
        s2 += -2.0f*w*f1 + 2.0f*z*f2 - 4.0f*y*f3;
        s3 +=  2.0f*x*f1 + 2.0f*y*f2;
    }

    /* ── Mag gradient ── */
    if (prv_norm3(&mx, &my, &mz)) {
        /* Magnetic reference in Earth frame (flattened to bx, bz) */
        float hx = 2.0f*(mx*(0.5f - y*y - z*z) + my*(x*y - w*z) + mz*(x*z + w*y));
        float hy = 2.0f*(mx*(x*y + w*z) + my*(0.5f - x*x - z*z) + mz*(y*z - w*x));
        float hz = 2.0f*(mx*(x*z - w*y) + my*(y*z + w*x) + mz*(0.5f - x*x - y*y));

        float bx = sqrtf(hx*hx + hy*hy);
        float bz = hz;

        /* Objective function f_b */
        float f4 = 2.0f*bx*(0.5f - y*y - z*z) + 2.0f*bz*(x*z - w*y) - mx;
        float f5 = 2.0f*bx*(x*y - w*z)          + 2.0f*bz*(w*x + y*z) - my;
        float f6 = 2.0f*bx*(w*y + x*z)          + 2.0f*bz*(0.5f - x*x - y*y) - mz;

        /* J_b^T * f_b */
        s0 += -2.0f*bz*y*f4 + (-2.0f*bx*z + 2.0f*bz*x)*f5 + 2.0f*bx*y*f6;
        s1 +=  2.0f*bz*z*f4 + ( 2.0f*bx*y + 2.0f*bz*w)*f5 + (2.0f*bx*z - 4.0f*bz*x)*f6;
        s2 += (-4.0f*bx*y - 2.0f*bz*w)*f4 + (2.0f*bx*x + 2.0f*bz*z)*f5 + (2.0f*bx*w - 4.0f*bz*y)*f6;
        s3 += (-4.0f*bx*z + 2.0f*bz*x)*f4 + (-2.0f*bx*w + 2.0f*bz*y)*f5 + 2.0f*bx*x*f6;
    }

    /* Normalise gradient step */
    float inv_s = prv_inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= inv_s;  s1 *= inv_s;  s2 *= inv_s;  s3 *= inv_s;

    /* ── Rate of change from gyro ── */
    float qdot0 = 0.5f*(-x*gx - y*gy - z*gz) - gains->beta*s0;
    float qdot1 = 0.5f*( w*gx + y*gz - z*gy) - gains->beta*s1;
    float qdot2 = 0.5f*( w*gy - x*gz + z*gx) - gains->beta*s2;
    float qdot3 = 0.5f*( w*gz + x*gy - y*gx) - gains->beta*s3;

    q[0] = w + qdot0 * dt;
    q[1] = x + qdot1 * dt;
    q[2] = y + qdot2 * dt;
    q[3] = z + qdot3 * dt;

    AHRS_QuatNorm(q);
}

/* ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── */

void Madgwick_UpdateIMU(float q[4],
                         const Madgwick_Gains_t *gains,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;

    if (prv_norm3(&ax, &ay, &az)) {
        float f1 = 2.0f*(x*z - w*y) - ax;
        float f2 = 2.0f*(w*x + y*z) - ay;
        float f3 = 1.0f - 2.0f*(x*x + y*y) - az;

        s0 = -2.0f*y*f1 + 2.0f*x*f2;
        s1 =  2.0f*z*f1 + 2.0f*w*f2 - 4.0f*x*f3;
        s2 = -2.0f*w*f1 + 2.0f*z*f2 - 4.0f*y*f3;
        s3 =  2.0f*x*f1 + 2.0f*y*f2;

        float inv_s = prv_inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= inv_s;  s1 *= inv_s;  s2 *= inv_s;  s3 *= inv_s;
    }

    float qdot0 = 0.5f*(-x*gx - y*gy - z*gz) - gains->beta*s0;
    float qdot1 = 0.5f*( w*gx + y*gz - z*gy) - gains->beta*s1;
    float qdot2 = 0.5f*( w*gy - x*gz + z*gx) - gains->beta*s2;
    float qdot3 = 0.5f*( w*gz + x*gy - y*gx) - gains->beta*s3;

    q[0] = w + qdot0 * dt;
    q[1] = x + qdot1 * dt;
    q[2] = y + qdot2 * dt;
    q[3] = z + qdot3 * dt;

    AHRS_QuatNorm(q);
}

/* ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── */

void Madgwick_UpdateGyro(float q[4],
                          float gx, float gy, float gz,
                          float dt)
{
    /* Identical to Mahony_UpdateGyro — pure integration, no correction */
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float half_dt = 0.5f * dt;

    q[0] = w + half_dt*(-x*gx - y*gy - z*gz);
    q[1] = x + half_dt*( w*gx + y*gz - z*gy);
    q[2] = y + half_dt*( w*gy - x*gz + z*gx);
    q[3] = z + half_dt*( w*gz + x*gy - y*gx);

    AHRS_QuatNorm(q);
}
