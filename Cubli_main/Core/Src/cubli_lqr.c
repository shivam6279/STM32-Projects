#include "cubli_lqr.h"
#include <math.h>
#include <string.h>

// ============================================================
//  INTERNAL CONSTANTS
// ============================================================

#define STATE_DIM       9
#define INPUT_DIM       3
#define RICCATI_ITER    500
#define RICCATI_TOL     1e-8f

// Cube geometry: reaction wheels mount along face normals.
// When balancing on corner, transform from body frame to
// wheel frame requires rotation by -35.264 deg about Y then
// 45 deg about X. These are exact for a perfect cube.
#define CORNER_ANGLE_Y  (-0.61548f)   // -35.264 deg in radians
#define CORNER_ANGLE_X  ( 0.78540f)   //  45.000 deg in radians


// ============================================================
//  SMALL MATRIX UTILITIES (fixed 9x9, 3x9, 3x3)
// ============================================================

static void mat_zero(float *A, int rows, int cols) {
    memset(A, 0, sizeof(float) * rows * cols);
}

static void mat_copy(const float *src, float *dst, int rows, int cols) {
    memcpy(dst, src, sizeof(float) * rows * cols);
}

// C = A * B,  A is (r x m), B is (m x c)
static void mat_mul(const float *A, const float *B, float *C,
                    int r, int m, int c) {
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++) {
            float s = 0.0f;
            for (int k = 0; k < m; k++)
                s += A[i*m + k] * B[k*c + j];
            C[i*c + j] = s;
        }
}

// C = A + B
static void mat_add(const float *A, const float *B, float *C, int r, int c) {
    for (int i = 0; i < r*c; i++) C[i] = A[i] + B[i];
}

// C = A - B
static void mat_sub(const float *A, const float *B, float *C, int r, int c) {
    for (int i = 0; i < r*c; i++) C[i] = A[i] - B[i];
}

// Transpose: B = A^T, A is (r x c)
static void mat_transpose(const float *A, float *B, int r, int c) {
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            B[j*r + i] = A[i*c + j];
}

// Scale: B = s * A
static void mat_scale(const float *A, float s, float *B, int r, int c) {
    for (int i = 0; i < r*c; i++) B[i] = s * A[i];
}

// Frobenius norm of difference (convergence check)
static float mat_diff_norm(const float *A, const float *B, int r, int c) {
    float s = 0.0f;
    for (int i = 0; i < r*c; i++) {
        float d = A[i] - B[i];
        s += d * d;
    }
    return sqrtf(s);
}

// 3x3 inverse via cofactor (exact, no pivoting needed for 3x3)
static int mat_inv3(const float A[9], float B[9]) {
    float det = A[0]*(A[4]*A[8] - A[5]*A[7])
              - A[1]*(A[3]*A[8] - A[5]*A[6])
              + A[2]*(A[3]*A[7] - A[4]*A[6]);
    if (fabsf(det) < 1e-10f) return -1;
    float inv = 1.0f / det;
    B[0] =  (A[4]*A[8] - A[5]*A[7]) * inv;
    B[1] = -(A[1]*A[8] - A[2]*A[7]) * inv;
    B[2] =  (A[1]*A[5] - A[2]*A[4]) * inv;
    B[3] = -(A[3]*A[8] - A[5]*A[6]) * inv;
    B[4] =  (A[0]*A[8] - A[2]*A[6]) * inv;
    B[5] = -(A[0]*A[5] - A[2]*A[3]) * inv;
    B[6] =  (A[3]*A[7] - A[4]*A[6]) * inv;
    B[7] = -(A[0]*A[7] - A[1]*A[6]) * inv;
    B[8] =  (A[0]*A[4] - A[1]*A[3]) * inv;
    return 0;
}

// NxN inverse via Gauss-Jordan (for 9x9 P matrix inversion)
static int mat_inv_n(const float *A, float *B, int n) {
    float M[9*9*2];  // augmented [A | I]
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            M[i*(2*n) + j]   = A[i*n + j];
            M[i*(2*n) + n+j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    for (int col = 0; col < n; col++) {
        // Find pivot
        int pivot = -1;
        float best = 0.0f;
        for (int row = col; row < n; row++) {
            float v = fabsf(M[row*(2*n) + col]);
            if (v > best) { best = v; pivot = row; }
        }
        if (best < 1e-10f) return -1;
        // Swap rows
        if (pivot != col) {
            for (int j = 0; j < 2*n; j++) {
                float tmp = M[col*(2*n)+j];
                M[col*(2*n)+j] = M[pivot*(2*n)+j];
                M[pivot*(2*n)+j] = tmp;
            }
        }
        float scale = 1.0f / M[col*(2*n)+col];
        for (int j = 0; j < 2*n; j++) M[col*(2*n)+j] *= scale;
        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            float f = M[row*(2*n)+col];
            for (int j = 0; j < 2*n; j++)
                M[row*(2*n)+j] -= f * M[col*(2*n)+j];
        }
    }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            B[i*n+j] = M[i*(2*n)+n+j];
    return 0;
}


// ============================================================
//  BUILD GEOMETRY MATRIX Tbf (3x3)
//  Maps wheel frame torques to body frame.
//  Fixed by cube geometry — no measurement needed.
// ============================================================
static void build_Tbf(float Tbf[9]) {
    float cy = cosf(CORNER_ANGLE_Y), sy = sinf(CORNER_ANGLE_Y);
    float cx = cosf(CORNER_ANGLE_X), sx = cosf(CORNER_ANGLE_X);

    // Ry(-35.264) * Rx(45)
    float Ry[9] = { cy,  0, sy,
                     0,  1,  0,
                   -sy,  0, cy };

    float Rx[9] = { 1,  0,   0,
                    0, cx, -sx,
                    0, sx,  cx };

    mat_mul(Ry, Rx, Tbf, 3, 3, 3);
}


// ============================================================
//  BUILD LINEARIZED STATE SPACE MATRICES A (9x9), B (9x3)
//
//  State: x = [alpha, beta, gamma, alpha_dot, beta_dot,
//              gamma_dot, wA, wB, wC]
//
//  Linearized around alpha=beta=gamma=0 (corner equilibrium)
// ============================================================
static void build_AB(const CubliParams *p, const float Tbf[9],
                     float A[STATE_DIM*STATE_DIM],
                     float B[STATE_DIM*INPUT_DIM]) {
    mat_zero(A, STATE_DIM, STATE_DIM);
    mat_zero(B, STATE_DIM, INPUT_DIM);

    float Ia = p->Iy - p->Iw;  // effective inertia about alpha axis
    float Ib = p->Ix - p->Iw;  // effective inertia about beta axis
    float Ic = p->Iz;           // yaw inertia

    float mgl = p->m * p->g * p->l;

    // -- A matrix --
    // alpha_dot  = alpha_dot
    A[0*STATE_DIM + 3] = 1.0f;
    // beta_dot   = beta_dot
    A[1*STATE_DIM + 4] = 1.0f;
    // gamma_dot  = gamma_dot
    A[2*STATE_DIM + 5] = 1.0f;

    // alpha_ddot = (mgl/Ia) * alpha
    A[3*STATE_DIM + 0] = mgl / Ia;
    // beta_ddot  = (mgl/Ib) * beta
    A[4*STATE_DIM + 1] = mgl / Ib;
    // gamma_ddot = 0 (no gravitational restoring torque about yaw)

    // Wheel speed states have no autonomous dynamics (A rows 6,7,8 = 0)

    // -- B matrix --
    // Torque coupling through Tbf:
    // [alpha_ddot, beta_ddot, gamma_ddot] += -(1/I_axis) * Tbf * [tauA, tauB, tauC]
    // [wA_dot, wB_dot, wC_dot]            +=  (1/Iw)    * I  * [tauA, tauB, tauC]

    // Body frame rows (3,4,5) — tilt and yaw acceleration
    for (int axis = 0; axis < 3; axis++) {
        float I_eff;
        if (axis == 0) I_eff = Ia;
        else if (axis == 1) I_eff = Ib;
        else I_eff = Ic;

        for (int wheel = 0; wheel < 3; wheel++) {
            B[(3+axis)*INPUT_DIM + wheel] = -Tbf[axis*3 + wheel] / I_eff;
        }
    }

    // Wheel speed rows (6,7,8)
    for (int wheel = 0; wheel < 3; wheel++) {
        B[(6+wheel)*INPUT_DIM + wheel] = 1.0f / p->Iw;
    }
}


// ============================================================
//  DISCRETE-TIME ALGEBRAIC RICCATI EQUATION SOLVER
//  Uses value iteration (backward recursion) on the DARE.
//  Converges for stabilizable systems — Cubli qualifies.
//
//  Solves: P = Q + A^T P A - A^T P B (R + B^T P B)^-1 B^T P A
//
//  Inputs:
//    Ad, Bd : discretized A, B (use lqr_init discretization)
//    Q (9x9), R (3x3) : cost matrices
//  Output:
//    P (9x9) : solution
//    K (3x9) : optimal gain
// ============================================================
static int solve_dare(const float Ad[STATE_DIM*STATE_DIM],
                      const float Bd[STATE_DIM*INPUT_DIM],
                      const float Q[STATE_DIM*STATE_DIM],
                      const float R[INPUT_DIM*INPUT_DIM],
                      float P[STATE_DIM*STATE_DIM],
                      float K[INPUT_DIM*STATE_DIM]) {
    // Initialize P = Q
    mat_copy(Q, P, STATE_DIM, STATE_DIM);

    float AdT[STATE_DIM*STATE_DIM];
    float BdT[INPUT_DIM*STATE_DIM];
    mat_transpose(Ad, AdT, STATE_DIM, STATE_DIM);
    mat_transpose(Bd, BdT, INPUT_DIM, STATE_DIM);

    float P_new[STATE_DIM*STATE_DIM];
    float tmp1[STATE_DIM*STATE_DIM];
    float tmp2[STATE_DIM*INPUT_DIM];
    float tmp3[INPUT_DIM*INPUT_DIM];
    float tmp3_inv[INPUT_DIM*INPUT_DIM];
    float tmp4[INPUT_DIM*STATE_DIM];
    float tmp5[STATE_DIM*STATE_DIM];

    for (int iter = 0; iter < RICCATI_ITER; iter++) {
        // S = R + Bd^T * P * Bd  (3x3)
        mat_mul(P, Bd, tmp2, STATE_DIM, STATE_DIM, INPUT_DIM);     // P*Bd
        mat_mul(BdT, tmp2, tmp3, INPUT_DIM, STATE_DIM, INPUT_DIM); // Bd^T*P*Bd
        mat_add(R, tmp3, tmp3, INPUT_DIM, INPUT_DIM);               // S = R + ...

        if (mat_inv3(tmp3, tmp3_inv) != 0) return -1;

        // K_iter = S^-1 * Bd^T * P * Ad
        mat_mul(BdT, P, tmp4, INPUT_DIM, STATE_DIM, STATE_DIM);    // Bd^T * P
        mat_mul(tmp4, Ad, tmp4, INPUT_DIM, STATE_DIM, STATE_DIM);  // Bd^T * P * Ad
        mat_mul(tmp3_inv, tmp4, K, INPUT_DIM, INPUT_DIM, STATE_DIM); // K_iter

        // P_new = Q + Ad^T * P * Ad - Ad^T * P * Bd * K_iter
        mat_mul(P, Ad, tmp1, STATE_DIM, STATE_DIM, STATE_DIM);      // P * Ad
        mat_mul(AdT, tmp1, P_new, STATE_DIM, STATE_DIM, STATE_DIM); // Ad^T*P*Ad
        mat_add(Q, P_new, P_new, STATE_DIM, STATE_DIM);             // Q + ...

        mat_mul(P, Bd, tmp2, STATE_DIM, STATE_DIM, INPUT_DIM);      // P * Bd
        mat_mul(AdT, tmp2, tmp5, STATE_DIM, STATE_DIM, INPUT_DIM);  // Ad^T*P*Bd  (9x3)
        mat_mul(tmp5, K, tmp1, STATE_DIM, INPUT_DIM, STATE_DIM);    // (9x3)*(3x9)
        mat_sub(P_new, tmp1, P_new, STATE_DIM, STATE_DIM);          // subtract

        if (mat_diff_norm(P_new, P, STATE_DIM, STATE_DIM) < RICCATI_TOL) {
            mat_copy(P_new, P, STATE_DIM, STATE_DIM);
            return 0;
        }
        mat_copy(P_new, P, STATE_DIM, STATE_DIM);
    }
    // Did not converge — increase RICCATI_ITER or check parameters
    return -1;
}


// ============================================================
//  DISCRETIZE A, B using zero-order hold (Euler approximation)
//  Sufficient for dt << system time constants.
//  At 1kHz (dt=0.001s) this is accurate for Cubli dynamics.
//
//  Ad = I + A*dt
//  Bd = B*dt
// ============================================================
static void discretize(const float A[STATE_DIM*STATE_DIM],
                       const float B[STATE_DIM*INPUT_DIM],
                       float dt,
                       float Ad[STATE_DIM*STATE_DIM],
                       float Bd[STATE_DIM*INPUT_DIM]) {
    for (int i = 0; i < STATE_DIM; i++)
        for (int j = 0; j < STATE_DIM; j++)
            Ad[i*STATE_DIM+j] = A[i*STATE_DIM+j] * dt + (i==j ? 1.0f : 0.0f);

    mat_scale(B, dt, Bd, STATE_DIM, INPUT_DIM);
}


// ============================================================
//  PUBLIC API
// ============================================================

int lqr_init(CubliLQR *lqr, const CubliParams *params) {
    if (!lqr || !params) return -1;

    // Basic sanity checks
    if (params->m <= 0 || params->Iw <= 0 || params->Kt <= 0) return -1;
    if (params->Ix <= 0 || params->Iy <= 0 || params->Iz <= 0) return -1;

    memcpy(&lqr->p, params, sizeof(CubliParams));
    lqr->initialized = 0;

    // Build geometry matrix
    build_Tbf((float*)lqr->Tbf);

    // Build continuous A, B
    float A[STATE_DIM*STATE_DIM];
    float B[STATE_DIM*INPUT_DIM];
    build_AB(params, (float*)lqr->Tbf, A, B);

    // Discretize at 1kHz
    float Ad[STATE_DIM*STATE_DIM];
    float Bd[STATE_DIM*INPUT_DIM];
    discretize(A, B, 0.001f, Ad, Bd);

    // Build Q (9x9 diagonal)
    float Q[STATE_DIM*STATE_DIM];
    mat_zero(Q, STATE_DIM, STATE_DIM);
    Q[0*STATE_DIM+0] = params->q_tilt;      // alpha
    Q[1*STATE_DIM+1] = params->q_tilt;      // beta
    Q[2*STATE_DIM+2] = params->q_yaw;       // gamma
    Q[3*STATE_DIM+3] = params->q_tilt_dot;  // alpha_dot
    Q[4*STATE_DIM+4] = params->q_tilt_dot;  // beta_dot
    Q[5*STATE_DIM+5] = params->q_yaw_dot;   // gamma_dot
    Q[6*STATE_DIM+6] = params->q_wheel;     // wA
    Q[7*STATE_DIM+7] = params->q_wheel;     // wB
    Q[8*STATE_DIM+8] = params->q_wheel;     // wC

    // Build R (3x3 diagonal)
    float R[INPUT_DIM*INPUT_DIM];
    mat_zero(R, INPUT_DIM, INPUT_DIM);
    R[0*INPUT_DIM+0] = params->r_torque;
    R[1*INPUT_DIM+1] = params->r_torque;
    R[2*INPUT_DIM+2] = params->r_torque;

    // Solve DARE
    float P[STATE_DIM*STATE_DIM];
    float K[INPUT_DIM*STATE_DIM];
    if (solve_dare(Ad, Bd, Q, R, P, K) != 0) return -1;

    memcpy(lqr->K, K, sizeof(K));
    lqr->initialized = 1;
    return 0;
}


void lqr_update(CubliLQR *lqr,
                const float state[9],
                float ref_gamma,
                float torque_out[3]) {
    if (!lqr->initialized) {
        torque_out[0] = torque_out[1] = torque_out[2] = 0.0f;
        return;
    }

    // Error state: subtract reference on yaw only
    float err[STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) err[i] = state[i];
    err[2] = state[2] - ref_gamma;  // yaw error

    // u = -K * err
    for (int i = 0; i < INPUT_DIM; i++) {
        float s = 0.0f;
        for (int j = 0; j < STATE_DIM; j++)
            s += lqr->K[i][j] * err[j];
        torque_out[i] = -s;
    }
}


void lqr_friction_compensate(CubliLQR *lqr,
                              float torque_out[3],
                              const float wheel_vel[3]) {
    for (int i = 0; i < 3; i++) {
        // Viscous friction: proportional to wheel speed
        torque_out[i] += lqr->p.Cw * wheel_vel[i];

        // Static friction: feedforward in direction of commanded torque
        if (fabsf(torque_out[i]) > 1e-6f) {
            torque_out[i] += (torque_out[i] > 0 ? 1.0f : -1.0f)
                             * lqr->p.tau_static;
        }
    }
}
