#include "cubli_lqr.h"
#include <math.h>
#include <string.h>

#define DARE_ITER  1000
#define DARE_TOL   1e-10
#define DEG2RAD    (3.14159265358979f / 180.0f)
#define DEG2RAD_D  (3.14159265358979  / 180.0)

// Corner geometry (exact for cube)
#define CORNER_ANGLE_Y  (-0.61548)   // -35.264 deg
#define CORNER_ANGLE_X  ( 0.78540)   //  45.000 deg

// ================================================================
//  DOUBLE MATRIX UTILITIES (init/DARE only)
// ================================================================

static void dm_zero(double *A, int r, int c) { memset(A, 0, sizeof(double)*r*c); }
static void dm_copy(const double *s, double *d, int r, int c) { memcpy(d, s, sizeof(double)*r*c); }

static void dm_mul(const double *A, const double *B, double *C, int n, int m, int p) {
    for (int i = 0; i < n; i++)
        for (int j = 0; j < p; j++) {
            double s = 0.0;
            for (int k = 0; k < m; k++) s += A[i*m+k] * B[k*p+j];
            C[i*p+j] = s;
        }
}

static void dm_add(const double *A, const double *B, double *C, int r, int c) {
    for (int i = 0; i < r*c; i++) C[i] = A[i] + B[i];
}

static void dm_sub(const double *A, const double *B, double *C, int r, int c) {
    for (int i = 0; i < r*c; i++) C[i] = A[i] - B[i];
}

static void dm_trans(const double *A, double *B, int r, int c) {
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            B[j*r+i] = A[i*c+j];
}

static double dm_diff_norm(const double *A, const double *B, int r, int c) {
    double s = 0.0;
    for (int i = 0; i < r*c; i++) { double d = A[i]-B[i]; s += d*d; }
    return sqrt(s);
}

static int dm_inv(const double *A, double *B, int n) {
    double M[10*20];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            M[i*(2*n)+j]   = A[i*n+j];
            M[i*(2*n)+n+j] = (i==j) ? 1.0 : 0.0;
        }
    }
    for (int col = 0; col < n; col++) {
        int pivot = -1; double best = 0.0;
        for (int row = col; row < n; row++) {
            double v = fabs(M[row*(2*n)+col]);
            if (v > best) { best = v; pivot = row; }
        }
        if (best < 1e-14) return -1;
        if (pivot != col) {
            for (int j = 0; j < 2*n; j++) {
                double tmp = M[col*(2*n)+j];
                M[col*(2*n)+j] = M[pivot*(2*n)+j];
                M[pivot*(2*n)+j] = tmp;
            }
        }
        double sc = 1.0 / M[col*(2*n)+col];
        for (int j = 0; j < 2*n; j++) M[col*(2*n)+j] *= sc;
        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            double f = M[row*(2*n)+col];
            for (int j = 0; j < 2*n; j++) M[row*(2*n)+j] -= f*M[col*(2*n)+j];
        }
    }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            B[i*n+j] = M[i*(2*n)+n+j];
    return 0;
}

// ================================================================
//  DARE SOLVER (double)
//  n = state dim, m = input dim
//  Ad(nxn), Bd(nxm), Q(nxn), R(mxm)
//  Output: K(mxn)
// ================================================================
static int dare_solve(const double *Ad, const double *Bd,
                      const double *Q, const double *R,
                      int n, int m, double *K) {
    double P[9*9], P_new[9*9];
    double AdT[9*9], BdT[3*9];
    double S[3*3], S_inv[3*3];
    double tmp1[9*9], tmp2[9*3], tmp3[3*9], tmp4[3*9];

    dm_copy(Q, P, n, n);
    dm_trans(Ad, AdT, n, n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            BdT[j*n+i] = Bd[i*m+j];

    for (int iter = 0; iter < DARE_ITER; iter++) {
        // S = R + Bd^T * P * Bd
        dm_mul(P,   Bd,  tmp2, n, n, m);
        dm_mul(BdT, tmp2, S,   m, n, m);
        dm_add(R, S, S, m, m);
        if (dm_inv(S, S_inv, m) != 0) return -1;

        // K = S^-1 * Bd^T * P * Ad
        dm_mul(BdT, P,  tmp3, m, n, n);
        dm_mul(tmp3, Ad, tmp4, m, n, n);
        dm_mul(S_inv, tmp4, K, m, m, n);

        // P_new = Q + Ad^T*P*Ad - Ad^T*P*Bd*K
        dm_mul(P,   Ad,  tmp1, n, n, n);
        dm_mul(AdT, tmp1, P_new, n, n, n);
        dm_add(Q, P_new, P_new, n, n);

        double tmp5[9*3], tmp6[9*9];
        dm_mul(P,   Bd,  tmp5, n, n, m);
        dm_mul(AdT, tmp5, tmp5, n, n, m);
        dm_mul(tmp5, K, tmp6, n, m, n);
        dm_sub(P_new, tmp6, P_new, n, n);

        if (dm_diff_norm(P_new, P, n, n) < DARE_TOL) return 0;
        dm_copy(P_new, P, n, n);
    }
    return -1;
}

// ================================================================
//  DISCRETIZE: Ad = I + A*dt, Bd = B*dt
// ================================================================
static void discretize(const double *A, const double *B,
                       double dt, int n, int m,
                       double *Ad, double *Bd) {
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            Ad[i*n+j] = A[i*n+j]*dt + (i==j ? 1.0 : 0.0);
    for (int i = 0; i < n*m; i++) Bd[i] = B[i]*dt;
}

// ================================================================
//  EDGE GAIN COMPUTE
//  State: [theta, theta_dot, omega]
//  A = [0, 1, 0; mgl/Ie, 0, -Iw/Ie; 0, 0, 0]
//  B = [0; -1/Ie; 1/Iw]
// ================================================================
static int compute_edge_gains(CubliEdgeGains *eg,
                              double m, double g, double l,
                              double Ie, double Iw,
                              double q_a, double q_r,
                              double q_w, double r_t) {
    double A[9] = {0, 1, 0,  m*g*l/Ie, 0, -Iw/Ie,  0, 0, 0};
    double B[3] = {0, -1.0/Ie, 1.0/Iw};
    double Ad[9], Bd[3];
    discretize(A, B, 0.001, 3, 1, Ad, Bd);

    double Q[9] = {0}; Q[0] = q_a; Q[4] = q_r; Q[8] = q_w;
    double R[1] = {r_t};
    double K[3];
    if (dare_solve(Ad, Bd, Q, R, 3, 1, K) != 0) return -1;
    eg->K[0] = (float)K[0];
    eg->K[1] = (float)K[1];
    eg->K[2] = (float)K[2];
    return 0;
}

// ================================================================
//  CORNER GAIN COMPUTE
// ================================================================
static void build_Tbf(double Tbf[9]) {
    double cy = cos(CORNER_ANGLE_Y), sy = sin(CORNER_ANGLE_Y);
    double cx = cos(CORNER_ANGLE_X), sx = sin(CORNER_ANGLE_X);
    double Ry[9] = {cy,0,sy, 0,1,0, -sy,0,cy};
    double Rx[9] = {1,0,0, 0,cx,-sx, 0,sx,cx};
    double tmp[9];
    dm_mul(Ry, Rx, tmp, 3, 3, 3);
    memcpy(Tbf, tmp, sizeof(double)*9);
}

static int compute_corner_gains(CubliCornerGains *cg,
                                const CubliPhysical *p,
                                const CubliWeights *w) {
    double Tbf[9];
    build_Tbf(Tbf);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            cg->Tbf[i][j] = (float)Tbf[i*3+j];

    double Ia = p->Iy - p->Iw;
    double Ib = p->Ix - p->Iw;
    double Ic = p->Iz;
    double mgl = p->m * p->g * p->l_corner;

    double A[81] = {0};
    A[0*9+3]=1; A[1*9+4]=1; A[2*9+5]=1;
    A[3*9+0]=mgl/Ia; A[4*9+1]=mgl/Ib;

    double B[27] = {0};
    double Ieff[3] = {Ia, Ib, Ic};
    for (int ax = 0; ax < 3; ax++)
        for (int wh = 0; wh < 3; wh++)
            B[(3+ax)*3+wh] = -Tbf[ax*3+wh] / Ieff[ax];
    for (int wh = 0; wh < 3; wh++)
        B[(6+wh)*3+wh] = 1.0 / p->Iw;

    double Ad[81], Bd[27];
    discretize(A, B, 0.001, 9, 3, Ad, Bd);

    double Q[81] = {0};
    Q[0*9+0]=w->corner_q_tilt;      Q[1*9+1]=w->corner_q_tilt;
    Q[2*9+2]=w->corner_q_yaw;       Q[3*9+3]=w->corner_q_tilt_rate;
    Q[4*9+4]=w->corner_q_tilt_rate; Q[5*9+5]=w->corner_q_yaw_rate;
    Q[6*9+6]=w->corner_q_wheel;     Q[7*9+7]=w->corner_q_wheel;
    Q[8*9+8]=w->corner_q_wheel;

    double R[9] = {0};
    R[0]=R[4]=R[8]=w->corner_r_torque;

    double K[27];
    if (dare_solve(Ad, Bd, Q, R, 9, 3, K) != 0) return -1;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 9; j++)
            cg->K[i][j] = (float)K[i*9+j];
    return 0;
}

// ================================================================
//  SETPOINT INTEGRATOR
// ================================================================
static float sp_update(float *integrator, float nominal, float error,
                       const CubliSetpointParams *p, float dt) {
    if (fabsf(error) > p->deadband)
        *integrator += p->ki_sp * error * dt;
    *integrator *= (1.0f - p->leak * dt);
    if (*integrator >  p->max_offset) *integrator =  p->max_offset;
    if (*integrator < -p->max_offset) *integrator = -p->max_offset;
    return nominal + *integrator;
}

// ================================================================
//  ORIENTATION DETECTION
//
//  Checks angle within a threshold, handling 360-degree wrap
//  for non-balance angles (multiples of 90).
// ================================================================
static float angle_diff(float a, float b) {
    // Smallest difference between two angles (degrees)
    float d = a - b;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return fabsf(d);
}

// Returns index of matching edge entry, or -1
static int detect_edge(const CubliParams *params,
                       float roll_deg, float pitch_deg,
                       uint8_t acquiring) {
    float bal_win = acquiring ? CUBLI_BALANCE_ACQUIRE_DEG
                              : CUBLI_BALANCE_WINDOW_DEG;

    for (int i = 0; i < CUBLI_NUM_EDGES; i++) {
        const CubliEdgeEntry *e = &params->edges[i];
        float bal_angle, nonbal_angle;

        if (e->balance_axis == 0) {
            bal_angle    = roll_deg;
            nonbal_angle = pitch_deg;
        } else if (e->balance_axis == 1) {
            bal_angle    = pitch_deg;
            nonbal_angle = roll_deg;
        } else {
            // Diagonal: check both axes against their refs
            float dr = angle_diff(roll_deg,  e->ref_balance_deg);
            float dp = angle_diff(pitch_deg, e->ref_nonbalance_deg);
            if (dr < bal_win && dp < bal_win) return i;
            continue;
        }

        float bal_err    = angle_diff(bal_angle,    e->ref_balance_deg);
        float nonbal_err = angle_diff(nonbal_angle, e->ref_nonbalance_deg);

        if (bal_err < bal_win && nonbal_err < CUBLI_NONBALANCE_WINDOW_DEG)
            return i;
    }
    return -1;
}

// Returns index of matching corner entry, or -1
static int detect_corner(const CubliParams *params,
                         float roll_deg, float pitch_deg,
                         uint8_t acquiring) {
    float win = acquiring ? CUBLI_CORNER_ACQUIRE_DEG
                          : CUBLI_CORNER_WINDOW_DEG;

    for (int i = 0; i < CUBLI_NUM_CORNERS; i++) {
        const CubliCornerEntry *c = &params->corners[i];
        if (angle_diff(roll_deg,  c->ref_roll_deg)  < win &&
            angle_diff(pitch_deg, c->ref_pitch_deg) < win)
            return i;
    }
    return -1;
}

// ================================================================
//  PUBLIC API
// ================================================================

int cubli_lqr_init(CubliLQR *ctrl, const CubliParams *params) {
    if (!ctrl || !params) return -1;
    memset(ctrl, 0, sizeof(CubliLQR));
    memcpy(&ctrl->params, params, sizeof(CubliParams));

    const CubliPhysical *p = &params->physical;
    const CubliWeights  *w = &params->weights;

    if (compute_edge_gains(&ctrl->edge_gains[0],
                           p->m, p->g, p->l_edge, p->I_edge_x, p->Iw,
                           w->edge_q_angle, w->edge_q_rate,
                           w->edge_q_wheel, w->edge_r_torque) != 0) return -1;

    if (compute_edge_gains(&ctrl->edge_gains[1],
                           p->m, p->g, p->l_edge, p->I_edge_y, p->Iw,
                           w->edge_q_angle, w->edge_q_rate,
                           w->edge_q_wheel, w->edge_r_torque) != 0) return -1;

    if (compute_edge_gains(&ctrl->edge_gains[2],
                           p->m, p->g, p->l_edge, p->I_edge_z, p->Iw,
                           w->edge_q_angle, w->edge_q_rate,
                           w->edge_q_wheel, w->edge_r_torque) != 0) return -1;

    if (compute_corner_gains(&ctrl->corner_gains, p, w) != 0) return -1;

    ctrl->active.mode        = CUBLI_MODE_INACTIVE;
    ctrl->active.entry_index = -1;
    ctrl->active.acquiring   = 1;
    ctrl->initialized        = 1;
    return 0;
}


int cubli_lqr_update(CubliLQR *ctrl,
                     const float angle_rad[3],
                     const float rate_rad_s[3],
                     const float wheel_rad_s[3],
                     float torque_out[3],
                     CubliMode *mode_out) {
    torque_out[0] = torque_out[1] = torque_out[2] = 0.0f;
    if (!ctrl->initialized) return -1;

    const float dt = 0.001f;
    const CubliSetpointParams *sp = &ctrl->params.setpoint;

    float roll_deg  = angle_rad[0] / DEG2RAD;
    float pitch_deg = angle_rad[1] / DEG2RAD;

    CubliActiveState *act = &ctrl->active;

    // ---- ORIENTATION DETECTION ----
    if (act->mode == CUBLI_MODE_INACTIVE) {
        // Try to acquire any edge
        int ei = detect_edge(&ctrl->params, roll_deg, pitch_deg, 1);
        if (ei >= 0) {
            const CubliEdgeEntry *e = &ctrl->params.edges[ei];
            act->mode        = CUBLI_MODE_EDGE;
            act->entry_index = ei;
            act->acquiring   = 0;
            act->setpoint_a  = e->nominal_sp_rad;
            act->integrator_a = 0.0f;
        } else {
            // Try corner
            int ci = detect_corner(&ctrl->params, roll_deg, pitch_deg, 1);
            if (ci >= 0) {
                const CubliCornerEntry *c = &ctrl->params.corners[ci];
                act->mode        = CUBLI_MODE_CORNER;
                act->entry_index = ci;
                act->acquiring   = 0;
                act->setpoint_a  = c->nominal_sp_alpha;
                act->setpoint_b  = c->nominal_sp_beta;
                act->setpoint_gamma = c->nominal_sp_gamma;
                act->integrator_a = 0.0f;
                act->integrator_b = 0.0f;
            }
        }
    } else {
        // Already active - check if still in balance window
        int still_valid = 0;
        if (act->mode == CUBLI_MODE_EDGE)
            still_valid = (detect_edge(&ctrl->params, roll_deg, pitch_deg, 0) == act->entry_index);
        else
            still_valid = (detect_corner(&ctrl->params, roll_deg, pitch_deg, 0) == act->entry_index);

        if (!still_valid) {
            act->mode        = CUBLI_MODE_INACTIVE;
            act->entry_index = -1;
            act->acquiring   = 1;
            if (mode_out) *mode_out = CUBLI_MODE_INACTIVE;
            return -1;
        }
    }

    if (mode_out) *mode_out = act->mode;
    if (act->mode == CUBLI_MODE_INACTIVE) return -1;

    // ---- EDGE CONTROL ----
    if (act->mode == CUBLI_MODE_EDGE) {
        const CubliEdgeEntry *e = &ctrl->params.edges[act->entry_index];
        const CubliEdgeGains *g = &ctrl->edge_gains[e->gain_axis];

        // Select and sign the balance angle
        float raw_angle = (e->balance_axis == 0) ? angle_rad[0] : angle_rad[1];
        float raw_rate  = (e->balance_axis == 0) ? rate_rad_s[0] : rate_rad_s[1];

        // For diagonal edges, project onto the diagonal
        if (e->balance_axis == 2) {
            raw_angle = (angle_rad[0] + angle_rad[1]) * 0.7071f;
            raw_rate  = (rate_rad_s[0] + rate_rad_s[1]) * 0.7071f;
        }

        raw_angle *= e->balance_sign;
        raw_rate  *= e->balance_sign;

        float error = raw_angle - act->setpoint_a;

        // Update setpoint
        act->setpoint_a = sp_update(&act->integrator_a,
                                    e->nominal_sp_rad,
                                    error, sp, dt);
        error = raw_angle - act->setpoint_a;

        float omega = wheel_rad_s[e->wheel_index];
        float tau = -(g->K[0] * error + g->K[1] * raw_rate + g->K[2] * omega);
        torque_out[e->wheel_index] = tau;
        return 0;
    }

    // ---- CORNER CONTROL ----
    const CubliCornerEntry *c = &ctrl->params.corners[act->entry_index];
    const CubliCornerGains *cg = &ctrl->corner_gains;

    float alpha = angle_rad[0];
    float beta  = angle_rad[1];
    float gamma = angle_rad[2];

    float err_a = alpha - act->setpoint_a;
    float err_b = beta  - act->setpoint_b;

    act->setpoint_a = sp_update(&act->integrator_a, c->nominal_sp_alpha, err_a, sp, dt);
    act->setpoint_b = sp_update(&act->integrator_b, c->nominal_sp_beta,  err_b, sp, dt);

    float x[9];
    x[0] = alpha - act->setpoint_a;
    x[1] = beta  - act->setpoint_b;
    x[2] = gamma - act->setpoint_gamma;
    x[3] = rate_rad_s[0];
    x[4] = rate_rad_s[1];
    x[5] = rate_rad_s[2];
    x[6] = wheel_rad_s[0];
    x[7] = wheel_rad_s[1];
    x[8] = wheel_rad_s[2];

    for (int i = 0; i < 3; i++) {
        float s = 0.0f;
        for (int j = 0; j < 9; j++) s += cg->K[i][j] * x[j];
        torque_out[i] = -s;
    }
    return 0;
}


void cubli_set_yaw_reference(CubliLQR *ctrl, float yaw_rad) {
    ctrl->active.setpoint_gamma = yaw_rad;
}

void cubli_reset_setpoints(CubliLQR *ctrl) {
    CubliActiveState *act = &ctrl->active;
    if (act->mode == CUBLI_MODE_EDGE && act->entry_index >= 0) {
        act->setpoint_a   = ctrl->params.edges[act->entry_index].nominal_sp_rad;
        act->integrator_a = 0.0f;
    } else if (act->mode == CUBLI_MODE_CORNER && act->entry_index >= 0) {
        const CubliCornerEntry *c = &ctrl->params.corners[act->entry_index];
        act->setpoint_a   = c->nominal_sp_alpha;
        act->setpoint_b   = c->nominal_sp_beta;
        act->integrator_a = 0.0f;
        act->integrator_b = 0.0f;
    }
}

CubliMode cubli_get_mode(const CubliLQR *ctrl) {
    return ctrl->active.mode;
}

int cubli_get_active_entry(const CubliLQR *ctrl) {
    return ctrl->active.entry_index;
}
