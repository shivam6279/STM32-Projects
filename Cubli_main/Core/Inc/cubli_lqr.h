#ifndef CUBLI_LQR_H
#define CUBLI_LQR_H

#include <stdint.h>

// ================================================================
//  CUBLI LQR CONTROLLER
//
//  Supports all 12 edges and 8 corners.
//
//  EDGE   - 3-state: [theta, theta_dot, omega_wheel]
//           3 gain matrices (one per axis group X/Y/Z),
//           reused across all 4 edges of the same axis.
//
//  CORNER - 9-state: [alpha, beta, gamma,
//                     alpha_dot, beta_dot, gamma_dot,
//                     omegaA, omegaB, omegaC]
//           Single gain matrix, reused across all 8 corners.
//
//  Setpoint correction: leaky integrator with deadband,
//  slow outer loop on top of LQR.
//
//  Output: torque in Nm. Convert to Iq: Iq = tau / Kt
// ================================================================

// ----------------------------------------------------------------
//  ORIENTATION DETECTION WINDOWS  (degrees)
// ----------------------------------------------------------------
#define CUBLI_BALANCE_ACQUIRE_DEG     1.0f
#define CUBLI_BALANCE_WINDOW_DEG     20.0f
#define CUBLI_NONBALANCE_WINDOW_DEG  10.0f
#define CUBLI_CORNER_ACQUIRE_DEG      1.0f
#define CUBLI_CORNER_WINDOW_DEG      20.0f

#define CUBLI_NUM_EDGES    12
#define CUBLI_NUM_CORNERS   8

// ----------------------------------------------------------------
//  BALANCE MODE
// ----------------------------------------------------------------
typedef enum {
    CUBLI_MODE_INACTIVE = 0,
    CUBLI_MODE_EDGE     = 1,
    CUBLI_MODE_CORNER   = 2,
} CubliMode;

// ----------------------------------------------------------------
//  EDGE TABLE ENTRY  (12 entries)
//
//  balance_axis:
//    0 = roll is balance angle, pitch is non-balance
//    1 = pitch is balance angle, roll is non-balance
//    2 = diagonal edge (Z-axis wheels) - both axes near 45 deg
//
//  balance_sign:
//    +1.0 or -1.0. Flips angle so positive error = tilt away from
//    motor. Determine empirically per edge.
//
//  ref_balance_deg:
//    Expected balance angle when balanced on this edge (~45 or ~-45).
//
//  ref_nonbalance_deg:
//    Expected non-balance angle. Must be a multiple of 90.
//    Unused for diagonal edges (balance_axis == 2).
//
//  gain_axis:
//    Which of the 3 gain matrices to use: 0=X, 1=Y, 2=Z
//
//  nominal_sp_rad:
//    Starting setpoint. Fill from physical measurement.
// ----------------------------------------------------------------
typedef struct {
    float    ref_balance_deg;
    float    ref_nonbalance_deg;
    uint8_t  balance_axis;
    float    balance_sign;
    uint8_t  wheel_index;
    uint8_t  gain_axis;
    float    nominal_sp_rad;
} CubliEdgeEntry;

// ----------------------------------------------------------------
//  CORNER TABLE ENTRY  (8 entries)
//
//  All 8 corners share one gain matrix.
//  ref_roll_deg, ref_pitch_deg: expected IMU angles on this corner.
//  Fill from measurement - different for each corner.
// ----------------------------------------------------------------
typedef struct {
    float    ref_roll_deg;
    float    ref_pitch_deg;
    float    nominal_sp_alpha;
    float    nominal_sp_beta;
    float    nominal_sp_gamma;
} CubliCornerEntry;

// ----------------------------------------------------------------
//  PHYSICAL PARAMETERS
// ----------------------------------------------------------------
typedef struct {
    float m;
    float g;
    float l_edge;
    float l_corner;
    float I_edge_x;
    float I_edge_y;
    float I_edge_z;
    float Ix;
    float Iy;
    float Iz;
    float Iw;
} CubliPhysical;

// ----------------------------------------------------------------
//  LQR COST WEIGHTS
// ----------------------------------------------------------------
typedef struct {
    float edge_q_angle;
    float edge_q_rate;
    float edge_q_wheel;
    float edge_r_torque;
    float corner_q_tilt;
    float corner_q_tilt_rate;
    float corner_q_yaw;
    float corner_q_yaw_rate;
    float corner_q_wheel;
    float corner_r_torque;
} CubliWeights;

// ----------------------------------------------------------------
//  SETPOINT INTEGRATOR PARAMETERS
// ----------------------------------------------------------------
typedef struct {
    float ki_sp;        // Integration rate (rad/s per rad error)
    float leak;         // Leak rate (fraction per second)
    float deadband;     // Error below which integrator freezes (rad)
    float max_offset;   // Max setpoint deviation from nominal (rad)
} CubliSetpointParams;

// ----------------------------------------------------------------
//  FULL PARAMETER STRUCT
// ----------------------------------------------------------------
typedef struct {
    CubliPhysical       physical;
    CubliWeights        weights;
    CubliSetpointParams setpoint;
    CubliEdgeEntry      edges[CUBLI_NUM_EDGES];
    CubliCornerEntry    corners[CUBLI_NUM_CORNERS];
} CubliParams;

// ----------------------------------------------------------------
//  INTERNAL STATE
// ----------------------------------------------------------------
typedef struct {
    float K[3];             // [K_angle, K_rate, K_wheel]
} CubliEdgeGains;

typedef struct {
    float K[3][9];
    float Tbf[3][3];
} CubliCornerGains;

typedef struct {
    CubliMode   mode;
    int         entry_index;    // Index into edges[] or corners[], -1 if inactive
    uint8_t     acquiring;      // 1 = searching, 0 = actively balancing
    float       setpoint_a;     // Primary axis setpoint (rad)
    float       setpoint_b;     // Secondary axis setpoint (rad, corner only)
    float       setpoint_gamma; // Yaw setpoint (rad, corner only)
    float       integrator_a;
    float       integrator_b;
} CubliActiveState;

typedef struct {
    CubliEdgeGains   edge_gains[3];     // X, Y, Z axis gain matrices
    CubliCornerGains corner_gains;
    CubliActiveState active;
    CubliParams      params;
    uint8_t          initialized;
} CubliLQR;

// ================================================================
//  PUBLIC API
// ================================================================

// Initialize and compute gain matrices. DARE runs in double.
// Returns 0 on success, -1 on failure.
int cubli_lqr_init(CubliLQR *ctrl, const CubliParams *params);

// Main update - call at 1kHz.
// angle_rad[3]   : [roll, pitch, yaw] from Mahony (rad)
// rate_rad_s[3]  : [p, q, r] from gyro (rad/s)
// wheel_rad_s[3] : [omegaA, omegaB, omegaC] (rad/s)
// torque_out[3]  : output [tauA, tauB, tauC] (Nm)
// Returns 0 if active, -1 if inactive.
int cubli_lqr_update(CubliLQR *ctrl,
                     const float angle_rad[3],
                     const float rate_rad_s[3],
                     const float wheel_rad_s[3],
                     float torque_out[3],
                     CubliMode *mode_out);

// Set yaw reference for corner mode (rad).
void cubli_set_yaw_reference(CubliLQR *ctrl, float yaw_rad);

// Reset setpoint integrators. Call after large disturbance or mode switch.
void cubli_reset_setpoints(CubliLQR *ctrl);

// Get current mode and active entry index (-1 if inactive).
CubliMode cubli_get_mode(const CubliLQR *ctrl);
int       cubli_get_active_entry(const CubliLQR *ctrl);

#endif // CUBLI_LQR_H
