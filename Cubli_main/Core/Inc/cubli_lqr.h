#ifndef CUBLI_LQR_H
#define CUBLI_LQR_H

#include <stdint.h>

// ============================================================
//  CUBLI LQR CONTROLLER
//  9-state corner balancing with yaw control + desaturation
//
//  State vector: x = [alpha, beta, gamma, alpha_dot, beta_dot,
//                     gamma_dot, wA, wB, wC]
//  Input vector: u = [tauA, tauB, tauC]  (Nm)
//
//  Output is motor torque — divide by Kt to get Iq current
//  command for your FOC driver.
// ============================================================


// ------------------------------------------------------------
//  SYSTEM PARAMETERS — fill these before calling lqr_init()
// ------------------------------------------------------------
typedef struct {

    // -- Physical parameters --
    float m;        // Total mass of cube (kg)
    float g;        // Gravity (9.81 m/s^2)
    float l;        // Distance from corner pivot to CoM (m)
    float Ix;       // Moment of inertia of full system about body x-axis (kg.m^2)
    float Iy;       // Moment of inertia of full system about body y-axis (kg.m^2)
    float Iz;       // Moment of inertia of full system about body z-axis (kg.m^2)
    float Iw;       // Moment of inertia of ONE reaction wheel about its spin axis (kg.m^2)
                    // Assumes all 3 wheels are identical

    // -- Friction compensation --
    float tau_static;   // Breakaway (stiction) torque (Nm)
                        // Measure: minimum current to start wheel * Kt
    float Cw;           // Viscous damping coefficient (Nm / (rad/s))
                        // Measure: spin wheel, cut power, fit deceleration curve

    // -- Motor --
    float Kt;       // Motor torque constant (Nm/A)
                    // Used externally to convert torque output to Iq current

    // -- LQR cost weights --
    // Q penalizes state error, R penalizes control effort
    // Increase q_tilt for faster tilt correction (risk: oscillation)
    // Increase q_yaw for tighter yaw control (risk: wheel saturation)
    // Increase q_wheel for faster desaturation (risk: yaw drift)
    // Increase r_torque to reduce control effort (risk: slower response)
    float q_tilt;       // Penalty on alpha, beta (tilt angles)
    float q_tilt_dot;   // Penalty on alpha_dot, beta_dot (tilt rates)
    float q_yaw;        // Penalty on gamma (yaw angle error)
    float q_yaw_dot;    // Penalty on gamma_dot
    float q_wheel;      // Penalty on wheel speed (desaturation)
    float r_torque;     // Penalty on torque input (all 3 motors equal)

} CubliParams;


// ------------------------------------------------------------
//  LQR STATE
// ------------------------------------------------------------
typedef struct {
    float K[3][9];      // LQR gain matrix (3 outputs x 9 states)
    float Tbf[3][3];    // Body-to-wheel frame transform
    CubliParams p;      // Copy of params
    uint8_t initialized;
} CubliLQR;


// ------------------------------------------------------------
//  API
// ------------------------------------------------------------

// Initialize controller and compute LQR gain matrix K.
// Returns 0 on success, -1 if parameters are invalid or
// if the Riccati solver fails to converge.
int lqr_init(CubliLQR *lqr, const CubliParams *params);

// Compute torque commands from current state.
//
// state[9]: [alpha, beta, gamma, alpha_dot, beta_dot,
//            gamma_dot, wA, wB, wC]
//   alpha, beta, gamma : Euler angles (rad), from Mahony filter
//   alpha_dot, beta_dot, gamma_dot : angular rates (rad/s), from gyro
//   wA, wB, wC : reaction wheel angular velocities (rad/s), from motor encoder/observer
//
// ref_gamma: desired yaw angle (rad)
//
// torque_out[3]: output torques [tauA, tauB, tauC] (Nm)
//   Convert to Iq: i_q = torque / Kt
void lqr_update(CubliLQR *lqr,
                const float state[9],
                float ref_gamma,
                float torque_out[3]);

// Apply friction compensation feedforward to torque commands.
// Call this AFTER lqr_update(), pass the same torque_out array.
// wheel_vel[3]: current wheel angular velocities [wA, wB, wC] (rad/s)
void lqr_friction_compensate(CubliLQR *lqr,
                              float torque_out[3],
                              const float wheel_vel[3]);

#endif // CUBLI_LQR_H
