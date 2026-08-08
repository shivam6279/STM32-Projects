#ifndef CUBLI_CONFIG_H
#define CUBLI_CONFIG_H

// ============================================================================
//  CUBLI 3D CONFIGURATION
//
//  Single source of truth for the robot's physical frame: how the body axes
//  are defined, how the IMU is mounted, how the wheels map to axes over CAN,
//  and the sign / numbering conventions everything else assumes. All of this
//  was pinned by bench measurement (see the frame test) -- if the IMU or a
//  motor is remounted, fix it HERE and nowhere else.
//
//  ---------------------------------------------------------------------------
//  BODY FRAME  (right-handed, validated on the bench)
//  ---------------------------------------------------------------------------
//    X  <-> roll   <-> wheel B
//    Y  <-> pitch  <-> wheel A
//    Z  <-> yaw    <-> wheel C   (Z up at rest)
//
//    At the desired resting pose the fused attitude reads roll 0 / pitch 0.
//    Axis <-> Euler is 1:1 (X=roll, Y=pitch, Z=yaw), confirmed by rotating the
//    cube about each marked axis and watching euler + gyro.
//
//  ---------------------------------------------------------------------------
//  MOTOR SIGN CONVENTION  (measured, do not re-derive)
//  ---------------------------------------------------------------------------
//    A POSITIVE 'P' command spins a wheel CLOCKWISE viewed from OUTSIDE its
//    mounting face, which produces +axis spin:
//        wheel A +cmd -> +Y spin      wheel B +cmd -> +X spin
//        wheel C +cmd -> +Z spin
//    By reaction the body torques about the NEGATIVE of that axis, which is
//    what the LQR B-matrix (B = [0; -1/Ie; +1/Iw]) assumes -- so LQR torque
//    maps straight to the 'P' command with no per-wheel sign flip.
//
//  ---------------------------------------------------------------------------
//  GEOMETRY NUMBERING  (see cubli_geometry.html)
//  ---------------------------------------------------------------------------
//    Face   0..5 : face  = 2*axis + side   (axis X=0 Y=1 Z=2, side -=0 +=1)
//                  opposite = face ^ 1;  wheels on faces 0(B) 2(A) 4(C)
//    Edge   0..11: edge   = 4*group + pos  (group = parallel axis;
//                  pos = loSign + 2*hiSign of the two constrained faces)
//    Corner 0..7 : corner = Xbit + 2*Ybit + 4*Zbit  (bit = 1 if +side)
//
//  ---------------------------------------------------------------------------
//  UNITS AT THE LQR BOUNDARY  (reminder for the input-conditioning layer)
//  ---------------------------------------------------------------------------
//    Fusion euler:  DEGREES   -> LQR wants radians   (x pi/180)
//    Gyro rates:    DEG/S     -> LQR wants rad/s     (x pi/180)
//    Wheel rpm_*:   RPM       -> LQR wants rad/s     (x 2*pi/60)
// ============================================================================

// ---- Body-axis indices -----------------------------------------------------
#define CUBLI_AXIS_X  0   // roll  / wheel B
#define CUBLI_AXIS_Y  1   // pitch / wheel A
#define CUBLI_AXIS_Z  2   // yaw   / wheel C

// ---- Wheel <-> CAN command IDs ---------------------------------------------
// Torque commands are sent with mode 'P' to these standard IDs.
#define CUBLI_CAN_WHEEL_A  0x300u   // spins about body Y (pitch)
#define CUBLI_CAN_WHEEL_B  0x310u   // spins about body X (roll)
#define CUBLI_CAN_WHEEL_C  0x320u   // spins about body Z (yaw)

// CAN ID for the wheel that actuates each body axis (index by CUBLI_AXIS_*).
#define CUBLI_CAN_FOR_AXIS_X  CUBLI_CAN_WHEEL_B
#define CUBLI_CAN_FOR_AXIS_Y  CUBLI_CAN_WHEEL_A
#define CUBLI_CAN_FOR_AXIS_Z  CUBLI_CAN_WHEEL_C

// ---- IMU mounting transform ------------------------------------------------
// Rotates raw sensor axes into the body frame. Rows are body axes, columns are
// sensor axes. Bench result: IMU sits rotated 180 deg about body Y, so sensor
// X and Z are negated. Use as the initializer for a const float[3][3].
#define CUBLI_IMU_MOUNT_INIT { \
	{ -1.0f,  0.0f,  0.0f },   /* body X = -sensor X */ \
	{  0.0f,  1.0f,  0.0f },   /* body Y = +sensor Y */ \
	{  0.0f,  0.0f, -1.0f },   /* body Z = -sensor Z */ \
}

#endif // CUBLI_CONFIG_H
