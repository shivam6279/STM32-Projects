#ifndef COGGING_H
#define COGGING_H

#include <stdint.h>

// Cogging torque compensation LUT (Savitzky-Golay smoothed, window=15)
// Indexed by mechanical encoder count (0-2047)
// Iq_ff = (cogging_lut[encoder_count] / 32767.0f) * COGGING_LUT_SCALE
// Coulomb friction: 0.263A
#define COGGING_LUT_SIZE 2048
#define COGGING_LUT_SCALE 0.2f

extern const int16_t cogging_lut[COGGING_LUT_SIZE];

#endif // COGGING_H
