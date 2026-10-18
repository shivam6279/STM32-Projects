#ifndef _JOG_H_
#define _JOG_H_

#include <stdbool.h>
#include <inttypes.h>
#include "motion.h"

// Velocity jog: each axis follows a target speed in mm/s instead of going to a point.
// JogTick (TIM7, every 10ms) keeps ~20ms of segments queued, so a new speed takes effect within
// ~30ms and the queue doesn't run dry when the main loop is busy. Axes are on their own (no shared
// path like MotionMove), speed ramps at the axis' a_max (trapezoid, no jerk limit yet).
// A target only lasts JOG_TIMEOUT_MS: keep calling JogSet or the axes ramp down and stop.
// While jogging, MotionMove / MotionSteps return MOTION_JOG.

#define JOG_PERIOD		10000	// ticks per segment, = the TIM7 period
#define JOG_TIMEOUT_MS	100

#define JOG_OK		0
#define JOG_BUSY	1	// motion still queued
#define JOG_FAULT	2	// step generator stopped

uint8_t JogStart();
void JogSet(const float *v);	// mm/s per axis (MOTION_AXES), + = the axis' + direction
void JogStop();					// ramp down, the axes go back to MotionMove once stopped
void JogTask();					// main loop: hands the axes back after a stop
bool JogActive();				// jog has the axes (from JogStart until stopped and synced)
bool JogFaulted();				// last jog ended because the step generator stopped
uint32_t JogLate();				// times an axis' queue ran dry while moving (shouldn't happen)
void JogTick();					// TIM7 interrupt

#endif
