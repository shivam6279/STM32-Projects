#ifndef _STEPGEN_H_
#define _STEPGEN_H_

#include <stdbool.h>
#include <inttypes.h>
#include "stepgen_fill.h"

// STEP1-8 on PE0-7, DIR1-8 on PE8-15 (order from the pin sheet), all plain GPIO.
// Axis n = nets STEP(n+1)/DIR(n+1).
// TIM6 ticks at STEP_TICK_HZ, each tick DMA1 S0 writes one word into GPIOE->BSRR.
// Circular buffer, refilled half at a time in the DMA HT/TC interrupt.

#define STEP_TICK_HZ	1000000	// 1us, max 500kHz per axis
#define STEP_HALF		512		// words per half buffer = refill deadline in us
#define STEP_DIR_SETUP	2		// ticks, TODO get from driver datasheet

#define STEP_OK				0
#define STEP_FAULT_UNDERRUN	1	// refill was late
#define STEP_FAULT_DMA		2
#define STEP_FAULT_ABORT	3
#define STEP_FAULT_DRIVER	4	// a TMC2209 DIAG pin went high

void StepInit();
void StepStart();
bool StepQueue(uint32_t axis, const StepSegment *seg);	// false if the axis queue is full
uint32_t StepQueueFree(uint32_t axis);
bool StepAxisIdle(uint32_t axis);
uint64_t StepNow();						// earliest tick a new segment can start at without being late
uint64_t StepCount(uint32_t axis);		// steps put out
int64_t StepPosition(uint32_t axis);	// signed step position (DIR high = +)
uint32_t StepLate(uint32_t axis);		// segments that started behind time
void StepAbort();
void StepDriverFault();					// stop stepping, drivers stay enabled (DIAG interrupt)
bool StepRunning();
void StepRecover();						// after StepAbort / a fault: drop the queues, start again
uint8_t StepFault();
bool StepDryRun(bool on);				// true: DMA writes a dummy word, no pins move (only while idle)
bool StepDry();							// dry run on
uint32_t StepIsrMaxUs();				// worst buffer refill time so far, us (budget: STEP_HALF us)
void StepIsrReset();

#endif
