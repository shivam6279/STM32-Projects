#include <inttypes.h>
#include <stdio.h>

#include "adc.h"
#include "board.h"
#include "boot.h"
#include "buttons.h"
#include "clock.h"
#include "console.h"
#include "esp.h"
#include "fdcan.h"
#include "motion.h"
#include "mpu.h"
#include "net.h"
#include "pwm.h"
#include "stepgen.h"
#include "tmc.h"
#include "uart.h"
#include "usb.h"

#ifdef STEPGEN_SELFTEST
#define TEST_STEPS	1000000
#define TEST_SEG	10000

uint32_t test_queued[STEP_AXES];
uint64_t test_next[STEP_AXES];

// Axis n steps at period 2+n, back to back constant rate segments, DIR flips every 100k steps.
// Drivers stay disabled.
void TestFeed() {
	uint32_t i;
	StepSegment s;

	for (i = 0; i < STEP_AXES; i++) {
		if (test_next[i] == 0) test_next[i] = StepNow() + 2 * STEP_HALF;
		while (test_queued[i] < TEST_STEPS) {
			bool dir = (test_queued[i] / TEST_SEG / 10) & 1;
			StepSegConst(&s, test_next[i], TEST_SEG, 2 + i, dir);
			if (!StepQueue(i, &s)) break;
			test_queued[i] += TEST_SEG;
			test_next[i] += s.duration;
		}
	}
}
#endif

int main(void) {
#ifdef STEPGEN_SELFTEST
	uint32_t i;
#endif

	BootInit(); // vector table is at APP_BASE, not 0x08000000

	ClockInit();
	MpuInit();
	BoardIoInit();

	BoardLed(0, true);
	PwmInit(); // coil drivers, all off
	ButtonsInit();

	StepInit();
	StepStart();
	TmcInit(); // driver UARTs, both banks
	MotionInit();

	setvbuf(stdout, NULL, _IONBF, 0); // Disable printf buffering
	UartInit();
	if (!UsbInit()) BoardLed(1, true); // LED1 on = no USB supply
	CanInit(); // both buses, 1M / 5M FD
	AdcInit(); // pressure sensors, not running until AdcStart()
	NetInit(); // ethernet + lwIP, 10.77.0.2
	EspInit(); // UART6 + IO expander, ESP32 left running

	while (1) {
		UsbTask();
		NetTask();
		ButtonsTask();
		ConsoleTask();
		TmcTask(); // driver setup at boot, DIAG faults

#ifdef STEPGEN_SELFTEST
		TestFeed();

		// LED1 on = every axis did exactly TEST_STEPS with no fault
		bool done = true;
		for (i = 0; i < STEP_AXES; i++) {
			if (!StepAxisIdle(i) || StepCount(i) != TEST_STEPS) done = false;
		}
		BoardLed(1, done && StepFault() == STEP_OK);
#endif
		__WFI();
	}
}
