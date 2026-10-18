#include "diags.h"
#include "adc.h"
#include "board.h"
#include "buttons.h"
#include "boot.h"
#include "clock.h"
#include "console.h"
#include "esp.h"
#include "eth.h"
#include "fdcan.h"
#include "ioexp.h"
#include "motion.h"
#include "net.h"
#include "jog.h"
#include "pad.h"
#include "scurve.h"
#include "pwm.h"
#include "stepgen.h"
#include "tmc.h"
#include "uart.h"
#include "usb.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


uint8_t diags_info(char*);
uint8_t diags_LED(char*);
uint8_t diags_step(char*);
void stepBench();
bool argFlag(char *cmd, const char *arg);
uint8_t diags_move(char*);
uint8_t diags_pos(char*);
uint8_t diags_axis(char*);
uint8_t tmcWords(char *cmd, char *buf, uint32_t len, char **w, uint8_t max);
uint8_t diags_reset(char*);
uint8_t diags_boot(char*);
uint8_t diags_can(char*);
uint8_t diags_pwm(char*);
uint8_t diags_btn(char*);
uint8_t diags_press(char*);
uint8_t diags_eth(char*);
uint8_t diags_tmc(char*);
uint8_t diags_esp(char*);
uint8_t diags_pad(char*);
void printDiagsMenu();

typedef uint8_t (*diags_function)(char*);

typedef struct diags_menu_item {
	char name[40];
	char cmd[10];
	diags_function func;
} diags_menu_item;

const diags_menu_item diags_list[] = {
	{ .name = "Board info",				.cmd = "info",	.func = diags_info	},
	{ .name = "Turn LEDs on or off",	.cmd = "led",	.func = diags_LED	},
	{ .name = "Stepper status",			.cmd = "step",	.func = diags_step	},
	{ .name = "Move axes (mm)",			.cmd = "move",	.func = diags_move	},
	{ .name = "Axis positions",			.cmd = "pos",	.func = diags_pos	},
	{ .name = "Axes: drivers, direction, steps/mm",	.cmd = "axis",	.func = diags_axis	},
	{ .name = "Software reset",			.cmd = "reset",	.func = diags_reset	},
	{ .name = "CAN bus commands",		.cmd = "can",	.func = diags_can	},
	{ .name = "Coil driver PWM outputs",	.cmd = "pwm",	.func = diags_pwm	},
	{ .name = "Button inputs",			.cmd = "btn",	.func = diags_btn	},
	{ .name = "Pressure sensors",		.cmd = "press",	.func = diags_press	},
	{ .name = "Ethernet",				.cmd = "eth",	.func = diags_eth	},
	{ .name = "Stepper drivers (TMC2209 UART)",	.cmd = "tmc",	.func = diags_tmc	},
	{ .name = "ESP32: reset, bootloader, UART bridge",	.cmd = "esp",	.func = diags_esp	},
	{ .name = "Xbox controller (through the ESP32)",	.cmd = "pad",	.func = diags_pad	},
	{ .name = "Reboot to the USB bootloader",	.cmd = "boot",	.func = diags_boot	},
};

unsigned char diags_list_len = sizeof(diags_list) / sizeof(diags_list[0]);

bool can_mon = false;
bool btn_mon = false;

void DiagsMenu() {
	unsigned char i;
	for (i = 0; i < diags_list_len; i++) {
		printf("%s - %s\n", diags_list[i].cmd, diags_list[i].name);
	}
	printf("help - This list\n");
	printf("exit - Back to G-code\n");
}

void DiagsRun(char *cmd) {
	unsigned char i, n;

	if (cmd[0] == '\0') return;

	if (strcmp(cmd, "help") == 0) {
		DiagsMenu();
		return;
	}
	for (i = 0; i < diags_list_len; i++) {
		n = strlen(diags_list[i].cmd);
		// whole word match only
		if (strncmp(cmd, diags_list[i].cmd, n) == 0 && (cmd[n] == ' ' || cmd[n] == '\0')) {
			diags_list[i].func(cmd);
			return;
		}
	}
	printf("Unknown command, try help\n");
}

// Background output for 'btn mon' / 'can mon', goes to ports in diags mode
void DiagsPoll() {
	if (btn_mon) {
		uint8_t c = ButtonsChanged(), s = ButtonsState(), b;
		for (b = 0; b < BUTTONS; b++) {
			if ((c >> b) & 1) printf("BTN%d: %d (%" PRIu32 " edges) at %" PRIu32 " ms\n", b + 1, (s >> b) & 1, ButtonsEdges(b), Millis());
		}
	}

	if (can_mon) {
		CanFrame f;
		uint8_t b, j;
		for (b = 0; b < CAN_BUSES; b++) {
			while (CanRead(b, &f)) {
				printf("CAN%d %s %0*" PRIX32 " [%d]", b + 1, f.fd ? (f.brs ? "FD+BRS" : "FD") : "classic", f.ext ? 8 : 3, f.id, f.len);
				for (j = 0; j < f.len; j++) printf(" %02X", f.data[j]);
				printf("\n");
			}
		}
	}
}

uint8_t diags_info(char *cmd) {
	const uint32_t *uid = (const uint32_t *)UID_BASE;
	(void)cmd;

	printf("MCU: STM32H723, DEV_ID 0x%03" PRIX32 ", REV_ID 0x%04" PRIX32 "\n", DBGMCU->IDCODE & 0xFFF, DBGMCU->IDCODE >> 16);
	printf("UID: %08" PRIX32 "%08" PRIX32 "%08" PRIX32 "\n", uid[0], uid[1], uid[2]);
	printf("SYSCLK %" PRIu32 " MHz, HCLK %" PRIu32 " MHz\n", (uint32_t)CLOCK_SYSCLK_HZ / 1000000, (uint32_t)CLOCK_HCLK_HZ / 1000000);
	printf("Uptime %" PRIu32 " ms\n", Millis());
	return true;
}

uint8_t diags_LED(char *cmd) {
	const char help_str[] = "\
led <led no> <state>:\n\
<led no> : 0 or 1\n\
<state> : 0: off, 1: on\n";

	// "led 0 1"
	if (strlen(cmd) < 7 || (cmd[4] != '0' && cmd[4] != '1') || (cmd[6] != '0' && cmd[6] != '1')) {
		fputs(help_str, stdout);
		return false;
	}
	BoardLed(cmd[4] - '0', cmd[6] == '1');
	printf("LED%c %s\n", cmd[4], cmd[6] == '1' ? "on" : "off");
	return true;
}

uint8_t diags_step(char *cmd) {
	uint32_t i;
	const char *faults[] = { "none", "underrun", "dma error", "stopped", "driver fault (DIAG)" };

	const char help_str[] = "\
step <drv> <steps> [steps/s] : that many STEP pulses on driver 0-7, - = DIR low. Default 1000/s. Turns the drivers on\n\
step : status (drivers, dry run, fault, every axis)\n\
en : drivers on (checks every TMC2209 has its settings and no fault first)\n\
off : drivers off. If anything is moving it stops, like stop\n\
stop : stop all motion now, drivers off. Needs step recover\n\
recover : after stop or a fault: drop what was queued, start again (drivers stay off)\n\
dry on / dry off : step generator to a dummy word / to the STEP/DIR pins\n\
bench : refill time test, in dry run\n\
isr : worst refill time since the last isr\n";

	if (argFlag(cmd, "-h") || argFlag(cmd, "--help")) {
		fputs(help_str, stdout);
		return true;
	}

	// step <drv> <steps> [rate]
	char *p = cmd + 4;
	while (*p == ' ') p++;
	if (*p >= '0' && *p <= '9') {
		uint32_t drv = strtoul(p, &p, 10);
		int32_t n = strtol(p, &p, 10);
		float rate = strtof(p, NULL);
		uint8_t res;
		if (rate == 0) rate = 1000;
		if (drv >= STEP_AXES || n == 0 || rate < 0) {
			fputs(help_str, stdout);
			return false;
		}
		if (!StepRunning()) {
			printf("Stopped (%s), step recover first\n", faults[StepFault() < 5 ? StepFault() : 0]);
			return false;
		}
		if (!BoardStepperEnabled()) {
			if (!TmcReady()) {
				printf("Drivers not ready, staying off\n");
				return false;
			}
			BoardStepperEnable(true);
			printf("Drivers on\n");
		}
		res = MotionSteps(drv, n, rate);
		if (res == MOTION_BUSY) printf("Queue full, try again\n");
		else if (res == MOTION_BAD) printf("Rate must be 1 - %d steps/s, and the run under ~71 min\n", MOTION_MAX_STEP_HZ);
		else if (res == MOTION_JOG) printf("Jog mode has the axes, pad mode off first\n");
		else if (res == MOTION_OK) {
			printf("Driver %" PRIu32 " (%s): %" PRId32 " steps, DIR %s, %.0f steps/s, %.2f s%s\n", drv, MotionDriverName(drv),
				n < 0 ? -n : n, n > 0 ? "high" : "low", (double)rate, (double)((n < 0 ? -n : n) / rate), StepDry() ? ", dry run" : "");
		}
		return res == MOTION_OK;
	}

	if (argFlag(cmd, "en")) {
		if (!StepRunning()) {
			printf("Stopped (%s), step recover first\n", faults[StepFault() < 5 ? StepFault() : 0]);
			return false;
		}
		if (!TmcReady()) {
			printf("Drivers not ready, staying off\n");
			return false;
		}
		BoardStepperEnable(true);
		printf("Drivers on%s\n", StepDry() ? " (dry run is on, nothing will step)" : "");
		return true;
	}
	if (argFlag(cmd, "off") && !argFlag(cmd, "dry")) {
		if (MotionIdle()) {
			BoardStepperEnable(false);
			printf("Drivers off\n");
		} else {
			StepAbort();
			printf("Was moving: stopped, drivers off. step recover to go again\n");
		}
		return true;
	}
	if (argFlag(cmd, "stop")) {
		StepAbort();
		printf("Stopped, drivers off. step recover to go again\n");
		return true;
	}
	if (argFlag(cmd, "recover")) {
		if (StepRunning()) {
			printf("Not stopped\n");
			return true;
		}
		if (!TmcReady()) {
			printf("Driver fault still there, not recovering\n");
			return false;
		}
		StepRecover();
		MotionSync();
		printf("Recovered, drivers are off (step en)\n");
		return true;
	}
	if (argFlag(cmd, "bench")) {
		stepBench();
		return true;
	}
	if (argFlag(cmd, "isr")) {
		printf("Worst refill since last 'step isr': %" PRIu32 " us of %d\n", StepIsrMaxUs(), STEP_HALF);
		StepIsrReset();
		return true;
	}
	if (argFlag(cmd, "dry")) {
		bool on = !argFlag(cmd, "off");
		if (!StepDryRun(on)) {
			printf("Axes are moving, try again when idle\n");
			return false;
		}
		printf("Dry run %s: STEP/DIR pins %s\n", on ? "on" : "off", on ? "don't move" : "are live");
		return true;
	}

	printf("Drivers %s, dry run %s, %s, fault: %s, clock at tick %" PRIu32 "\n",
		BoardStepperEnabled() ? "on" : "off", StepDry() ? "on" : "off", StepRunning() ? "running" : "stopped",
		faults[StepFault() < 5 ? StepFault() : 0], (uint32_t)StepNow());
	for (i = 0; i < STEP_AXES; i++) {
		printf("Driver %" PRIu32 " (%s): %s, position %" PRId32 ", %" PRIu32 " steps out, %" PRIu32 " queued segments, %" PRIu32 " late\n",
			i, MotionDriverName(i), StepAxisIdle(i) ? "idle" : "busy", (int32_t)StepPosition(i), (uint32_t)StepCount(i),
			STEP_QUEUE_LEN - StepQueueFree(i), StepLate(i));
	}
	return true;
}

// Feed segments to all 8 axes until 'count' each are queued, then wait for idle. Keeps USB alive.
bool benchFeed(const StepSegment *segs, uint32_t nseg, uint32_t count, uint64_t *t) {
	uint32_t ax, done[STEP_AXES] = { 0 }, all, k;
	StepSegment s;
	uint64_t base[STEP_AXES];
	uint32_t start = Millis();

	for (ax = 0; ax < STEP_AXES; ax++) base[ax] = *t;
	do {
		all = 1;
		for (ax = 0; ax < STEP_AXES; ax++) {
			while (done[ax] < count && StepQueueFree(ax) > nseg) {
				for (k = 0; k < nseg; k++) {
					s = segs[k];
					s.start += base[ax];
					s.dir = (done[ax] & 1) == 0;	// out and back, ends where it started
					StepQueue(ax, &s);
				}
				base[ax] += segs[nseg - 1].start + segs[nseg - 1].duration;
				done[ax]++;
			}
			if (done[ax] < count) all = 0;
		}
		UsbTask();
		if (StepFault() != STEP_OK || Millis() - start > 20000) return false;
	} while (!all);
	while (!MotionIdle()) {
		UsbTask();
		if (StepFault() != STEP_OK) return false;
	}
	*t = base[0];
	return true;
}

// Worst case refill time with all 8 axes busy, DMA pointed at a dummy word so nothing moves
void stepBench() {
	StepSegment segs[SCURVE_PHASES];
	SCurve sc;
	uint32_t n, ax;
	uint64_t t;
	bool ok;

	if (!MotionIdle()) {
		printf("Axes are moving, try again when idle\n");
		return;
	}
	StepDryRun(true);
	printf("Dry run: DMA -> dummy, STEP/DIR pins don't move\n");

	// 1: constant 333kHz on all 8 axes (2.7M steps/s), 20 x 20000 steps out and back
	StepIsrReset();
	t = StepNow() + 2 * STEP_HALF;
	StepSegConst(&segs[0], 0, 20000, 3, true);
	ok = benchFeed(segs, 1, 20, &t);
	printf("Constant 333kHz x8: %s, worst refill %" PRIu32 " us of %d\n", ok ? "ok" : "FAULT", StepIsrMaxUs(), STEP_HALF);

	// 2: S-curves, every step goes through the solver. 16000 steps on all 8 axes at the planner's
	// total budget: 500k steps/s / 8 = 62.5kHz peak each, the worst the planner can ask for
	StepIsrReset();
	t = StepNow() + 2 * STEP_HALF;
	SCurvePlan(&sc, 0.0625 / 16000, 1e-5 / 16000, 5e-9 / 16000);
	n = SCurveSegments(&sc, 16000, true, 0, segs);
	ok = benchFeed(segs, n, 10, &t);
	printf("S-curve budget, 8 x 62.5kHz peak (%.1f ms moves): %s, worst refill %" PRIu32 " us of %d\n",
		(double)sc.total / 1000, ok ? "ok" : "FAULT", StepIsrMaxUs(), STEP_HALF);

	for (ax = 0; ax < STEP_AXES; ax++) {
		if (StepLate(ax)) printf("Driver %" PRIu32 ": %" PRIu32 " late segments\n", ax, StepLate(ax));
	}
	StepDryRun(false);
	printf("Back on the pins\n");
}

// "move X10 Y-2.5 F200": absolute move in mm, F = feed in mm/s (default 50). -r = relative
uint8_t diags_move(char *cmd) {
	float target[MOTION_AXES], feed = 50;
	uint8_t i, mask = 0, res;
	char *p = cmd + 4;
	bool rel = false;
	SCurve sc;

	for (i = 0; i < MOTION_AXES; i++) target[i] = MotionPosition(i);
	while (*p) {
		while (*p == ' ') p++;
		if (*p == '\0') break;
		if (strncmp(p, "-r", 2) == 0 && (p[2] == ' ' || p[2] == '\0')) {
			rel = true;
			p += 2;
			continue;
		}
		char c = *p >= 'a' && *p <= 'z' ? *p - 32 : *p;
		float v = strtof(p + 1, &p);
		if (c == 'F') {
			feed = v;
			continue;
		}
		for (i = 0; i < MOTION_AXES && motion_axes[i].name != c; i++);
		if (i == MOTION_AXES) {
			printf("move [-r] [X..] [Y..] ... [F mm/s] : move in mm, absolute (-r relative), axes ");
			for (i = 0; i < MOTION_AXES; i++) printf("%c", motion_axes[i].name);
			printf("\n");
			return false;
		}
		target[i] = v;
		mask |= 1 << i;
	}
	if (rel) {
		for (i = 0; i < MOTION_AXES; i++) {
			if ((mask >> i) & 1) target[i] += MotionPosition(i);
		}
	}

	res = MotionMove(target, mask, feed);
	if (res == MOTION_BUSY) printf("Queue full, try again\n");
	else if (res == MOTION_NOMOVE) printf("Already there\n");
	else if (res == MOTION_BAD) printf("Bad move\n");
	else if (res == MOTION_FAULT) printf("Stopped, step recover first\n");
	else if (res == MOTION_JOG) printf("Jog mode has the axes, pad mode off first\n");
	else {
		MotionLastPlan(&sc);
		printf("Queued: %.1f ms (jerk %.1f, accel %.1f, cruise %.1f ms)%s%s\n",
			(double)sc.total / 1000, (double)sc.tj / 1000, (double)sc.ta / 1000, (double)sc.tv / 1000,
			StepDry() ? ", dry run" : "", BoardStepperEnabled() ? "" : ", drivers are off (step en)");
	}
	return res == MOTION_OK;
}

uint8_t diags_pos(char *cmd) {
	uint8_t i;
	(void)cmd;
	for (i = 0; i < MOTION_AXES; i++) {
		printf("%c: planned %.4f, actual %.4f mm\n", motion_axes[i].name, (double)MotionPosition(i), (double)MotionActual(i));
	}
	printf("%s\n", MotionIdle() ? "Idle" : "Moving");
	return true;
}

uint8_t diags_reset(char *cmd) {
	uint32_t t;
	(void)cmd;

	printf("Resetting\n");
	t = Millis();
	while (Millis() - t < 50) UsbTask(); // let the message go out
	NVIC_SystemReset();
	return true;
}

// Fill a test frame: mix of std/ext IDs, FD+BRS / FD / classic, every FD length
void canTestFrame(CanFrame *f, uint32_t n) {
	static const uint8_t lens[] = { 0, 1, 8, 12, 16, 20, 24, 32, 48, 64 };
	uint32_t i;

	f->ext = n & 1;
	f->id = f->ext ? (0x1ABC000 + n) & 0x1FFFFFFF : (0x100 + n) & 0x7FF;
	f->fd = (n % 5) != 4;			// every 5th is classic
	f->brs = f->fd && (n % 3) != 2;	// some FD frames without BRS
	f->len = f->fd ? lens[n % 10] : (n % 9);
	for (i = 0; i < f->len; i++) f->data[i] = (uint8_t)(n * 7 + i * 13);
}

bool canSame(const CanFrame *a, const CanFrame *b) {
	return a->id == b->id && a->ext == b->ext && a->fd == b->fd && a->brs == b->brs
		&& a->len == b->len && memcmp(a->data, b->data, a->len) == 0;
}

// Send n frames on tx, expect each one on rx. Returns number of bad frames.
uint32_t canXfer(uint8_t tx, uint8_t rx, uint32_t n) {
	CanFrame f, g;
	uint32_t i, t, bad = 0, missing = 0, start = Millis();

	while (CanRead(rx, &g)); // flush
	for (i = 0; i < n; i++) {
		canTestFrame(&f, i);
		t = Millis();
		while (!CanSend(tx, &f)) {
			if (Millis() - t > 50) break;
		}
		t = Millis();
		while (!CanRead(rx, &g)) {
			if (Millis() - t > 20) break;
		}
		if (Millis() - t > 20) {
			missing++;
			if (missing > 5) break; // nothing's getting through, stop
			continue;
		}
		if (!canSame(&f, &g)) bad++;
	}
	if (tx != rx) printf("CAN%d -> CAN%d: ", tx + 1, rx + 1);
	else printf("CAN%d loopback: ", tx + 1);
	printf("%" PRIu32 " frames, %" PRIu32 " missing, %" PRIu32 " wrong, %" PRIu32 " ms\n", i, missing, bad, Millis() - start);
	return bad + missing + (n - i);
}

void canPrintStatus(uint8_t bus) {
	const char *lec[] = { "none", "stuff", "form", "ack", "bit1", "bit0", "crc", "-" };
	CanStatus s;

	CanGetStatus(bus, &s);
	printf("CAN%d: %s, TEC %d REC %d, last err %s / data %s, TDCV %d tq, rx %" PRIu32 " tx %" PRIu32 " dropped %" PRIu32 "\n",
		bus + 1, s.bus_off ? "BUS OFF" : s.passive ? "error passive" : s.warning ? "warning" : "ok",
		s.tec, s.rec, lec[s.lec], lec[s.dlec], s.tdcv, s.rx_count, s.tx_count, s.rx_dropped);
}

uint8_t diags_can(char *cmd) {
	CanFrame f;
	uint32_t n, bad;
	unsigned long v;
	char *p;
	uint8_t bus;

	const char help_str[] = "\
can : status of both buses\n\
can test [n] : n frames CAN1 -> CAN2 and back (buses must be wired together)\n\
can loop <1|2> [n] : internal loopback on one bus, nothing on the wire\n\
can send <1|2> <id hex> [bytes hex] : send an FD+BRS frame (id > 7FF = extended)\n\
can mon : toggle printing received frames\n";

	p = cmd + 3;
	while (*p == ' ') p++;

	if (*p == '\0') {
		canPrintStatus(0);
		canPrintStatus(1);
	} else if (strncmp(p, "test", 4) == 0) {
		n = strtoul(p + 4, NULL, 10);
		if (n == 0) n = 1000;
		bad = canXfer(0, 1, n) + canXfer(1, 0, n);
		canPrintStatus(0);
		canPrintStatus(1);
		printf("%s\n", bad ? "FAIL" : "PASS");
	} else if (strncmp(p, "loop", 4) == 0) {
		bus = strtoul(p + 4, &p, 10) - 1;
		if (bus >= CAN_BUSES) {
			fputs(help_str, stdout);
			return false;
		}
		n = strtoul(p, NULL, 10);
		if (n == 0) n = 1000;
		CanLoopback(bus, true);
		bad = canXfer(bus, bus, n);
		CanLoopback(bus, false);
		printf("%s\n", bad ? "FAIL" : "PASS");
	} else if (strncmp(p, "send", 4) == 0) {
		bus = strtoul(p + 4, &p, 10) - 1;
		if (bus >= CAN_BUSES) {
			fputs(help_str, stdout);
			return false;
		}
		memset(&f, 0, sizeof(f));
		f.id = strtoul(p, &p, 16);
		f.ext = f.id > 0x7FF;
		f.fd = true;
		f.brs = true;
		f.len = 0;
		while (f.len < 64) {
			v = strtoul(p, &p, 16);
			while (*p == ' ') p++;
			if (*p == '\0' && v == 0 && f.len == 0) break;
			f.data[f.len++] = v;
			if (*p == '\0') break;
		}
		f.len = CanRoundLen(f.len);
		printf(CanSend(bus, &f) ? "Sent\n" : "TX FIFO full\n");
	} else if (strncmp(p, "mon", 3) == 0) {
		can_mon = !can_mon;
		printf("CAN monitor %s\n", can_mon ? "on" : "off");
	} else {
		fputs(help_str, stdout);
		return false;
	}
	return true;
}

uint8_t diags_pwm(char *cmd) {
	uint8_t ch;
	char *p;
	float duty;

	const char help_str[] = "\
pwm : show all coil driver outputs\n\
pwm <0-7> <duty %> : set an output, 0-7 = PWM1-8, e.g. pwm 2 50\n\
pwm meas <0-7> : measure the duty on the pin\n\
pwm off : all off\n";

	p = cmd + 3;
	while (*p == ' ') p++;

	if (*p == '\0') {
		for (ch = 0; ch < PWM_CHANNELS; ch++) printf("PWM%d: %.1f%%\n", ch + 1, PwmGet(ch) * 100);
		printf("%d Hz\n", PWM_FREQ);
	} else if (strncmp(p, "off", 3) == 0) {
		PwmAllOff();
		printf("All off\n");
	} else if (strncmp(p, "meas", 4) == 0) {
		ch = strtoul(p + 4, NULL, 10);
		if (ch >= PWM_CHANNELS) {
			fputs(help_str, stdout);
			return false;
		}
		printf("PWM%d: set %.1f%%, measured %.1f%%\n", ch + 1,PwmGet(ch) * 100, PwmMeasure(ch) * 100);
	} else if (*p >= '0' && *p <= '9') {
		ch = strtoul(p, &p, 10);
		duty = strtof(p, NULL);
		if (ch >= PWM_CHANNELS || duty < 0 || duty > 100) {
			fputs(help_str, stdout);
			return false;
		}
		PwmSet(ch, duty / 100);
		printf("PWM%d: %.1f%%\n", ch + 1, PwmGet(ch) * 100);
	} else {
		fputs(help_str, stdout);
		return false;
	}
	return true;
}

// ---- Argument / streaming helpers ----

// True if the whole word arg (e.g. "-s") is in cmd
bool argFlag(char *cmd, const char *arg) {
	char *p = cmd;
	unsigned int n = strlen(arg);

	while ((p = strstr(p, arg)) != NULL) {
		if ((p == cmd || p[-1] == ' ') && (p[n] == ' ' || p[n] == '\0')) return true;
		p += n;
	}
	return false;
}

// Number after arg (e.g. "-f 50"), false if arg isn't there
bool argFloat(char *cmd, const char *arg, float *val) {
	char *p = cmd;
	unsigned int n = strlen(arg);

	while ((p = strstr(p, arg)) != NULL) {
		if ((p == cmd || p[-1] == ' ') && p[n] == ' ') {
			*val = strtof(p + n, NULL);
			return true;
		}
		p += n;
	}
	return false;
}

// Channel list like "0", "1,5,7" or "2-3,5,6-7" -> bitmask. Picks the first word that starts
// with a digit and isn't the value of -f. No list = all channels. Returns 0 if the list is bad.
uint32_t argChannels(char *cmd, uint8_t count) {
	uint32_t mask = 0, a, b;
	char *p = cmd, *prev = "";
	uint32_t all = count >= 32 ? 0xFFFFFFFF : (1u << count) - 1;

	while (*p) {
		while (*p == ' ') p++;
		if (*p >= '0' && *p <= '9' && strncmp(prev, "-f", 2) != 0) break;
		prev = p;
		while (*p && *p != ' ') p++;
	}
	if (*p == '\0') return all;

	while (*p && *p != ' ') {
		a = strtoul(p, &p, 10);
		b = a;
		if (*p == '-') b = strtoul(p + 1, &p, 10);
		if (a > b || b >= count) return 0;
		for (; a <= b; a++) mask |= 1u << a;
		if (*p == ',') p++;
		else if (*p && *p != ' ') return 0;
	}
	return mask;
}

// Wait out one stream period, keeping USB and the buttons running. True if 'x' was typed.
bool streamWait(uint32_t start, uint32_t period_ms) {
	int c;
	do {
		UsbTask();
		ButtonsTask();
		PadTask(); // jog keeps following the sticks while streaming
		// only the port that started the stream: the other one may be a G-code host
		while ((c = ConsoleReadChar()) >= 0) {
			if (c == 'x') return true;
		}
	} while (Millis() - start < period_ms);
	return false;
}

// -f [x] in Hz -> stream period in ms. Default 100ms like the other boards.
bool streamPeriod(char *cmd, uint32_t *period_ms) {
	float f;
	*period_ms = 100;
	if (argFloat(cmd, "-f", &f)) {
		if (f < 1.0f || f > 10000.0f) {
			printf("Invalid frequency\n");
			return false;
		}
		*period_ms = (uint32_t)(1000.0f / f);
		printf("Streaming at %.1f Hz\n", (double)f);
	}
	return true;
}

uint8_t diags_btn(char *cmd) {
	uint8_t i, state = ButtonsState(), raw = ButtonsRaw();
	uint32_t mask, period, t;
	bool first;

	const char help_str[] = "\
btn [ch] : show buttons (debounced), ch = 0-7 for BTN1-8, e.g. 2-3,5,6-7. Default all\n\
-s : stream the pin levels until 'x' is entered\n\
-f [x] : streaming frequency in Hz [1, 10000], default 10\n\
mon : toggle printing debounced changes as they happen\n";

	if (argFlag(cmd, "-h") || argFlag(cmd, "--help")) {
		fputs(help_str, stdout);
		return true;
	}
	if (argFlag(cmd, "mon")) {
		btn_mon = !btn_mon;
		ButtonsChanged(); // drop anything old
		printf("Button monitor %s\n", btn_mon ? "on" : "off");
		return true;
	}

	mask = argChannels(cmd, BUTTONS);
	if (mask == 0) {
		printf("Bad channel list\n");
		fputs(help_str, stdout);
		return false;
	}

	if (argFlag(cmd, "-s")) {
		if (!streamPeriod(cmd, &period)) return false;
		// header, then one line of 0/1 per period, raw pin levels
		first = true;
		for (i = 0; i < BUTTONS; i++) {
			if ((mask >> i) & 1) {
				printf("%sBTN%d", first ? "" : ", ", i + 1);
				first = false;
			}
		}
		printf("\n");
		while (1) {
			t = Millis();
			raw = ButtonsRaw();
			first = true;
			for (i = 0; i < BUTTONS; i++) {
				if ((mask >> i) & 1) {
					printf("%s%d", first ? "" : ", ", (raw >> i) & 1);
					first = false;
				}
			}
			printf("\n");
			if (streamWait(t, period)) return true;
		}
	}

	for (i = 0; i < BUTTONS; i++) {
		if (!((mask >> i) & 1)) continue;
		printf("BTN%d: %d", i + 1, (state >> i) & 1);
		if (((raw ^ state) >> i) & 1) printf(" (settling)");
		printf(", %" PRIu32 " edges\n", ButtonsEdges(i));
	}
	return true;
}

uint8_t diags_press(char *cmd) {
	uint8_t i;
	uint32_t mask, period, t;
	bool first, was_running = AdcRunning();

	const char help_str[] = "\
press [ch] : pressure sensors, ch = 0-1 for PRESS0-1, e.g. 0 or 0-1. Default all\n\
-s : stream pressure in kPa until 'x' is entered\n\
-f [x] : streaming frequency in Hz [1, 10000], default 10\n\
start / stop : leave the ADC converting continuously, or stop it\n";

	if (argFlag(cmd, "-h") || argFlag(cmd, "--help")) {
		fputs(help_str, stdout);
		return true;
	}
	if (argFlag(cmd, "start")) {
		AdcStart();
		printf("ADC running\n");
		return true;
	}
	if (argFlag(cmd, "stop")) {
		AdcStop();
		printf("ADC stopped\n");
		return true;
	}

	mask = argChannels(cmd, ADC_CHANNELS);
	if (mask == 0) {
		printf("Bad channel list\n");
		fputs(help_str, stdout);
		return false;
	}

	// Run the ADC just for this command if nobody else has it on, let a few sequences land
	if (!was_running) {
		AdcStart();
		t = Millis();
		while (Millis() - t < 10) UsbTask();
	}

	if (argFlag(cmd, "-s")) {
		if (!streamPeriod(cmd, &period)) {
			if (!was_running) AdcStop();
			return false;
		}
		first = true;
		for (i = 0; i < ADC_CHANNELS; i++) {
			if ((mask >> i) & 1) {
				printf("%sPRESS%d", first ? "" : ", ", i);
				first = false;
			}
		}
		printf("\n");
		while (1) {
			t = Millis();
			first = true;
			for (i = 0; i < ADC_CHANNELS; i++) {
				if ((mask >> i) & 1) {
					printf("%s%.2f", first ? "" : ", ", (double)PressKpa(i));
					first = false;
				}
			}
			printf("\n");
			if (streamWait(t, period)) break;
		}
	} else {
		for (i = 0; i < ADC_CHANNELS; i++) {
			if ((mask >> i) & 1) {
				printf("PRESS%d: %.2f kPa, %.4f V, raw %" PRIu32 "\n", i, (double)PressKpa(i), (double)AdcVolts(i), adc_raw[i]);
			}
		}
	}

	if (!was_running) AdcStop();
	return true;
}

void printIp(uint32_t ip) {
	// lwIP keeps addresses in network order, first octet in the low byte here
	printf("%" PRIu32 ".%" PRIu32 ".%" PRIu32 ".%" PRIu32, ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, ip >> 24);
}

// Raw frames through the PHY's digital loopback: MAC, DMA, RMII and PHY, no cable needed
uint32_t ethLoopTest(uint32_t n) {
	uint8_t mac[6], *tx, *rx;
	uint32_t i, j, len, rlen, t, bad = 0, missing = 0, first_missing = 0;

	NetGetMac(mac);
	EthPhyLoopback(true);
	t = Millis();
	while (Millis() - t < 50);

	// One warm-up frame, not counted, then flush
	if ((tx = EthTxBuffer()) != NULL) {
		memset(tx, 0, 60);
		memcpy(tx, mac, 6);
		memcpy(tx + 6, mac, 6);
		tx[12] = 0x88;
		tx[13] = 0xB5;
		EthTxSend(60);
	}
	t = Millis();
	while (Millis() - t < 10);
	while ((rx = EthRxPeek(&rlen)) != NULL) EthRxRelease();

	for (i = 0; i < n; i++) {
		len = 60 + (i * 37) % 1455; // 60..1514 bytes
		t = Millis();
		while ((tx = EthTxBuffer()) == NULL) {
			if (Millis() - t > 20) break;
		}
		if (tx == NULL) {
			missing++;
			continue;
		}
		memcpy(tx, mac, 6);			// to us
		memcpy(tx + 6, mac, 6);		// from us
		tx[12] = 0x88;				// local experimental ethertype
		tx[13] = 0xB5;
		for (j = 14; j < len; j++) tx[j] = (uint8_t)(i + j * 7);
		EthTxSend(len);

		t = Millis();
		while ((rx = EthRxPeek(&rlen)) == NULL) {
			if (Millis() - t > 20) break;
		}
		if (rx == NULL) {
			if (missing++ == 0) first_missing = i;
			if (missing > 5) break;
			continue;
		}
		if (rlen != len) bad++;
		else {
			for (j = 14; j < len; j++) {
				if (rx[j] != (uint8_t)(i + j * 7)) {
					bad++;
					break;
				}
			}
		}
		EthRxRelease();
	}
	EthPhyLoopback(false);
	printf("PHY loopback: %" PRIu32 " frames, %" PRIu32 " missing, %" PRIu32 " wrong\n", i, missing, bad);
	if (missing) printf("first missing: frame %" PRIu32 "\n", first_missing);
	return bad + missing + (n - i);
}

uint8_t diags_eth(char *cmd) {
	uint8_t mac[6], i;
	uint16_t v;
	uint32_t period, t, reg, val, n;
	EthLink l;
	EthStats s;
	char *p, ip[16], mask[16];

	const char help_str[] = "\
eth : status\n\
-s : stream link state and frame counts until 'x' is entered\n\
-f [x] : streaming frequency in Hz [1, 10000], default 10\n\
phy : dump PHY registers 0-31\n\
phy <reg> [val] : read / write a PHY register (hex)\n\
ip <a.b.c.d> [mask] : change the address (until reset)\n\
loop [n] : n raw frames through the PHY loopback, no cable needed\n";

	p = cmd + 3;
	while (*p == ' ') p++;

	if (argFlag(cmd, "-h") || argFlag(cmd, "--help")) {
		fputs(help_str, stdout);
		return true;
	}
	if (!NetUp()) {
		const char *why[] = { "ok", "not started", "PHY answers on MDIO but there's no 50MHz REF_CLK on PA1",
			"nothing answers on MDIO", "no PHY on MDIO and no 50MHz REF_CLK" };
		printf("Ethernet didn't start: %s\n", why[EthStatus() < 5 ? EthStatus() : 1]);
		if (EthPhyAddr() < 0 || strncmp(p, "phy", 3) != 0) {
			if (EthPhyAddr() >= 0) printf("PHY: addr %d, ID %08" PRIX32 " (eth phy works)\n", EthPhyAddr(), EthPhyId());
			return false;
		}
	}

	if (strncmp(p, "phy", 3) == 0) {
		p += 3;
		while (*p == ' ') p++;
		if (*p == '\0') {
			for (i = 0; i < 32; i++) {
				if (EthMdioRead(i, &v)) printf("%02X: %04X%s", i, v, (i % 8) == 7 ? "\n" : "  ");
				else printf("%02X: ----%s", i, (i % 8) == 7 ? "\n" : "  ");
			}
			return true;
		}
		reg = strtoul(p, &p, 16);
		while (*p == ' ') p++;
		if (reg > 31) {
			fputs(help_str, stdout);
			return false;
		}
		if (*p != '\0') {
			val = strtoul(p, NULL, 16);
			printf(EthMdioWrite(reg, val) ? "Wrote %02" PRIX32 " = %04" PRIX32 "\n" : "MDIO write failed\n", reg, val);
		}
		if (EthMdioRead(reg, &v)) printf("%02" PRIX32 ": %04X\n", reg, v);
		else printf("MDIO read failed\n");
		return true;
	}

	if (strncmp(p, "ip", 2) == 0) {
		p += 2;
		if (sscanf(p, " %15s %15s", ip, mask) < 1 || !NetSetIp(ip, sscanf(p, " %15s %15s", ip, mask) == 2 ? mask : NULL)) {
			fputs(help_str, stdout);
			return false;
		}
		printf("IP ");
		printIp(NetIp());
		printf(" / ");
		printIp(NetMask());
		printf("\n");
		return true;
	}

	if (strncmp(p, "loop", 4) == 0) {
		n = strtoul(p + 4, NULL, 10);
		if (n == 0) n = 1000;
		printf("%s\n", ethLoopTest(n) ? "FAIL" : "PASS");
		return true;
	}

	if (argFlag(cmd, "-s")) {
		if (!streamPeriod(cmd, &period)) return false;
		printf("link, speed, full duplex, rx, tx, rx errors\n");
		while (1) {
			t = Millis();
			l = NetLink();
			EthGetStats(&s);
			printf("%d, %d, %d, %" PRIu32 ", %" PRIu32 ", %" PRIu32 "\n", l.up, l.speed, l.full_duplex, s.rx_frames, s.tx_frames, s.rx_errors);
			if (streamWait(t, period)) return true;
			NetTask();
		}
	}

	l = NetLink();
	EthGetStats(&s);
	NetGetMac(mac);
	printf("PHY: addr %d, ID %08" PRIX32 "\n", EthPhyAddr(), EthPhyId());
	printf("Link: %d", l.up);
	if (l.up) printf(", %d Mbit/s %s duplex", l.speed, l.full_duplex ? "full" : "half");
	printf("\nMAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	printf("IP: ");
	printIp(NetIp());
	printf(" / ");
	printIp(NetMask());
	printf(", discovery on UDP %d\n", NET_DISCOVERY_PORT);
	printf("Frames: rx %" PRIu32 ", tx %" PRIu32 ", rx errors %" PRIu32 ", rx overruns %" PRIu32 ", tx busy %" PRIu32 "\n",
		s.rx_frames, s.tx_frames, s.rx_errors, s.rx_overruns, s.tx_busy);
	return true;
}

uint8_t diags_boot(char *cmd) {
	uint32_t t;
	(void)cmd;

	StepAbort(); // drivers off, step DMA stopped
	PwmAllOff();
	printf("Rebooting to USB bootloader\n");
	t = Millis();
	while (Millis() - t < 50) UsbTask(); // let the message go out
	UsbDetach(); // so the PC sees the COM port go away cleanly
	t = Millis();
	while (Millis() - t < 20);
	BootToLoader();
	return true;
}

// ---- ESP32 ----

static char esp_line[64];
static uint8_t esp_line_len = 0;

// Lines from the ESP32 app, "!boot" = reboot to the bootloader (flash over WiFi). Rest is log
void DiagsEspTask() {
	int c;

	while ((c = EspCtlReadChar()) >= 0) {
		if (c != '\r' && c != '\n') {
			if (esp_line_len < sizeof(esp_line) - 1) esp_line[esp_line_len++] = c;
			continue;
		}
		esp_line[esp_line_len] = '\0';
		esp_line_len = 0;
		if (strcmp(esp_line, "!boot") == 0) {
			if (!MotionIdle()) {
				EspWrite("!busy\n", 6);
				continue;
			}
			EspWrite("!ok\n", 4);
			while (!EspTxIdle());
			diags_boot("");
		}
	}
}

// EN / BOOT pin levels, false if the IO expander doesn't answer
bool espLevels(uint8_t *en, uint8_t *boot) {
	uint16_t lv;
	if (!IoExpRead(&lv)) {
		printf("IO expander not answering at 0x%02X\n", IOEXP_ADDR);
		return false;
	}
	*en = (lv >> IOEXP_ESP_EN) & 1;
	*boot = (lv >> IOEXP_ESP_BOOT) & 1;
	return true;
}

// Command to the ESP32 app, print the reply up to its OK
bool espQuery(const char *q, uint32_t timeout_ms) {
	char line[128];
	uint8_t len = 0;
	uint32_t t = Millis();
	int c;

	EspCtlFlush();
	EspWrite(q, strlen(q));
	EspWrite("\n", 1);
	while (Millis() - t < timeout_ms) {
		UsbTask();
		while ((c = EspCtlReadChar()) >= 0) {
			if (c == '\r') continue;
			if (c != '\n') {
				if (len < sizeof(line) - 1) line[len++] = c;
				continue;
			}
			line[len] = '\0';
			len = 0;
			if (strcmp(line, "OK") == 0) return true;
			printf("%s\n", line);
			if (strncmp(line, "ERR", 3) == 0) return false;
		}
	}
	printf("No answer from the ESP32 (no app, or it's in the bootloader?)\n");
	return false;
}

uint8_t diags_esp(char *cmd) {
	EspStats st;
	uint32_t period, t, v;
	uint8_t en, boot;
	char *p;

	const char help_str[] = "\
esp : EN / BOOT pin levels, UART6 baud and byte counts, WiFi console client\n\
-s : stream EN, BOOT until 'x' is entered\n\
-f [x] : streaming frequency in Hz [1, 10000], default 10\n\
esp reset : reset, runs the app\n\
esp boot : reset into the ROM bootloader (BOOT low through the reset)\n\
esp en <0|1> : 0 = hold EN low (in reset), 1 = let go\n\
esp io9 <0|1> : BOOT pin (IO9), 0 = pull low, 1 = let go\n\
esp baud <n> : UART6 baud. In the bridge the USB port sets it\n\
esp bridge : this port <-> ESP32 UART0. 1s quiet, +++, 1s quiet to get back\n\
esp scan : WiFi networks the ESP32 can see\n\
esp wifi : WiFi state, IP, RSSI, last disconnect reason\n\
esp wifi -s : stream connected, RSSI until 'x' is entered (-f as above)\n\
esp pad : Xbox controller (BLE on the ESP32): connected, paired address, battery, state\n\
esp pad pair : forget it, pair the next controller in pairing mode (hold its pair button)\n\
esp pad log : BLE events + the paired controller's advertisements since the last log\n\
(live state on the H7 side: pad -s)\n";

	if (argFlag(cmd, "-h") || argFlag(cmd, "--help")) {
		fputs(help_str, stdout);
		return true;
	}

	p = cmd + 3;
	while (*p == ' ') p++;

	if (*p == '\0' || *p == '-') {
		if (argFlag(cmd, "-s")) {
			if (!streamPeriod(cmd, &period)) return false;
			printf("EN, BOOT\n");
			while (1) {
				t = Millis();
				if (!espLevels(&en, &boot)) return false;
				printf("%d, %d\n", en, boot);
				if (streamWait(t, period)) return true;
			}
		}
		if (espLevels(&en, &boot)) {
			printf("EN: %d%s\n", en, IoExpIsLow(IOEXP_ESP_EN) ? " (held low)" : "");
			printf("BOOT: %d%s\n", boot, IoExpIsLow(IOEXP_ESP_BOOT) ? " (held low)" : "");
		}
		EspGetStats(&st);
		printf("UART6: %" PRIu32 " baud, rx %" PRIu32 ", tx %" PRIu32 ", overrun %" PRIu32 ", framing %" PRIu32 ", dropped %" PRIu32 "\n",
			EspGetBaud(), st.rx, st.tx, st.overrun, st.frame, st.rx_dropped);
		printf("WiFi console: %s\n", EspConConnected() ? "client connected" : "no client");
	} else if (strncmp(p, "reset", 5) == 0) {
		if (!EspReset(false)) {
			printf("IO expander not answering at 0x%02X\n", IOEXP_ADDR);
			return false;
		}
		printf("ESP32 reset\n");
	} else if (strncmp(p, "boot", 4) == 0) {
		EspFlushRx();
		if (!EspReset(true)) {
			printf("IO expander not answering at 0x%02X\n", IOEXP_ADDR);
			return false;
		}
		printf("ESP32 in download mode\n");
	} else if (strncmp(p, "en", 2) == 0 || strncmp(p, "io9", 3) == 0) {
		uint8_t pin = p[0] == 'e' ? IOEXP_ESP_EN : IOEXP_ESP_BOOT;
		p += pin == IOEXP_ESP_EN ? 2 : 3;
		while (*p == ' ') p++;
		if (*p != '0' && *p != '1') {
			fputs(help_str, stdout);
			return false;
		}
		if (!IoExpLow(pin, *p == '0')) {
			printf("IO expander not answering at 0x%02X\n", IOEXP_ADDR);
			return false;
		}
		printf("%s %s\n", pin == IOEXP_ESP_EN ? "EN" : "BOOT", *p == '0' ? "held low" : "let go");
	} else if (strncmp(p, "baud", 4) == 0) {
		v = strtoul(p + 4, NULL, 10);
		if (v == 0) {
			fputs(help_str, stdout);
			return false;
		}
		EspBaud(v);
		printf("UART6 at %" PRIu32 " baud\n", EspGetBaud());
	} else if (strncmp(p, "scan", 4) == 0) {
		printf("Scanning...\n");
		return espQuery("scan", 10000);
	} else if (strncmp(p, "wifi", 4) == 0) {
		if (argFlag(cmd, "-s")) {
			if (!streamPeriod(cmd, &period)) return false;
			printf("connected, RSSI\n");
			while (1) {
				t = Millis();
				if (!espQuery("link", 500)) return false;
				if (streamWait(t, period)) return true;
			}
		}
		return espQuery("wifi", 1000);
	} else if (strncmp(p, "pad", 3) == 0) {
		p += 3;
		while (*p == ' ') p++;
		if (strncmp(p, "pair", 4) == 0) return espQuery("pad pair", 1000);
		if (strncmp(p, "log", 3) == 0) return espQuery("pad log", 1000);
		return espQuery("pad", 1000);
	} else if (strncmp(p, "bridge", 6) == 0) {
		if (!ConsoleBridge()) return false;
	} else {
		fputs(help_str, stdout);
		return false;
	}
	return true;
}

// ---- Xbox controller ----

static const char *pad_names[PAD_BUTTONS] = {
	"A", "B", "X", "Y", "LB", "RB", "LS", "RS", "up", "down", "left", "right", "view", "menu", "share", "xbox",
};

static void padPrint(const EspPad *p, bool up) {
	uint8_t i;
	bool any = false;

	printf("%d, %u, %u, %u, %u, %u, %u, ", up, p->lx, p->ly, p->rx, p->ry, p->lt, p->rt);
	for (i = 0; i < PAD_BUTTONS; i++) {
		if (!((p->buttons >> i) & 1)) continue;
		printf("%s%s", any ? " " : "", pad_names[i]);
		any = true;
	}
	printf("%s\n", any ? "" : "-");
}

// pad mode [name]
static bool padMode(char *p) {
	uint8_t m;

	while (*p == ' ') p++;
	if (*p == '\0') {
		printf("Mode: %s\n", PadModeName(PadMode()));
		return true;
	}
	for (m = 0; m < PAD_MODES; m++) {
		if (strcmp(p, PadModeName(m)) == 0) break;
	}
	if (m == PAD_MODES) {
		printf("Modes: off, jog\n");
		return false;
	}
	if (!PadSetMode(m)) return false;
	if (m == PAD_MODE_JOG) {
		printf("Jog: left stick X / Y %.1f mm/s, RT / LT Z up / down %.1f mm/s%s\n",
			(double)pad_speed_xy, (double)pad_speed_z, StepDry() ? ", dry run" : "");
		printf("Centre the sticks and let go of the triggers to start. pad mode off to stop\n");
	} else {
		printf("Mode: off\n");
	}
	return true;
}

// pad speed [xy <mm/s>] [z <mm/s>]
static bool padSpeed(char *cmd) {
	float v;

	if (argFloat(cmd, "xy", &v)) {
		if (v <= 0 || v > 1000) {
			printf("xy speed must be 0 - 1000 mm/s\n");
			return false;
		}
		pad_speed_xy = v;
	}
	if (argFloat(cmd, "z", &v)) {
		if (v <= 0 || v > 1000) {
			printf("z speed must be 0 - 1000 mm/s\n");
			return false;
		}
		pad_speed_z = v;
	}
	printf("Full stick: XY %.1f mm/s, Z %.1f mm/s (still capped at each axis' max feed, M203)\n",
		(double)pad_speed_xy, (double)pad_speed_z);
	return true;
}

uint8_t diags_pad(char *cmd) {
	EspPad p;
	EspPadStats st;
	uint32_t period, t;
	bool up;
	char *a;

	const char help_str[] = "\
pad : Xbox controller state from the ESP32 pad packets, packet counts\n\
-s : stream connected, sticks (0-65535), triggers (0-1023), buttons until 'x' is entered\n\
-f [x] : streaming frequency in Hz [1, 10000], default 10\n\
pad mode : what the controller does: off (default at boot) or jog\n\
pad mode jog : left stick = X / Y, RT / LT = Z up / down. Centre the sticks to start\n\
pad mode off : stop (ramps down), axes back to G-code\n\
pad speed [xy <mm/s>] [z <mm/s>] : speed at full stick / trigger\n\
Nothing for 150ms = treated as gone (sticks centred, all let go). Pairing: esp pad pair\n";

	if (argFlag(cmd, "-h") || argFlag(cmd, "--help")) {
		fputs(help_str, stdout);
		return true;
	}

	a = cmd + 3;
	while (*a == ' ') a++;
	if (strncmp(a, "mode", 4) == 0 && (a[4] == ' ' || a[4] == '\0')) return padMode(a + 4);
	if (strncmp(a, "speed", 5) == 0 && (a[5] == ' ' || a[5] == '\0')) return padSpeed(a);

	if (argFlag(cmd, "-s")) {
		if (!streamPeriod(cmd, &period)) return false;
		printf("connected, LX, LY, RX, RY, LT, RT, buttons\n");
		while (1) {
			t = Millis();
			up = EspPadGet(&p);
			padPrint(&p, up);
			if (streamWait(t, period)) return true;
		}
	}

	up = EspPadGet(&p);
	EspPadGetStats(&st);
	printf("Pad: %s, battery %u%%\n", up ? "connected" : "not connected", p.battery);
	printf("Mode: %s", PadModeName(PadMode()));
	if (PadMode() == PAD_MODE_JOG) printf(", %s", PadArmed() ? "moving on input" : "waiting for centred sticks");
	else if (JogFaulted()) printf(" (last jog stopped by a fault)");
	if (JogLate()) printf(", queue ran dry %" PRIu32 " times", JogLate());
	printf("\n");
	printf("Packets: %" PRIu32 " ok, %" PRIu32 " bad, %" PRIu32 " missed, ", st.good, st.bad, st.missed);
	if (st.age_ms == UINT32_MAX) printf("none yet\n");
	else printf("last %" PRIu32 " ms ago\n", st.age_ms);
	printf("connected, LX, LY, RX, RY, LT, RT, buttons\n");
	padPrint(&p, up);
	return true;
}

// ---- TMC2209 drivers ----

static const uint32_t dir_pins[STEP_AXES] = {
	PIN_DIR1_MASK, PIN_DIR2_MASK, PIN_DIR3_MASK, PIN_DIR4_MASK,
	PIN_DIR5_MASK, PIN_DIR6_MASK, PIN_DIR7_MASK, PIN_DIR8_MASK,
};
static const uint32_t step_pins[STEP_AXES] = {
	PIN_STEP1_MASK, PIN_STEP2_MASK, PIN_STEP3_MASK, PIN_STEP4_MASK,
	PIN_STEP5_MASK, PIN_STEP6_MASK, PIN_STEP7_MASK, PIN_STEP8_MASK,
};

// Driver list word: "all", or 0-based like "3", "0-2", "1,5,7". 0 if bad
uint8_t tmcList(char *w) {
	if (w == NULL) return 0;
	if (strcmp(w, "all") == 0) return (1 << TMC_DRIVERS) - 1;
	if (*w < '0' || *w > '9') return 0;
	return argChannels(w, TMC_DRIVERS);
}

// Split a copy of cmd into words
uint8_t tmcWords(char *cmd, char *buf, uint32_t len, char **w, uint8_t max) {
	uint8_t n = 0, i;
	char *p;

	strncpy(buf, cmd, len - 1);
	buf[len - 1] = '\0';
	for (p = strtok(buf, " "); p && n < max; p = strtok(NULL, " ")) w[n++] = p;
	for (i = n; i < max; i++) w[i] = NULL;
	return n;
}

// Which driver each DIR line goes to: flip one DIR pin at a time and see whose IOIN DIR bit
// follows. No STEP pulses, so nothing moves. The pin goes back to where it was.
// Flip one pin, see which driver's IOIN bit follows, put the pin back
void tmcMapPin(const char *name, uint8_t i, uint32_t pin, uint32_t bit, const uint32_t *base, const uint8_t *err) {
	uint32_t was = GPIOE->ODR & pin, now;
	uint8_t d, hits = 0;

	GPIOE->BSRR = was ? (pin << 16) : pin;
	printf("%s%d (%s): ", name, i + 1, MotionDriverName(i));
	for (d = 0; d < TMC_DRIVERS; d++) {
		if (err[d] != TMC_OK || TmcRead(d, TMC_IOIN, &now) != TMC_OK) continue;
		if ((base[d] ^ now) & bit) {
			printf("%sdriver %d (bank %d node %d)", hits ? ", " : "", d, d / TMC_NODES + 1, d % TMC_NODES);
			hits++;
		}
	}
	GPIOE->BSRR = was ? pin : (pin << 16);
	printf("%s\n", hits ? "" : "no driver saw it");
}

// Which driver each DIR and STEP line goes to, from the pin levels the drivers read back (IOIN).
// Drivers off: a STEP pin flip is one pulse into a disabled driver, nothing moves
void tmcMap() {
	uint32_t base[TMC_DRIVERS];
	uint8_t err[TMC_DRIVERS], i, d;
	bool was_on = BoardStepperEnabled();

	BoardStepperEnable(false);
	for (d = 0; d < TMC_DRIVERS; d++) err[d] = TmcRead(d, TMC_IOIN, &base[d]);
	for (i = 0; i < STEP_AXES; i++) tmcMapPin("DIR", i, dir_pins[i], TMC_IOIN_DIR, base, err);
	for (i = 0; i < STEP_AXES; i++) tmcMapPin("STEP", i, step_pins[i], TMC_IOIN_STEP, base, err);
	if (was_on) printf("Drivers were on, now off (step en)\n");
}

void tmcFaultRow(uint8_t d) {
	uint32_t g, s;
	uint8_t e;

	printf("%d, %s, %d, ", d, MotionDriverName(d), (TmcDiagPins() >> d) & 1);
	e = TmcRead(d, TMC_GSTAT, &g);
	if (e == TMC_OK) e = TmcRead(d, TMC_DRV_STATUS, &s);
	if (e != TMC_OK) {
		printf("%s\n", TmcError(e));
		return;
	}
	printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
		(g & TMC_GSTAT_RESET) != 0, (g & TMC_GSTAT_DRV_ERR) != 0, (g & TMC_GSTAT_UV_CP) != 0,
		(s & TMC_DRV_OT) != 0, (s & TMC_DRV_OTPW) != 0, (s & TMC_DRV_S2GA) != 0, (s & TMC_DRV_S2GB) != 0,
		(s & TMC_DRV_S2VSA) != 0, (s & TMC_DRV_S2VSB) != 0, (s & TMC_DRV_OLA) != 0, (s & TMC_DRV_OLB) != 0,
		(s & TMC_DRV_T120) != 0, (s & TMC_DRV_T143) != 0, (s & TMC_DRV_T150) != 0, (s & TMC_DRV_T157) != 0);
}

const char tmc_fault_header[] = "DRV, AXIS, DIAG, RESET, DRV_ERR, UV_CP, OT, OTPW, S2GA, S2GB, S2VSA, S2VSB, OLA, OLB, T120, T143, T150, T157\n";

uint8_t tmcFaults(char *cmd, uint8_t mask) {
	uint8_t d, e;
	uint32_t g, s, period, t;
	char txt[160];
	bool any = false;

	if (argFlag(cmd, "-s")) {
		if (!streamPeriod(cmd, &period)) return false;
		fputs(tmc_fault_header, stdout);
		while (1) {
			t = Millis();
			for (d = 0; d < TMC_DRIVERS; d++) {
				if ((mask >> d) & 1) tmcFaultRow(d);
			}
			if (streamWait(t, period)) return true;
		}
	}

	fputs(tmc_fault_header, stdout);
	for (d = 0; d < TMC_DRIVERS; d++) {
		if ((mask >> d) & 1) tmcFaultRow(d);
	}
	// Plain words for anything set (GSTAT reset alone isn't a fault, just "powered up since")
	for (d = 0; d < TMC_DRIVERS; d++) {
		if (!((mask >> d) & 1)) continue;
		e = TmcRead(d, TMC_GSTAT, &g);
		if (e == TMC_OK) e = TmcRead(d, TMC_DRV_STATUS, &s);
		if (e != TMC_OK) continue;
		if (!((g & (TMC_GSTAT_DRV_ERR | TMC_GSTAT_UV_CP)) || (s & 0xFF) || ((TmcDiagPins() >> d) & 1))) continue;
		TmcFaultText(g, s, txt, sizeof(txt));
		printf("Faults: %d (%s) %s%s\n", d, MotionDriverName(d), txt, (TmcIgnored() >> d) & 1 ? " [ignored]" : "");
		any = true;
	}
	if (!any) printf("No faults\n");
	return true;
}

void tmcConfigRows(uint8_t mask) {
	uint8_t d, e, irun, ihold;
	uint32_t gconf, chop;
	TmcConfig *c;
	bool vs;
	float run_a, hold_a;

	printf("DRV, AXIS, RUN SET, RUN A, IRUN, HOLD SET, HOLD A, IHOLD, VSENSE, USTEP, INTERP, MODE, DRIVER\n");
	for (d = 0; d < TMC_DRIVERS; d++) {
		if (!((mask >> d) & 1)) continue;
		c = &tmc_config[d];
		vs = TmcVsenseFor(c->run);
		run_a = TmcCurrent(c->run, vs, &irun);
		hold_a = TmcCurrent(c->hold, vs, &ihold);
		printf("%d, %s, %.2f, %.2f, %d, %.2f, %.2f, %d, ", d, MotionDriverName(d),
			(double)c->run, (double)run_a, irun, (double)c->hold, (double)hold_a, ihold);

		// The rest is read back from the driver (IHOLD_IRUN is write only)
		e = TmcRead(d, TMC_GCONF, &gconf);
		if (e == TMC_OK) e = TmcRead(d, TMC_CHOPCONF, &chop);
		if (e != TMC_OK) {
			printf("-, -, -, -, %s\n", TmcError(e));
			continue;
		}
		uint16_t ustep = 256 >> ((chop & TMC_CHOP_MRES) >> TMC_CHOP_MRES_POS);
		bool intpol = (chop & TMC_CHOP_INTPOL) != 0;
		bool spread = (gconf & TMC_GCONF_EN_SPREADCYCLE) != 0;
		bool same = ((chop & TMC_CHOP_VSENSE) != 0) == vs && ustep == c->ustep && intpol == c->intpol && spread == c->spread
			&& (gconf & TMC_GCONF_PDN_DISABLE) && (gconf & TMC_GCONF_MSTEP_REG_SELECT) && !(gconf & TMC_GCONF_I_SCALE_ANALOG);
		printf("%d, %d, %d, %s, %s\n", (chop & TMC_CHOP_VSENSE) != 0, ustep, intpol, spread ? "spread" : "stealth",
			same ? "ok" : "differs, run tmc setup");
	}
}

// tmc set <list> <field> <value>
uint8_t tmcSet(char **w) {
	uint8_t mask = tmcList(w[2]), d, e;
	char *field = w[3], *val = w[4];
	float f;
	uint16_t u;
	float mm;

	if (mask == 0 || field == NULL || val == NULL) {
		printf("tmc set <list|all> <run|hold|ustep|interp|mode> <value>\n");
		return false;
	}
	if (!MotionIdle()) {
		printf("Motion running, try again when idle\n");
		return false;
	}
	f = strtof(val, NULL);

	// Check first, so a bad value changes nothing
	for (d = 0; d < TMC_DRIVERS; d++) {
		if (!((mask >> d) & 1)) continue;
		if (strcmp(field, "run") == 0) {
			if (f <= 0 || f > TMC_IRMS_MAX) {
				printf("run must be > 0 and <= %.1f A (datasheet RMS limit)\n", (double)TMC_IRMS_MAX);
				return false;
			}
			if (f < tmc_config[d].hold) {
				printf("Driver %d: run %.2f A is below its hold %.2f A, lower hold first\n", d, (double)f, (double)tmc_config[d].hold);
				return false;
			}
		} else if (strcmp(field, "hold") == 0) {
			if (f < 0 || f > tmc_config[d].run) {
				printf("Driver %d: hold must be 0 - %.2f A (its run current)\n", d, (double)tmc_config[d].run);
				return false;
			}
		} else if (strcmp(field, "ustep") == 0) {
			u = (uint16_t)f;
			if (u < 1 || u > 256 || (u & (u - 1)) != 0) {
				printf("ustep must be 1, 2, 4 ... 256\n");
				return false;
			}
		} else if (strcmp(field, "interp") == 0) {
			if (strcmp(val, "0") != 0 && strcmp(val, "1") != 0) {
				printf("interp must be 0 or 1\n");
				return false;
			}
		} else if (strcmp(field, "mode") == 0) {
			if (strcmp(val, "stealth") != 0 && strcmp(val, "spread") != 0) {
				printf("mode must be stealth or spread\n");
				return false;
			}
		} else {
			printf("Unknown setting %s. run, hold, ustep, interp, mode\n", field);
			return false;
		}
	}

	// Drivers of one axis (Y1 + Y2) have to step the same distance, so they get the same microsteps
	if (strcmp(field, "ustep") == 0) {
		uint8_t all = mask;
		for (d = 0; d < TMC_DRIVERS; d++) {
			int8_t ax = MotionAxisOf(d);
			if ((mask >> d) & 1 && ax >= 0) all |= motion_axes[ax].drivers;
		}
		if (all != mask) printf("Ganged drivers get the same microsteps, setting those too\n");
		mask = all;
	}
	uint8_t scaled = 0;

	for (d = 0; d < TMC_DRIVERS; d++) {
		if (!((mask >> d) & 1)) continue;
		TmcConfig *c = &tmc_config[d];
		if (strcmp(field, "run") == 0) c->run = f;
		else if (strcmp(field, "hold") == 0) c->hold = f;
		else if (strcmp(field, "interp") == 0) c->intpol = f != 0;
		else if (strcmp(field, "mode") == 0) c->spread = strcmp(val, "spread") == 0;
		else {
			// Steps/mm follows the microsteps (once per axis), and the axis stays where it is in mm
			int8_t ax = MotionAxisOf(d);
			u = (uint16_t)f;
			if (ax >= 0 && !((scaled >> ax) & 1)) {
				mm = MotionPosition(ax);
				motion_axes[ax].steps_per_mm *= (float)u / c->ustep;
				MotionSync();
				MotionSetPosition(ax, mm);
				scaled |= 1 << ax;
				printf("Axis %c now %.3f steps/mm\n", motion_axes[ax].name, (double)motion_axes[ax].steps_per_mm);
			}
			c->ustep = u;
		}
		e = TmcSetup(d);
		if (e != TMC_OK) printf("Driver %d: %s\n", d, TmcError(e));
	}
	tmcConfigRows(mask);
	return true;
}

uint8_t diags_tmc(char *cmd) {
	char buf[96], *w[6];
	uint8_t mask, d, e, b, n, found, reg;
	uint32_t v, ifcnt, gstat;
	float f;

	const char help_str[] = "\
tmc : read every TMC2209. Driver 0-7 = DIR1-8 = axis, bank 1 = 0-3 (UART5 PC12), bank 2 = 4-7 (UART7 PB4)\n\
faults [list] : GSTAT + DRV_STATUS flags, list like 3, 0-2, 1,5,7. Default all\n\
  -s : stream them until 'x' is entered, -f [x] : frequency in Hz [1, 10000], default 10\n\
clear [list] : clear the latched GSTAT flags\n\
config [list] : settings, and what the driver really has\n\
set <list|all> run <A> : run current, A RMS (max 1.4)\n\
set <list|all> hold <A> : standstill current, A RMS\n\
set <list|all> ustep <n> : microsteps 1-256, the axis steps/mm follows\n\
set <list|all> interp <0|1> : interpolate to 256 microsteps\n\
set <list|all> mode <stealth|spread> : StealthChop or SpreadCycle\n\
setup [list] : write the settings again\n\
ignore [list|none] : drivers left out of the DIAG / ready checks (no motor fitted). No list = show\n\
map : which driver each DIR and STEP line goes to (flips the pins with the drivers off)\n\
baud [x] : show / set the UART baud rate\n\
reg <drv> <reg> [value] : read a register, or write it if value is given\n";

	tmcWords(cmd, buf, sizeof(buf), w, 6);
	// w[0] = "tmc", w[1] = sub command

	if (argFlag(cmd, "-h") || argFlag(cmd, "--help")) {
		fputs(help_str, stdout);
		return true;
	}

	if (w[1] && strcmp(w[1], "faults") == 0) {
		mask = (w[2] && w[2][0] != '-') ? tmcList(w[2]) : (1 << TMC_DRIVERS) - 1;
		if (mask == 0) {
			printf("Bad driver list\n");
			return false;
		}
		return tmcFaults(cmd, mask);
	}

	if (w[1] && strcmp(w[1], "clear") == 0) {
		mask = w[2] ? tmcList(w[2]) : (1 << TMC_DRIVERS) - 1;
		if (mask == 0) {
			printf("Bad driver list\n");
			return false;
		}
		for (d = 0; d < TMC_DRIVERS; d++) {
			if (!((mask >> d) & 1)) continue;
			e = TmcWrite(d, TMC_GSTAT, TMC_GSTAT_RESET | TMC_GSTAT_DRV_ERR | TMC_GSTAT_UV_CP);
			if (e == TMC_OK) e = TmcRead(d, TMC_GSTAT, &gstat);
			if (e == TMC_OK) printf("Driver %d: GSTAT now 0x%" PRIX32 "%s\n", d, gstat & 7, gstat & TMC_GSTAT_UV_CP ? " (uv_cp isn't latched, it's still there)" : "");
			else printf("Driver %d: %s\n", d, TmcError(e));
		}
		return true;
	}

	if (w[1] && strcmp(w[1], "config") == 0) {
		mask = w[2] ? tmcList(w[2]) : (1 << TMC_DRIVERS) - 1;
		if (mask == 0) {
			printf("Bad driver list\n");
			return false;
		}
		tmcConfigRows(mask);
		return true;
	}

	if (w[1] && strcmp(w[1], "set") == 0) return tmcSet(w);

	if (w[1] && strcmp(w[1], "setup") == 0) {
		mask = w[2] ? tmcList(w[2]) : (1 << TMC_DRIVERS) - 1;
		if (mask == 0) {
			printf("Bad driver list\n");
			return false;
		}
		for (d = 0; d < TMC_DRIVERS; d++) {
			if ((mask >> d) & 1) printf("Driver %d: %s\n", d, TmcError(TmcSetup(d)));
		}
		return true;
	}

	if (w[1] && strcmp(w[1], "ignore") == 0) {
		if (w[2]) {
			mask = strcmp(w[2], "none") == 0 ? 0 : tmcList(w[2]);
			if (mask == 0 && strcmp(w[2], "none") != 0) {
				printf("Bad driver list\n");
				return false;
			}
			TmcIgnore(mask);
		}
		mask = TmcIgnored();
		printf("Ignored drivers:");
		for (d = 0; d < TMC_DRIVERS; d++) {
			if ((mask >> d) & 1) printf(" %d (%s)", d, MotionDriverName(d));
		}
		printf("%s\n", mask ? "" : " none");
		return true;
	}

	if (w[1] && strcmp(w[1], "map") == 0) {
		if (!MotionIdle()) {
			printf("Motion running, try again when idle\n");
			return false;
		}
		tmcMap();
		return true;
	}

	if (w[1] && strcmp(w[1], "baud") == 0) {
		if (w[2]) {
			f = strtof(w[2], NULL);
			if (f < 9000 || f > 500000) {
				printf("Baud must be 9000-500000\n");
				return false;
			}
			TmcSetBaud((uint32_t)f);
		}
		printf("TMC UART %" PRIu32 " baud\n", TmcBaud());
		return true;
	}

	if (w[1] && strcmp(w[1], "reg") == 0) {
		if (w[2] == NULL || w[3] == NULL) {
			fputs(help_str, stdout);
			return false;
		}
		d = strtoul(w[2], NULL, 0);
		reg = strtoul(w[3], NULL, 0);
		if (d >= TMC_DRIVERS || reg > 0x7F) {
			fputs(help_str, stdout);
			return false;
		}
		if (w[4]) {
			v = strtoul(w[4], NULL, 0);
			e = TmcWrite(d, reg, v);
			printf("Write 0x%02X = 0x%08" PRIX32 ": %s\n", reg, v, TmcError(e));
		} else {
			e = TmcRead(d, reg, &v);
			if (e == TMC_OK) printf("0x%02X = 0x%08" PRIX32 "\n", reg, v);
			else printf("0x%02X: %s\n", reg, TmcError(e));
		}
		return e == TMC_OK;
	}

	if (w[1]) {
		printf("Unknown: %s\n", w[1]);
		fputs(help_str, stdout);
		return false;
	}

	// Plain tmc: who answers, pin levels
	printf("%" PRIu32 " baud, firmware has the drivers %s\n", TmcBaud(), BoardStepperEnabled() ? "enabled" : "disabled");
	printf("DRV, AXIS, BANK, NODE, RESULT, VERSION, IFCNT, GSTAT, ENN, MS1, MS2, DIAG, PDN_UART, STEP, DIR, SPREAD\n");
	found = 0;
	for (d = 0; d < TMC_DRIVERS; d++) {
		b = d / TMC_NODES + 1;
		n = d % TMC_NODES;
		printf("%d, %s, %d, %d, ", d, MotionDriverName(d), b, n);
		e = TmcRead(d, TMC_IOIN, &v);
		if (e == TMC_OK) e = TmcRead(d, TMC_IFCNT, &ifcnt);
		if (e == TMC_OK) e = TmcRead(d, TMC_GSTAT, &gstat);
		if (e != TMC_OK) {
			printf("%s\n", TmcError(e));
			continue;
		}
		found++;
		printf("ok, 0x%02" PRIX32 ", %" PRIu32 ", 0x%" PRIX32 ", %d, %d, %d, %d, %d, %d, %d, %d\n",
			TMC_VERSION(v), ifcnt & 0xFF, gstat & 0x7,
			(v & TMC_IOIN_ENN) != 0, (v & TMC_IOIN_MS1) != 0, (v & TMC_IOIN_MS2) != 0,
			(v & TMC_IOIN_DIAG) != 0, (v & TMC_IOIN_PDN_UART) != 0, (v & TMC_IOIN_STEP) != 0,
			(v & TMC_IOIN_DIR) != 0, (v & TMC_IOIN_SPREAD) != 0);
	}
	printf("%d of %d drivers answered\n", found, TMC_DRIVERS);
	return found == TMC_DRIVERS;
}

// ---- Axes ----

uint8_t diags_axis(char *cmd) {
	char buf[64], *w[5];
	uint8_t i, d, first;
	int8_t ax;
	float mm;

	const char help_str[] = "\
axis : which drivers each axis moves, direction flips, steps/mm and limits\n\
invert <drv> <0|1> : flip driver 0-7's DIR so + goes the axis' + way (X right, Y towards you, Z up)\n";

	if (argFlag(cmd, "-h") || argFlag(cmd, "--help")) {
		fputs(help_str, stdout);
		return true;
	}
	tmcWords(cmd, buf, sizeof(buf), w, 5);

	if (w[1] && strcmp(w[1], "invert") == 0) {
		if (w[2] == NULL || w[3] == NULL || (d = strtoul(w[2], NULL, 10)) >= MOTION_DRIVERS
			|| (strcmp(w[3], "0") != 0 && strcmp(w[3], "1") != 0)) {
			fputs(help_str, stdout);
			return false;
		}
		if (!MotionIdle()) {
			printf("Motion running, try again when idle\n");
			return false;
		}
		// The axis keeps its position in mm, only the direction changes
		ax = MotionAxisOf(d);
		mm = ax >= 0 ? MotionPosition(ax) : 0;
		if (w[3][0] == '1') motion_invert |= 1 << d;
		else motion_invert &= ~(1 << d);
		MotionSync();
		if (ax >= 0) MotionSetPosition(ax, mm);
	} else if (w[1]) {
		fputs(help_str, stdout);
		return false;
	}

	printf("AXIS, DRIVERS, STEPS/MM, V MAX, A MAX, J MAX\n");
	for (i = 0; i < MOTION_AXES; i++) {
		printf("%c, ", motion_axes[i].name);
		first = 1;
		for (d = 0; d < MOTION_DRIVERS; d++) {
			if ((motion_axes[i].drivers >> d) & 1) {
				printf("%s%d", first ? "" : "+", d);
				first = 0;
			}
		}
		printf("%s, %.3f, %.1f, %.1f, %.1f\n", first ? "none" : "", (double)motion_axes[i].steps_per_mm,
			(double)motion_axes[i].v_max, (double)motion_axes[i].a_max, (double)motion_axes[i].j_max);
	}
	printf("DRV, NAME, INVERT\n");
	for (d = 0; d < MOTION_DRIVERS; d++) printf("%d, %s, %d\n", d, MotionDriverName(d), (motion_invert >> d) & 1);
	return true;
}
