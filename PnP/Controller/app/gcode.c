#include "gcode.h"
#include "adc.h"
#include "board.h"
#include "boot.h"
#include "buttons.h"
#include "clock.h"
#include "console.h"
#include "home.h"
#include "motion.h"
#include "pwm.h"
#include "settings.h"
#include "stepgen.h"
#include "tmc.h"
#include "usb.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_WORDS	24

// What a port is waiting on before it can answer "ok"
#define WAIT_NONE		0
#define WAIT_MOVE		1	// G0/G1 that didn't fit in the step queues yet
#define WAIT_IDLE		2	// M400
#define WAIT_DWELL		3	// G4: motion done, then a timed wait
#define WAIT_HOME		4	// G28: motion done, then home (home.c), axes that cannot home are zeroed
#define WAIT_DISABLE	5	// M18/M84: motion done, then drivers off

typedef struct {
	uint8_t wait;
	float target[MOTION_AXES];
	uint8_t mask;
	float feed;
	bool idle_seen;			// G4: motion finished, dwell timer running. G28: homing started
	uint32_t dwell_start, dwell_ms;
	int32_t last_line;		// Marlin style line numbers (N..)
} gcode_port;

typedef struct {
	char letter;
	bool has_value;
	float value;
} gcode_word;

static gcode_port gp[CON_PORTS];
static bool relative = false;	// G91
static float feed = 50;			// mm/s, modal, set with F (mm/min like Marlin)
static bool halted = false;		// after M112 or a step generator fault, until M999

static void Ok() {
	printf("ok\n");
}

// Split "G1 X10 Y-2.5 F3000" (or "G1X10Y-2.5F3000") into letter/value pairs
static uint8_t Words(const char *s, gcode_word *w) {
	uint8_t n = 0;
	char *end;

	while (*s && n < MAX_WORDS) {
		if (*s >= 'A' && *s <= 'Z') {
			w[n].letter = *s;
			w[n].value = strtof(s + 1, &end);
			w[n].has_value = end != s + 1;
			s = w[n].has_value ? end : s + 1;
			n++;
		} else {
			s++;
		}
	}
	return n;
}

// Axis index for a G-code letter, Marlin's I J K = A B C. -1 if not an axis
static int8_t Axis(char c) {
	uint8_t i;
	if (c == 'I') c = 'A';
	else if (c == 'J') c = 'B';
	else if (c == 'K') c = 'C';
	for (i = 0; i < MOTION_AXES; i++) {
		if (motion_axes[i].name == c) return i;
	}
	return -1;
}

static bool CheckHalt() {
	if (!halted && (!StepRunning() || StepFault() != STEP_OK)) halted = true;
	if (halted) printf("Error:Stopped, send M999 to resume\n");
	return halted;
}

// ---- running commands ----

// Drivers on, but only if every TMC2209 still has its settings and no fault (one that lost
// power is back at full current)
static bool Enable() {
	if (BoardStepperEnabled()) return true;
	if (!TmcReady()) {
		printf("Error:Drivers not ready, steppers stay off\n");
		return false;
	}
	BoardStepperEnable(true);
	return true;
}

static void TryMove(uint8_t p) {
	gcode_port *g = &gp[p];
	uint8_t res = MotionMove(g->target, g->mask, g->feed);

	if (res == MOTION_BUSY) return; // queue full, try again next time round
	g->wait = WAIT_NONE;
	if (res == MOTION_BAD) printf("Error:Bad move\n");
	else if (res == MOTION_JOG) printf("Error:Jog mode (Xbox controller) has the axes, move ignored\n");
	else if (res == MOTION_FAULT) {
		halted = true;
		printf("Error:Stopped, send M999 to resume\n");
	}
	Ok();
}

void GcodeResume(uint8_t p) {
	gcode_port *g = &gp[p];
	uint8_t i;

	switch (g->wait) {
		case WAIT_MOVE:
			TryMove(p);
			break;
		case WAIT_IDLE:
			if (MotionIdle()) {
				g->wait = WAIT_NONE;
				Ok();
			}
			break;
		case WAIT_DWELL:
			if (!g->idle_seen) {
				if (!MotionIdle()) break;
				g->idle_seen = true;
				g->dwell_start = Millis();
			}
			if (Millis() - g->dwell_start >= g->dwell_ms) {
				g->wait = WAIT_NONE;
				Ok();
			}
			break;
		case WAIT_HOME: {
			uint8_t res, hm;
			if (!g->idle_seen) {
				if (!MotionIdle() || HomeBusy()) break;
				// Axes not set up for homing (no dir / sgthrs / travel) just get zeroed, like before
				hm = g->mask & HomeReadyMask();
				for (i = 0; i < MOTION_AXES; i++) {
					if ((g->mask >> i) & 1 && !((hm >> i) & 1)) {
						MotionSetPosition(i, 0);
						if (home_axes[i].dir != 0) printf("echo:%c can't home yet (home set), position set to 0\n", motion_axes[i].name);
					}
				}
				res = HomeStart(hm);
				if (res != HOME_OK || hm == 0) {
					if (res != HOME_OK) printf("Error:G28: %s\n", HomeError(res));
					g->wait = WAIT_NONE;
					Ok();
					break;
				}
				g->idle_seen = true;
			}
			if (HomeBusy()) break;
			if (HomeResult() != HOME_OK) printf("Error:G28: %s\n", HomeError(HomeResult()));
			g->wait = WAIT_NONE;
			Ok();
			break;
		}
		case WAIT_DISABLE:
			if (MotionIdle()) {
				BoardStepperEnable(false);
				g->wait = WAIT_NONE;
				Ok();
			}
			break;
	}
}

bool GcodeBusy(uint8_t p) {
	return gp[p].wait != WAIT_NONE;
}

// ---- reports ----

static void ReportAxes(const char *code, float *v) {
	uint8_t i;
	printf("echo:  %s", code);
	for (i = 0; i < MOTION_AXES; i++) printf(" %c%.2f", motion_axes[i].name, (double)v[i]);
	printf("\n");
}

// First driver an axis moves, -1 = none
static int8_t FirstDriver(uint8_t ax) {
	uint8_t d;
	for (d = 0; d < MOTION_DRIVERS; d++) {
		if ((motion_axes[ax].drivers >> d) & 1) return d;
	}
	return -1;
}

static void ReportSettings(uint16_t which) {
	float v[MOTION_AXES];
	uint8_t i;

	if (which == 0 || which == 92) {
		for (i = 0; i < MOTION_AXES; i++) v[i] = motion_axes[i].steps_per_mm;
		ReportAxes("M92", v);
	}
	if (which == 0 || which == 203) {
		for (i = 0; i < MOTION_AXES; i++) v[i] = motion_axes[i].v_max;
		ReportAxes("M203", v);
	}
	if (which == 0 || which == 201) {
		for (i = 0; i < MOTION_AXES; i++) v[i] = motion_axes[i].a_max;
		ReportAxes("M201", v);
	}
	if (which == 0 || which == 204) printf("echo:  M204 S%.2f\n", (double)motion_accel);
	if (which == 0 || which == 205) {
		for (i = 0; i < MOTION_AXES; i++) v[i] = motion_axes[i].j_max;
		ReportAxes("M205", v);
	}
	if (which == 0 || which == 906) {
		for (i = 0; i < MOTION_AXES; i++) v[i] = FirstDriver(i) < 0 ? 0 : tmc_config[FirstDriver(i)].run * 1000;
		ReportAxes("M906", v);
	}
	if (which == 0 || which == 914) {
		for (i = 0; i < MOTION_AXES; i++) v[i] = home_axes[i].sgthrs;
		ReportAxes("M914", v);
	}
}

// M906 X<mA> ...: run current of every driver on the axis (Marlin units, RMS), or report.
// Hold stays at or under run. Written to the drivers straight away, so only while idle
static void TmcCurrentSetting(gcode_word *w, uint8_t n) {
	uint8_t k, d;
	int8_t ax;
	bool any = false;
	float a;

	for (k = 1; k < n; k++) {
		ax = Axis(w[k].letter);
		if (ax < 0 || !w[k].has_value) continue;
		a = w[k].value / 1000;
		if (a <= 0 || a > TMC_IRMS_MAX) {
			printf("Error:%c current must be > 0 and <= %.0f mA\n", w[k].letter, (double)(TMC_IRMS_MAX * 1000));
			continue;
		}
		if (!MotionIdle()) {
			printf("Error:Not while moving\n");
			return;
		}
		for (d = 0; d < MOTION_DRIVERS; d++) {
			if (!((motion_axes[ax].drivers >> d) & 1)) continue;
			tmc_config[d].run = a;
			if (tmc_config[d].hold > a) tmc_config[d].hold = a;
			if (TmcSetup(d) != TMC_OK) printf("Error:Driver %d didn't take it\n", d);
		}
		any = true;
	}
	if (!any) ReportSettings(906);
}

// M914 X<sgthrs> ...: StallGuard threshold for homing (Marlin's sensorless homing sensitivity), or report
static void StallSetting(gcode_word *w, uint8_t n) {
	uint8_t k;
	int8_t ax;
	bool any = false;

	for (k = 1; k < n; k++) {
		ax = Axis(w[k].letter);
		if (ax < 0 || !w[k].has_value) continue;
		if (w[k].value < 0 || w[k].value > 255) {
			printf("Error:%c must be 0 - 255\n", w[k].letter);
			continue;
		}
		home_axes[ax].sgthrs = (uint8_t)w[k].value;
		any = true;
	}
	if (!any) ReportSettings(914);
}

// M92 / M201 / M203 / M205: set the per axis value for every axis letter given, or report
static void AxisSetting(uint8_t code, gcode_word *w, uint8_t n) {
	uint8_t k;
	int8_t ax;
	bool any = false;

	for (k = 1; k < n; k++) {
		ax = Axis(w[k].letter);
		if (ax < 0 || !w[k].has_value) continue;
		if (w[k].value <= 0) {
			printf("Error:%c must be > 0\n", w[k].letter);
			continue;
		}
		if (code == 92) motion_axes[ax].steps_per_mm = w[k].value;
		else if (code == 203) motion_axes[ax].v_max = w[k].value;
		else if (code == 201) motion_axes[ax].a_max = w[k].value;
		else motion_axes[ax].j_max = w[k].value;
		any = true;
	}
	if (!any) ReportSettings(code);
	if (any && code == 92) MotionSync(); // positions are counted in steps, re-read them in the new units
}

static void Pressure() {
	bool was = AdcRunning();
	uint32_t t;

	if (!was) {
		AdcStart();
		t = Millis();
		while (Millis() - t < 10);
	}
	printf("PRESS0:%.2f PRESS1:%.2f\n", (double)PressKpa(0), (double)PressKpa(1));
	if (!was) AdcStop();
}

// ---- line handling ----

static bool LineNumber(uint8_t p, char *s) {
	gcode_port *g = &gp[p];
	char *star = strchr(s, '*');
	uint8_t cs = 0;
	char *c;
	int32_t n;

	if (*s != 'N') {
		if (star) *star = '\0'; // checksum without a line number: ignore it
		return true;
	}
	n = strtol(s + 1, &c, 10);
	if (star == NULL) {
		printf("Error:No Checksum with line number, Last Line: %" PRId32 "\nResend: %" PRId32 "\nok\n", g->last_line, g->last_line + 1);
		return false;
	}
	for (c = s; c < star; c++) cs ^= (uint8_t)*c;
	if (cs != (uint8_t)atoi(star + 1)) {
		printf("Error:checksum mismatch, Last Line: %" PRId32 "\nResend: %" PRId32 "\nok\n", g->last_line, g->last_line + 1);
		return false;
	}
	*star = '\0';
	if (n != g->last_line + 1 && strstr(s, "M110") == NULL) {
		printf("Error:Line Number is not Last Line Number+1, Last Line: %" PRId32 "\nResend: %" PRId32 "\nok\n", g->last_line, g->last_line + 1);
		return false;
	}
	g->last_line = n;
	// Drop the N word itself
	c = s + 1;
	while (*c == '-' || (*c >= '0' && *c <= '9')) c++;
	memmove(s, c, strlen(c) + 1);
	return true;
}

// Uppercase, comments out (";..." and "(...)")
static void Clean(char *s) {
	char *r = s, *w = s;
	int paren = 0;
	while (*r) {
		char c = *r++;
		if (c == ';') break;
		if (c == '(') paren++;
		else if (c == ')') {
			if (paren) paren--;
		} else if (!paren) {
			*w++ = (c >= 'a' && c <= 'z') ? c - 32 : c;
		}
	}
	*w = '\0';
}

bool GcodeEmergency(const char *line) {
	char buf[24];
	strncpy(buf, line, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	Clean(buf);
	char *s = buf;
	while (*s == ' ') s++;
	if (*s == 'N') {
		s++;
		while (*s == '-' || (*s >= '0' && *s <= '9')) s++;
		while (*s == ' ') s++;
	}
	return strncmp(s, "M112", 4) == 0 && !(s[4] >= '0' && s[4] <= '9');
}

void GcodeLine(uint8_t p, char *line) {
	gcode_port *g = &gp[p];
	gcode_word w[MAX_WORDS];
	char buf[128];
	uint8_t n, k, i, res;
	int8_t ax;
	int code;

	strncpy(buf, line, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	Clean(buf);
	{
		char *s = buf;
		while (*s == ' ') s++;
		memmove(buf, s, strlen(s) + 1);
	}
	if (!LineNumber(p, buf)) return;

	n = Words(buf, w);
	if (n == 0) {
		Ok(); // empty line or just a comment
		return;
	}
	code = (int)w[0].value;

	if (w[0].letter == 'G') {
		switch (code) {
			case 0:
			case 1:
				if (CheckHalt()) break;
				for (i = 0; i < MOTION_AXES; i++) g->target[i] = MotionPosition(i);
				g->mask = 0;
				for (k = 1; k < n; k++) {
					if (w[k].letter == 'F' && w[k].has_value && w[k].value > 0) feed = w[k].value / 60; // mm/min -> mm/s
					ax = Axis(w[k].letter);
					if (ax < 0 || !w[k].has_value) continue;
					g->target[ax] = relative ? g->target[ax] + w[k].value : w[k].value;
					g->mask |= 1 << ax;
				}
				if (g->mask == 0) break; // just a feed change
				if (!Enable()) break; // Marlin enables the drivers on a move
				g->feed = feed;
				g->wait = WAIT_MOVE;
				TryMove(p);
				return;
			case 4:
				g->dwell_ms = 0;
				for (k = 1; k < n; k++) {
					if (w[k].letter == 'P') g->dwell_ms = (uint32_t)w[k].value;
					if (w[k].letter == 'S') g->dwell_ms = (uint32_t)(w[k].value * 1000);
				}
				g->idle_seen = false;
				g->wait = WAIT_DWELL;
				GcodeResume(p);
				return;
			case 20:
				printf("Error:Inches aren't supported, use G21\n");
				break;
			case 21:
				break;
			case 28:
				if (CheckHalt()) break;
				g->mask = 0;
				for (k = 1; k < n; k++) {
					ax = Axis(w[k].letter);
					if (ax >= 0) g->mask |= 1 << ax;
				}
				if (g->mask == 0) g->mask = (1 << MOTION_AXES) - 1;
				g->idle_seen = false;
				g->wait = WAIT_HOME;
				GcodeResume(p);
				return;
			case 90:
				relative = false;
				break;
			case 91:
				relative = true;
				break;
			case 92:
				for (k = 1; k < n; k++) {
					ax = Axis(w[k].letter);
					if (ax >= 0 && w[k].has_value) MotionSetPosition(ax, w[k].value);
				}
				break;
			default:
				printf("echo:Unknown command: \"%s\"\n", line);
				break;
		}
		Ok();
		return;
	}

	if (w[0].letter == 'M') {
		switch (code) {
			case 17:
				Enable();
				break;
			case 18:
			case 84:
				g->wait = WAIT_DISABLE;
				GcodeResume(p);
				return;
			case 92:
			case 201:
			case 203:
			case 205:
				AxisSetting(code, w, n);
				break;
			case 105:
				printf("ok T:0.00 /0.00\n"); // no heaters, but hosts poll this
				return;
			case 106:
			case 107: {
				uint8_t ch = 0;
				float s = code == 106 ? 255 : 0;
				for (k = 1; k < n; k++) {
					if (w[k].letter == 'P') ch = (uint8_t)w[k].value;
					if (w[k].letter == 'S' && code == 106) s = w[k].value;
				}
				if (ch >= PWM_CHANNELS) printf("Error:P must be 0-%d\n", PWM_CHANNELS - 1);
				else PwmSet(ch, s / 255);
				break;
			}
			case 110:
				for (k = 1; k < n; k++) {
					if (w[k].letter == 'N') g->last_line = (int32_t)w[k].value;
				}
				break;
			case 112: {
				uint8_t q;
				StepAbort();
				PwmAllOff();
				halted = true;
				// Anything waiting on any port is dropped
				for (q = 0; q < CON_PORTS; q++) {
					if (q != p && gp[q].wait != WAIT_NONE) {
						uint8_t mask = ConsoleOutMask();
						gp[q].wait = WAIT_NONE;
						ConsoleOut(1 << q);
						printf("Error:Emergency stop (M112)\nok\n");
						ConsoleOut(mask);
					}
				}
				g->wait = WAIT_NONE;
				printf("Error:Emergency stop (M112), send M999 to resume\n");
				break;
			}
			case 114:
				for (i = 0; i < MOTION_AXES; i++) printf("%c:%.3f ", motion_axes[i].name, (double)MotionPosition(i));
				printf("Count");
				for (i = 0; i < MOTION_AXES; i++) printf(" %c:%" PRId32, motion_axes[i].name, (int32_t)MotionCount(i));
				printf("\n");
				break;
			case 115: {
				const uint32_t *uid = (const uint32_t *)UID_BASE;
				printf("FIRMWARE_NAME:PnP-Controller (Marlin dialect) PROTOCOL_VERSION:1.0 MACHINE_TYPE:PnP EXTRUDER_COUNT:0 AXIS_COUNT:%d UUID:%08" PRIX32 "%08" PRIX32 "%08" PRIX32 "\n",
					MOTION_AXES, uid[0], uid[1], uid[2]);
				break;
			}
			case 119: {
				uint8_t st = ButtonsState();
				printf("Reporting endstop status\n");
				for (i = 0; i < BUTTONS; i++) printf("btn%d: %d\n", i + 1, (st >> i) & 1);
				break;
			}
			case 204:
				for (k = 1; k < n; k++) {
					if ((w[k].letter == 'S' || w[k].letter == 'P' || w[k].letter == 'T') && w[k].has_value) {
						motion_accel = w[k].value;
					}
				}
				if (n == 1) ReportSettings(204);
				break;
			case 400:
				g->wait = WAIT_IDLE;
				GcodeResume(p);
				return;
			case 500:
				res = SettingsSave();
				if (res == SET_OK) printf("echo:Settings Stored\n");
				else printf("Error:Settings not saved: %s\n", SettingsError(res));
				break;
			case 501:
				res = SettingsLoad();
				if (res == SET_OK) printf("echo:Stored settings retrieved\n");
				else printf("Error:Settings not loaded: %s\n", SettingsError(res));
				break;
			case 502:
				res = SettingsDefaults();
				if (res == SET_OK) printf("echo:Hardcoded Default Settings Loaded\n");
				else printf("Error:%s\n", SettingsError(res));
				break;
			case 503:
				ReportSettings(0);
				break;
			case 800:
				Pressure();
				break;
			case 906:
				TmcCurrentSetting(w, n);
				break;
			case 914:
				StallSetting(w, n);
				break;
			case 997: { // firmware update: reboot into the USB bootloader, like diags boot
				uint32_t t;
				StepAbort();
				PwmAllOff();
				printf("echo:Rebooting to USB bootloader\n");
				t = Millis();
				while (Millis() - t < 50) UsbTask();
				UsbDetach();
				t = Millis();
				while (Millis() - t < 20);
				BootToLoader();
				break;
			}
			case 999:
				if (halted) {
					if (!TmcReady()) {
						printf("Error:Driver fault still there, not resuming\n");
						break;
					}
					StepRecover();
					MotionSync();
					halted = false;
					printf("echo:Resumed\n");
				}
				break;
			default:
				printf("echo:Unknown command: \"%s\"\n", line);
				break;
		}
		Ok();
		return;
	}

	if (w[0].letter == 'T') {
		Ok(); // one tool
		return;
	}

	printf("echo:Unknown command: \"%s\"\n", line);
	Ok();
}
