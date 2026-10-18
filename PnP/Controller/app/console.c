#include "console.h"
#include "clock.h"
#include "diags.h"
#include "esp.h"
#include "gcode.h"
#include "uart.h"
#include "usb.h"

#include <stdio.h>
#include <string.h>

#define LINE_LEN 128
#define BRIDGE_GUARD_MS 1000	// quiet around the +++
#define BRIDGE_ESC_LEN 3

typedef struct {
	int (*read)();
	uint32_t (*write)(const char *, uint32_t);
	bool (*connected)();	// NULL = always there, greeted once at boot
	char line[LINE_LEN];
	unsigned char len;
	bool held;		// a whole line is waiting (G-code: the last command is still running)
	bool diags;		// diags mode instead of G-code
	bool was_connected;
} con_port;

static con_port ports[CON_PORTS] = {
	{ .read = UsbReadChar, .write = UsbWrite, .connected = UsbConnected },
	{ .read = UartReadChar, .write = UartWrite },
	{ .read = EspConReadChar, .write = EspConWrite, .connected = EspConConnected },
};

static uint8_t out_mask = (1 << CON_PORTS) - 1;

static int8_t bridge_port = -1;	// -1 = none
static uint32_t bridge_rx_ms;	// last byte from the host
static uint8_t bridge_plus;		// '+'s held back

void ConsoleOut(uint8_t mask) {
	out_mask = mask;
}

uint8_t ConsoleOutMask() {
	return out_mask;
}

int ConsoleReadChar() {
	uint8_t p;
	int c;
	for (p = 0; p < CON_PORTS; p++) {
		if (((out_mask >> p) & 1) && (c = ports[p].read()) >= 0) return c;
	}
	return -1;
}

// printf ends up here. \n -> \r\n for terminals (G-code hosts don't mind the \r)
int _write(int file, char *ptr, int len) {
	int i, start = 0;
	uint8_t p;
	(void)file;

	for (p = 0; p < CON_PORTS; p++) {
		if (!((out_mask >> p) & 1) || p == bridge_port) continue;
		start = 0;
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				ports[p].write(ptr + start, i - start);
				ports[p].write("\r\n", 2);
				start = i + 1;
			}
		}
		ports[p].write(ptr + start, len - start);
	}
	return len;
}

static void Greeting(uint8_t p) {
	ConsoleOut(1 << p);
	if (ports[p].diags) {
		printf("\nPnP controller\n");
		DiagsMenu();
		printf("[PnP]: ");
	} else {
		// What Marlin prints at reset, hosts look for "start"
		printf("start\necho:PnP controller, Marlin G-code. Type diags for the menu\n");
	}
}

static bool IsWord(const char *line, const char *w) {
	while (*line == ' ') line++;
	while (*w) {
		char c = *line >= 'A' && *line <= 'Z' ? *line + 32 : *line;
		if (c != *w) return false;
		line++;
		w++;
	}
	while (*line == ' ') line++;
	return *line == '\0';
}

// Diags mode: echo, backspace, [PnP]: prompt
static void ReadDiags(uint8_t p) {
	con_port *cp = &ports[p];
	int c;

	while ((c = cp->read()) >= 0) {
		if (c == '\r' || c == '\n') {
			if (c == '\n' && cp->len == 0) continue; // second half of \r\n
			printf("\n");
			cp->line[cp->len] = '\0';
			cp->len = 0;
			if (IsWord(cp->line, "exit")) {
				cp->diags = false;
				printf("Back to G-code\n");
				return;
			}
			DiagsRun(cp->line);
			if (bridge_port == p) return; // esp bridge, the rest goes to the ESP
			ConsoleOut(1 << p); // a command may have changed it
			printf("[PnP]: ");
		} else if (c == '\b' || c == 0x7F) {
			if (cp->len > 0) {
				cp->len--;
				printf("\b \b");
			}
		} else if (cp->len < LINE_LEN - 1 && c >= ' ') {
			cp->line[cp->len++] = c;
			putchar(c);
		}
	}
}

// G-code mode: collect one line, no echo. Stops reading once a line is complete, so the next
// line waits in the serial buffer until the current command has answered "ok".
static void ReadGcode(uint8_t p) {
	con_port *cp = &ports[p];
	int c;

	while (!cp->held && (c = cp->read()) >= 0) {
		if (c == '\r' || c == '\n') {
			if (cp->len == 0) continue;
			cp->line[cp->len] = '\0';
			cp->len = 0;
			cp->held = true;
		} else if (cp->len < LINE_LEN - 1) {
			cp->line[cp->len++] = c;
		}
	}
}

bool ConsoleBridge() {
	uint8_t p;

	if (bridge_port >= 0) {
		printf("Already bridged on another port\n");
		return false;
	}
	for (p = 0; p < CON_PORTS; p++) {
		if (out_mask == 1 << p) break;
	}
	if (p == CON_PORTS) return false;
	if (p == CON_ESP) {
		printf("Can't bridge the ESP32 to itself, use USB or the ST-Link UART\n");
		return false;
	}

	printf("Bridge open at %" PRIu32 " baud. 1s quiet, +++, 1s quiet to get back\n", p == CON_USB ? UsbBaud() : ESP_BAUD);

	EspBaud(p == CON_USB ? UsbBaud() : ESP_BAUD);
	EspLinkPause(true); // WiFi console off while the UART is raw
	EspFlushRx();
	ports[p].len = 0;
	bridge_plus = 0;
	bridge_rx_ms = Millis();
	bridge_port = p;
	return true;
}

static void BridgeFlushPlus() {
	while (bridge_plus) {
		EspWrite("+", 1);
		bridge_plus--;
	}
}

// Port <-> ESP32 UART. 1s quiet, +++, 1s quiet ends it
static void BridgeTask(uint8_t p) {
	con_port *cp = &ports[p];
	char buf[256];
	uint32_t n = 0, now = Millis();
	int c;

	// esptool changes the baud after its stub loads
	if (p == CON_USB) EspBaud(UsbBaud());

	while (EspTxRoom() > BRIDGE_ESC_LEN && (c = cp->read()) >= 0) {
		if (c == '+' && bridge_plus < BRIDGE_ESC_LEN && (bridge_plus > 0 || now - bridge_rx_ms >= BRIDGE_GUARD_MS)) {
			bridge_plus++;
		} else {
			char ch = c;
			BridgeFlushPlus();
			EspWrite(&ch, 1);
		}
		bridge_rx_ms = now;
	}

	if (bridge_plus && now - bridge_rx_ms >= BRIDGE_GUARD_MS) {
		if (bridge_plus == BRIDGE_ESC_LEN) {
			bridge_plus = 0;
			bridge_port = -1;
			EspBaud(ESP_BAUD); // esptool may have left it at 921600
			EspLinkPause(false);
			ConsoleOut(1 << p);
			printf("\nBridge closed\n[PnP]: ");
			return;
		}
		BridgeFlushPlus();
	}

	if (p == CON_USB) {
		uint32_t room = UsbWriteRoom();
		while (n < sizeof(buf) && n < room && (c = EspReadChar()) >= 0) buf[n++] = c;
		if (n) UsbWriteNow(buf, n);
	} else {
		while (n < sizeof(buf) && (c = EspReadChar()) >= 0) buf[n++] = c;
		if (n) cp->write(buf, n);
	}
}

void ConsoleTask() {
	uint8_t p, diags_mask = 0;

	for (p = 0; p < CON_PORTS; p++) {
		con_port *cp = &ports[p];
		bool up = cp->connected ? cp->connected() : true;

		// Greeting: UART once at boot, USB / WiFi whenever a terminal opens the port
		if (up && !cp->was_connected && p != bridge_port) Greeting(p);
		cp->was_connected = up;
		ConsoleOut(1 << p);

		if (p == bridge_port) {
			BridgeTask(p);
			continue;
		}
		if (cp->diags) {
			ReadDiags(p);
		} else {
			if (GcodeBusy(p)) GcodeResume(p);
			ReadGcode(p);
			if (cp->held) {
				if (IsWord(cp->line, "diags") && !GcodeBusy(p)) {
					cp->held = false;
					cp->diags = true;
					Greeting(p);
				} else if (GcodeEmergency(cp->line)) {
					// M112 doesn't wait behind a running command (Marlin's emergency parser)
					cp->held = false;
					GcodeLine(p, cp->line);
				} else if (!GcodeBusy(p)) {
					cp->held = false;
					GcodeLine(p, cp->line);
				}
			}
		}
		if (cp->diags) diags_mask |= 1 << p;
	}

	if (bridge_port < 0) DiagsEspTask();

	// Monitor output only goes to ports in diags mode
	if (diags_mask) {
		ConsoleOut(diags_mask);
		DiagsPoll();
	}
	ConsoleOut((1 << CON_PORTS) - 1);
}
