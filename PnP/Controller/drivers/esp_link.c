#include "esp.h"
#include "clock.h"

// UART6 carries three channels, so the ESP32's TCP console can be a command port like USB and the
// Xbox controller state can come in while the ESP still gets its own commands. DLE (0x10) + a code, both ways:
//   DLE C		console bytes follow
//   DLE P		one pad packet follows (ESP -> H7 only, EspPad below)
//   DLE E		ESP channel follows (the default: its log, scan / wifi / link, !boot)
//   DLE DLE	a 0x10 in the data
// ESP -> H7: DLE + = a client is on the console port, DLE - = none
// H7 -> ESP: DLE S / DLE Q = stop / go on sending console bytes (our buffer), DLE ? = send + or -
// Console and pad frames always end with DLE E, and the receiver drops back to the ESP channel after
// FRAME_MS of quiet, so a reset on either side mid frame can't leave it stuck.

#define DLE 0x10
#define CON_LEN 1024	// power of 2
#define CTL_LEN 256		// power of 2
#define CON_STOP (CON_LEN * 3 / 4)	// ESP sends at most 128 bytes at a time
#define CON_GO (CON_LEN / 4)
#define FRAME_MS 100
#define WRITE_TIMEOUT_MS 50
#define PAD_LEN 18		// packet bytes, last one is the sum of the others
#define PAD_STALE_MS 150	// ESP sends every 20ms while the pad is connected

#define CH_ESP 0
#define CH_CON 1
#define CH_PAD 2

static char con_buf[CON_LEN];
static uint32_t con_head, con_tail;
static char ctl_buf[CTL_LEN];
static uint32_t ctl_head, ctl_tail;
static uint8_t ch;		// channel being received
static bool in_dle;
static uint8_t pad_rx[PAD_LEN];
static uint8_t pad_rx_len;
static EspPad pad;			// last good packet
static uint32_t pad_ms;		// when it came
static EspPadStats pad_stats;
static bool stopped;	// sent DLE S
static bool connected;	// client on the ESP's console port
static bool replaced;	// a new client replaced the old one
static bool paused;		// esp bridge has the UART
static uint32_t last_ms;

static void Code(char c) {
	char b[2] = { DLE, c };
	EspWrite(b, 2);
}

static uint16_t U16(const uint8_t *b) {
	return b[0] | b[1] << 8;
}

// End of a pad frame: check it, unpack it
static void PadFrame() {
	uint8_t sum = 0, i;

	if (pad_rx_len != PAD_LEN) {
		pad_stats.bad++;
		return;
	}
	for (i = 0; i < PAD_LEN - 1; i++) sum += pad_rx[i];
	if (sum != pad_rx[PAD_LEN - 1]) {
		pad_stats.bad++;
		return;
	}
	if (pad_stats.good && (uint8_t)(pad_rx[0] - pad.seq) != 1) pad_stats.missed += (uint8_t)(pad_rx[0] - pad.seq - 1);
	pad.seq = pad_rx[0];
	pad.connected = pad_rx[1] & 1;
	pad.battery = pad_rx[2];
	pad.lx = U16(pad_rx + 3);
	pad.ly = U16(pad_rx + 5);
	pad.rx = U16(pad_rx + 7);
	pad.ry = U16(pad_rx + 9);
	pad.lt = U16(pad_rx + 11);
	pad.rt = U16(pad_rx + 13);
	pad.buttons = U16(pad_rx + 15);
	pad_ms = Millis();
	pad_stats.good++;
}

static void Channel(uint8_t c) {
	if (ch == CH_PAD) PadFrame();
	ch = c;
	pad_rx_len = 0;
}

static void Put(char c) {
	if (ch == CH_PAD) {
		if (pad_rx_len < PAD_LEN) pad_rx[pad_rx_len] = c;
		if (pad_rx_len < 255) pad_rx_len++;
	} else if (ch == CH_CON) {
		if (con_head - con_tail < CON_LEN) con_buf[con_head++ & (CON_LEN - 1)] = c;
	} else if (ctl_head - ctl_tail < CTL_LEN) {
		ctl_buf[ctl_head++ & (CTL_LEN - 1)] = c;
	}
}

static void ConFlush() {
	con_tail = con_head;
	if (stopped) {
		Code('Q');
		stopped = false;
	}
}

// UART6 -> the two buffers
static void Poll() {
	uint32_t now = Millis();
	int c;

	if (paused) return;
	while ((c = EspReadChar()) >= 0) {
		if (now - last_ms > FRAME_MS) {
			// Half a frame and then nothing: drop it (a pad packet is checked at its DLE E)
			ch = CH_ESP;
			in_dle = false;
			pad_rx_len = 0;
		}
		last_ms = now;
		if (in_dle) {
			in_dle = false;
			switch (c) {
				case DLE: Put(DLE); break;
				case 'C': Channel(CH_CON); break;
				case 'P': Channel(CH_PAD); break;
				case 'E': Channel(CH_ESP); break;
				case '+':
				case '-':
					// New client or gone, either way what's left is stale
					if (c == '+' && connected) replaced = true;
					connected = c == '+';
					ConFlush();
					break;
			}
		} else if (c == DLE) {
			in_dle = true;
		} else {
			Put(c);
		}
	}
	if (!stopped && con_head - con_tail > CON_STOP) {
		Code('S');
		stopped = true;
	}
}

void EspLinkInit() {
	Code('?');
}

void EspLinkPause(bool p) {
	if (p == paused) return;
	paused = p;
	connected = false;
	ch = CH_ESP;
	in_dle = false;
	pad_rx_len = 0;
	pad_ms = Millis() - PAD_STALE_MS; // stale until the next packet
	con_tail = con_head;
	ctl_tail = ctl_head;
	if (!p) {
		EspFlushRx();
		stopped = false;
		Code('Q');
		Code('?');
	}
}

bool EspConConnected() {
	Poll();
	// A new client took over: looks disconnected once so the console greets it
	if (replaced) {
		replaced = false;
		return false;
	}
	return connected;
}

int EspConReadChar() {
	char c;

	Poll();
	if (con_head == con_tail) return -1;
	c = con_buf[con_tail++ & (CON_LEN - 1)];
	if (stopped && con_head - con_tail < CON_GO) {
		Code('Q');
		stopped = false;
	}
	return c;
}

// Waits for room, false if the UART stays full past the timeout
static bool Send(const char *b, uint32_t n, uint32_t t) {
	while (EspTxRoom() < n) {
		if (Millis() - t > WRITE_TIMEOUT_MS) return false;
	}
	EspWrite(b, n);
	return true;
}

uint32_t EspConWrite(const char *buf, uint32_t len) {
	static const char start[2] = { DLE, 'C' }, end[2] = { DLE, 'E' }, dle2[2] = { DLE, DLE };
	uint32_t i, t = Millis();

	if (paused || !connected || len == 0) return len; // dropped, like USB with no terminal
	if (!Send(start, 2, t)) return 0;
	for (i = 0; i < len; i++) {
		if (!(buf[i] == DLE ? Send(dle2, 2, t) : Send(buf + i, 1, t))) break;
	}
	Send(end, 2, Millis()); // always close the frame
	return i;
}

int EspCtlReadChar() {
	char c;

	Poll();
	if (ctl_head == ctl_tail) return -1;
	c = ctl_buf[ctl_tail++ & (CTL_LEN - 1)];
	return c;
}

void EspCtlFlush() {
	Poll();
	ctl_tail = ctl_head;
}

bool EspPadGet(EspPad *p) {
	Poll();
	if (pad_stats.good && Millis() - pad_ms < PAD_STALE_MS && pad.connected) {
		*p = pad;
		return true;
	}
	// Gone or stale: sticks centred, triggers and buttons let go
	*p = (EspPad){ .seq = pad.seq, .battery = pad.battery, .lx = 32767, .ly = 32767, .rx = 32767, .ry = 32767 };
	return false;
}

void EspPadGetStats(EspPadStats *s) {
	Poll();
	*s = pad_stats;
	s->age_ms = pad_stats.good ? Millis() - pad_ms : UINT32_MAX;
}
