#ifndef _ESP_H_
#define _ESP_H_

#include <stdbool.h>
#include <inttypes.h>

// ESP32-C3-MINI-1 (U3). UART6 (PC6 TX / PC7 RX) = its UART0
// EN and BOOT (IO9) are on the IO expander

#define ESP_BAUD		115200	// ESP app, also its ROM bootloader
#define ESP_BOOT_BAUD	921600	// our bootloader, flashing over WiFi

// esp.c
bool EspInit();						// false if the IO expander doesn't answer
bool EspReset(bool download);		// pulse EN. download: BOOT held low through it = ROM bootloader

// esp_uart.c, bootloader uses this too
void EspUartInit(uint32_t baud);
void EspBaud(uint32_t baud);		// waits for TX to finish first
uint32_t EspGetBaud();
uint32_t EspWrite(const char *buf, uint32_t len);	// doesn't wait, returns how much fit
uint32_t EspTxRoom();
bool EspTxIdle();					// everything sent
int EspReadChar();					// -1 if nothing
void EspFlushRx();

// esp_link.c, app only. UART6 split into the console channel (TCP console on the ESP, a command
// port like USB) and the ESP channel (its log and commands). EspRead/EspWrite still go straight to
// the UART, ESP channel writes need nothing else.
void EspLinkInit();					// asks the ESP whether a console client is on
void EspLinkPause(bool pause);		// esp bridge has the UART raw
bool EspConConnected();
int EspConReadChar();
uint32_t EspConWrite(const char *buf, uint32_t len);	// dropped if no client
int EspCtlReadChar();				// ESP channel, -1 if nothing
void EspCtlFlush();

// Xbox controller state, sent by the ESP on every change and every 20ms while connected.
// Packet (18 bytes, little endian): seq, flags (bit 0 = connected), battery %, LX, LY, RX, RY
// (u16, 0-65535, 32767 = centre), LT, RT (u16, 0-1023), buttons (u16, PAD_ bits), sum of the rest
#define PAD_A		(1 << 0)
#define PAD_B		(1 << 1)
#define PAD_X		(1 << 2)
#define PAD_Y		(1 << 3)
#define PAD_LB		(1 << 4)
#define PAD_RB		(1 << 5)
#define PAD_LS		(1 << 6)
#define PAD_RS		(1 << 7)
#define PAD_UP		(1 << 8)
#define PAD_DOWN	(1 << 9)
#define PAD_LEFT	(1 << 10)
#define PAD_RIGHT	(1 << 11)
#define PAD_VIEW	(1 << 12)
#define PAD_MENU	(1 << 13)
#define PAD_SHARE	(1 << 14)
#define PAD_XBOX	(1 << 15)
#define PAD_BUTTONS	16

typedef struct {
	uint8_t seq;
	bool connected;
	uint8_t battery;
	uint16_t lx, ly, rx, ry;
	uint16_t lt, rt;
	uint16_t buttons;
} EspPad;

typedef struct {
	uint32_t good, bad, missed;	// packets: ok, wrong length / sum, seq gaps
	uint32_t age_ms;			// since the last good one, UINT32_MAX = never
} EspPadStats;

bool EspPadGet(EspPad *p);			// false = no pad, or nothing for 150ms: p is centred / let go
void EspPadGetStats(EspPadStats *s);

// Byte counts and errors since boot
typedef struct {
	uint32_t rx, tx;
	uint32_t overrun, frame, rx_dropped;
} EspStats;
void EspGetStats(EspStats *s);

#endif
