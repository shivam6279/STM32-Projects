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

// Byte counts and errors since boot
typedef struct {
	uint32_t rx, tx;
	uint32_t overrun, frame, rx_dropped;
} EspStats;
void EspGetStats(EspStats *s);

#endif
