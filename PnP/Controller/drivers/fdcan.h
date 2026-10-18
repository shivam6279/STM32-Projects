#ifndef _FDCAN_H_
#define _FDCAN_H_

#include <stdbool.h>
#include <inttypes.h>

// FDCAN1 (PB8 RX / PB9 TX) and FDCAN2 (PB5 RX / PB6 TX), TJA1051TK/3 transceivers.
// CAN FD, 1 Mbit/s nominal, 5 Mbit/s data (TJA1051 is rated to 5 Mbit/s in the FD fast phase).
// RX: everything goes to FIFO0, the ISR copies it into a software queue. TX: TX FIFO.

#define CAN_BUSES	2

typedef struct {
	uint32_t id;
	bool ext;		// 29 bit ID
	bool fd;		// FD frame (else classic CAN)
	bool brs;		// switch to the data bit rate (FD only)
	uint8_t len;	// bytes, FD: 0-8, 12, 16, 20, 24, 32, 48, 64
	uint8_t data[64];
} CanFrame;

typedef struct {
	uint8_t lec;		// last error code, nominal phase
	uint8_t dlec;		// last error code, data phase
	uint8_t tec, rec;	// error counters
	bool passive, warning, bus_off;
	uint8_t tdcv;		// measured transmitter delay, in tq
	uint32_t rx_count, tx_count, rx_dropped;
} CanStatus;

void CanInit();
void CanLoopback(uint8_t bus, bool on);	// internal loopback, nothing goes on the bus
bool CanSend(uint8_t bus, const CanFrame *f);	// false if the TX FIFO is full
bool CanRead(uint8_t bus, CanFrame *f);	// false if nothing received
void CanGetStatus(uint8_t bus, CanStatus *s);
uint8_t CanRoundLen(uint8_t len);		// round up to a valid FD length

#endif
