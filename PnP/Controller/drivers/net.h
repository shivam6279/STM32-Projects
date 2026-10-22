#ifndef _NET_H_
#define _NET_H_

#include <stdbool.h>
#include <inttypes.h>
#include "eth.h"

// lwIP on top of eth.c. Fixed address on a dedicated subnet for the link to the Pi,
// plus a discovery responder so the Pi can find us if the subnet ever changes.
//   Pi:   send any UDP packet to 255.255.255.255:NET_DISCOVERY_PORT
//   here: broadcast back "PnP ip=... mask=... mac=... uid=..." on the same port
// TCP NET_CON_PORT: the G-code / diags console (console.c), raw bytes like the USB COM port.
// One client, a new one replaces it. Same port number as the ESP32's WiFi console.

#define NET_IP				"10.77.0.2"
#define NET_MASK			"255.255.255.0"
#define NET_DISCOVERY_PORT	41077
#define NET_LINK_POLL_MS	500
#define NET_CON_PORT		3234

bool NetInit();			// false if the ethernet hardware didn't come up
void NetTask();			// main loop: RX frames into lwIP, lwIP timers
bool NetUp();			// ethernet hardware ok
EthLink NetLink();		// last link state from the poller
void NetGetMac(uint8_t *mac);
uint32_t NetIp();		// network byte order, like lwIP
uint32_t NetMask();
bool NetSetIp(const char *ip, const char *mask);

int NetConReadChar();	// TCP console, -1 if nothing
uint32_t NetConWrite(const char *buf, uint32_t len);
bool NetConConnected();
uint32_t NetConPeer();	// client address (network byte order), 0 if none

#endif
