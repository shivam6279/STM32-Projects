#ifndef _NET_H_
#define _NET_H_

#include <stdbool.h>
#include <inttypes.h>
#include "eth.h"

// lwIP on top of eth.c. Fixed address on a dedicated subnet for the link to the Pi,
// plus a discovery responder so the Pi can find us if the subnet ever changes.
//   Pi:   send any UDP packet to 255.255.255.255:NET_DISCOVERY_PORT
//   here: broadcast back "PnP ip=... mask=... mac=... uid=..." on the same port

#define NET_IP				"10.77.0.2"
#define NET_MASK			"255.255.255.0"
#define NET_DISCOVERY_PORT	41077
#define NET_LINK_POLL_MS	500

bool NetInit();			// false if the ethernet hardware didn't come up
void NetTask();			// main loop: RX frames into lwIP, lwIP timers
bool NetUp();			// ethernet hardware ok
EthLink NetLink();		// last link state from the poller
void NetGetMac(uint8_t *mac);
uint32_t NetIp();		// network byte order, like lwIP
uint32_t NetMask();
bool NetSetIp(const char *ip, const char *mask);

#endif
