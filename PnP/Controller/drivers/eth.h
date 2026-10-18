#ifndef _ETH_H_
#define _ETH_H_

#include <stdbool.h>
#include <inttypes.h>

// Ethernet MAC + DP83826 PHY over RMII. PHY makes the 50MHz REF_CLK (PA1).
// No lwIP in here, net.c does that. RX/TX are copy in / copy out from DMA buffers in
// non-cacheable RAM, polled from the main loop. The ETH interrupt just wakes the CPU.
// Link handling only uses the standard IEEE registers (0-5). The one DP83826 register used is
// RCSR, to force RMII leader mode (see EthInit).

#define ETH_MTU_FRAME	1536	// buffer size, max frame 1518

// Standard (clause 22) PHY registers
#define PHY_BMCR		0
#define PHY_BMSR		1
#define PHY_ID1			2
#define PHY_ID2			3
#define PHY_ANAR		4
#define PHY_ANLPAR		5
#define PHY_BISCR		0x16	// DP83826 BIST control (loopback select)
#define PHY_RCSR		0x17	// DP83826 RMII control (datasheet 8.5.1.23)

typedef struct {
	bool up;
	bool full_duplex;
	uint16_t speed;		// 10 / 100
} EthLink;

typedef struct {
	uint32_t rx_frames, rx_errors, rx_overruns;
	uint32_t tx_frames, tx_busy;
} EthStats;

#define ETH_OK					0
#define ETH_ERR_NOT_STARTED		1
#define ETH_ERR_NO_REFCLK		2	// PHY answers on MDIO but no 50MHz on PA1, MAC can't start
#define ETH_ERR_NO_PHY			3	// MAC ok but nothing answers on MDIO
#define ETH_ERR_NO_PHY_NO_CLK	4	// neither

bool EthInit(const uint8_t *mac);	// false if the PHY or its 50MHz clock didn't show up
uint8_t EthStatus();
bool EthReady();
int8_t EthPhyAddr();				// -1 if no PHY answered
uint32_t EthPhyId();				// ID1 << 16 | ID2
bool EthMdioRead(uint8_t reg, uint16_t *val);
bool EthMdioWrite(uint8_t reg, uint16_t val);

EthLink EthLinkUpdate();			// reads the PHY, sets MAC speed/duplex to match
void EthPhyLoopback(bool on);		// PHY digital loopback, 100M full duplex

uint8_t *EthTxBuffer();				// next free TX buffer, NULL if all busy
void EthTxSend(uint32_t len);		// send what was put in EthTxBuffer()
uint8_t *EthRxPeek(uint32_t *len);	// next received frame, NULL if none
void EthRxRelease();				// give it back to the DMA

void EthGetStats(EthStats *s);

#endif
