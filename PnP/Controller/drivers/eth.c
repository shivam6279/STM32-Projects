#include "eth.h"
#include "board.h"
#include "clock.h"
#include "irq_prio.h"
#include "mpu.h"
#include "stm32h7xx.h"

#include <string.h>

// Init order and defaults follow ST's HAL (stm32h7xx_hal_eth.c: HAL_ETH_Init, ETH_MACDMAConfig,
// HAL_ETH_Start). Descriptor bits from stm32h7xx_hal_eth.h (ETH_DMATXNDESCRF_*, ETH_DMARXNDESC*).
#define RX_DESC		8
#define TX_DESC		4

#define RDES3_OWN	0x80000000	// read format
#define RDES3_IOC	0x40000000
#define RDES3_BUF1V	0x01000000
#define RDES3_CTXT	0x40000000	// write back format
#define RDES3_FD	0x20000000
#define RDES3_LD	0x10000000
#define RDES3_ES	0x00008000
#define RDES3_PL	0x00007FFF

#define TDES2_B1L	0x00003FFF
#define TDES3_OWN	0x80000000
#define TDES3_FD	0x20000000
#define TDES3_LD	0x10000000
#define TDES3_FL	0x00007FFF

// DP83826 datasheet (SNLS647I, page 15), "Reset Timing" / "Power Up Timing" tables:
//   reset pulse width       >= 25us (T1, no debounce cap on RST_N)
//   reset to SMI (MDIO) ready  2ms  (T2)
//   power-up to SMI ready     50ms  (T4)
// Millis() is 1ms resolution, so the pulse is 1ms (>= 25us).
#define PHY_RESET_PULSE_MS	1
#define PHY_RESET_SMI_MS	2
#define PHY_POWERUP_SMI_MS	50

typedef struct {
	volatile uint32_t des0, des1, des2, des3;
} EthDesc;

static EthDesc rx_desc[RX_DESC] DMA_BUF;
static EthDesc tx_desc[TX_DESC] DMA_BUF;
static uint8_t rx_buf[RX_DESC][ETH_MTU_FRAME] DMA_BUF;
static uint8_t tx_buf[TX_DESC][ETH_MTU_FRAME] DMA_BUF;

static uint32_t rx_idx, tx_idx;
static int8_t phy_addr = -1;
static uint32_t phy_id;
static bool ready = false;
static uint8_t status = ETH_ERR_NOT_STARTED;
static EthStats stats;
static volatile uint32_t rx_overruns;

static void Wait(uint32_t ms) {
	uint32_t t = Millis();
	while (Millis() - t <= ms);
}

static bool MdioWait() {
	uint32_t t = Millis();
	while (ETH->MACMDIOAR & ETH_MACMDIOAR_MB) {
		if (Millis() - t > 2) return false;
	}
	return true;
}

static bool MdioRead(uint8_t addr, uint8_t reg, uint16_t *val) {
	if (!MdioWait()) return false;
	ETH->MACMDIOAR = (ETH->MACMDIOAR & ETH_MACMDIOAR_CR) | (addr << 21) | (reg << 16) | ETH_MACMDIOAR_MOC_RD | ETH_MACMDIOAR_MB;
	if (!MdioWait()) return false;
	*val = ETH->MACMDIODR & 0xFFFF;
	return true;
}

static bool MdioWrite(uint8_t addr, uint8_t reg, uint16_t val) {
	if (!MdioWait()) return false;
	ETH->MACMDIODR = val;
	ETH->MACMDIOAR = (ETH->MACMDIOAR & ETH_MACMDIOAR_CR) | (addr << 21) | (reg << 16) | ETH_MACMDIOAR_MOC_WR | ETH_MACMDIOAR_MB;
	return MdioWait();
}

// Work even if the MAC didn't start, for poking at the PHY
bool EthMdioRead(uint8_t reg, uint16_t *val) {
	return phy_addr >= 0 && MdioRead(phy_addr, reg, val);
}

bool EthMdioWrite(uint8_t reg, uint16_t val) {
	return phy_addr >= 0 && MdioWrite(phy_addr, reg, val);
}

static void RingsInit() {
	uint32_t i;

	for (i = 0; i < TX_DESC; i++) {
		tx_desc[i].des0 = tx_desc[i].des1 = tx_desc[i].des2 = tx_desc[i].des3 = 0;
	}
	for (i = 0; i < RX_DESC; i++) {
		rx_desc[i].des0 = (uint32_t)rx_buf[i];
		rx_desc[i].des1 = 0;
		rx_desc[i].des2 = 0;
		rx_desc[i].des3 = RDES3_OWN | RDES3_BUF1V | RDES3_IOC;
	}
	tx_idx = rx_idx = 0;

	ETH->DMACTDRLR = TX_DESC - 1;
	ETH->DMACTDLAR = (uint32_t)tx_desc;
	ETH->DMACTDTPR = (uint32_t)tx_desc;
	ETH->DMACRDRLR = RX_DESC - 1;
	ETH->DMACRDLAR = (uint32_t)rx_desc;
	__DMB();
	ETH->DMACRDTPR = (uint32_t)&rx_desc[RX_DESC - 1];
}

bool EthInit(const uint8_t *mac) {
	uint8_t a;
	uint16_t id1, id2;
	uint32_t t;
	bool clock_ok = true;

	ready = false;

	// PHY has been held in reset since BoardIoInit. Make sure its power-up time has passed
	// (cold boot gets here quickly), give it a clean reset pulse, then wait for MDIO.
	while (Millis() < PHY_POWERUP_SMI_MS);
	PIN_PHY_RST_PORT->BSRR = PIN_PHY_RST_MASK << 16;
	Wait(PHY_RESET_PULSE_MS);
	PIN_PHY_RST_PORT->BSRR = PIN_PHY_RST_MASK;
	Wait(PHY_RESET_SMI_MS);

	// RMII select before the MAC comes out of reset (SYSCFG_ETH_RMII = EPIS_SEL_2)
	SYSCFG->PMCR = (SYSCFG->PMCR & ~SYSCFG_PMCR_EPIS_SEL) | SYSCFG_PMCR_EPIS_SEL_2;
	(void)SYSCFG->PMCR;
	RCC->AHB1ENR |= RCC_AHB1ENR_ETH1MACEN | RCC_AHB1ENR_ETH1TXEN | RCC_AHB1ENR_ETH1RXEN;
	(void)RCC->AHB1ENR;

	// MDC = HCLK / 124 for HCLK >= 250MHz (HAL_ETH_SetMDIOClockRange) -> ~2.1MHz.
	// MDIO runs off HCLK, so it works before the PHY's 50MHz clock is there.
	ETH->MACMDIOAR = ETH_MACMDIOAR_CR_DIV124;

	// Find the PHY: first address that answers with a sane ID
	phy_addr = -1;
	for (a = 0; a < 32; a++) {
		if (MdioRead(a, PHY_ID1, &id1) && MdioRead(a, PHY_ID2, &id2) && id1 != 0xFFFF && id1 != 0x0000) {
			phy_addr = a;
			phy_id = ((uint32_t)id1 << 16) | id2;
			break;
		}
	}

	// Board straps the DP83826 as RMII follower (SOR1 Strap10 = 1, datasheet table 8-12), which
	// wants 50MHz in on XI, but it has a 25MHz crystal. Force RMII leader: RCSR (0x17) bit 7 = 0
	// = 25MHz crystal reference, and the PHY drives 50MHz out on RX_CLK/50MHz_RMII -> PA1.
	// TODO fix the strap on the PHY's RX_DV pin (pull-down) and drop this.
	if (phy_addr >= 0 && MdioRead(phy_addr, PHY_RCSR, &id1)) {
		MdioWrite(phy_addr, PHY_RCSR, id1 & ~(1 << 7));
		Wait(2);
	}

	// Software reset. Only finishes if the PHY's 50MHz REF_CLK is coming in on PA1.
	ETH->DMAMR |= ETH_DMAMR_SWR;
	t = Millis();
	while (ETH->DMAMR & ETH_DMAMR_SWR) {
		if (Millis() - t > 100) {
			clock_ok = false;
			break;
		}
	}
	// SWR resets the MAC registers, MDIO setup included
	ETH->MACMDIOAR = ETH_MACMDIOAR_CR_DIV124;
	ETH->MAC1USTCR = CLOCK_HCLK_HZ / 1000000 - 1;

	if (phy_addr < 0) {
		status = clock_ok ? ETH_ERR_NO_PHY : ETH_ERR_NO_PHY_NO_CLK;
		return false;
	}
	if (!clock_ok) {
		status = ETH_ERR_NO_REFCLK;
		return false;
	}

	// MAC: auto pad/CRC strip, 100M full duplex until the link says otherwise. Checksums in software.
	ETH->MACCR = ETH_MACCR_ACS | ETH_MACCR_CST | ETH_MACCR_DM | ETH_MACCR_FES;
	ETH->MACPFR = 0;	// our MAC + broadcast
	ETH->MACA0HR = ((uint32_t)mac[5] << 8) | mac[4];
	ETH->MACA0LR = ((uint32_t)mac[3] << 24) | ((uint32_t)mac[2] << 16) | ((uint32_t)mac[1] << 8) | mac[0];

	// DMA / MTL: store and forward, 32 beat bursts, 4 word descriptors (DSL = 0)
	ETH->DMASBMR = ETH_DMASBMR_AAL | ETH_DMASBMR_FB;
	ETH->DMACCR &= ~ETH_DMACCR_DSL;
	ETH->DMACTCR = (32 << ETH_DMACTCR_TPBL_Pos);
	ETH->DMACRCR = (32 << ETH_DMACRCR_RPBL_Pos) | (ETH_MTU_FRAME << ETH_DMACRCR_RBSZ_Pos);
	ETH->MTLTQOMR |= ETH_MTLTQOMR_TSF;
	ETH->MTLRQOMR |= ETH_MTLRQOMR_RSF;

	RingsInit();

	ETH->DMACIER = ETH_DMACIER_NIE | ETH_DMACIER_RIE | ETH_DMACIER_AIE | ETH_DMACIER_RBUE | ETH_DMACIER_FBEE;
	NVIC_SetPriority(ETH_IRQn, IRQ_PRIO_ETH);
	NVIC_EnableIRQ(ETH_IRQn);

	// Start (HAL_ETH_Start order)
	ETH->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;
	ETH->MTLTQOMR |= ETH_MTLTQOMR_FTQ;
	ETH->DMACTCR |= ETH_DMACTCR_ST;
	ETH->DMACRCR |= ETH_DMACRCR_SR;
	ETH->DMACSR = ETH_DMACSR_TPS | ETH_DMACSR_RPS;

	ready = true;
	status = ETH_OK;
	return true;
}

uint8_t EthStatus() {
	return status;
}

bool EthReady() {
	return ready;
}

int8_t EthPhyAddr() {
	return phy_addr;
}

uint32_t EthPhyId() {
	return phy_id;
}

EthLink EthLinkUpdate() {
	EthLink l = { false, false, 0 };
	uint16_t bmcr, bmsr, anar, anlpar, common;

	if (!ready) return l;
	// BMSR link bit latches low, read twice for the current state
	if (!EthMdioRead(PHY_BMSR, &bmsr) || !EthMdioRead(PHY_BMSR, &bmsr) || !EthMdioRead(PHY_BMCR, &bmcr)) return l;
	l.up = (bmsr >> 2) & 1;

	if ((bmcr & (1 << 12)) && (bmsr & (1 << 5)) && EthMdioRead(PHY_ANAR, &anar) && EthMdioRead(PHY_ANLPAR, &anlpar)) {
		// Autoneg done: best mode both ends advertise
		common = anar & anlpar;
		if (common & (1 << 8)) { l.speed = 100; l.full_duplex = true; }
		else if (common & (1 << 7)) { l.speed = 100; l.full_duplex = false; }
		else if (common & (1 << 6)) { l.speed = 10; l.full_duplex = true; }
		else { l.speed = 10; l.full_duplex = false; }
	} else {
		// Forced (or loopback): from BMCR
		l.speed = (bmcr & (1 << 13)) ? 100 : 10;
		l.full_duplex = (bmcr >> 8) & 1;
	}

	if (l.up) {
		uint32_t cr = ETH->MACCR & ~(ETH_MACCR_FES | ETH_MACCR_DM);
		if (l.speed == 100) cr |= ETH_MACCR_FES;
		if (l.full_duplex) cr |= ETH_MACCR_DM;
		ETH->MACCR = cr;
	}
	return l;
}

void EthPhyLoopback(bool on) {
	static uint16_t biscr;

	// MII loopback (DP83826 datasheet 8.3.14.2): BMCR bit 14 + BISCR bit 2, autoneg off,
	// forced 100M full duplex. Off: put BISCR back, autoneg on + restart.
	if (on) {
		EthMdioRead(PHY_BISCR, &biscr);
		EthMdioWrite(PHY_BMCR, (1 << 14) | (1 << 13) | (1 << 8));
		EthMdioWrite(PHY_BISCR, biscr | (1 << 2));
		ETH->MACCR |= ETH_MACCR_FES | ETH_MACCR_DM;
	} else {
		EthMdioWrite(PHY_BISCR, biscr & ~(1 << 2));
		EthMdioWrite(PHY_BMCR, (1 << 12) | (1 << 9));
	}
}

uint8_t *EthTxBuffer() {
	if (!ready || (tx_desc[tx_idx].des3 & TDES3_OWN)) {
		stats.tx_busy++;
		return NULL;
	}
	return tx_buf[tx_idx];
}

void EthTxSend(uint32_t len) {
	EthDesc *d = &tx_desc[tx_idx];

	d->des0 = (uint32_t)tx_buf[tx_idx];
	d->des1 = 0;
	d->des2 = len & TDES2_B1L;
	__DMB();
	d->des3 = TDES3_OWN | TDES3_FD | TDES3_LD | (len & TDES3_FL);
	__DMB();
	tx_idx = (tx_idx + 1) % TX_DESC;
	ETH->DMACTDTPR = (uint32_t)&tx_desc[tx_idx];	// tail = next free (HAL does the same)
	stats.tx_frames++;
}

uint8_t *EthRxPeek(uint32_t *len) {
	EthDesc *d;

	while (ready) {
		d = &rx_desc[rx_idx];
		if (d->des3 & RDES3_OWN) return NULL;
		// One buffer holds a whole frame, so anything that isn't a clean FD+LD frame is junk
		if ((d->des3 & (RDES3_CTXT | RDES3_FD | RDES3_LD | RDES3_ES)) == (RDES3_FD | RDES3_LD)) {
			*len = d->des3 & RDES3_PL;
			stats.rx_frames++;
			return rx_buf[rx_idx];
		}
		stats.rx_errors++;
		EthRxRelease();
	}
	return NULL;
}

void EthRxRelease() {
	EthDesc *d = &rx_desc[rx_idx];

	d->des0 = (uint32_t)rx_buf[rx_idx];
	d->des1 = 0;
	d->des2 = 0;
	__DMB();
	d->des3 = RDES3_OWN | RDES3_BUF1V | RDES3_IOC;
	__DMB();
	ETH->DMACRDTPR = (uint32_t)d;	// tail = last one handed back (HAL ETH_UpdateDescriptor)
	rx_idx = (rx_idx + 1) % RX_DESC;
}

void EthGetStats(EthStats *s) {
	*s = stats;
	s->rx_overruns = rx_overruns;
}

// Only here to wake the main loop out of WFI. Frames are handled in NetTask().
void ETH_IRQHandler() {
	uint32_t sr = ETH->DMACSR;
	if (sr & ETH_DMACSR_RBU) rx_overruns++;
	ETH->DMACSR = sr;
}
