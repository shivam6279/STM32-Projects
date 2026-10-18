#include "fdcan.h"
#include "clock.h"
#include "irq_prio.h"
#include "stm32h7xx.h"

#include <string.h>

// Bit timing, 80MHz kernel clock (CLOCK_FDCAN_HZ)
// Nominal 1 Mbit/s: 80 tq = 1 + 63 + 16, sample point 80%
// Data 5 Mbit/s:    16 tq = 1 + 11 + 4,  sample point 75%
#define NOM_BRP		1
#define NOM_SEG1	63
#define NOM_SEG2	16
#define NOM_SJW		16
#define DATA_BRP	1
#define DATA_SEG1	11
#define DATA_SEG2	4
#define DATA_SJW	4
// Transmitter delay compensation: TJA1051 loop delay is up to 250ns (datasheet table 8), longer
// than a 200ns data bit, so the secondary sample point is needed. Put it at the sample point.
#define TDC_OFFSET	(DATA_BRP * (1 + DATA_SEG1))

_Static_assert(CLOCK_FDCAN_HZ / NOM_BRP / (1 + NOM_SEG1 + NOM_SEG2) == 1000000, "nominal isn't 1 Mbit/s");
_Static_assert(CLOCK_FDCAN_HZ / DATA_BRP / (1 + DATA_SEG1 + DATA_SEG2) == 5000000, "data isn't 5 Mbit/s");

// Message RAM: 10KB (2560 words) at SRAMCAN_BASE, shared by FDCAN1/2/3 (HAL FDCAN_MESSAGE_RAM_SIZE).
// Same layout for each bus, FDCAN3 isn't pinned out so it gets nothing.
#define RAM_WORDS		(0x2800 / 4)
#define SID_NBR			32		// standard filters, 1 word each (none used yet)
#define XID_NBR			16		// extended filters, 2 words each
#define RXF0_NBR		32		// RX FIFO0 elements (max 64)
#define TXF_NBR			16		// TX FIFO elements (max 32)
#define ELEM_WORDS		18		// 2 header + 64 data bytes
#define BUS_WORDS		(SID_NBR + XID_NBR * 2 + RXF0_NBR * ELEM_WORDS + TXF_NBR * ELEM_WORDS)

#define SID_OFF			0
#define XID_OFF			(SID_OFF + SID_NBR)
#define RXF0_OFF		(XID_OFF + XID_NBR * 2)
#define TXF_OFF			(RXF0_OFF + RXF0_NBR * ELEM_WORDS)

_Static_assert(BUS_WORDS * CAN_BUSES <= RAM_WORDS, "FDCAN message RAM overflow");

// Element header bits (HAL FDCAN_ELEMENT_MASK_*)
#define E_XTD		(1u << 30)
#define E_STDID_POS	18
#define E_DLC_POS	16
#define E_BRS		(1u << 20)
#define E_FDF		(1u << 21)

#define RXQ_LEN		32	// software queue per bus, power of 2

static FDCAN_GlobalTypeDef *const can[CAN_BUSES] = { FDCAN1, FDCAN2 };
static const IRQn_Type can_irq[CAN_BUSES] = { FDCAN1_IT0_IRQn, FDCAN2_IT0_IRQn };

static CanFrame rxq[CAN_BUSES][RXQ_LEN];
static volatile uint32_t rxq_head[CAN_BUSES], rxq_tail[CAN_BUSES];
static volatile uint32_t rx_count[CAN_BUSES], tx_count[CAN_BUSES], rx_dropped[CAN_BUSES];

static const uint8_t dlc_len[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

static uint32_t LenToDlc(uint8_t len) {
	uint32_t dlc;
	for (dlc = 0; dlc < 15 && dlc_len[dlc] < len; dlc++);
	return dlc;
}

uint8_t CanRoundLen(uint8_t len) {
	return dlc_len[LenToDlc(len)];
}

static volatile uint32_t *Ram(uint8_t bus) {
	return (volatile uint32_t *)SRAMCAN_BASE + bus * BUS_WORDS;
}

static void Setup(uint8_t bus, bool loopback) {
	FDCAN_GlobalTypeDef *c = can[bus];
	uint32_t base = bus * BUS_WORDS; // word offset into the message RAM
	volatile uint32_t *ram = Ram(bus);
	uint32_t i;

	NVIC_DisableIRQ(can_irq[bus]);

	// Out of clock stop, into init with config access
	c->CCCR &= ~FDCAN_CCCR_CSR;
	while (c->CCCR & FDCAN_CCCR_CSA);
	c->CCCR |= FDCAN_CCCR_INIT;
	while (!(c->CCCR & FDCAN_CCCR_INIT));
	c->CCCR |= FDCAN_CCCR_CCE;

	// FD with bit rate switching, auto retransmit on
	c->CCCR &= ~(FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_DAR);
	c->CCCR |= FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE;
	if (loopback) {
		// internal loopback: TX pin stays recessive, frames come straight back
		c->CCCR |= FDCAN_CCCR_TEST | FDCAN_CCCR_MON;
		c->TEST |= FDCAN_TEST_LBCK;
	}

	c->NBTP = ((NOM_SJW - 1) << FDCAN_NBTP_NSJW_Pos) | ((NOM_BRP - 1) << FDCAN_NBTP_NBRP_Pos)
		| ((NOM_SEG1 - 1) << FDCAN_NBTP_NTSEG1_Pos) | ((NOM_SEG2 - 1) << FDCAN_NBTP_NTSEG2_Pos);
	c->DBTP = FDCAN_DBTP_TDC | ((DATA_BRP - 1) << FDCAN_DBTP_DBRP_Pos)
		| ((DATA_SEG1 - 1) << FDCAN_DBTP_DTSEG1_Pos) | ((DATA_SEG2 - 1) << FDCAN_DBTP_DTSEG2_Pos)
		| ((DATA_SJW - 1) << FDCAN_DBTP_DSJW_Pos);
	c->TDCR = (TDC_OFFSET << FDCAN_TDCR_TDCO_Pos) | (0 << FDCAN_TDCR_TDCF_Pos);

	// Message RAM layout, offsets in words
	c->SIDFC = ((base + SID_OFF) << FDCAN_SIDFC_FLSSA_Pos) | (0 << FDCAN_SIDFC_LSS_Pos);
	c->XIDFC = ((base + XID_OFF) << FDCAN_XIDFC_FLESA_Pos) | (0 << FDCAN_XIDFC_LSE_Pos);
	c->RXF0C = ((base + RXF0_OFF) << FDCAN_RXF0C_F0SA_Pos) | (RXF0_NBR << FDCAN_RXF0C_F0S_Pos);
	c->RXF1C = 0;
	c->RXBC = 0;
	c->RXESC = (7 << FDCAN_RXESC_F0DS_Pos);	// 64 byte data field
	c->TXBC = ((base + TXF_OFF) << FDCAN_TXBC_TBSA_Pos) | (TXF_NBR << FDCAN_TXBC_TFQS_Pos);	// TX FIFO mode
	c->TXESC = (7 << FDCAN_TXESC_TBDS_Pos);
	c->TXEFC = 0;
	for (i = 0; i < BUS_WORDS; i++) ram[i] = 0;

	// No filters yet: accept everything (std and ext) into FIFO0, reject remote frames
	c->GFC = (0 << FDCAN_GFC_ANFS_Pos) | (0 << FDCAN_GFC_ANFE_Pos) | (1 << FDCAN_GFC_RRFS_Pos) | (1 << FDCAN_GFC_RRFE_Pos);

	// New message in FIFO0 -> line 0
	c->IR = 0xFFFFFFFF;
	c->IE = FDCAN_IE_RF0NE;
	c->ILS = 0;
	c->ILE = FDCAN_ILE_EINT0;

	rxq_head[bus] = rxq_tail[bus] = 0;

	c->CCCR &= ~FDCAN_CCCR_INIT;
	while (c->CCCR & FDCAN_CCCR_INIT);

	NVIC_SetPriority(can_irq[bus], IRQ_PRIO_CAN);
	NVIC_EnableIRQ(can_irq[bus]);
}

void CanInit() {
	uint8_t i;

	ClockCanInit();
	RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
	(void)RCC->APB1HENR;

	for (i = 0; i < CAN_BUSES; i++) Setup(i, false);
}

void CanLoopback(uint8_t bus, bool on) {
	if (bus < CAN_BUSES) Setup(bus, on);
}

bool CanSend(uint8_t bus, const CanFrame *f) {
	FDCAN_GlobalTypeDef *c;
	volatile uint32_t *e;
	uint32_t idx, i, n, w;

	if (bus >= CAN_BUSES || f->len > (f->fd ? 64 : 8)) return false;
	c = can[bus];
	if (c->TXFQS & FDCAN_TXFQS_TFQF) return false;

	idx = (c->TXFQS >> FDCAN_TXFQS_TFQPI_Pos) & 0x1F;
	e = Ram(bus) + TXF_OFF + idx * ELEM_WORDS;
	e[0] = f->ext ? (E_XTD | (f->id & 0x1FFFFFFF)) : ((f->id & 0x7FF) << E_STDID_POS);
	e[1] = (LenToDlc(f->len) << E_DLC_POS) | (f->fd ? E_FDF : 0) | (f->fd && f->brs ? E_BRS : 0);

	// Data, padded with zeros up to the DLC length
	n = dlc_len[LenToDlc(f->len)];
	for (i = 0; i < n; i += 4) {
		w = 0;
		for (uint32_t b = 0; b < 4; b++) {
			if (i + b < f->len) w |= (uint32_t)f->data[i + b] << (8 * b);
		}
		e[2 + i / 4] = w;
	}

	c->TXBAR = 1 << idx;
	tx_count[bus]++;
	return true;
}

bool CanRead(uint8_t bus, CanFrame *f) {
	if (bus >= CAN_BUSES || rxq_head[bus] == rxq_tail[bus]) return false;
	*f = rxq[bus][rxq_tail[bus] & (RXQ_LEN - 1)];
	rxq_tail[bus]++;
	return true;
}

void CanGetStatus(uint8_t bus, CanStatus *s) {
	FDCAN_GlobalTypeDef *c = can[bus];
	uint32_t psr = c->PSR, ecr = c->ECR;

	s->lec = (psr >> FDCAN_PSR_LEC_Pos) & 7;
	s->dlec = (psr >> FDCAN_PSR_DLEC_Pos) & 7;
	s->passive = (psr >> FDCAN_PSR_EP_Pos) & 1;
	s->warning = (psr >> FDCAN_PSR_EW_Pos) & 1;
	s->bus_off = (psr >> FDCAN_PSR_BO_Pos) & 1;
	s->tdcv = (psr >> FDCAN_PSR_TDCV_Pos) & 0x7F;
	s->tec = (ecr >> FDCAN_ECR_TEC_Pos) & 0xFF;
	s->rec = (ecr >> FDCAN_ECR_REC_Pos) & 0x7F;
	s->rx_count = rx_count[bus];
	s->tx_count = tx_count[bus];
	s->rx_dropped = rx_dropped[bus];
}

// Drain FIFO0 into the software queue
static void RxIsr(uint8_t bus) {
	FDCAN_GlobalTypeDef *c = can[bus];
	volatile uint32_t *e;
	uint32_t idx, r0, r1, i;
	CanFrame *f;

	c->IR = FDCAN_IR_RF0N;
	while ((c->RXF0S >> FDCAN_RXF0S_F0FL_Pos) & 0x7F) {
		idx = (c->RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 0x3F;
		e = Ram(bus) + RXF0_OFF + idx * ELEM_WORDS;
		r0 = e[0];
		r1 = e[1];

		if (rxq_head[bus] - rxq_tail[bus] < RXQ_LEN) {
			f = &rxq[bus][rxq_head[bus] & (RXQ_LEN - 1)];
			f->ext = (r0 & E_XTD) != 0;
			f->id = f->ext ? (r0 & 0x1FFFFFFF) : ((r0 >> E_STDID_POS) & 0x7FF);
			f->fd = (r1 & E_FDF) != 0;
			f->brs = (r1 & E_BRS) != 0;
			f->len = dlc_len[(r1 >> E_DLC_POS) & 0xF];
			if (!f->fd && f->len > 8) f->len = 8;
			for (i = 0; i < f->len; i += 4) {
				uint32_t w = e[2 + i / 4];
				memcpy(&f->data[i], &w, (f->len - i) < 4 ? (f->len - i) : 4);
			}
			rxq_head[bus]++;
			rx_count[bus]++;
		} else {
			rx_dropped[bus]++;
		}
		c->RXF0A = idx;
	}
}

void FDCAN1_IT0_IRQHandler() {
	RxIsr(0);
}

void FDCAN2_IT0_IRQHandler() {
	RxIsr(1);
}
