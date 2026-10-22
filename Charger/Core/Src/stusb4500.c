// STUSB4500 USB-PD sink status reader. See stusb4500.h.

#include "stusb4500.h"
#include <stdio.h>
#include <string.h>

/* USB-PD Fixed Supply PDO (sink), 32-bit:
 *   B31:30 = 00 (fixed supply)
 *   B19:10 = voltage, 50 mV/LSB
 *   B9:0   = operational current, 10 mA/LSB
 * USB-PD Fixed Request Data Object (RDO), 32-bit:
 *   B30:28 = object position of the source PDO that was accepted (1-based)
 *   B19:10 = operating current, 10 mA/LSB
 *   B9:0   = max operating current, 10 mA/LSB
 * The RDO carries no voltage; the voltage is whatever the selected PDO says. */
#define PDO_VOLT_SHIFT      10u
#define PDO_VOLT_MASK       0x3FFu
#define PDO_VOLT_LSB_mV     50u
#define PDO_CURR_MASK       0x3FFu
#define PDO_CURR_LSB_mA     10u
#define RDO_OBJPOS_SHIFT    28u
#define RDO_OBJPOS_MASK     0x7u

// Read 4 consecutive bytes as a little-endian u32.
static bq_status_t read_u32(uint8_t reg, uint32_t *out) {
	uint8_t b[4];
	bq_status_t st = BQ_BusRead(STUSB_ADDR7, reg, b, 4u);
	if (st != BQ_OK) {
		return st;
	}
	*out = (uint32_t)b[0] | ((uint32_t)b[1] << 8)
		 | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
	return BQ_OK;
}

static void decode_pdo(uint32_t raw, stusb_pdo_t *out) {
	out->raw        = raw;
	out->voltage_mV = (uint16_t)(((raw >> PDO_VOLT_SHIFT) & PDO_VOLT_MASK)
								 * PDO_VOLT_LSB_mV);
	out->current_mA = (uint16_t)((raw & PDO_CURR_MASK) * PDO_CURR_LSB_mA);
}

bq_status_t STUSB4500_Probe(void) {
	/* No magic ID constant is needed (and DEVICE_ID is not in the public
	 * register map): if the part is powered it ACKs its address, and if VBUS is
	 * down the read NACKs. That is exactly the distinction we care about. */
	uint8_t v;
	return BQ_BusRead(STUSB_ADDR7, STUSB_REG_PORT_STATUS_1, &v, 1u);
}

bq_status_t STUSB4500_ReadStatus(stusb_status_t *out) {
	bq_status_t st;

	memset(out, 0, sizeof(*out));

	// Status block REG0D..REG11 is contiguous, so take it in one burst.
	uint8_t blk[5];
	st = BQ_BusRead(STUSB_ADDR7, STUSB_REG_PORT_STATUS_0, blk, sizeof(blk));
	if (st != BQ_OK) {
		return st;
	}
	out->port_status[0] = blk[0];
	out->port_status[1] = blk[1];
	out->typec_mon[0]   = blk[2];
	out->typec_mon[1]   = blk[3];
	out->cc_status      = blk[4];
	out->attached = (uint8_t)((blk[1] & STUSB_PORT1_ATTACHED) ? 1u : 0u);

	st = BQ_BusRead(STUSB_ADDR7, STUSB_REG_DPM_PDO_NUMB, &out->pdo_count, 1u);
	if (st != BQ_OK) {
		return st;
	}
	out->pdo_count &= 0x3u;                  // 1..3 active sink PDOs
	if (out->pdo_count > 3u) { out->pdo_count = 3u; }

	// The sink PDOs currently loaded from NVM (4 bytes each, back to back).
	for (uint8_t i = 0u; i < out->pdo_count; i++) {
		uint32_t raw;
		st = read_u32((uint8_t)(STUSB_REG_DPM_SNK_PDO1 + (i * 4u)), &raw);
		if (st != BQ_OK) {
			return st;
		}
		decode_pdo(raw, &out->pdo[i]);
		out->pdo_valid = (uint8_t)(i + 1u);
	}

	st = read_u32(STUSB_REG_RDO_STATUS, &out->rdo_raw);
	if (st != BQ_OK) {
		return st;
	}
	out->rdo_object_pos = (uint8_t)((out->rdo_raw >> RDO_OBJPOS_SHIFT)
									& RDO_OBJPOS_MASK);
	out->rdo_op_mA  = (uint16_t)(((out->rdo_raw >> PDO_VOLT_SHIFT) & PDO_CURR_MASK)
								 * PDO_CURR_LSB_mA);
	out->rdo_max_mA = (uint16_t)((out->rdo_raw & PDO_CURR_MASK) * PDO_CURR_LSB_mA);
	return BQ_OK;
}

bq_status_t STUSB4500_ReadContract(uint16_t *voltage_mV, uint16_t *current_mA) {
	stusb_status_t st_buf;
	bq_status_t st = STUSB4500_ReadStatus(&st_buf);
	if (st != BQ_OK) {
		return st;
	}
	/* Object position is 1-based and indexes the SOURCE capabilities, but the
	 * sink PDO at the same position is the one we asked to be matched, so its
	 * voltage is the contract voltage. Position 0 means nothing negotiated. */
	if (st_buf.rdo_object_pos == 0u ||
		st_buf.rdo_object_pos > st_buf.pdo_valid) {
		return BQ_BADID;
	}
	*voltage_mV = st_buf.pdo[st_buf.rdo_object_pos - 1u].voltage_mV;
	*current_mA = st_buf.rdo_op_mA;
	return BQ_OK;
}

void STUSB4500_PrintStatus(const stusb_status_t *st) {
	printf("PD: attach=%u PORT=%02X %02X MON=%02X %02X CC=%02X pdos=%u\n",
		   st->attached, st->port_status[0], st->port_status[1],
		   st->typec_mon[0], st->typec_mon[1], st->cc_status, st->pdo_count);
	for (uint8_t i = 0u; i < st->pdo_valid; i++) {
		printf("PD: SNK_PDO%u = %umV %umA (raw %08lX)\n",
			   (unsigned)(i + 1u), st->pdo[i].voltage_mV, st->pdo[i].current_mA,
			   (unsigned long)st->pdo[i].raw);
	}
	if (st->rdo_object_pos == 0u) {
		printf("PD: no contract (RDO %08lX)\n", (unsigned long)st->rdo_raw);
	} else {
		uint16_t v = (st->rdo_object_pos <= st->pdo_valid)
				   ? st->pdo[st->rdo_object_pos - 1u].voltage_mV : 0u;
		printf("PD: contract obj=%u %umV op=%umA max=%umA (raw %08lX)\n",
			   st->rdo_object_pos, v, st->rdo_op_mA, st->rdo_max_mA,
			   (unsigned long)st->rdo_raw);
	}
}
