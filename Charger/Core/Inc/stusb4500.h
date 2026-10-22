// STUSB4500 USB-PD sink status reader. Shares I2C1 with the BQ25798 via the
// generic BQ_BusRead/BQ_BusWrite helpers in bq25798.c.
//
// The chip is a STANDALONE sink: its sink PDO profiles live in NVM and it
// negotiates with no host involvement. This module is read-only -- it exists so
// the firmware can report (and eventually react to) what was actually
// negotiated, not to drive the negotiation. NVM programming is deliberately not
// implemented here; that is a one-time bench operation.
//
// NOTE: the STUSB4500 is powered from VBUS, so it only answers while USB is
// plugged in. Every call here fails harmlessly on battery.
#ifndef STUSB4500_H
#define STUSB4500_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bq25798.h"      // bq_status_t + BQ_BusRead/BQ_BusWrite

// 7-bit I2C address. 0x28 with ADDR0/ADDR1 both low (0x29/0x2A/0x2B otherwise).
#define STUSB_ADDR7             0x28u

// ---- Register addresses ---------------------------------------------------
// The register map is NOT in the datasheet (DS12499); it is published
// separately as "STUSB45 Register Map" and in UM2650.
#define STUSB_REG_TYPEC_REV_LOW    0x06u
#define STUSB_REG_USBPD_REV_LOW    0x08u
#define STUSB_REG_PORT_STATUS_0    0x0Du   // transition/alert flags
#define STUSB_REG_PORT_STATUS_1    0x0Eu   // current state (bit0 = attached)
#define STUSB_REG_TYPEC_MON_0      0x0Fu
#define STUSB_REG_TYPEC_MON_1      0x10u
#define STUSB_REG_CC_STATUS        0x11u
#define STUSB_REG_DPM_PDO_NUMB     0x70u   // how many sink PDOs are active (1..3)
#define STUSB_REG_DPM_SNK_PDO1     0x85u   // 4 bytes each, LE; PDO2 = +4, PDO3 = +8
#define STUSB_REG_RDO_STATUS       0x91u   // 4 bytes, LE: the negotiated RDO

#define STUSB_PORT1_ATTACHED       (1u << 0)   // REG0E bit0

// A decoded USB-PD Fixed Supply object (sink PDO or the negotiated contract).
typedef struct {
	uint32_t raw;            // the 32-bit object as read
	uint16_t voltage_mV;     // PDO only: B19:10, 50 mV/LSB
	uint16_t current_mA;     // PDO: B9:0 max current, 10 mA/LSB
} stusb_pdo_t;

typedef struct {
	uint8_t     attached;        // PORT_STATUS_1 bit0
	uint8_t     port_status[2];  // raw REG0D/REG0E
	uint8_t     typec_mon[2];    // raw REG0F/REG10
	uint8_t     cc_status;       // raw REG11
	uint8_t     pdo_count;       // DPM_PDO_NUMB (1..3)
	uint8_t     rdo_object_pos;  // RDO B30:28 -- which source PDO was taken
	uint16_t    rdo_op_mA;       // RDO B19:10, 10 mA/LSB
	uint16_t    rdo_max_mA;      // RDO B9:0,   10 mA/LSB
	uint32_t    rdo_raw;
	stusb_pdo_t pdo[3];          // the sink PDOs currently loaded from NVM
	uint8_t     pdo_valid;       // how many entries of pdo[] were read
} stusb_status_t;

/* Probe the device: returns BQ_OK only if it ACKs on the bus. Call this before
 * anything else; on battery (no VBUS) it will fail, which is expected. */
bq_status_t STUSB4500_Probe(void);

// Full status snapshot. Partial reads leave the rest of *out zeroed.
bq_status_t STUSB4500_ReadStatus(stusb_status_t *out);

/* Negotiated contract only -- the cheap call for the 1 Hz loop. voltage_mV is
 * taken from the sink PDO the RDO selected (the RDO itself carries no voltage).
 * Returns BQ_BADID if the RDO names an object position we have no PDO for. */
bq_status_t STUSB4500_ReadContract(uint16_t *voltage_mV, uint16_t *current_mA);

// Print a one-line summary of *st over the debug UART.
void STUSB4500_PrintStatus(const stusb_status_t *st);

#ifdef __cplusplus
}
#endif
#endif // STUSB4500_H
