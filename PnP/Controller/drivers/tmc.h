#ifndef _TMC_H_
#define _TMC_H_

#include <stdbool.h>
#include <inttypes.h>

// TMC2209 stepper drivers (datasheet rev 1.09). Two banks of 4 on a single wire UART each:
//   bank 1: UART5_TX on PC12 (net UART_BANK1), drivers 0-3
//   bank 2: UART7_TX on PB4  (net UART_BANK2), drivers 4-7
// Driver n = DIR(n+1)/STEP(n+1)/DIAG(n+1) = axis n. Bank n / 4, node address n % 4 (MS1/MS2).
// The UART runs half duplex on the TX pin. Polled and blocking: a register read takes about
// 1ms at 115200, so it's for setup, faults and diags, never in the motion path.
//
// DIAG1-8 (PD8-15) go high on a driver error (datasheet 15.4): power-on reset, charge pump
// undervoltage, overtemperature, short. The rising edge stops the steppers in the interrupt,
// TmcTask reads the driver over UART, reports, and turns the drivers off.

#define TMC_DRIVERS			8
#define TMC_BANKS			2
#define TMC_NODES			4
#define TMC_BAUD_DEFAULT	115200	// datasheet: 9000 to fCLK/16
#define TMC_RSENSE			0.11f	// ohms, BRA/BRB
#define TMC_IRMS_MAX		1.4f	// A RMS, datasheet design guideline per coil

// Registers
#define TMC_GCONF			0x00
#define TMC_GSTAT			0x01
#define TMC_IFCNT			0x02	// +1 for every write the driver accepted
#define TMC_NODECONF		0x03	// SENDDELAY in bits 11..8, >= 2 with several nodes on a wire
#define TMC_IOIN			0x06
#define TMC_IHOLD_IRUN		0x10	// write only
#define TMC_CHOPCONF		0x6C
#define TMC_DRV_STATUS		0x6F

// GCONF
#define TMC_GCONF_I_SCALE_ANALOG	(1 << 0)
#define TMC_GCONF_INTERNAL_RSENSE	(1 << 1)
#define TMC_GCONF_EN_SPREADCYCLE	(1 << 2)
#define TMC_GCONF_PDN_DISABLE		(1 << 6)
#define TMC_GCONF_MSTEP_REG_SELECT	(1 << 7)

// GSTAT
#define TMC_GSTAT_RESET		(1 << 0)
#define TMC_GSTAT_DRV_ERR	(1 << 1)
#define TMC_GSTAT_UV_CP		(1 << 2)

// CHOPCONF
#define TMC_CHOP_VSENSE		(1 << 17)
#define TMC_CHOP_MRES_POS	24
#define TMC_CHOP_MRES		(0xFu << TMC_CHOP_MRES_POS)
#define TMC_CHOP_INTPOL		(1u << 28)

// DRV_STATUS
#define TMC_DRV_OTPW		(1 << 0)
#define TMC_DRV_OT			(1 << 1)
#define TMC_DRV_S2GA		(1 << 2)
#define TMC_DRV_S2GB		(1 << 3)
#define TMC_DRV_S2VSA		(1 << 4)
#define TMC_DRV_S2VSB		(1 << 5)
#define TMC_DRV_OLA			(1 << 6)
#define TMC_DRV_OLB			(1 << 7)
#define TMC_DRV_T120		(1 << 8)
#define TMC_DRV_T143		(1 << 9)
#define TMC_DRV_T150		(1 << 10)
#define TMC_DRV_T157		(1 << 11)
#define TMC_DRV_CS_ACTUAL(s)	(((s) >> 16) & 0x1F)
#define TMC_DRV_STEALTH		(1u << 30)
#define TMC_DRV_STST		(1u << 31)

// IOIN
#define TMC_IOIN_ENN		(1 << 0)
#define TMC_IOIN_MS1		(1 << 2)
#define TMC_IOIN_MS2		(1 << 3)
#define TMC_IOIN_DIAG		(1 << 4)
#define TMC_IOIN_PDN_UART	(1 << 6)
#define TMC_IOIN_STEP		(1 << 7)
#define TMC_IOIN_SPREAD		(1 << 8)
#define TMC_IOIN_DIR		(1 << 9)
#define TMC_VERSION(ioin)	((ioin) >> 24)	// 0x21

// Results
#define TMC_OK				0
#define TMC_NO_ECHO			1	// our own bytes didn't come back off the wire: line stuck / shorted
#define TMC_NO_REPLY		2	// nobody answered at that address
#define TMC_BAD_REPLY		3	// answer with a bad sync / address / register / CRC
#define TMC_NOT_ACCEPTED	4	// IFCNT says writes went missing, or a read back didn't match

// What we want each driver to be set to. Written by TmcSetup, kept in RAM (FRAM TODO)
typedef struct {
	float run;			// A RMS
	float hold;			// A RMS, standstill
	uint16_t ustep;		// 1, 2, 4 ... 256
	bool intpol;		// interpolate to 256 microsteps
	bool spread;		// SpreadCycle instead of StealthChop
} TmcConfig;

extern TmcConfig tmc_config[TMC_DRIVERS];

void TmcInit();
void TmcTask();							// main loop: first time setup, DIAG faults
void TmcDiagIsr();						// EXTI lines 8-15
void TmcSetBaud(uint32_t baud);
uint32_t TmcBaud();
uint8_t TmcRead(uint8_t drv, uint8_t reg, uint32_t *val);
uint8_t TmcWrite(uint8_t drv, uint8_t reg, uint32_t val);
uint8_t TmcSetup(uint8_t drv);			// write tmc_config[drv] and check it landed
bool TmcReady();						// every driver not ignored: set up, no fault. Before enabling
uint8_t TmcDiagPins();					// DIAG levels, bit n = driver n
uint8_t TmcIgnored();
void TmcIgnore(uint8_t mask);			// drivers left out of the checks (no motor fitted / known bad)
const char *TmcError(uint8_t err);
void TmcFaultText(uint32_t gstat, uint32_t drv, char *buf, uint32_t len);	// "uv_cp, s2ga" etc

// Current scale for a current in A RMS: picks vsense, fills IRUN-style CS (0-31). Returns the
// current that really gives
float TmcCurrent(float amps, bool vsense, uint8_t *cs);
bool TmcVsenseFor(float run);

#endif
