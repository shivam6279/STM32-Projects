#include "tmc.h"
#include "board.h"
#include "clock.h"
#include "irq_prio.h"
#include "stepgen.h"
#include "stm32h7xx.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Datagrams (datasheet 4.1), bytes go out LSB first, 32 bit data high byte first:
//   write   0x05, node, reg | 0x80, d3, d2, d1, d0, crc
//   read    0x05, node, reg, crc
//   reply   0x05, 0xFF, reg, d3, d2, d1, d0, crc
// 0x05 = sync nibble 1010 (LSB first) + reserved 0000.
// The driver answers after SENDDELAY (8 bit times by default, up to 15*8).
// A gap of more than 63 bit times inside a datagram resets its receiver, and after any
// error it wants 12 bit times of idle line before the next datagram.

#define TMC_SYNC		0x05
#define TMC_MASTER		0xFF
#define IDLE_BITS		24		// idle before every datagram, > the 12 bit error recovery
#define ECHO_BITS		30		// a byte is 10 bits, so 3 bytes worth of slack
#define REPLY_BITS		200		// first reply byte: SENDDELAY max 15*8 bits + 1 byte + slack

// Current: I_RMS = (CS + 1) / 32 * VFS / (RSENSE + 20mohm) / sqrt(2)   (datasheet 9)
// VFS = 325mV (vsense = 0) or 180mV (vsense = 1), with I_scale_analog = 0 (internal reference,
// VREF isn't used on this board)
#define VFS_LOW			0.325f
#define VFS_HIGH		0.180f
#define R_INTERNAL		0.02f
#define IHOLDDELAY		4		// steps of 2^18 clocks per current decrement down to hold
#define SENDDELAY		2		// 3*8 bit times, the minimum with several nodes on a wire

#define DIAG_PORT		3		// EXTICR value for port D
#define DIAG_LINES		0xFF00	// EXTI lines 8-15

#define ON_PD8_15(n) (PIN_DIAG##n##_PORT_IDX == DIAG_PORT && PIN_DIAG##n##_NUM >= 8)
_Static_assert(ON_PD8_15(1) && ON_PD8_15(2) && ON_PD8_15(3) && ON_PD8_15(4) && ON_PD8_15(5) && ON_PD8_15(6) && ON_PD8_15(7) && ON_PD8_15(8),
	"DIAG pins expected on PD8-PD15");

static const uint8_t diag_pin[TMC_DRIVERS] = {
	PIN_DIAG1_NUM, PIN_DIAG2_NUM, PIN_DIAG3_NUM, PIN_DIAG4_NUM,
	PIN_DIAG5_NUM, PIN_DIAG6_NUM, PIN_DIAG7_NUM, PIN_DIAG8_NUM,
};

static USART_TypeDef *const bus[TMC_BANKS] = { UART5, UART7 };
static uint32_t baud = TMC_BAUD_DEFAULT;
static uint8_t line_drv[16];		// EXTI line -> driver
static volatile uint8_t pending;	// DIAG went high, not looked at yet
static volatile uint8_t ignored;
static bool booted = false;

// TODO real currents per axis. Until then low enough for the smallest motor here (NEMA 8)
TmcConfig tmc_config[TMC_DRIVERS] = {
	{ 0.2f, 0.1f, 16, true, false },
	{ 0.2f, 0.1f, 16, true, false },
	{ 0.2f, 0.1f, 16, true, false },
	{ 0.2f, 0.1f, 16, true, false },
	{ 0.2f, 0.1f, 16, true, false },
	{ 0.2f, 0.1f, 16, true, false },
	{ 0.2f, 0.1f, 16, true, false },
	{ 0.2f, 0.1f, 16, true, false },
};

// ---- UART ----

// DWT cycle counter is started by StepInit
static uint32_t BitCycles() {
	return CLOCK_SYSCLK_HZ / baud;
}

static void Idle(uint32_t bits) {
	uint32_t t = DWT->CYCCNT;
	while (DWT->CYCCNT - t < bits * BitCycles());
}

static bool GetByte(USART_TypeDef *u, uint8_t *c, uint32_t bits) {
	uint32_t t = DWT->CYCCNT, n = bits * BitCycles();
	while (!(u->ISR & USART_ISR_RXNE_RXFNE)) {
		if (DWT->CYCCNT - t > n) return false;
	}
	*c = u->RDR;
	return true;
}

// CRC8-ATM, x^8 + x^2 + x + 1, LSB first, from 0 (datasheet 4.2)
static uint8_t Crc(const uint8_t *d, uint8_t n) {
	uint8_t crc = 0, c, i, j;
	for (i = 0; i < n; i++) {
		c = d[i];
		for (j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (c & 1)) crc = (crc << 1) ^ 0x07;
			else crc <<= 1;
			c >>= 1;
		}
	}
	return crc;
}

// Half duplex: the receiver hears everything on the wire, our own bytes too. Each byte is
// checked as it comes back, so a stuck or shorted line shows up as TMC_NO_ECHO.
static uint8_t Send(USART_TypeDef *u, const uint8_t *d, uint8_t n) {
	uint8_t i, c;

	Idle(IDLE_BITS);
	while (u->ISR & USART_ISR_RXNE_RXFNE) (void)u->RDR;
	u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;

	for (i = 0; i < n; i++) {
		while (!(u->ISR & USART_ISR_TXE_TXFNF));
		u->TDR = d[i];
		if (!GetByte(u, &c, ECHO_BITS) || c != d[i]) return TMC_NO_ECHO;
	}
	return TMC_OK;
}

// Kernel clock = PCLK1 (USART28SEL reset value), 8N1, oversampling 16, no FIFO, half duplex
void TmcSetBaud(uint32_t b) {
	uint8_t i;

	baud = b;
	for (i = 0; i < TMC_BANKS; i++) {
		USART_TypeDef *u = bus[i];
		u->CR1 = 0;
		u->PRESC = 0;
		u->BRR = (CLOCK_PCLK1_HZ + baud / 2) / baud;
		u->CR2 = 0;
		u->CR3 = USART_CR3_HDSEL | USART_CR3_OVRDIS;
		u->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
	}
}

uint32_t TmcBaud() {
	return baud;
}

uint8_t TmcRead(uint8_t drv, uint8_t reg, uint32_t *val) {
	USART_TypeDef *u = bus[drv / TMC_NODES];
	uint8_t req[4], rep[8], i, err;

	req[0] = TMC_SYNC;
	req[1] = drv % TMC_NODES;
	req[2] = reg & 0x7F;
	req[3] = Crc(req, 3);
	err = Send(u, req, 4);
	if (err != TMC_OK) return err;

	if (!GetByte(u, &rep[0], REPLY_BITS)) return TMC_NO_REPLY;
	for (i = 1; i < 8; i++) {
		if (!GetByte(u, &rep[i], ECHO_BITS)) return TMC_BAD_REPLY;
	}
	if ((rep[0] & 0x0F) != TMC_SYNC || rep[1] != TMC_MASTER || rep[2] != req[2] || rep[7] != Crc(rep, 7)) {
		return TMC_BAD_REPLY;
	}
	*val = ((uint32_t)rep[3] << 24) | ((uint32_t)rep[4] << 16) | ((uint32_t)rep[5] << 8) | rep[6];
	return TMC_OK;
}

// No answer to a write. Check it landed with IFCNT
uint8_t TmcWrite(uint8_t drv, uint8_t reg, uint32_t val) {
	uint8_t d[8];

	d[0] = TMC_SYNC;
	d[1] = drv % TMC_NODES;
	d[2] = reg | 0x80;
	d[3] = val >> 24;
	d[4] = val >> 16;
	d[5] = val >> 8;
	d[6] = val;
	d[7] = Crc(d, 7);
	return Send(bus[drv / TMC_NODES], d, 8);
}

const char *TmcError(uint8_t err) {
	switch (err) {
		case TMC_OK:			return "ok";
		case TMC_NO_ECHO:		return "no echo (line stuck?)";
		case TMC_NO_REPLY:		return "no reply";
		case TMC_BAD_REPLY:		return "bad reply";
		case TMC_NOT_ACCEPTED:	return "settings didn't stick";
	}
	return "?";
}

// ---- settings ----

static float FullScale(bool vsense) {
	return (vsense ? VFS_HIGH : VFS_LOW) / (TMC_RSENSE + R_INTERNAL) / sqrtf(2);
}

// vsense = 1 gives finer steps, use it whenever the run current fits in its range
bool TmcVsenseFor(float run) {
	return run <= FullScale(true);
}

float TmcCurrent(float amps, bool vsense, uint8_t *cs) {
	float full = FullScale(vsense);
	int32_t c = lroundf(amps / full * 32) - 1;
	if (c < 0) c = 0;
	if (c > 31) c = 31;
	*cs = c;
	return (c + 1) * full / 32;
}

// MRES: 256 microsteps = 0, 128 = 1 ... full step = 8
static uint32_t Mres(uint16_t ustep) {
	uint32_t m = 0;
	while ((256 >> m) > ustep && m < 8) m++;
	return m;
}

// Everything in tmc_config[drv], plus what this board needs over UART:
//   SENDDELAY >= 2      several nodes per wire
//   pdn_disable         PDN_UART is the UART, not the standstill pin
//   mstep_reg_select    MS1/MS2 are the address, microsteps come from MRES
//   I_scale_analog = 0  internal current reference, VREF isn't used
// Then GSTAT cleared, IFCNT checked (5 writes) and GCONF / CHOPCONF read back
uint8_t TmcSetup(uint8_t drv) {
	TmcConfig *c = &tmc_config[drv];
	uint32_t c0, c1, gconf, chop, v;
	uint8_t e, irun, ihold;
	bool vsense = TmcVsenseFor(c->run);

	if ((e = TmcRead(drv, TMC_IFCNT, &c0)) != TMC_OK) return e;
	if ((e = TmcRead(drv, TMC_GCONF, &gconf)) != TMC_OK) return e;
	if ((e = TmcRead(drv, TMC_CHOPCONF, &chop)) != TMC_OK) return e;

	gconf &= ~(TMC_GCONF_I_SCALE_ANALOG | TMC_GCONF_INTERNAL_RSENSE | TMC_GCONF_EN_SPREADCYCLE);
	gconf |= TMC_GCONF_PDN_DISABLE | TMC_GCONF_MSTEP_REG_SELECT;
	if (c->spread) gconf |= TMC_GCONF_EN_SPREADCYCLE;

	// Chopper timing (TOFF, HSTRT, HEND, TBL) stays at the driver's defaults
	chop &= ~(TMC_CHOP_MRES | TMC_CHOP_INTPOL | TMC_CHOP_VSENSE);
	chop |= Mres(c->ustep) << TMC_CHOP_MRES_POS;
	if (c->intpol) chop |= TMC_CHOP_INTPOL;
	if (vsense) chop |= TMC_CHOP_VSENSE;

	TmcCurrent(c->run, vsense, &irun);
	TmcCurrent(c->hold, vsense, &ihold);

	TmcWrite(drv, TMC_NODECONF, SENDDELAY << 8);
	TmcWrite(drv, TMC_GCONF, gconf);
	TmcWrite(drv, TMC_CHOPCONF, chop);
	TmcWrite(drv, TMC_IHOLD_IRUN, ihold | (irun << 8) | (IHOLDDELAY << 16));
	TmcWrite(drv, TMC_GSTAT, TMC_GSTAT_RESET | TMC_GSTAT_DRV_ERR | TMC_GSTAT_UV_CP);

	if ((e = TmcRead(drv, TMC_IFCNT, &c1)) != TMC_OK) return e;
	if (((c1 - c0) & 0xFF) != 5) return TMC_NOT_ACCEPTED;
	if ((e = TmcRead(drv, TMC_GCONF, &v)) != TMC_OK) return e;
	if ((v & 0x3FF) != (gconf & 0x3FF)) return TMC_NOT_ACCEPTED;
	if ((e = TmcRead(drv, TMC_CHOPCONF, &v)) != TMC_OK) return e;
	if (v != chop) return TMC_NOT_ACCEPTED;
	return TMC_OK;
}

// ---- faults ----

uint8_t TmcDiagPins() {
	uint32_t idr = GPIOD->IDR;
	uint8_t i, v = 0;
	for (i = 0; i < TMC_DRIVERS; i++) v |= ((idr >> diag_pin[i]) & 1) << i;
	return v;
}

uint8_t TmcIgnored() {
	return ignored;
}

void TmcIgnore(uint8_t mask) {
	ignored = mask;
}

void TmcFaultText(uint32_t gstat, uint32_t drv, char *buf, uint32_t len) {
	static const struct { uint32_t bit; bool in_gstat; const char *name; } f[] = {
		{ TMC_GSTAT_RESET, true, "reset (power lost?)" },
		{ TMC_GSTAT_DRV_ERR, true, "drv_err" },
		{ TMC_GSTAT_UV_CP, true, "uv_cp (charge pump undervoltage)" },
		{ TMC_DRV_OT, false, "ot (overtemperature)" },
		{ TMC_DRV_OTPW, false, "otpw (overtemperature warning)" },
		{ TMC_DRV_S2GA, false, "s2ga (short to GND, A)" },
		{ TMC_DRV_S2GB, false, "s2gb (short to GND, B)" },
		{ TMC_DRV_S2VSA, false, "s2vsa (low side short, A)" },
		{ TMC_DRV_S2VSB, false, "s2vsb (low side short, B)" },
		{ TMC_DRV_OLA, false, "ola (open load, A)" },
		{ TMC_DRV_OLB, false, "olb (open load, B)" },
	};
	uint32_t i, n = 0;

	buf[0] = '\0';
	for (i = 0; i < sizeof(f) / sizeof(f[0]); i++) {
		if (!((f[i].in_gstat ? gstat : drv) & f[i].bit)) continue;
		n += snprintf(buf + n, len > n ? len - n : 0, "%s%s", n ? ", " : "", f[i].name);
	}
	if (n == 0) snprintf(buf, len, "none");
}

// Before the drivers get enabled: every driver we use must still have its settings (GSTAT reset
// = it lost power since, so it's back to full current) and no fault
bool TmcReady() {
	uint8_t d, e;
	uint32_t g;
	bool ok = true;
	char txt[96];

	for (d = 0; d < TMC_DRIVERS; d++) {
		if ((ignored >> d) & 1) continue;
		e = TmcRead(d, TMC_GSTAT, &g);
		if (e == TMC_OK && (g & TMC_GSTAT_RESET)) {
			e = TmcSetup(d);
			if (e == TMC_OK) e = TmcRead(d, TMC_GSTAT, &g);
		}
		if (e != TMC_OK) {
			printf("Error:Driver %d %s\n", d, TmcError(e));
			ok = false;
		} else if (g & (TMC_GSTAT_DRV_ERR | TMC_GSTAT_UV_CP) || (TmcDiagPins() >> d) & 1) {
			TmcFaultText(g, 0, txt, sizeof(txt));
			printf("Error:Driver %d fault: %s\n", d, txt);
			ok = false;
		}
	}
	return ok;
}

// Rising DIAG: stop stepping now, the rest happens in TmcTask
void TmcDiagIsr() {
	uint32_t pr = EXTI_D1->PR1 & DIAG_LINES;
	uint8_t line, hit = 0;

	EXTI_D1->PR1 = pr;
	for (line = 8; line < 16; line++) {
		if (pr & (1 << line)) hit |= 1 << line_drv[line];
	}
	hit &= ~ignored;
	if (hit) {
		pending |= hit;
		StepDriverFault();
	}
}

void EXTI15_10_IRQHandler() {
	TmcDiagIsr();
}

void TmcInit() {
	uint8_t i;

	RCC->APB1LENR |= RCC_APB1LENR_UART5EN | RCC_APB1LENR_UART7EN;
	(void)RCC->APB1LENR;

	// The line has to idle high (datasheet 4.1.1) and the driver drives it when it answers, so
	// open drain + pull-up: we can never fight it. Pins come from the table as AF push-pull
	PIN_UART5_TX_PORT->OTYPER |= PIN_UART5_TX_MASK;
	PIN_UART7_TX_PORT->OTYPER |= PIN_UART7_TX_MASK;
	PIN_UART5_TX_PORT->PUPDR = (PIN_UART5_TX_PORT->PUPDR & ~(3 << (PIN_UART5_TX_NUM * 2))) | (1 << (PIN_UART5_TX_NUM * 2));
	PIN_UART7_TX_PORT->PUPDR = (PIN_UART7_TX_PORT->PUPDR & ~(3 << (PIN_UART7_TX_NUM * 2))) | (1 << (PIN_UART7_TX_NUM * 2));
	TmcSetBaud(baud);

	// DIAG1-8: EXTI lines 8-15 on port D, rising edge. EXTI9_5 is enabled by the buttons
	for (i = 8; i < 16; i++) {
		SYSCFG->EXTICR[i / 4] = (SYSCFG->EXTICR[i / 4] & ~(0xF << ((i % 4) * 4))) | (DIAG_PORT << ((i % 4) * 4));
	}
	for (i = 0; i < TMC_DRIVERS; i++) line_drv[diag_pin[i]] = i;
	EXTI->RTSR1 |= DIAG_LINES;
	EXTI_D1->PR1 = DIAG_LINES;
	EXTI_D1->IMR1 |= DIAG_LINES;
	NVIC_SetPriority(EXTI15_10_IRQn, IRQ_PRIO_DIAG);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Read what went wrong, report it, drivers off. Positions can't be trusted after this, so it
// stays stopped until M999
static void HandleFaults(uint8_t mask) {
	uint8_t d, e;
	uint32_t g, s;
	char txt[160];

	for (d = 0; d < TMC_DRIVERS; d++) {
		if (!((mask >> d) & 1)) continue;
		e = TmcRead(d, TMC_GSTAT, &g);
		if (e == TMC_OK) e = TmcRead(d, TMC_DRV_STATUS, &s);
		if (e != TMC_OK) {
			printf("Error:Driver %d DIAG high, can't read it: %s\n", d, TmcError(e));
			continue;
		}
		TmcFaultText(g, s, txt, sizeof(txt));
		printf("Error:Driver %d DIAG high: %s\n", d, txt);
		if (g & TMC_GSTAT_RESET) {
			e = TmcSetup(d);
			printf("%s:Driver %d settings written again: %s\n", e == TMC_OK ? "echo" : "Error", d, TmcError(e));
		}
	}
	BoardStepperEnable(false); // ENN high also clears the short flags
	printf("Error:Steppers stopped and disabled, send M999 to resume\n");
}

void TmcTask() {
	uint8_t d, e, p, n = 0;

	if (!booted) {
		booted = true;
		for (d = 0; d < TMC_DRIVERS; d++) {
			if ((e = TmcSetup(d)) == TMC_OK) n++;
			else printf("Error:Driver %d setup: %s\n", d, TmcError(e));
		}
		printf("echo:%d of %d TMC2209 drivers set up\n", n, TMC_DRIVERS);
		// Already high at boot = no edge to catch
		p = TmcDiagPins() & ~ignored;
		if (p) {
			pending |= p;
			StepDriverFault();
		}
	}

	__disable_irq();
	p = pending;
	pending = 0;
	__enable_irq();
	if (p) HandleFaults(p);
}
