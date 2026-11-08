#include "settings.h"
#include "bootflag.h"
#include "flash.h"
#include "home.h"
#include "jog.h"
#include "motion.h"
#include "pad.h"
#include "probe.h"
#include "pwm.h"
#include "stepgen.h"
#include "tmc.h"
#include "stm32h7xx.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define MAGIC		0x31544553	// "SET1"
#define SLOTS		(FLASH_SECTOR / SETTINGS_SLOT)
#define SLOT_WORDS	(SETTINGS_SLOT / 4)
#define HDR_WORDS	(FLASH_WORD / 4)

// Everything that gets saved. Change it = bump SETTINGS_LAYOUT, or old saves load as garbage
typedef struct {
	MotionAxis axes[MOTION_AXES];
	uint8_t invert;
	float accel;
	TmcConfig tmc[TMC_DRIVERS];
	uint8_t tmc_ignored;
	HomeAxis home[MOTION_AXES];
	float pad_xy, pad_xy_fast, pad_z;
	PwmProfile pwm[PWM_CHANNELS];	// layout 3
	HomeSwitch home_sw[MOTION_AXES];	// layout 4
	ProbeConfig probe;				// layout 5, keep it last (LAYOUT4_SIZE)
} Settings;

// Older layouts are this one cut short: still loaded, what they don't have comes from the defaults
#define LAYOUT2_SIZE	offsetof(Settings, pwm)
#define LAYOUT3_SIZE	offsetof(Settings, home_sw)
#define LAYOUT4_SIZE	offsetof(Settings, probe)

// First flash word of a slot, written last: a slot without a good header is never loaded
typedef struct {
	uint32_t magic;
	uint32_t layout;
	uint32_t size;		// sizeof(Settings)
	uint32_t seq;		// +1 every save, the highest wins
	uint32_t crc;		// of the Settings bytes
	uint32_t spare[2];
	uint32_t hcrc;		// of the 7 words above
} Header;

_Static_assert(sizeof(Header) == FLASH_WORD, "header is one flash word");
_Static_assert(sizeof(Header) + sizeof(Settings) <= SETTINGS_SLOT, "settings don't fit a slot");
_Static_assert(FLASH_SECTOR % SETTINGS_SLOT == 0 && SETTINGS_SLOT % FLASH_WORD == 0, "slot size");
_Static_assert(SETTINGS_BASE == BOOT_BASE + SETTINGS_SECTOR * FLASH_SECTOR, "settings sector address");

static uint32_t buf[SLOT_WORDS];	// one slot, read or about to be written
static Settings defaults;			// the built in values, taken at boot before loading
static int16_t cur = -1;			// slot with the newest good save, -1 = none
static uint16_t next_free;			// first slot after the last one used (SLOTS = full, erase first)
static uint32_t seq;
static bool loaded;					// SettingsBoot found a save
static volatile bool probing, probe_fault;
static Settings saved;				// the save in buf, filled out to this layout

static uint32_t Crc(const void *p, uint32_t n) {
	const uint8_t *b = p;
	uint32_t c = 0xFFFFFFFF, k;
	while (n--) {
		c ^= *b++;
		for (k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320 & -(c & 1));
	}
	return ~c;
}

static uint32_t SlotAddr(uint16_t i) {
	return SETTINGS_BASE + (uint32_t)i * SETTINGS_SLOT;
}

// A flash word cut off half written (power lost during a save) fails its ECC, and reading it is a
// bus fault. While probing, the fault handler skips the load and flags it instead of hanging
void BusFault_Handler() __attribute__((naked));
void BusFault_Handler() {
	__asm volatile(
		"tst lr, #4\n"
		"ite eq\n"
		"mrseq r0, msp\n"
		"mrsne r0, psp\n"
		"b BusFaultFrame\n");
}

void BusFaultFrame(uint32_t *frame) __attribute__((used));
void BusFaultFrame(uint32_t *frame) {
	uint16_t op;

	if (!probing) while (1); // same as the default handler
	if (SCB->CFSR & SCB_CFSR_PRECISERR_Msk) {
		// Precise: the stacked PC is the load itself, step over it (16 or 32 bit Thumb)
		op = *(uint16_t *)frame[6];
		frame[6] += (op & 0xF800) >= 0xE800 ? 4 : 2;
	}
	SCB->CFSR = SCB_CFSR_BUSFAULTSR_Msk; // write 1 to clear
	probe_fault = true;
}

static bool ProbeRead(uint32_t addr, uint32_t *dst, uint32_t words) {
	uint32_t i;

	probe_fault = false;
	SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
	probing = true;
	__DSB();
	__ISB();
	for (i = 0; i < words && !probe_fault; i++) dst[i] = ((volatile const uint32_t *)addr)[i];
	__DSB();
	probing = false;
	SCB->SHCSR &= ~SCB_SHCSR_BUSFAULTENA_Msk;
	FLASH->CCR1 = FLASH_CCR_CLR_SNECCERR | FLASH_CCR_CLR_DBECCERR;
	return !probe_fault;
}

static bool Blank(const uint32_t *w, uint32_t n) {
	while (n--) {
		if (*w++ != 0xFFFFFFFF) return false;
	}
	return true;
}

static bool Good(const uint32_t *w) {
	const Header *h = (const Header *)w;
	bool layout = (h->layout == SETTINGS_LAYOUT && h->size == sizeof(Settings)) || (h->layout == 2 && h->size == LAYOUT2_SIZE) || (h->layout == 3 && h->size == LAYOUT3_SIZE) || (h->layout == 4 && h->size == LAYOUT4_SIZE);
	return h->magic == MAGIC && layout && h->hcrc == Crc(h, sizeof(Header) - 4) && h->crc == Crc(&w[HDR_WORDS], h->size);
}

// Newest good save, and where the next one goes
static void Scan() {
	uint16_t i;
	bool ok;

	cur = -1;
	seq = 0;
	next_free = 0;
	for (i = 0; i < SLOTS; i++) {
		ok = ProbeRead(SlotAddr(i), buf, SLOT_WORDS);
		if (ok && Blank(buf, SLOT_WORDS)) continue;
		next_free = i + 1; // used, or a broken one: either way not writable before an erase
		if (ok && Good(buf) && ((Header *)buf)->seq >= seq) {
			cur = i;
			seq = ((Header *)buf)->seq;
		}
	}
}

static void Gather(Settings *s) {
	memset(s, 0, sizeof(*s)); // padding too, it's in the CRC
	memcpy(s->axes, motion_axes, sizeof(s->axes));
	s->invert = motion_invert;
	s->accel = motion_accel;
	memcpy(s->tmc, tmc_config, sizeof(s->tmc));
	s->tmc_ignored = TmcIgnored();
	memcpy(s->home, home_axes, sizeof(s->home));
	s->pad_xy = pad_speed_xy;
	s->pad_xy_fast = pad_speed_xy_fast;
	s->pad_z = pad_speed_z;
	memcpy(s->pwm, pwm_profiles, sizeof(s->pwm));
	memcpy(s->home_sw, home_switch, sizeof(s->home_sw));
	s->probe = probe_config;
}

// At boot TmcTask does the driver setup. Later every driver gets its new config now
static void Apply(const Settings *s, bool boot) {
	uint8_t d, n = 0;

	memcpy(motion_axes, s->axes, sizeof(s->axes));
	motion_invert = s->invert;
	motion_accel = s->accel;
	memcpy(tmc_config, s->tmc, sizeof(s->tmc));
	TmcIgnore(s->tmc_ignored);
	memcpy(home_axes, s->home, sizeof(s->home));
	pad_speed_xy = s->pad_xy;
	pad_speed_xy_fast = s->pad_xy_fast;
	pad_speed_z = s->pad_z;
	memcpy(pwm_profiles, s->pwm, sizeof(s->pwm));
	memcpy(home_switch, s->home_sw, sizeof(s->home_sw));
	probe_config = s->probe;
	MotionSync(); // steps/mm may have changed, positions are counted in steps
	if (boot) return;
	for (d = 0; d < TMC_DRIVERS; d++) {
		if (TmcSetup(d) == TMC_OK) n++;
	}
	printf("echo:%d of %d drivers set up with it\n", n, TMC_DRIVERS);
}

// The good save in buf: an older layout's missing fields are the defaults
static const Settings *Saved() {
	saved = defaults;
	memcpy(&saved, &buf[HDR_WORDS], ((const Header *)buf)->size);
	return &saved;
}

static bool Idle() {
	return MotionIdle() && !JogActive() && !HomeBusy() && !ProbeBusy() && PadMode() == PAD_MODE_OFF;
}

void SettingsBoot() {
	Gather(&defaults);
	Scan();
	if (cur < 0 || !ProbeRead(SlotAddr(cur), buf, SLOT_WORDS) || !Good(buf)) return;
	Apply(Saved(), true);
	loaded = true;
}

uint8_t SettingsSave() {
	Header *h = (Header *)buf;
	uint32_t e = 0, i, n;
	uint16_t slot = next_free;
	bool paused, erase = slot >= SLOTS;

	if (!Idle()) return SET_BUSY;
	memset(buf, 0, sizeof(buf));
	Gather((Settings *)&buf[HDR_WORDS]);
	h->magic = MAGIC;
	h->layout = SETTINGS_LAYOUT;
	h->size = sizeof(Settings);
	h->seq = seq + 1;
	h->crc = Crc(&buf[HDR_WORDS], sizeof(Settings));
	h->hcrc = Crc(h, sizeof(Header) - 4);
	n = (sizeof(Settings) + FLASH_WORD - 1) / FLASH_WORD * HDR_WORDS;

	// Stopped step generator (fault / E-stop) has no deadline to miss, a running one gets paused
	paused = StepPause();
	if (!paused && StepRunning()) return SET_BUSY;
	FlashUnlock();
	if (erase) {
		e = FlashEraseSector(SETTINGS_SECTOR);
		slot = 0;
	}
	for (i = HDR_WORDS; !e && i < HDR_WORDS + n; i += HDR_WORDS) e = FlashWriteWord(SlotAddr(slot) + i * 4, &buf[i]);
	if (!e) e = FlashWriteWord(SlotAddr(slot), buf);
	FlashLock();
	SCB_InvalidateDCache_by_Addr((void *)SETTINGS_BASE, FLASH_SECTOR);
	if (paused) StepResume();

	Scan(); // read back: the new slot has to be the newest good one
	if (e || cur != slot) return SET_FLASH;
	return SET_OK;
}

uint8_t SettingsLoad() {
	if (!Idle()) return SET_BUSY;
	Scan();
	if (cur < 0) return SET_NONE;
	if (!ProbeRead(SlotAddr(cur), buf, SLOT_WORDS) || !Good(buf)) return SET_FLASH;
	Apply(Saved(), false);
	return SET_OK;
}

uint8_t SettingsDefaults() {
	if (!Idle()) return SET_BUSY;
	Apply(&defaults, false);
	return SET_OK;
}

uint8_t SettingsErase() {
	uint32_t e;
	bool paused;

	if (!Idle()) return SET_BUSY;
	paused = StepPause();
	if (!paused && StepRunning()) return SET_BUSY;
	FlashUnlock();
	e = FlashEraseSector(SETTINGS_SECTOR);
	FlashLock();
	SCB_InvalidateDCache_by_Addr((void *)SETTINGS_BASE, FLASH_SECTOR);
	if (paused) StepResume();
	Scan();
	return e || next_free ? SET_FLASH : SET_OK;
}

// ---- diff ----

static uint16_t n_diff;
static bool diff_print;

static void DiffF(const char *who, const char *what, float was, float now) {
	if (was == now) return;
	n_diff++;
	if (diff_print) printf("%s %s: %g -> %g\n", who, what, (double)was, (double)now);
}

static void DiffI(const char *who, const char *what, int32_t was, int32_t now) {
	if (was == now) return;
	n_diff++;
	if (diff_print) printf("%s %s: %" PRId32 " -> %" PRId32 "\n", who, what, was, now);
}

static void DiffS(const char *who, const char *what, const char *was, const char *now, bool same) {
	if (same) return;
	n_diff++;
	if (diff_print) printf("%s %s: %s -> %s\n", who, what, was, now);
}

static const char *Mask(uint8_t m, char *b) {
	uint8_t d, n = 0;
	for (d = 0; d < 8; d++) {
		if ((m >> d) & 1) n += sprintf(b + n, "%s%d", n ? "," : "", d);
	}
	if (n == 0) strcpy(b, "none");
	return b;
}

static const char *End(int8_t dir) {
	return dir < 0 ? "-" : (dir > 0 ? "+" : "off");
}

// Field by field, base -> live
static void Diff(const Settings *a, const Settings *b) {
	char who[12], m1[20], m2[20];
	uint8_t i;

	for (i = 0; i < MOTION_AXES; i++) {
		const MotionAxis *x = &a->axes[i], *y = &b->axes[i];
		snprintf(who, sizeof(who), "%c", y->name);
		DiffS(who, "drivers", Mask(x->drivers, m1), Mask(y->drivers, m2), x->drivers == y->drivers);
		DiffF(who, "steps/mm (M92)", x->steps_per_mm, y->steps_per_mm);
		DiffF(who, "max feed mm/s (M203)", x->v_max, y->v_max);
		DiffF(who, "max accel mm/s^2 (M201)", x->a_max, y->a_max);
		DiffF(who, "max jerk mm/s^3 (M205)", x->j_max, y->j_max);
	}
	DiffS("Axes", "DIR inverted drivers", Mask(a->invert, m1), Mask(b->invert, m2), a->invert == b->invert);
	DiffF("Path", "accel mm/s^2 (M204)", a->accel, b->accel);
	for (i = 0; i < TMC_DRIVERS; i++) {
		const TmcConfig *x = &a->tmc[i], *y = &b->tmc[i];
		snprintf(who, sizeof(who), "Driver %d", i);
		DiffF(who, "run A", x->run, y->run);
		DiffF(who, "hold A", x->hold, y->hold);
		DiffI(who, "ustep", x->ustep, y->ustep);
		DiffI(who, "interp", x->intpol, y->intpol);
		DiffS(who, "mode", x->spread ? "spread" : "stealth", y->spread ? "spread" : "stealth", x->spread == y->spread);
	}
	DiffS("Drivers", "ignored", Mask(a->tmc_ignored, m1), Mask(b->tmc_ignored, m2), a->tmc_ignored == b->tmc_ignored);
	for (i = 0; i < MOTION_AXES; i++) {
		const HomeAxis *x = &a->home[i], *y = &b->home[i];
		snprintf(who, sizeof(who), "Home %c", b->axes[i].name);
		DiffS(who, "end", End(x->dir), End(y->dir), x->dir == y->dir);
		DiffF(who, "speed", x->speed, y->speed);
		DiffI(who, "sgthrs", x->sgthrs, y->sgthrs);
		DiffF(who, "current", x->current, y->current);
		DiffF(who, "travel", x->travel, y->travel);
		DiffF(who, "backoff", x->backoff, y->backoff);
		DiffF(who, "pos", x->pos, y->pos);
		DiffF(who, "pre", x->pre, y->pre);
		DiffS(who, "mode", a->home_sw[i].mode == HOME_SWITCH ? "switch" : "soft", b->home_sw[i].mode == HOME_SWITCH ? "switch" : "soft",
			a->home_sw[i].mode == b->home_sw[i].mode);
		DiffI(who, "btn", a->home_sw[i].btn, b->home_sw[i].btn);
		DiffI(who, "level", a->home_sw[i].level, b->home_sw[i].level);
	}
	DiffF("Pad", "speed xy", a->pad_xy, b->pad_xy);
	DiffF("Pad", "speed fast", a->pad_xy_fast, b->pad_xy_fast);
	DiffF("Pad", "speed z", a->pad_z, b->pad_z);
	for (i = 0; i < PWM_CHANNELS; i++) {
		const PwmProfile *x = &a->pwm[i], *y = &b->pwm[i];
		snprintf(who, sizeof(who), "PWM%d (%d)", i + 1, i);
		DiffF(who, "strike", x->strike, y->strike);
		DiffI(who, "strike ms", x->strike_ms, y->strike_ms);
		DiffF(who, "hold", x->hold, y->hold);
	}
	{
		const ProbeConfig *x = &a->probe, *y = &b->probe;
		DiffF("Probe", "F mm/s", x->feed, y->feed);
		DiffI("Probe", "P channel", x->ch, y->ch);
		DiffF("Probe", "S kPa", x->drop, y->drop);
		DiffI("Probe", "N samples", x->need, y->need);
		DiffI("Probe", "A average", x->avg, y->avg);
		DiffI("Probe", "D blank ms", x->blank_ms, y->blank_ms);
		DiffI("Probe", "R reference ms", x->ref_ms, y->ref_ms);
		DiffI("Probe", "W rolling ms", x->roll_ms, y->roll_ms);
		DiffF("Probe", "V min vacuum kPa", x->min_vac, y->min_vac);
		DiffF("Probe", "U rise kPa", x->rise, y->rise);
		DiffI("Probe", "T timeout ms", (int32_t)x->timeout_ms, (int32_t)y->timeout_ms);
		DiffF("Probe", "L z min", x->z_lo, y->z_lo);
		DiffF("Probe", "H z max", x->z_hi, y->z_hi);
	}
}

uint16_t SettingsDiff(bool vs_defaults, bool print) {
	static Settings base, live;

	if (vs_defaults || cur < 0) {
		base = defaults;
	} else if (ProbeRead(SlotAddr(cur), buf, SLOT_WORDS) && Good(buf)) {
		base = *Saved();
	} else {
		if (print) printf("Newest save can't be read\n");
		return 0;
	}
	Gather(&live);
	n_diff = 0;
	diff_print = print;
	Diff(&base, &live);
	return n_diff;
}

void SettingsInfo() {
	printf("Storage: flash sector %d (0x%08X), %d slots of %d bytes, layout %d, %u bytes saved\n",
		SETTINGS_SECTOR, SETTINGS_BASE, SLOTS, SETTINGS_SLOT, SETTINGS_LAYOUT, (unsigned)sizeof(Settings));
	if (cur < 0) printf("Nothing saved, running on the built in defaults\n");
	else printf("Newest save: #%" PRIu32 " in slot %d, %d slots left before an erase\n", seq, cur, SLOTS - next_free);
	printf("At boot: %s\n", loaded ? "loaded the newest save" : "built in defaults");
	printf("Unsaved changes: %u (vs the %s, config diff lists them)\n", SettingsDiff(false, false), cur < 0 ? "defaults" : "save");
}

const char *SettingsError(uint8_t e) {
	switch (e) {
		case SET_OK:	return "ok";
		case SET_BUSY:	return "not while moving, jogging, homing or in pad mode";
		case SET_FLASH:	return "flash erase / write / read back failed";
		case SET_NONE:	return "nothing saved yet";
	}
	return "?";
}
