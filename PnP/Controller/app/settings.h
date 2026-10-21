#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include <stdbool.h>
#include <inttypes.h>

// Settings kept over a reboot: emulated EEPROM in flash sector 3 (SETTINGS_BASE, bootflag.h).
// The app links to sectors 1-2 and the flasher only erases what the image needs, so a firmware
// update leaves them alone.
//
// Saved: axes (M92 M201 M203 M204 M205, DIR inverts), TMC2209 config + ignored drivers, homing,
// pad speeds. Changes take effect at once and only stick after SettingsSave (M500, config save).
//
// Each save goes in the next free SETTINGS_SLOT with a sequence number and CRC, the newest good
// one wins at boot. The sector is erased only when every slot is used. A slot from another layout
// (SETTINGS_LAYOUT) or with a bad CRC is skipped, so the worst case is the built in defaults.
// While the flash is busy nothing runs from it: a save needs everything idle and pauses the step
// generator around it (a write is ~ms, an erase ~1s, USB / UART can hiccup during that).

#define SETTINGS_SLOT	1024
#define SETTINGS_LAYOUT	2		// bump whenever the saved struct changes. 2: HomeAxis.pre

#define SET_OK		0
#define SET_BUSY	1	// motion, jog, homing or pad mode running
#define SET_FLASH	2	// erase / write / read back failed
#define SET_NONE	3	// nothing saved yet

void SettingsBoot();		// main, after the modules' init, before the loop: newest saved -> live
uint8_t SettingsSave();
uint8_t SettingsLoad();		// newest saved -> live, drivers set up again
uint8_t SettingsDefaults();	// built in values -> live, not saved until SettingsSave
uint8_t SettingsErase();	// sector wiped: next boot comes up on the defaults
void SettingsInfo();		// slots used, sequence, layout, unsaved changes
uint16_t SettingsDiff(bool defaults, bool print);	// live vs the newest save (or the defaults): count, each one printed
const char *SettingsError(uint8_t e);

#endif
