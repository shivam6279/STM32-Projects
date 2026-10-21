#ifndef _BOOTFLAG_H_
#define _BOOTFLAG_H_

#include <inttypes.h>

// Shared between the bootloader and the app.
// BOOT_FLAG is the first word of DTCM. Both linker scripts start DTCM at 0x20000010 so
// nothing else uses it, startup doesn't touch it, and it survives a system reset.

#define BOOT_FLAG		(*(volatile uint32_t *)0x20000000)
#define BOOT_STAY		0xB007D00D	// app -> bootloader: stay in the bootloader after reset

#define BOOT_BASE		0x08000000	// sector 0
#define APP_BASE		0x08020000	// sectors 1-3
#define APP_SIZE		(384 * 1024)	// what the bootloader allows. The app links to 256K, sector 3 is settings
#define SETTINGS_SECTOR	3			// app/settings.c. The bootloader can still erase it (flash_usb.py --erase-settings)
#define SETTINGS_BASE	0x08060000
#define FLASH_SECTOR	(128 * 1024)

#endif
