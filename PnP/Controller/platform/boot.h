#ifndef _BOOT_H_
#define _BOOT_H_

// App side of the bootloader (boot/). The app lives at APP_BASE, the bootloader in sector 0.

void BootInit();		// first thing in main: point VTOR at the app's vector table
void BootToLoader();	// reset into the bootloader and stay there (PnP-Boot COM port)

#endif
