#include "boot.h"
#include "bootflag.h"
#include "stm32h7xx.h"

extern uint32_t g_pfnVectors[];

void BootInit() {
	// The bootloader sets this before jumping, but not if we got here some other way (SWD)
	SCB->VTOR = (uint32_t)g_pfnVectors;
	__DSB();
}

void BootToLoader() {
	__disable_irq();
	BOOT_FLAG = BOOT_STAY;
	// Read it back before resetting. Without this the DTCM write can still be in flight
	// when the reset hits and the bootloader sees 0 (DSB alone wasn't enough here).
	while (BOOT_FLAG != BOOT_STAY);
	__DSB();
	NVIC_SystemReset();
}
