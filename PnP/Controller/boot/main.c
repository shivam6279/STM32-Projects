#include "board.h"
#include "bootflag.h"
#include "clock.h"
#include "loader.h"
#include "usb.h"

// Bootloader, sector 0. Goes straight to the app unless the app asked us to stay
// (BOOT_FLAG) or there's no valid app. Otherwise shows up as the PnP-Boot COM port, and
// listens on UART6 (ESP32) too.

void JumpToApp() {
	uint32_t sp = *(volatile uint32_t *)APP_BASE;
	uint32_t pc = *(volatile uint32_t *)(APP_BASE + 4);

	// Still in reset state apart from STEPPER_EN, which the app keeps: nothing to undo
	SCB->VTOR = APP_BASE;
	__set_MSP(sp);
	((void (*)())pc)();
	while (1);
}

int main(void) {
	uint32_t blink_tick = 0;
	bool led = false;

	BoardStepperSafe(); // drivers off first, ENN has no pull-up
	if (BOOT_FLAG != BOOT_STAY && AppValid(false)) JumpToApp();
	BOOT_FLAG = 0;

	ClockInit();
	BoardIoInit(); // same safe pin states as the app, drivers disabled
	UsbInit();
	LoaderInit(); // UART6 to the ESP32, for flashing over WiFi

	while (1) {
		UsbTask();
		LoaderTask();

		// LED0 blinks fast in the bootloader
		if (Millis() - blink_tick >= 100) {
			blink_tick += 100;
			led = !led;
			BoardLed(0, led);
		}
	}
}
