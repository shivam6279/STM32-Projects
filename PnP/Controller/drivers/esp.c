#include "esp.h"
#include "clock.h"
#include "ioexp.h"

#define RESET_MS 20	// EN low. 10k/100nF on EN, it's down in well under 1ms
#define STRAP_MS 50	// BOOT stays low this long after EN goes back up

static void Wait(uint32_t ms) {
	uint32_t t = Millis();
	while (Millis() - t < ms);
}

bool EspInit() {
	EspUartInit(ESP_BAUD);
	EspLinkInit();
	return IoExpInit();
}

bool EspReset(bool download) {
	bool ok = true;

	if (download) ok &= IoExpLow(IOEXP_ESP_BOOT, true);
	ok &= IoExpLow(IOEXP_ESP_EN, true);
	Wait(RESET_MS);
	ok &= IoExpLow(IOEXP_ESP_EN, false);
	if (download) {
		Wait(STRAP_MS);
		ok &= IoExpLow(IOEXP_ESP_BOOT, false);
	}
	return ok;
}
