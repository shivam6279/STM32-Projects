#include "usb.h"
#include "clock.h"
#include "irq_prio.h"
#include "stm32h7xx.h"
#include "tusb.h"

#define WRITE_TIMEOUT_MS 50

bool UsbInit() {
	uint32_t t;

	// USB PHY supply, VDD33USB straight from 3.3V. TODO check against schematic
	PWR->CR3 |= PWR_CR3_USB33DEN;
	t = Millis();
	while (!(PWR->CR3 & PWR_CR3_USB33RDY)) {
		if (Millis() - t > 10) return false;
	}

	ClockUsbInit(); // 48MHz from PLL3Q

	RCC->AHB1ENR |= RCC_AHB1ENR_USB1OTGHSEN;
	RCC->AHB1LPENR &= ~RCC_AHB1LPENR_USB1OTGHSULPILPEN; // no ULPI with the internal PHY, leaving it on breaks USB in WFI
	(void)RCC->AHB1ENR;

	NVIC_SetPriority(OTG_HS_IRQn, IRQ_PRIO_USB);

	tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL };
	tusb_init(BOARD_TUD_RHPORT, &dev_init);
	return true;
}

void UsbTask() {
	tud_task();
}

bool UsbConnected() {
	return tud_cdc_connected();
}

uint32_t UsbWrite(const char *buf, uint32_t len) {
	uint32_t done = 0, t = Millis();

	if (!tud_cdc_connected()) return len; // nobody listening, drop it

	while (done < len) {
		done += tud_cdc_write(buf + done, len - done);
		if (done < len) {
			// FIFO full, push it out
			tud_cdc_write_flush();
			tud_task();
			if (Millis() - t > WRITE_TIMEOUT_MS) break;
		}
	}
	tud_cdc_write_flush();
	return done;
}

uint32_t UsbRead(char *buf, uint32_t len) {
	return tud_cdc_read(buf, len);
}

int UsbReadChar() {
	if (!tud_cdc_available()) return -1;
	return tud_cdc_read_char();
}

void UsbDetach() {
	tud_disconnect();
}

void OTG_HS_IRQHandler() {
	tud_int_handler(BOARD_TUD_RHPORT);
}

// tinyusb timing hooks
uint32_t tusb_time_millis_api(void) {
	return Millis();
}

void tusb_time_delay_ms_api(uint32_t ms) {
	uint32_t t = Millis();
	while (Millis() - t < ms);
}
