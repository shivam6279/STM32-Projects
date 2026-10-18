#include "tusb.h"
#include "stm32h7xx.h"

// Bootloader build overrides these
#ifndef USB_PID
#define USB_PID		0x4001
#endif
#ifndef USB_PRODUCT
#define USB_PRODUCT	"PnP Controller"
#endif
#ifndef USB_CDC_NAME
#define USB_CDC_NAME	"PnP-Diags"
#endif

#define USB_VID		0xCafe	// TODO get a real VID/PID (pid.codes), this is tinyusb's test VID

enum {
	ITF_NUM_CDC = 0,
	ITF_NUM_CDC_DATA,
	ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF	0x81
#define EPNUM_CDC_OUT	0x02
#define EPNUM_CDC_IN	0x82

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

enum {
	STR_LANG = 0,
	STR_MANUFACTURER,
	STR_PRODUCT,
	STR_SERIAL,
	STR_CDC,
};

static const tusb_desc_device_t desc_device = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = TUSB_CLASS_MISC,		// IAD, so more interfaces (DFU) can be added later
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = USB_VID,
	.idProduct = USB_PID,
	.bcdDevice = 0x0100,
	.iManufacturer = STR_MANUFACTURER,
	.iProduct = STR_PRODUCT,
	.iSerialNumber = STR_SERIAL,
	.bNumConfigurations = 1
};

static const uint8_t desc_config[] = {
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STR_CDC, EPNUM_CDC_NOTIF, 16, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

static const char *strings[] = {
	[STR_MANUFACTURER] = "Shivam",
	[STR_PRODUCT] = USB_PRODUCT,
	[STR_CDC] = USB_CDC_NAME,
};

static uint16_t desc_str[33];

const uint8_t *tud_descriptor_device_cb(void) {
	return (const uint8_t *)&desc_device;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
	(void)index;
	return desc_config;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
	uint8_t len;
	(void)langid;

	if (index == STR_LANG) {
		desc_str[1] = 0x0409; // English
		len = 1;
	} else if (index == STR_SERIAL) {
		// 96 bit unique ID as 24 hex chars
		const uint32_t *uid = (const uint32_t *)UID_BASE;
		for (len = 0; len < 24; len++) {
			uint8_t nib = (uid[len / 8] >> (28 - (len % 8) * 4)) & 0xF;
			desc_str[1 + len] = nib < 10 ? '0' + nib : 'A' + nib - 10;
		}
	} else {
		if (index >= sizeof(strings) / sizeof(strings[0]) || strings[index] == NULL) return NULL;
		const char *s = strings[index];
		for (len = 0; s[len] && len < 32; len++) desc_str[1 + len] = s[len];
	}

	// first word: length in bytes + descriptor type
	desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
	return desc_str;
}
