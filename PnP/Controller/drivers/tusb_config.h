#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#define CFG_TUSB_MCU				OPT_MCU_STM32H7
#define CFG_TUSB_OS					OPT_OS_NONE
#define CFG_TUSB_DEBUG				0

#define BOARD_TUD_RHPORT			0	// H723 has one USB (OTG_HS), tinyusb calls it port 0
#define CFG_TUD_ENABLED				1
#define CFG_TUD_MAX_SPEED			OPT_MODE_FULL_SPEED	// internal FS PHY on PA11/PA12

#define CFG_TUD_DWC2_DMA_ENABLE		0	// FIFO mode, CPU copies, so no cache issues
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN			__attribute__((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE		64

#define CFG_TUD_CDC					1
#define CFG_TUD_MSC					0
#define CFG_TUD_HID					0
#define CFG_TUD_MIDI				0
#define CFG_TUD_VENDOR				0

#define CFG_TUD_CDC_RX_BUFSIZE		512
#define CFG_TUD_CDC_TX_BUFSIZE		2048
#define CFG_TUD_CDC_EP_BUFSIZE		64

#endif
