#include "loader.h"
#include "bootflag.h"
#include "clock.h"
#include "flash.h"
#include "usb.h"
#include "stm32h7xx.h"

#include <string.h>

#define VERSION		1
#define MAX_DATA	1024
#define HDR_LEN		3

#define ERR_CMD		1	// unknown command
#define ERR_ARGS	2	// bad length / offset / sector
#define ERR_ERASE	3
#define ERR_WRITE	4
#define ERR_CRC		5
#define ERR_FIRST	6	// never got the first 32 bytes
#define ERR_READ	7	// readback doesn't match what was written

static uint8_t pkt[HDR_LEN + 8 + MAX_DATA];
static uint32_t pkt_len = 0;
static uint32_t first_word[FLASH_WORD / 4];
static bool have_first = false;

static uint32_t Crc32(uint32_t crc, const uint8_t *p, uint32_t len) {
	uint32_t i;
	crc = ~crc;
	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
	}
	return ~crc;
}

static uint32_t Get32(const uint8_t *p) {
	return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void Reply(char k) {
	UsbWrite(&k, 1);
}

static void ReplyErr(uint8_t code) {
	char r[2] = { 'E', code };
	UsbWrite(r, 2);
}

bool AppValid() {
	uint32_t sp = *(volatile uint32_t *)APP_BASE;
	uint32_t pc = *(volatile uint32_t *)(APP_BASE + 4);

	bool sp_ok = (sp > 0x20000000 && sp <= 0x20020000) || (sp > 0x24000000 && sp <= 0x24050000);
	bool pc_ok = (pc & 1) && pc >= APP_BASE && pc < APP_BASE + APP_SIZE;
	return sp_ok && pc_ok;
}

static void CmdInfo() {
	uint8_t r[15] = { 'K', 'P', 'N', 'P', 'B', VERSION & 0xFF, VERSION >> 8 };
	uint32_t base = APP_BASE, size = APP_SIZE;
	memcpy(&r[7], &base, 4);
	memcpy(&r[11], &size, 4);
	UsbWrite((char *)r, sizeof(r));
}

static void CmdErase(const uint8_t *p, uint32_t len) {
	uint32_t sector;

	if (len != 4) return ReplyErr(ERR_ARGS);
	sector = Get32(p);
	if (sector < APP_BASE / FLASH_SECTOR - BOOT_BASE / FLASH_SECTOR || sector >= (APP_BASE - BOOT_BASE + APP_SIZE) / FLASH_SECTOR) {
		return ReplyErr(ERR_ARGS); // never let it erase the bootloader
	}
	if (sector == 1) have_first = false;

	FlashUnlock();
	if (FlashEraseSector(sector)) {
		FlashLock();
		return ReplyErr(ERR_ERASE);
	}
	FlashLock();
	Reply('K');
}

static void CmdWrite(const uint8_t *p, uint32_t len) {
	uint32_t off, n, i;
	uint32_t word[FLASH_WORD / 4];

	if (len < 4) return ReplyErr(ERR_ARGS);
	off = Get32(p);
	p += 4;
	n = len - 4;
	if ((off % FLASH_WORD) || (n % FLASH_WORD) || off + n > APP_SIZE) return ReplyErr(ERR_ARGS);

	FlashUnlock();
	for (i = 0; i < n; i += FLASH_WORD) {
		memcpy(word, p + i, FLASH_WORD);
		if (off + i == 0) {
			// vector table, hold it back until the CRC checks out
			memcpy(first_word, word, FLASH_WORD);
			have_first = true;
			continue;
		}
		if (FlashWriteWord(APP_BASE + off + i, word)) {
			FlashLock();
			return ReplyErr(ERR_WRITE);
		}
		if (memcmp((void *)(APP_BASE + off + i), word, FLASH_WORD)) {
			FlashLock();
			return ReplyErr(ERR_READ);
		}
	}
	FlashLock();
	Reply('K');
}

static void CmdVerify(const uint8_t *p, uint32_t len) {
	uint32_t size, crc, err;

	if (len != 8) return ReplyErr(ERR_ARGS);
	size = Get32(p);
	crc = Get32(p + 4);
	if (size < FLASH_WORD || size > APP_SIZE || (size % FLASH_WORD)) return ReplyErr(ERR_ARGS);
	if (!have_first) return ReplyErr(ERR_FIRST);

	// CRC over the held back first word + what's in flash after it
	if (Crc32(Crc32(0, (uint8_t *)first_word, FLASH_WORD), (uint8_t *)(APP_BASE + FLASH_WORD), size - FLASH_WORD) != crc) {
		return ReplyErr(ERR_CRC);
	}

	FlashUnlock();
	err = FlashWriteWord(APP_BASE, first_word);
	FlashLock();
	if (err) return ReplyErr(ERR_WRITE);
	if (memcmp((void *)APP_BASE, first_word, FLASH_WORD) || !AppValid()) return ReplyErr(ERR_READ);
	have_first = false;
	Reply('K');
}

static void CmdRun() {
	uint32_t t;

	Reply('K');
	t = Millis();
	while (Millis() - t < 50) UsbTask(); // let the reply go out
	UsbDetach();
	t = Millis();
	while (Millis() - t < 20);
	BOOT_FLAG = 0;
	while (BOOT_FLAG != 0); // make sure it landed, see BootToLoader()
	__DSB();
	NVIC_SystemReset(); // app starts from a clean reset
}

static void Handle(uint8_t cmd, const uint8_t *p, uint32_t len) {
	switch (cmd) {
		case 'I': CmdInfo(); break;
		case 'E': CmdErase(p, len); break;
		case 'W': CmdWrite(p, len); break;
		case 'V': CmdVerify(p, len); break;
		case 'R': CmdRun(); break;
		default: ReplyErr(ERR_CMD); break;
	}
}

void LoaderTask() {
	uint32_t want, len;

	// Read as much of the current packet as we can
	if (pkt_len < HDR_LEN) {
		pkt_len += UsbRead((char *)pkt + pkt_len, HDR_LEN - pkt_len);
		if (pkt_len < HDR_LEN) return;
	}
	len = pkt[1] | (pkt[2] << 8);
	if (len > sizeof(pkt) - HDR_LEN) {
		ReplyErr(ERR_ARGS);
		pkt_len = 0; // out of sync, host has to start over
		return;
	}
	want = HDR_LEN + len;
	pkt_len += UsbRead((char *)pkt + pkt_len, want - pkt_len);
	if (pkt_len < want) return;

	Handle(pkt[0], pkt + HDR_LEN, len);
	pkt_len = 0;
}
