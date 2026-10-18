#include "loader.h"
#include "bootflag.h"
#include "clock.h"
#include "esp.h"
#include "flash.h"
#include "usb.h"
#include "stm32h7xx.h"

#include <string.h>

#define VERSION		2
#define MAX_DATA	1024
#define HDR_LEN		3
#define CRC_LEN		4
#define LINK_IDLE_MS	100	// WiFi can stall a bit

#define ERR_CMD		1	// unknown command
#define ERR_ARGS	2	// bad length / offset / sector
#define ERR_ERASE	3
#define ERR_WRITE	4	// write failed, or the flash there isn't erased
#define ERR_CRC		5	// image CRC
#define ERR_FIRST	6	// never got the first 32 bytes
#define ERR_READ	7	// readback doesn't match what was written
#define ERR_PKT		8	// packet CRC

#define INFO_MAGIC	0x49504E50	// "PNPI"
#define VEC_SIZE	7			// vector 7 is reserved, holds the image size

typedef struct {
	uint32_t (*read)(char *buf, uint32_t len);
	uint32_t (*write)(const char *buf, uint32_t len);
	uint8_t pkt[HDR_LEN + 8 + MAX_DATA + CRC_LEN];
	uint32_t len;
	uint32_t last_ms;
} link;

static uint32_t UartRead(char *buf, uint32_t len) {
	uint32_t n = 0;
	int c;
	while (n < len && (c = EspReadChar()) >= 0) buf[n++] = c;
	return n;
}

static link links[2] = {
	{ .read = UsbRead, .write = UsbWrite },
	{ .read = UartRead, .write = EspWrite },
};
static link *cur; // the one being answered

static uint32_t first_word[FLASH_WORD / 4];
static bool have_first = false;
static uint32_t crc_table[256];

static uint32_t Crc32(uint32_t crc, const uint8_t *p, uint32_t len) {
	uint32_t i, j, c;

	if (crc_table[1] == 0) {
		for (i = 0; i < 256; i++) {
			c = i;
			for (j = 0; j < 8; j++) c = (c >> 1) ^ (0xEDB88320 & -(c & 1));
			crc_table[i] = c;
		}
	}
	crc = ~crc;
	while (len--) crc = (crc >> 8) ^ crc_table[(crc ^ *p++) & 0xFF];
	return ~crc;
}

static uint32_t Get32(const uint8_t *p) {
	return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void Reply(char k) {
	cur->write(&k, 1);
}

static void ReplyErr(uint8_t code) {
	char r[2] = { 'E', code };
	cur->write(r, 2);
}

// CRC the host computed: the image as sent, vector 7 = 0
static uint32_t ImageCrc(const uint32_t *first, uint32_t size) {
	uint32_t w[FLASH_WORD / 4];
	memcpy(w, first, FLASH_WORD);
	w[VEC_SIZE] = 0;
	return Crc32(Crc32(0, (uint8_t *)w, FLASH_WORD), (uint8_t *)(APP_BASE + FLASH_WORD), size - FLASH_WORD);
}

bool AppValid() {
	const uint32_t *vec = (const uint32_t *)APP_BASE;
	const uint32_t *info;
	uint32_t sp = vec[0], pc = vec[1], size = vec[VEC_SIZE];

	bool sp_ok = (sp > 0x20000000 && sp <= 0x20020000) || (sp > 0x24000000 && sp <= 0x24050000);
	bool pc_ok = (pc & 1) && pc >= APP_BASE && pc < APP_BASE + APP_SIZE;
	if (!sp_ok || !pc_ok) return false;

	// No size = flashed over SWD
	if (size == 0) return true;

	if (size < FLASH_WORD || (size % FLASH_WORD) || size + FLASH_WORD > APP_SIZE) return false;
	info = (const uint32_t *)(APP_BASE + size);
	return info[0] == INFO_MAGIC && info[1] == size && info[2] == ImageCrc(vec, size);
}

// Same data already there is fine (resent packet)
static uint8_t WriteWord(uint32_t addr, const uint32_t *word) {
	const uint32_t *f = (const uint32_t *)addr;
	uint32_t i;

	if (memcmp(f, word, FLASH_WORD) == 0) return 0;
	for (i = 0; i < FLASH_WORD / 4; i++) {
		if (f[i] != 0xFFFFFFFF) return ERR_WRITE;
	}
	if (FlashWriteWord(addr, word)) return ERR_WRITE;
	if (memcmp(f, word, FLASH_WORD)) return ERR_READ;
	return 0;
}

static void CmdInfo() {
	uint8_t r[15] = { 'K', 'P', 'N', 'P', 'B', VERSION & 0xFF, VERSION >> 8 };
	uint32_t base = APP_BASE, size = APP_SIZE;
	memcpy(&r[7], &base, 4);
	memcpy(&r[11], &size, 4);
	cur->write((char *)r, sizeof(r));
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
	uint8_t err;

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
		if ((err = WriteWord(APP_BASE + off + i, word))) {
			FlashLock();
			return ReplyErr(err);
		}
	}
	FlashLock();
	Reply('K');
}

static void CmdVerify(const uint8_t *p, uint32_t len) {
	uint32_t size, crc, info[FLASH_WORD / 4], vec[FLASH_WORD / 4];
	uint8_t err;

	if (len != 8) return ReplyErr(ERR_ARGS);
	size = Get32(p);
	crc = Get32(p + 4);
	if (size < FLASH_WORD || size + FLASH_WORD > APP_SIZE || (size % FLASH_WORD)) return ReplyErr(ERR_ARGS);
	if (!have_first) return ReplyErr(ERR_FIRST);
	if (first_word[VEC_SIZE] != 0) return ReplyErr(ERR_ARGS); // vector 7 has to be free for the size

	// CRC over the held back first word + what's in flash after it
	if (ImageCrc(first_word, size) != crc) return ReplyErr(ERR_CRC);

	// Info word after the image, then the vector table with the size in it
	memset(info, 0xFF, sizeof(info));
	info[0] = INFO_MAGIC;
	info[1] = size;
	info[2] = crc;
	memcpy(vec, first_word, FLASH_WORD);
	vec[VEC_SIZE] = size;

	FlashUnlock();
	err = WriteWord(APP_BASE + size, info);
	if (!err) err = WriteWord(APP_BASE, vec);
	FlashLock();
	if (err) return ReplyErr(err);
	if (!AppValid()) return ReplyErr(ERR_READ);
	Reply('K');
}

static void CmdRun() {
	uint32_t t;

	Reply('K');
	t = Millis();
	while (Millis() - t < 50 || !EspTxIdle()) UsbTask(); // let the reply go out
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

static void LinkTask(link *l) {
	uint32_t want, len, n;

	// Stuck half packet, drop it
	if (l->len && Millis() - l->last_ms > LINK_IDLE_MS) l->len = 0;

	if (l->len < HDR_LEN) {
		n = l->read((char *)l->pkt + l->len, HDR_LEN - l->len);
		if (n) l->last_ms = Millis();
		l->len += n;
		if (l->len < HDR_LEN) return;
	}
	cur = l;
	len = l->pkt[1] | (l->pkt[2] << 8);
	if (len > sizeof(l->pkt) - HDR_LEN - CRC_LEN) {
		ReplyErr(ERR_ARGS);
		l->len = 0; // out of sync, host has to start over
		return;
	}
	want = HDR_LEN + len + CRC_LEN;
	n = l->read((char *)l->pkt + l->len, want - l->len);
	if (n) l->last_ms = Millis();
	l->len += n;
	if (l->len < want) return;
	l->len = 0;

	if (Crc32(0, l->pkt, HDR_LEN + len) != Get32(l->pkt + HDR_LEN + len)) return ReplyErr(ERR_PKT);
	Handle(l->pkt[0], l->pkt + HDR_LEN, len);
}

void LoaderInit() {
	EspUartInit(ESP_BOOT_BAUD);
}

void LoaderTask() {
	LinkTask(&links[0]);
	LinkTask(&links[1]);
}
