#ifndef _FLASH_H_
#define _FLASH_H_

#include <inttypes.h>

// Internal flash, bank 1. Writes are one 256 bit flash word (8 x 32 bit) at a time,
// 32 byte aligned, and each flash word can only be written once after an erase.
// While the flash is busy, code running from flash just stalls (single bank).

#define FLASH_WORD 32

void FlashUnlock();
void FlashLock();
uint32_t FlashEraseSector(uint32_t sector);					// 0 = ok, else FLASH->SR1 error bits
uint32_t FlashWriteWord(uint32_t addr, const uint32_t *data);	// 8 words

#endif
