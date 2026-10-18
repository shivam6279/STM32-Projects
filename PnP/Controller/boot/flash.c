#include "flash.h"
#include "stm32h7xx.h"

// Sequence follows ST's HAL (stm32h7xx_hal_flash.c / _flash_ex.c, single bank parts)

#define FLASH_KEY1	0x45670123
#define FLASH_KEY2	0xCDEF89AB

#define SR_ERRORS	(FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR \
	| FLASH_SR_RDPERR | FLASH_SR_RDSERR | FLASH_SR_SNECCERR | FLASH_SR_DBECCERR | FLASH_SR_CRCRDERR)

#define CCR_ALL		(FLASH_CCR_CLR_EOP | FLASH_CCR_CLR_WRPERR | FLASH_CCR_CLR_PGSERR | FLASH_CCR_CLR_STRBERR \
	| FLASH_CCR_CLR_INCERR | FLASH_CCR_CLR_OPERR | FLASH_CCR_CLR_RDPERR | FLASH_CCR_CLR_RDSERR \
	| FLASH_CCR_CLR_SNECCERR | FLASH_CCR_CLR_DBECCERR | FLASH_CCR_CLR_CRCEND | FLASH_CCR_CLR_CRCRDERR)

static uint32_t Wait() {
	uint32_t err;

	while (FLASH->SR1 & (FLASH_SR_QW | FLASH_SR_BSY));
	err = FLASH->SR1 & SR_ERRORS;
	FLASH->CCR1 = CCR_ALL;
	return err;
}

void FlashUnlock() {
	if (FLASH->CR1 & FLASH_CR_LOCK) {
		FLASH->KEYR1 = FLASH_KEY1;
		FLASH->KEYR1 = FLASH_KEY2;
	}
	FLASH->CCR1 = CCR_ALL;
}

void FlashLock() {
	FLASH->CR1 |= FLASH_CR_LOCK;
}

uint32_t FlashEraseSector(uint32_t sector) {
	uint32_t err;

	Wait();
	// x64 parallelism (PSIZE = 3, HAL FLASH_VOLTAGE_RANGE_4). TODO check RM0468 VDD range for x64
	FLASH->CR1 = (FLASH->CR1 & ~(FLASH_CR_PSIZE | FLASH_CR_SNB)) | FLASH_CR_PSIZE | (sector << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
	FLASH->CR1 |= FLASH_CR_START;
	err = Wait();
	FLASH->CR1 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
	return err;
}

uint32_t FlashWriteWord(uint32_t addr, const uint32_t *data) {
	volatile uint32_t *dst = (volatile uint32_t *)addr;
	uint32_t i, err;

	Wait();
	FLASH->CR1 |= FLASH_CR_PG;
	__ISB();
	__DSB();
	for (i = 0; i < FLASH_WORD / 4; i++) dst[i] = data[i];
	__ISB();
	__DSB();
	err = Wait();
	FLASH->CR1 &= ~FLASH_CR_PG;
	return err;
}
