#include "EEPROM.h"
#include "main.h"
#include <string.h>

eeprom_data_t eeprom_data;

void ee_read() {
    memcpy(&eeprom_data, (void*)EEPROM_START_ADDR, sizeof(eeprom_data_t));
}

void ee_write() {
    // 1. Check if the data is already identical to save Flash endurance
    if (memcmp(&eeprom_data, (void*)EEPROM_START_ADDR, sizeof(eeprom_data_t)) == 0) {
        return;
    }

    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError;
    uint32_t numQuadWords = sizeof(eeprom_data_t) / 16;

    HAL_FLASH_Unlock();

    // 2. Erase the 8KB sector
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Banks     = EEPROM_FLASH_BANK;
    eraseInit.Sector    = EEPROM_FLASH_SECTOR;
    eraseInit.NbSectors = 1;

    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    // 3. Program in 128-bit chunks
    for (uint32_t i = 0; i < numQuadWords; i++) {
        // H5 requires the source data address to be a uint32_t/pointer to a 128-bit block
        uint32_t targetAddr = EEPROM_START_ADDR + (i * 16);
        uint32_t sourceAddr = (uint32_t)&eeprom_data + (i * 16);

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, targetAddr, sourceAddr);

        if (status != HAL_OK) break;
    }

    HAL_FLASH_Lock();
    return;
}
