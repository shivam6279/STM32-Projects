/* App: sectors 1-3 (0x08020000).
 * STM32H723VET6: 512 KB flash (4 x 128 KB sectors), DTCM 128 KB, AXI SRAM1 128 KB.
 * Only AXI SRAM1 is used: SRAM2 (0x24020000) is shared with ITCM, size set by option bytes.
 * DTCM starts at 0x20000010: 0x20000000-0x2000000F is BOOT_FLAG (platform/bootflag.h).
 */

MEMORY
{
	FLASH   (rx)  : ORIGIN = 0x08020000, LENGTH = 384K
	DTCMRAM (rw)  : ORIGIN = 0x20000010, LENGTH = 128K - 16
	RAM_DMA (rw)  : ORIGIN = 0x24000000, LENGTH = 32K
	RAM_D1  (rw)  : ORIGIN = 0x24008000, LENGTH = 96K
}

INCLUDE sections.ld
