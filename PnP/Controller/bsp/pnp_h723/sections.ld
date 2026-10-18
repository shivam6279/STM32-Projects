/* Shared sections for the app and the bootloader. MEMORY comes from mem_app.ld / mem_boot.ld.
 *
 * DTCM: stack, .data, .bss (CPU only; DMA1/2 can't reach DTCM). First 16 bytes are reserved for
 * BOOT_FLAG (platform/bootflag.h), shared between bootloader and app.
 * RAM_DMA: DMA buffers, MPU non-cacheable (platform/mpu.c). Size must match the MPU region.
 * RAM_D1: spare AXI SRAM for big CPU data.
 */

ENTRY(Reset_Handler)

_Min_Stack_Size = 0x2000;

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);

SECTIONS
{
	.isr_vector :
	{
		. = ALIGN(4);
		KEEP(*(.isr_vector))
		. = ALIGN(4);
	} >FLASH

	.text :
	{
		. = ALIGN(4);
		*(.text)
		*(.text*)
		*(.glue_7)
		*(.glue_7t)
		*(.eh_frame)
		KEEP(*(.init))
		KEEP(*(.fini))
		. = ALIGN(4);
		_etext = .;
	} >FLASH

	.rodata :
	{
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} >FLASH

	.ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
	.ARM :
	{
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} >FLASH

	.preinit_array :
	{
		PROVIDE_HIDDEN(__preinit_array_start = .);
		KEEP(*(.preinit_array*))
		PROVIDE_HIDDEN(__preinit_array_end = .);
	} >FLASH
	.init_array :
	{
		PROVIDE_HIDDEN(__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array*))
		PROVIDE_HIDDEN(__init_array_end = .);
	} >FLASH
	.fini_array :
	{
		PROVIDE_HIDDEN(__fini_array_start = .);
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array*))
		PROVIDE_HIDDEN(__fini_array_end = .);
	} >FLASH

	_sidata = LOADADDR(.data);

	.data :
	{
		. = ALIGN(4);
		_sdata = .;
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} >DTCMRAM AT> FLASH

	.bss (NOLOAD) :
	{
		. = ALIGN(4);
		_sbss = .;
		__bss_start__ = _sbss;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		__bss_end__ = _ebss;
	} >DTCMRAM

	/* Not touched by startup, keeps its value across a system reset */
	.noinit (NOLOAD) :
	{
		. = ALIGN(4);
		*(.noinit)
		*(.noinit*)
		. = ALIGN(4);
	} >DTCMRAM

	/* No heap: malloc is not used. end/_end provided for newlib's _sbrk symbol only. */
	._user_stack (NOLOAD) :
	{
		. = ALIGN(8);
		PROVIDE(end = .);
		PROVIDE(_end = .);
		. = . + _Min_Stack_Size;
		. = ALIGN(8);
	} >DTCMRAM

	/* Not zeroed or initialized by startup: owners fill before DMA starts. */
	.dma_buf (NOLOAD) :
	{
		__dma_buf_region_start__ = .;
		*(.dma_buf)
		*(.dma_buf*)
		__dma_buf_end__ = .;
	} >RAM_DMA

	.ram_d1 (NOLOAD) :
	{
		*(.ram_d1)
		*(.ram_d1*)
	} >RAM_D1

	.ARM.attributes 0 : { *(.ARM.attributes) }
}

ASSERT(__dma_buf_region_start__ == ORIGIN(RAM_DMA), "RAM_DMA must start at the MPU region base")
ASSERT(ORIGIN(RAM_DMA) % LENGTH(RAM_DMA) == 0, "MPU region base must be size-aligned")
