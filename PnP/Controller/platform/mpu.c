#include "mpu.h"
#include "stm32h7xx.h"

extern uint32_t __dma_buf_region_start__;

void MpuInit() {
	ARM_MPU_Disable();

	// Region 0: DMA buffers (step, ADC, ethernet), 32KB, normal non-cacheable (TEX=1 C=0 B=0), no exec
	ARM_MPU_SetRegion(ARM_MPU_RBAR(0, (uint32_t)&__dma_buf_region_start__),
		ARM_MPU_RASR(1, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0x00, ARM_MPU_REGION_SIZE_32KB));

	ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk); // default map for everything else

	SCB_EnableICache();
	SCB_EnableDCache();
}
