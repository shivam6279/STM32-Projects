#ifndef _MPU_H_
#define _MPU_H_

// DMA buffers go in .dma_buf (first 32KB of AXI SRAM), mapped non-cacheable by the MPU.
// Means no cache clean needed before DMA reads them, which is easy to forget in an ISR.
#define DMA_BUF __attribute__((section(".dma_buf"), aligned(32)))
// Big CPU-only data in the spare AXI SRAM (RAM_D1, cached, not zeroed at boot)
#define RAM_D1 __attribute__((section(".ram_d1")))

void MpuInit();

#endif
