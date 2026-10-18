#ifndef _MPU_H_
#define _MPU_H_

// DMA buffers go in .dma_buf (first 32KB of AXI SRAM), mapped non-cacheable by the MPU.
// Means no cache clean needed before DMA reads them, which is easy to forget in an ISR.
#define DMA_BUF __attribute__((section(".dma_buf"), aligned(32)))

void MpuInit();

#endif
