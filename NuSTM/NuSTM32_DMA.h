#ifndef __NUSTM32_DMA_H
#define __NUSTM32_DMA_H
#include "Nu_STM32Def.h"

// DMA
void NuEnableDMA(uint16_t nPort, uint16_t nDMAChannel, uint32_t nStream,	uint32_t DMA_PeripheralBaseAddr,
	uint32_t DMA_Memory0BaseAddr, uint32_t DMA_BufferSize, uint32_t DMA_DIR);
#endif


