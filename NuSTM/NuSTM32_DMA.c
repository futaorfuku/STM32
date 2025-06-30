#include "Nu_STM32.h"

uint32_t GetDMAChannel(uint32_t nChannel)
{
	uint32_t DMA_Channels[] = {
		DMA_Channel_0, DMA_Channel_1, DMA_Channel_2, DMA_Channel_3,
		DMA_Channel_4, DMA_Channel_5, DMA_Channel_6, DMA_Channel_7};
	return DMA_Channels[nChannel];
}

DMA_Stream_TypeDef* NuGetDMAStream(uint16_t nPort, uint32_t nStream)
{
	DMA_Stream_TypeDef* DMA1_Streams[] = {
		DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
		DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7};
	DMA_Stream_TypeDef* DMA2_Streams[] = {
		DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
		DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
	DMA_Stream_TypeDef** DMAS_Streams[] = {DMA1_Streams, DMA2_Streams};
	return DMAS_Streams[nPort - ST_DMA_1][nStream];
}

void NuEnableDMA(uint16_t nPort, uint16_t nDMAChannel, uint32_t nStream, uint32_t dmaPeripheralBaseAddr,
	uint32_t dmaMemory0BaseAddr, uint32_t dmaBufferSize, uint32_t dmaDIR)
{			
	DMA_Stream_TypeDef* DMA_STREAMx = NuGetDMAStream(nPort, nStream);
	DMA_DeInit(DMA_STREAMx);
	NuEnableRCCClock(nPort);
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = GetDMAChannel(nDMAChannel);
	DMA_InitStructure.DMA_Memory0BaseAddr = dmaMemory0BaseAddr; 
	DMA_InitStructure.DMA_PeripheralBaseAddr = dmaPeripheralBaseAddr;
	DMA_InitStructure.DMA_DIR = dmaDIR;
	DMA_InitStructure.DMA_BufferSize = dmaBufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA_STREAMx, &DMA_InitStructure);
	//
	DMA_Cmd(DMA_STREAMx, ENABLE);
}

