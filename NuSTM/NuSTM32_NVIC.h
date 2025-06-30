#ifndef __NUSTM32_NVIC_H
#define __NUSTM32_NVIC_H
#include "Nu_STM32Def.h"

// NVIC
void NuNVICConfig(uint32_t NVIC_PriorityGroup, uint8_t nvic_IRQChannel, uint8_t nvic_IRQChannelPreemptionPriority, 
	uint8_t nvic_IRQChannelSubPriority, FunctionalState nvic_IRQChannelCmd);


#endif


