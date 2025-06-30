#ifndef __NUSTM32_CLK_H
#define __NUSTM32_CLK_H
#include "Nu_STM32Def.h"

#ifdef CODE_F103
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#else
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#endif

typedef struct
{
	uint32_t clock;
	uint32_t bus;
} STM32407G_BUS;

void NuEnableRCCClock(uint16_t nPort);


#endif


