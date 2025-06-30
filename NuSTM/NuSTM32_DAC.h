#ifdef _USE_DAC_
#ifndef __NUSTM32_DAC_H
#define __NUSTM32_DAC_H
#include "NuSTM32_BASIC.h"

#define DAC_Channel_All ((uint32_t)0x00000011)
uint32_t NuGetDACFIFOAddress(void);
void NuOPModeEnableDAC(uint32_t nDACChannel, uint32_t uClockCycle, double (*channel1)(), double (*channel2)());
void NuDACConfig(uint32_t nDACChannel, uint32_t uClockCycle);
void DACCallBack(void);
void JSetDAC12(DWORD param[]);
#endif
#endif

