#ifndef __NUSTM32_BASIC_H
#define __NUSTM32_BASIC_H
#include "Nu_STM32Def.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "NuSTM32_CLK.h"
#include "NuSTM32_GPIO.h"
#include "NuSTM32_TIM.h"
#include "NuSTM32_NVIC.h"
#include "NuSTM32_DMA.h"

/* ADC CDR register base address */
#define OPMODE_ADC		0x1
#define OPMODE_CAN1		0x2
#define OPMODE_PWM		0x4
#define OPMODE_USART	0x10
#define OPMODE_DAC		0x20
#define OPMODE_CAN2		0x80
#define OPMODE_ITRI_APP		0x100
#define OPMODE_ELAN_APP		0x200


///////////////////////
// MAIN
extern uint16_t ADCFIFORingBuffer[];
extern bool bProtectedSessionInProcess;
extern uint16_t nOperationMode;
extern uint64_t nMilliSecondCnt;
extern bool m_bSystemReportInited;
extern bool m_bAPPPowerOn;
void NuInit(int nFastLoopHz, int nNormalLoopHz);
double DACChannel1(void);
double DACChannel2(void);
void STInit(int nFastLoopHz, int nNormalLoopHz);

#endif


