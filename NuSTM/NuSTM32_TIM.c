#include "NuSTM32_BASIC.h"
#include "NuSTM32_TIM.h"
#include "NuSTM32_PWM.h"

#define SYSTEM_HZ								(4 _MHz)
#define SYSTEM_CLOCKCYCLES			(TIM1_CLOCK / SYSTEM_HZ)
#define NORMALLOOP_CLOCKCYCLES 	(TIM1_CLOCK / NORMALLOOP_HZ)
#define FASTLOOP_CLOCKCYCLES 		(TIM1_CLOCK / FASTLOOP_HZ)

double dRealTimeClock = 0;

TIM_TypeDef* GetTIM_TypeDef(uint32_t nPort)
{
	TIM_TypeDef* TIMxs[] = {TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9, TIM10, TIM11, TIM12, TIM13, TIM14};
	return TIMxs[nPort - ST_TIMER_1];
}

void SetTimePeriod(TIM_TimeBaseInitTypeDef* pTIM_TimeBaseStructure, double dClockCycles)
{
	if (dClockCycles < 0x1000)
	{
		pTIM_TimeBaseStructure->TIM_Prescaler = 0;
		pTIM_TimeBaseStructure->TIM_Period = dClockCycles - 1;
	}
	else
	{
		double dX = dClockCycles / 65535;
		int nX = dX;
		if (nX != dX)
			nX++;
		pTIM_TimeBaseStructure->TIM_Prescaler = nX - 1;
		pTIM_TimeBaseStructure->TIM_Period = dClockCycles / nX - 1;
	}
}

void NuTIMInitConfig(uint16_t nPort, uint32_t uCycles, uint16_t tim_CounterMode, 
  uint16_t tim_ClockDivision, uint8_t tim_RepetitionCounter)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TypeDef* TIMx = GetTIM_TypeDef(nPort);
	TIM_DeInit(TIMx);
	SetTimePeriod(&TIM_TimeBaseInitStrue, uCycles);
	TIM_TimeBaseInitStrue.TIM_CounterMode = tim_CounterMode;	
	TIM_TimeBaseInitStrue.TIM_ClockDivision = tim_ClockDivision;
	TIM_TimeBaseInitStrue.TIM_RepetitionCounter = tim_RepetitionCounter;
	NuEnableRCCClock(nPort);	
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStrue);	
}

void NuEnableTIM(uint16_t nPort, uint32_t uCycles, uint16_t tim_CounterMode, 
  uint16_t tim_ClockDivision, uint8_t tim_RepetitionCounter)
{ //https://zhuanlan.zhihu.com/p/480892146
	TIM_TypeDef* TIMx = GetTIM_TypeDef(nPort);
	NuTIMInitConfig(nPort, uCycles, tim_CounterMode, tim_ClockDivision, tim_RepetitionCounter);
	TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIMx, ENABLE);
}

void NuEnableIRQTIM(uint16_t nPort, uint32_t uCycles)
{
	NuEnableTIM(nPort, uCycles, TIM_CounterMode_Up, TIM_CKD_DIV1, TIM_OPMode_Repetitive);
	NuEnableTIM_NVIC(nPort, NVIC_PriorityGroup_0, 1);
}

void NuEnableTIMTRO(uint16_t nPort, uint32_t uCycles, uint16_t tim_CounterMode, 
  uint16_t tim_ClockDivision, uint8_t tim_RepetitionCounter)
{ //https://zhuanlan.zhihu.com/p/480892146
	TIM_TypeDef* TIMx = GetTIM_TypeDef(nPort);
	NuTIMInitConfig(nPort, uCycles, tim_CounterMode, tim_ClockDivision, tim_RepetitionCounter);
 	TIM_SelectOutputTrigger(TIMx, TIM_TRGOSource_Update);
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIMx, ENABLE);
}

uint32_t NuTimeBaseHzConfig(TIM_TimeBaseInitTypeDef* pTIM_TimeBaseStructure, uint32_t uHz, uint16_t tim_CounterMode)
{
  double dTimerPeriod = (double) 168000000 / uHz; /* Compute the value to be set in ARR register to generate signal frequency uHz */
	SetTimePeriod(pTIM_TimeBaseStructure, dTimerPeriod);
	pTIM_TimeBaseStructure->TIM_CounterMode = tim_CounterMode;
	pTIM_TimeBaseStructure->TIM_ClockDivision = 0;
	pTIM_TimeBaseStructure->TIM_RepetitionCounter = 0;
	return dTimerPeriod;
}

void NuTimeOCInitStructure(TIM_OCInitTypeDef* pTIM_OCInitStructure, uint16_t tim_OCMode, bool bStateEnable,
	uint16_t tim_OCIdleState, uint16_t tim_OCNIdleState)
{
  pTIM_OCInitStructure->TIM_OCMode = tim_OCMode;
  pTIM_OCInitStructure->TIM_OutputState = bStateEnable? TIM_OutputState_Enable: TIM_OutputState_Disable;
  pTIM_OCInitStructure->TIM_OutputNState = bStateEnable? TIM_OutputNState_Enable: TIM_OutputNState_Disable;
  pTIM_OCInitStructure->TIM_OCPolarity = TIM_OCPolarity_High;
  pTIM_OCInitStructure->TIM_OCNPolarity = TIM_OCNPolarity_High;
  pTIM_OCInitStructure->TIM_OCIdleState = tim_OCIdleState;
  pTIM_OCInitStructure->TIM_OCNIdleState = tim_OCNIdleState;
}

void NuTIMBDTRInit(TIM_TypeDef* TIMx, uint16_t tim_DeadTime)
{
	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitTypeDef   TIM_BDTRInitStructure;
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  TIM_BDTRInitStructure.TIM_DeadTime = tim_DeadTime;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIMx, &TIM_BDTRInitStructure);
}

uint32_t nHalfSecondCnt = 0;

void Clock(GPortPin gpp)
{
	nHalfSecondCnt++;
	if (nHalfSecondCnt == 5000)
	{
		nHalfSecondCnt = 0;
		NuSetBit(gpp, !NuGetBit(gpp));
	}
}

uint64_t CyclesToNanoSecond(uint32_t nClockCycles)
{
	return (uint64_t) nClockCycles * 1000000000 / TIM1_CLOCK;
}


void NuInit(int nFastLoopHz, int nNormalLoopHz)
{
#ifdef JSON_APP
	JsonInit();
#endif
	mgrPWM.m_nCallFunction = 0;
	if (nFastLoopHz == 0)
		NuEnableIRQTIM(ST_TIMER_5, FASTLOOP_CLOCKCYCLES);
	else
		NuEnableIRQTIM(ST_TIMER_5, TIM1_CLOCK / nFastLoopHz);
	if (nNormalLoopHz == 0)
		NuEnableIRQTIM(ST_TIMER_7, NORMALLOOP_CLOCKCYCLES);
	else
		NuEnableIRQTIM(ST_TIMER_7, TIM1_CLOCK / nNormalLoopHz);
	NuEnableIRQTIM(ST_TIMER_2, SYSTEM_CLOCKCYCLES);

}

void NuTimeReport(char szText[MAX_JSON_VALUE])
{
	memset(szText, 0, MAX_JSON_VALUE);
	int nSeconds = nMilliSecondCnt / 1000;
	sprintf(szText, "%02d:%02d:%02d", nSeconds / 3600, nSeconds / 60 % 60, nSeconds % 60);
}
