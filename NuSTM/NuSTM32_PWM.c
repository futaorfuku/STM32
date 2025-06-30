#include "NuSTM32_PWM.h"
#include "NuSTM32_JSON.h"
#define DEVICELOOP_HZ				328125 //218750 // 134400 // 187500 // 328125
#define DEVICE_CLOCKCYCLES	(TIM1_CLOCK / DEVICELOOP_HZ)

PWM_MANAGER mgrPWM;
uint16_t NuGetBLDCPins(void);


//////////////////////////////
// PWM

void NuEnableTIMCC_NVIC(uint32_t nPortPWM)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	switch (nPortPWM)
	{
	case ST_PWM_1:
		NVIC_InitStruct.NVIC_IRQChannel = TIM1_CC_IRQn;
		break;
	case ST_PWM_8:
		NVIC_InitStruct.NVIC_IRQChannel = TIM8_CC_IRQn;
		break;
	}
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStruct);
}

uint32_t NuPWMChannelConfig(uint32_t nPortTimer, uint32_t uHz, int nChannels, uint16_t nDutyInit)
{
	PWMOCInit pwmo;
	uint32_t nTimerPeriod = NuTimeBaseHzConfig(&pwmo.TIM_TimeBaseStructure, uHz, TIM_CounterMode_Up);;
	for (int i = 0; i < nChannels; i++)
		pwmo.ChannelPulse[i] = nDutyInit;
	pwmo.m_nPortTimer = nPortTimer;
	NuTimeOCInitStructure(&pwmo.TIM_OCInitStructure, TIM_OCMode_PWM1, true, TIM_OCIdleState_Reset, TIM_OCNIdleState_Reset);
	NuPWMChannelInit(&pwmo, nChannels);
	return nTimerPeriod;
}

void NuPWMOutputEnable(uint32_t nPortTimer)
{
	// Brake
	TIM_TypeDef* TIMx = GetTIM_TypeDef(nPortTimer);
	NuTIMBDTRInit(TIMx, 0);
	// Init
	TIM_CCPreloadControl(TIMx, ENABLE);
	TIM_Cmd(TIMx, ENABLE);
	TIM_CtrlPWMOutputs(TIMx, ENABLE);
}

void NuPWMChannelInit(PWMOCInit* p, int nChannels)
{
	TIM_TypeDef* TIMx = GetTIM_TypeDef(p->m_nPortTimer);
	TIM_TimeBaseInit(TIMx, &p->TIM_TimeBaseStructure);	
	void (*TIM_OCInit)(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
	void* function[4] = {TIM_OC1Init, TIM_OC2Init, TIM_OC3Init, TIM_OC4Init};
	for (int i = 0; i < nChannels; i++)
	{
		p->TIM_OCInitStructure.TIM_Pulse = p->ChannelPulse[i];
		TIM_OCInit = function[i];
		TIM_OCInit(TIMx, &p->TIM_OCInitStructure);
	}
}

void NuPWMChannelEnable(PWMOCInit* p, int nChannels)
{
	TIM_TypeDef* TIMx = GetTIM_TypeDef(p->m_nPortTimer);
	NuPWMChannelInit(p, nChannels);
  TIM_SelectMasterSlaveMode(TIMx, TIM_MasterSlaveMode_Enable);
	TIM_Cmd(TIMx, ENABLE);
	TIM_CtrlPWMOutputs(TIMx, ENABLE);
}

void NuPWMMasterChannelEnable(PWMOCInit* p, int nChannels)
{
	TIM_TypeDef* TIMx = GetTIM_TypeDef(p->m_nPortTimer);
	NuPWMChannelInit(p, nChannels);
	TIM_SelectMasterSlaveMode(TIMx, TIM_MasterSlaveMode_Enable);
	NuTimeOCInitStructure(&p->TIM_OCInitStructure, TIM_OCMode_Active, false, NULL, NULL);
	TIM_OC3Init(TIMx, &p->TIM_OCInitStructure);
	TIM_SelectOutputTrigger(TIMx, TIM_TRGOSource_OC3Ref);
	TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
}

void NuPWMSlaveChannelEnable(PWMOCInit* pSlave, int nChannels, PWMOCInit* pMaster, int nMasterCounter,
		int nSlaveCounter)
{
	TIM_TypeDef* TimMaster = GetTIM_TypeDef(pMaster->m_nPortTimer);
	TIM_TypeDef* TimSlave = GetTIM_TypeDef(pSlave->m_nPortTimer);
	uint16_t InputTimerTriggers[] = {TIM_TS_ITR0, TIM_TS_ITR1, TIM_TS_ITR2, TIM_TS_ITR3};
	NuPWMChannelInit(pSlave, nChannels);
 	TIM_SelectSlaveMode(TimSlave, TIM_SlaveMode_Trigger);
	TIM_SelectInputTrigger(TimSlave, InputTimerTriggers[pMaster->m_nPortTimer - ST_TIMER_1]);
	TIM_Cmd(TimMaster, ENABLE); // TIM1 Main Output Enable
	TIM_Cmd(TimSlave, ENABLE); // TIM8 Main Output Enable
	TIM_SetCounter(TimMaster, nMasterCounter);
	TIM_SetCounter(TimSlave, nSlaveCounter);
	TIM_CtrlPWMOutputs(TimMaster, ENABLE);
  TIM_CtrlPWMOutputs(TimSlave, ENABLE);
} 

PWMOCInit PWMTIM_2ChannelConfig(PWMSpec pwms, bool bNeedReport)
{
	NuEnableTIMCC_NVIC(pwms.m_nPortPWM);
	NuEnableAFPPA(pwms.m_gppa, pwms.m_nPortPWM == ST_PWM_1? GPIO_AF_TIM1: GPIO_AF_TIM8);
	PWMOCInit pwmo;
	pwmo.m_nPortTimer = NULL;
	NuTimeBaseHzConfig(&pwmo.TIM_TimeBaseStructure, pwms.m_uHz, TIM_CounterMode_CenterAligned1);
	uint32_t Duty[2] = {pwms.m_uDuty - pwms.m_uDeadTime / 2, pwms.m_uDuty + pwms.m_uDeadTime / 2};	
	if (pwms.m_nClockCycles >= 0x10000)
	{
		if (bNeedReport)
			JsonWriteIntoJsonPair("ERROR", "FREQUENCY TOO LOW");
		return pwmo;
	}
	for (int i = 0; i < 2; i++)
		pwmo.ChannelPulse[i] = pwms.m_nClockCycles * Duty[i] / 1000.;
	NuTimeOCInitStructure(&pwmo.TIM_OCInitStructure, TIM_OCMode_PWM1, true, TIM_OCIdleState_Reset, TIM_OCNIdleState_Reset);
	pwmo.m_nPortTimer = pwms.m_nPortPWM - ST_PWM_1 + ST_TIMER_1;
	NuEnableRCCClock(pwmo.m_nPortTimer);
	if (bNeedReport)
	{
		JsonWriteIntIntoJsonPair("Period(ns)", CyclesToNanoSecond(pwms.m_nClockCycles));
		for (int i = 0; i < 2; i++)
		{
			JsonWriteIntIntoJsonPair("CH", i + 1);
			JsonWriteIntIntoJsonPair("ON(ns)", CyclesToNanoSecond(pwmo.ChannelPulse[i]));
			JsonWriteIntIntoJsonPair("Error(ns)",	CyclesToNanoSecond(pwms.m_nClockCycles * Duty[i] / 1000. - pwmo.ChannelPulse[i]));
		}
	}
	return pwmo;
}

void NuPWMTIM18Config(PWMSpec pwms1, PWMSpec pwms2, int nMasterCounter, int nSlaveCounter, bool bNeedReport)
{
	PWMOCInit pwmo1 = PWMTIM_2ChannelConfig(pwms1, bNeedReport);
	if (!pwmo1.m_nPortTimer)
		return;
	PWMOCInit pwmo8 = PWMTIM_2ChannelConfig(pwms2, bNeedReport);
	NuPWMMasterChannelEnable(&pwmo1, pwms1.m_gppa.m_nPins);
	NuPWMSlaveChannelEnable(&pwmo8, pwms2.m_gppa.m_nPins, &pwmo1, nMasterCounter, nSlaveCounter);
}

void NuOPModeEnableCLLCPWM(PWM_MANAGER* pMgr, int32_t nDutyDelay13, bool bNeedReport)
{
	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	pMgr->m_nCallFunction = PWM_CALL_HALF_BRIDGE;
	int nCenterOffsetCycles = (int)(pMgr->m_pwmSpec[0].m_uDeadTime_ns - pMgr->m_pwmSpec[1].m_uDeadTime_ns) / 12;
	int nMasterCounter = 30;
	int nSlaveCounter = nDutyDelay13 / 6 + 39 - nCenterOffsetCycles;
	NuPWMTIM18Config(pMgr->m_pwmSpec[0], pMgr->m_pwmSpec[1], nMasterCounter, nSlaveCounter, bNeedReport);
	JsonWriteIntoJsonPair("CLLCPWM", "ON"); 
}

void NuPWMTIM1Config(uint32_t nPort, uint32_t uHz, uint32_t uDuty1, uint32_t uDeadTime1, uint32_t uDuty2, 
	uint32_t uDeadTime2, GPortPinArray ppa)
{
	NuEnableTIMCC_NVIC(nPort);
	NuEnableAFPPA(ppa, nPort == ST_PWM_1? GPIO_AF_TIM1: GPIO_AF_TIM8);
	PWMOCInit pwmo;
	uint32_t TimerPeriod = NuTimeBaseHzConfig(&pwmo.TIM_TimeBaseStructure, uHz, TIM_CounterMode_CenterAligned1);;
	uint32_t Duty[4] = {
		1000 - uDuty1 + uDeadTime1 / 2, uDuty1 - uDeadTime1 / 2, 
		1000 - uDuty2 + uDeadTime2 / 2, uDuty2  - uDeadTime2 / 2};	
  for (int i = 0; i < 4; i++)
		pwmo.ChannelPulse[i] = (uint16_t) (((uint32_t) Duty[i] * (TimerPeriod - 1)) / 1000);
	pwmo.m_nPortTimer = ST_TIMER_1;
	NuTimeOCInitStructure(&pwmo.TIM_OCInitStructure, TIM_OCMode_PWM2, true, TIM_OCIdleState_Reset, TIM_OCNIdleState_Reset);
	NuPWMChannelEnable(&pwmo, 4);
}

void NuOPModeEnablePWM(uint32_t uHz, uint32_t uDuty1, uint32_t uDeadTime1, uint32_t uDuty2, 
	uint32_t uDeadTime2)
{
	GPortPinArray ppa = {2, A8, B14};
	NuPWMTIM1Config(ST_PWM_1, uHz, uDuty1, uDeadTime1, uDuty2, uDeadTime2, ppa);
	JsonWriteIntoJsonPair("PWM", "ON"); 
}

static __IO uint32_t* m_ppIDR[MAX_PWM_CHANNELS];
static uint16_t m_ppGPIO[MAX_PWM_CHANNELS];
static uint32_t add[8] = {1, 2, 4, 8, 16, 32, 64, 128};
static uint32_t m_PWMFIFORingBuffer[FIFOSIZE];
static uint32_t* m_pPWMFIFORingBuffer;

void NuPWMManagerSnapShotStart()
{
	mgrPWM.m_nPWMFIFOCount = 0;
	mgrPWM.m_bSnapShot = true;
	mgrPWM.m_bDownloading = false;
	uint32_t nCoprimeGCD = GCD(DEVICELOOP_HZ, mgrPWM.m_nHzSignal);
	mgrPWM.m_nBufferSize = DEVICELOOP_HZ / nCoprimeGCD;
	m_pPWMFIFORingBuffer = &m_PWMFIFORingBuffer[0];
	bProtectedSessionInProcess = true;
	mgrPWM.m_nppp = 0;
	memset(m_PWMFIFORingBuffer, 0, sizeof(uint32_t) * mgrPWM.m_nBufferSize);
	for (int i = 0; i < mgrPWM.m_nTimers; i++)
	{
		for (int j = 0; j < mgrPWM.m_pwmSpec[i].m_ppa.m_nPins; j++)
		{
			PortPin pp = mgrPWM.m_pwmSpec[i].m_ppa.m_pp[j];
			m_ppIDR[mgrPWM.m_nppp] = &pp.m_typedef->IDR;
			m_ppGPIO[mgrPWM.m_nppp] = pp.m_GPIO;
			mgrPWM.m_nppp++;
		}
	}
	NuEnableIRQTIM(ST_TIMER_4, DEVICE_CLOCKCYCLES);
}

void NuGetPWMPins(uint32_t* pByte)
{
	uint32_t nPinValue[8] = {
		*m_ppIDR[0] & m_ppGPIO[0], 
		*m_ppIDR[1] & m_ppGPIO[1], 
		*m_ppIDR[2] & m_ppGPIO[2], 
		*m_ppIDR[3] & m_ppGPIO[3],};
	for (int i = 0; i < mgrPWM.m_nppp; i++)
		*pByte += nPinValue[i]? add[i]: 0;
}

void NuPWMManagerSnapShotSave()
{
	NuGetPWMPins(m_pPWMFIFORingBuffer);
	m_pPWMFIFORingBuffer++;
	mgrPWM.m_nPWMFIFOCount++;
	if (mgrPWM.m_nPWMFIFOCount >= mgrPWM.m_nBufferSize)
	{
		TIM_DeInit(TIM4);
		bProtectedSessionInProcess = false;
		mgrPWM.m_nFIFOStart = 0;
		mgrPWM.m_nTransferSize = 40;
		mgrPWM.m_bDownloading = true;
		mgrPWM.m_bSnapShot = false;		
	}
}


////////////////////////////////////////
// JSON

bool bJDownloadPWMInProcess = false;

void JPrintPWM(DWORD param[])
{
	char szText[MAX_JSON_VALUE];
	memset(szText, 0, MAX_JSON_VALUE);
	memcpy(szText, (void*) param[0], MAX_JSON_VALUE);
	if (strFind(szText, "ON") == 0)
		NuPWMManagerSnapShotStart();
}

void JDownloadPWM()
{
	if (!mgrPWM.m_bDownloading)
		return;
	if (bJDownloadPWMInProcess)
		return;
	bJDownloadPWMInProcess = true;
	JsonWriteOpen();
	if (mgrPWM.m_nFIFOStart == 0)
	{
		char szPin[MAX_JSON_VALUE];
		memset(szPin, 0, MAX_JSON_VALUE);
		JsonWriteIntoJsonPair("Chunk", mgrPWM.m_nCallFunction == PWM_CALL_HALF_BRIDGE? "PWM": "BLDC"); 
		JsonWriteIntIntoJsonPair("FS", DEVICELOOP_HZ); 
		JsonWriteIntIntoJsonPair("FX", mgrPWM.m_nHzSignal); 
	}
	char szText[MAX_JSON_VALUE];
	memset(szText, 0, MAX_JSON_VALUE);
	int k = 0;
	for (int j = mgrPWM.m_nFIFOStart; k < mgrPWM.m_nTransferSize && j < mgrPWM.m_nBufferSize; j++, k++)
	{
		strcat(szText, k == 0? "[": ",");				
		char szPin[MAX_JSON_VALUE];
		memset(szPin, 0, MAX_JSON_VALUE);
		sprintf(szPin, "%X", m_PWMFIFORingBuffer[j]);
		strcat(szText, szPin);
		mgrPWM.m_nFIFOStart++;
	}
	strcat(szText, "]");
	JsonWriteIntoJsonPair(mgrPWM.m_nCallFunction == PWM_CALL_HALF_BRIDGE? "PWM": "BLDC", szText);
	if (mgrPWM.m_nFIFOStart >= mgrPWM.m_nBufferSize)
	{
		JsonWriteIntoJsonPair("Chunk", "End");
		JsonWriteClose();
		mgrPWM.m_bDownloading = false;
	}
	else
		JsonWriteClose();
	nJsonWriteEnd = strlen(szJsonWriteLeft);
	bJDownloadPWMInProcess = false;
}

void RecalculatePWMSpec(PWMSpec* ppwms)
{
	ppwms->m_nClockCycles = (int) (TIM1_CLOCK / ppwms->m_uHz);
	ppwms->m_uHz = TIM1_CLOCK / ppwms->m_nClockCycles;
	ppwms->m_uDeadTime = (uint64_t) (ppwms->m_uHz * ppwms->m_uDeadTime_ns * 2) / 1e6;
	ppwms->m_ppa.m_nPins = ppwms->m_gppa.m_nPins;
	for (int j = 0; j < ppwms->m_gppa.m_nPins; j++)
		ppwms->m_ppa.m_pp[j] = PP(ppwms->m_gppa.m_gpp[j]);
}

void RecalculatePWMManager(PWM_MANAGER* pMgr, int nTimers)
{
	pMgr->m_nTimers = nTimers;
	for (int i = 0; i < nTimers; i++)
		RecalculatePWMSpec(&pMgr->m_pwmSpec[i]);
}

