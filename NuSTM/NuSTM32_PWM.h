#ifndef __NUSTM32_PWM_H
#define __NUSTM32_PWM_H
#include "NuSTM32_BASIC.h"
#define ST_PWM_1		0x41
#define ST_PWM_8		0x48
#define MAX_PWM_CHANNELS	6

typedef struct
{
	uint32_t m_nPortPWM;
	uint32_t m_uHz;
	uint32_t m_uDuty;
	uint32_t m_uDeadTime_ns;
	GPortPinArray m_gppa;
	uint32_t m_uDeadTime;
	uint32_t m_nClockCycles;
	PortPinArray m_ppa;
} PWMSpec;

typedef struct
{
	bool m_bSnapShot;
	bool m_bDownloading;
	int m_nFIFOStart;
	int m_nTimers;
	int m_nBufferSize;
	int m_nTransferSize;
	uint16_t m_nPWMFIFOCount;
	uint32_t m_nModePWM;
	uint16_t m_nCallFunction;
	uint32_t m_nHzSignal;
	PWMSpec m_pwmSpec[2];	

	int m_nppp;
} PWM_MANAGER;

typedef struct
{
	uint32_t m_nPortTimer;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	uint16_t ChannelPulse[MAX_PWM_CHANNELS];
} PWMOCInit;


extern PWM_MANAGER mgrPWM;

void NuPWMOutputEnable(uint32_t nPortTimer);
uint32_t NuPWMChannelConfig(uint32_t nPortTimer, uint32_t uHz, int nChannels, uint16_t nDutyInit);
void NuPWMManagerSnapShotStart(void);
void NuPWMManagerSnapShotSave(void);
bool NuPWMManagerSnapShotCompleted(void);
void NuOPModeEnableCLLCPWM(PWM_MANAGER* pMgr, int32_t nDutyDelay13, bool bNeedReport);
void NuPWMChannelInit(PWMOCInit* p, int nChannels);
void NuEnableTIMCC_NVIC(uint32_t nPortPWM);
///////////////////////////////
// JSON
void JPrintPWM(DWORD param[]);
void JDownloadPWM(void);
void RecalculatePWMSpec(PWMSpec* ppwms);
void RecalculatePWMManager(PWM_MANAGER* pMgr, int nTimers);
#endif


