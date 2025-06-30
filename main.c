#include "ITRI_APP.h"
#include "ELAN_APP.h"
uint16_t nOperationMode = OPMODE_USART | OPMODE_ITRI_APP;// | OPMODE_ITRI_APP;// | OPMODE_DAC;// | OPMODE_ADC;
uint64_t nFastLoopCnt = 0;
uint64_t nNormalLoopCnt = 0;
bool m_bAPPPowerOn = true;
bool m_bSystemReportInited = false;

void PLC_FastLoop(void)
{
	nFastLoopCnt++;
	if (bProtectedSessionInProcess)
		return;
	Clock(D12);
	if (!JPopWriteBufferEmpty(30))
		return;
	//if (mgrADC.m_bFIFOADC)
		//JDownloadADC();
	//if (mgrPWM.m_bDownloading)
		//JDownloadPWM();
	if (nOperationMode & OPMODE_ITRI_APP)
		ITRICarFastLoop(nFastLoopCnt);
	/*
	if (nOperationMode & OPMODE_ELAN_APP)
		ELANCLLCFastLoop();
	*/
}

void PLC_NormalLoop(void)
{
	nNormalLoopCnt++;
	if (bProtectedSessionInProcess)
		return;	
	if (nOperationMode & OPMODE_ITRI_APP)
	{
		if (nNormalLoopCnt % 2 == 0)
			ITRICarNormalLoop();		
		if (nNormalLoopCnt % 20 == 0)
			ITRICarReport();
	}
	/*
	if (nOperationMode & OPMODE_ELAN_APP)
	{
		if (nNormalLoopCnt % 20 == 0)
		{
			ELANCLLCNormalLoop();	
			ELANCLLCReport();
		}
	}
	*/
	if (nNormalLoopCnt % 10 && m_bAPPPowerOn != NuGetBit(C4))
	{
		m_bAPPPowerOn = !m_bAPPPowerOn;
		fprintf(USART_FILE, "{\"P\"=\"%d\"}", m_bAPPPowerOn);
	}
}

int main(void)
{
	STInit(0, 0);
	if (nOperationMode & OPMODE_ITRI_APP)
		ITRICarMain();
	/*
	if (nOperationMode & OPMODE_ELAN_APP)
		ELANCLLCMain();
	*/
	if (nOperationMode & OPMODE_USART)
		NuOPModeEnableUSART(USART_PORT);
#ifdef _USE_DAC_
	if (nOperationMode & OPMODE_DAC)
		NuOPModeEnableDAC(DAC_Channel_1, 256, DACChannel1, DACChannel2);
#endif
#ifdef _USE_CAN_
	if (nOperationMode & OPMODE_CAN1)
		NuOPModeEnableCAN(ST_CAN_1, CAN_NODE_A);
	if (nOperationMode & OPMODE_CAN2)
		NuOPModeEnableCAN(ST_CAN_2, CAN_NODE_B);
#endif
	JsonWriteClose();
	JsonPrint();
	bProtectedSessionInProcess = false;
	while (true)
  { 
  }
}

void SysTick_Handler(void)
{
	nMilliSecondCnt++;
  if (nOperationMode & OPMODE_ITRI_APP)
	{
		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
		TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
	}
}
