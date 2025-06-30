#ifdef ELAN_APP
#include "ELAN_APP.h"

int32_t m_nTIM18Offset_ns = 0;

bool ELANCLLCNormalLoopInProcess = false;
bool ELANCLLCFastLoopInProcess = false;


void ELANCLLCReportWrite(bool bReadWrite)
{ 
	char szText[MAX_JSON_VALUE];
	NuTimeReport(szText);
	if (bReadWrite)
	{	
		fprintf(USART_FILE, "{\"SetValues\"=\"%s,0,0,0,0,%d,%d,%d,%d,%d\"}",
			szText, 
			mgrPWM.m_pwmSpec[0].m_uHz,
			mgrPWM.m_pwmSpec[0].m_uDuty,
			mgrPWM.m_pwmSpec[0].m_uDeadTime_ns,
			mgrPWM.m_pwmSpec[1].m_uDeadTime_ns,
			m_nTIM18Offset_ns);
	}
	else
		fprintf(USART_FILE, "{\"SetValues\"=\"%s,0,0,0,0\"}",	szText);		
}

void ELANCLLCInitReport(void)
{
	fprintf(USART_FILE, "{\"OpenCSVFile\" = \"D:/test123.csv\"}"); 
	fprintf(USART_FILE, "{\"SetKeys\" = \"Time,2,3,4,5,Hz,Duty,DeadTime1(ns),DeadTime2(ns),Offset(ns)\"}"); 
	fprintf(USART_FILE, "{\"PASSWORD\" = \"OFF\"}"); 
	ELANCLLCReportWrite(true);		
	m_bSystemReportInited = true;
}

void ELANCLLCReport(void)
{
	if (!m_bSystemReportInited)
	{
		ELANCLLCInitReport();
		NuOPModeEnableCLLCPWM(&mgrPWM, m_nTIM18Offset_ns, false);
	}
	if (!mgrADC.m_bFIFOADC &&	!mgrPWM.m_bDownloading)
		ELANCLLCReportWrite(false);	
}

void ELANCLLCNormalLoop(void)
{ 
	if (!ProcessBlocking(&ELANCLLCNormalLoopInProcess))
		return;
//
	ProcessUnblocking(&ELANCLLCNormalLoopInProcess);
}

void ELANCLLCFastLoop(void)
{
	if (!ProcessBlocking(&ELANCLLCFastLoopInProcess))
		return;
//
	ProcessUnblocking(&ELANCLLCFastLoopInProcess);
}

void ELANCLLCMain(void)
{
	mgrPWM.m_nHzSignal = 250 _kHz;
	mgrPWM.m_pwmSpec[0] = (PWMSpec) {ST_PWM_1, 249 _kHz, 500, 100, (GPortPinArray) {2, A8, B14}};
	mgrPWM.m_pwmSpec[1] = (PWMSpec) {ST_PWM_8, 249 _kHz, 600, 100, (GPortPinArray) {2, C6, B0}};
	RecalculatePWMManager(&mgrPWM, 2);
	TIM_DeInit(TIM1); 
	NuOPModeEnableCLLCPWM(&mgrPWM, m_nTIM18Offset_ns, true);
	int nADCArrangement = 2;
	uint32_t fs;
	switch (nADCArrangement) 
	{
		case 1:
			fs = 2.8 _MHz;
			NuOPModeEnableADC(ST_nCH_1ADC, fs, mgrPWM.m_nHzSignal, (GPortPinArray) {3, A1, A2, A3});
			break;
		case 2:
			fs = 4.2 _MHz;
			NuOPModeEnableADC(ST_1CH_3ADC, fs, mgrPWM.m_nHzSignal, (GPortPinArray) {3, A1, A2, A3});
			break;
		case 3:
			fs = 2.8 _MHz;
			NuOPModeEnableADC(ST_nCH_nADC, fs, mgrPWM.m_nHzSignal, (GPortPinArray) {3, A1, A2, A3});
			break;
		case 4:
			fs = 2.8 _MHz;
			NuOPModeEnableADC(ST_3nCH_3ADC, fs, mgrPWM.m_nHzSignal, (GPortPinArray) {3, A1, A1, A1});
			break;
	}
	JsonWriteIntoJsonPair("ELANCLLC", "ON"); 
}

void JELANApp(DWORD param[])
{
	if (!NuGetBit(C4))
	{
		char szText[MAX_JSON_VALUE];
		JsonPartition(param, szText);
		mgrPWM.m_pwmSpec[0].m_uHz = mgrPWM.m_pwmSpec[1].m_uHz = Bound(300 _kHz, 240 _kHz, Getint32(szText, ","));
		mgrPWM.m_pwmSpec[0].m_uDuty = mgrPWM.m_pwmSpec[1].m_uDuty = Bound(1000, 10, Getint32(szText, ","));
		mgrPWM.m_pwmSpec[0].m_uDeadTime_ns = Bound(200, 10, Getint32(szText, ","));
		mgrPWM.m_pwmSpec[1].m_uDeadTime_ns = Bound(200, 10, Getint32(szText, ","));
		RecalculatePWMManager(&mgrPWM, 2);
		m_nTIM18Offset_ns = Bound(200, -200, Getint32(szText, "]"));
		bProtectedSessionInProcess = true;
		NuOPModeEnableCLLCPWM(&mgrPWM, m_nTIM18Offset_ns, false);
		bProtectedSessionInProcess = false; 
	}
	ELANCLLCInitReport();
}
#endif
