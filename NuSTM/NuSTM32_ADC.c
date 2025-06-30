#include "NuSTM32_ADC.h"
#include "NuSTM32_JSON.h"
#define ADC_CDR_ADDRESS	0
#define ADC_DR_ADDRESS	1
#define ADC_CHANNELS 18
uint16_t ADCFIFORingBuffer[MAX_ADC * FIFOSIZE] = {0};
uint16_t ADCSNAPSHOT[MAX_ADC][FIFOSIZE];
NuADCtrl mgrADC;
	
uint16_t GetADCChannelFromGPortPin(GPortPin gpp)
{
	GPortPin ADCGPPA[ADC_CHANNELS] = {A0, A1, A2, A3, A4, A5, A6, A8, B0,	B1, C0, C1, C2, C3, C5, none, none};
	for (int i = 0; i < ADC_CHANNELS; i++)
	{
		if (gpp == ADCGPPA[i])
			return i;
	}
	return 0;
}

void NuADCCommonConfig(uint32_t adc_Mode, uint32_t adc_TwoSamplingDelay, uint32_t adc_DMAAccessMode)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = adc_Mode;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = adc_TwoSamplingDelay;
	ADC_CommonInitStructure.ADC_DMAAccessMode = adc_DMAAccessMode;
	ADC_CommonInitStructure.ADC_Prescaler = mgrADC.m_adcPrescaler;
	ADC_CommonInit(&ADC_CommonInitStructure);
}

void ADCInitConfig(ADC_InitTypeDef* pADC_InitStructure, uint32_t adcResolution, uint8_t adcNbrOfConversion,
	FunctionalState adc_ScanConvMode, FunctionalState adc_ContinuousConvMode)
{
	uint32_t adc_ExternalTrigConvEdge =	ADC_ExternalTrigConvEdge_Rising;
	uint32_t adc_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	pADC_InitStructure->ADC_Resolution = adcResolution; 
	pADC_InitStructure->ADC_NbrOfConversion = adcNbrOfConversion;
	pADC_InitStructure->ADC_ScanConvMode = adc_ScanConvMode;
	pADC_InitStructure->ADC_ContinuousConvMode = adc_ContinuousConvMode;
	pADC_InitStructure->ADC_ExternalTrigConvEdge = adc_ExternalTrigConvEdge;
	pADC_InitStructure->ADC_ExternalTrigConv = adc_ExternalTrigConv;
	pADC_InitStructure->ADC_DataAlign = ADC_DataAlign_Right;
}


STM32407G_ADC NuGetADC(uint16_t nPort)
{
	STM32407G_ADC adc;	
	switch (nPort)
	{
		case ST_ADC_1:
			adc.ADCx = ADC1;
			adc.nStream = 0; // [0, 4]
			break;
		case ST_ADC_2:
			adc.ADCx = ADC2;
			adc.nStream = 2; // [2, 3]
			break;
		case ST_ADC_3:
			adc.ADCx = ADC3;
			adc.nStream = 1; // [0, 1]
			break;
	}
	adc.nDMAChannel = nPort - ST_ADC_1;
	adc.ADCx_DR_ADDRESS = ((uint32_t) adc.ADCx + 0x4C);
	adc.ADCx_CDR_ADDRESS = ((uint32_t) ADC + 0x8);
	return adc;
}

void NuEnableDMA_ADC(uint16_t nPort, DWORD nBaseAddress, uint32_t dmaMemory0BaseAddr, uint32_t dmaBufferSize, uint32_t dmaDIR)
{
	STM32407G_ADC adc = NuGetADC(nPort);
	NuEnableDMA(ST_DMA_2, adc.nDMAChannel, adc.nStream,
		nBaseAddress == ADC_DR_ADDRESS? adc.ADCx_DR_ADDRESS: adc.ADCx_CDR_ADDRESS,
		dmaMemory0BaseAddr, dmaBufferSize, dmaDIR);
}

////////////////////////////////////////
FunctionalState bEnableADC = DISABLE;

void NuADC_Cmd(FunctionalState NewState)
{
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE); 
	ADC_Cmd(ADC1, NewState);
	ADC_Cmd(ADC2, NewState);
	ADC_Cmd(ADC3, NewState);
	bEnableADC = NewState;
}

void NuEnableADCCmd()
{
	if (!bEnableADC)
		NuADC_Cmd(ENABLE);
}

void ADC12bInit(int nChannels, uint16_t nPortStart, uint16_t nPortEnd)
{	
	ADC_InitTypeDef ADC_InitStructure;
	ADCInitConfig(&ADC_InitStructure, ADC_Resolution_12b, nChannels, nChannels > 1, ENABLE);
	for (int i = nPortStart; i <= nPortEnd; i++)
		ADC_Init(NuGetADC(i).ADCx, &ADC_InitStructure);
}

void Nu3InterleavedFastestADCConfig(uint16_t nPort, GPortPinArray gppa)
{
	uint8_t nADChannel = GetADCChannelFromGPortPin(mgrADC.m_gppa.m_gpp[0]);	
	NuADCCommonConfig(ADC_TripleMode_Interl, ADC_TwoSamplingDelay_5Cycles, ADC_DMAAccessMode_2);
	NuEnableDMA_ADC(nPort, ADC_CDR_ADDRESS, (uint32_t) mgrADC.m_dmaMemory0Base[0],	
		mgrADC.m_adcBufferSize, DMA_DIR_PeripheralToMemory);
	ADC12bInit(gppa.m_nPins, ST_ADC_1, ST_ADC_3);
	ADC_RegularChannelConfig(ADC1, nADChannel, 1, mgrADC.m_adcSampleTime);
	ADC_RegularChannelConfig(ADC2, nADChannel, 1, mgrADC.m_adcSampleTime);
	ADC_RegularChannelConfig(ADC3, nADChannel, 1, mgrADC.m_adcSampleTime);
 	ADC_SoftwareStartConv(ADC1); 
	NuADC_Cmd(ENABLE);
}

void NuEnableADC(ADC_TypeDef* ADCx)
{
	ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE); 
	ADC_DMACmd(ADCx, ENABLE);
	ADC_Cmd(ADCx, ENABLE);	
	ADC_SoftwareStartConv(ADCx);
}

void ADC12bConfig(uint16_t nPort, GPortPinArray gppa, bool bIndepdendent)
{
	// Max. 2.8MHz Single Channel
	ADC_TypeDef* ADCx = NuGetADC(nPort).ADCx;
	ADC12bInit(gppa.m_nPins, nPort, nPort);	
	NuADCCommonConfig(bIndepdendent? ADC_Mode_Independent: ADC_TripleMode_Interl, 
		ADC_TwoSamplingDelay_5Cycles, ADC_DMAAccessMode_Disabled);	
	NuEnableDMA_ADC(nPort, ADC_DR_ADDRESS, (uint32_t) mgrADC.m_dmaMemory0Base[nPort - ST_ADC_1], 
		mgrADC.m_adcBufferSize, DMA_DIR_PeripheralToMemory);;
	for (int i = 0; i < gppa.m_nPins; i++)
		ADC_RegularChannelConfig(ADCx, GetADCChannelFromGPortPin(gppa.m_gpp[i]), i + 1, mgrADC.m_adcSampleTime); 
	NuEnableADC(ADCx);
}

void InitADCManager(uint32_t nHzSampling, int nHzSignal, GPortPinArray gppaADC)
{
	mgrADC.m_nHzSampling = nHzSampling;
	mgrADC.m_nHzSignal = _iabs(nHzSignal);
	mgrADC.m_adcTransferSize = 48;
	mgrADC.m_adcPrescaler = ADC_Prescaler_Div2;
	mgrADC.m_adcSampleTime = ADC_SampleTime_3Cycles; // 2.8MHz
	mgrADC.m_adcBufferSize = nHzSignal < 0? -nHzSignal: 2 * nHzSampling / GCD(nHzSampling, nHzSignal);
	memcpy(mgrADC.m_szText, "ON", MAX_JSON_KEY);
	mgrADC.m_bFIFOADC = false;
	mgrADC.m_gppa = gppaADC;
	mgrADC.m_nDataDivision = 2;
	mgrADC.m_nADCModules = 1;
	mgrADC.m_nChannelsPerADCModule = 1;
	ADC_DeInit();
	for (int  i = 0; i < MAX_ADC; i++)
	{
		mgrADC.m_bADCOutput[i] = false;
		mgrADC.m_dmaMemory0Base[i] = &ADCFIFORingBuffer[i * FIFOSIZE];
		mgrADC.m_nPortADC[i] = ST_ADC_1 + i;
		NuEnableRCCClock(mgrADC.m_nPortADC[i]);
	}
	NuEnableAnalogPPA(gppaADC);	
	TIM_ITConfig(TIM1, TIM_IT_CC1|TIM_IT_Update, ENABLE);
}

void CalculateSampleTime(uint32_t nHz)
{
	if (nHz <= 43750)
	{
		mgrADC.m_adcPrescaler = ADC_Prescaler_Div4;
		mgrADC.m_adcSampleTime = ADC_SampleTime_480Cycles;
	}
}

void NuOPModeEnableADC(uint32_t nADCMode, int nHzSampling, int nHzSignal, GPortPinArray gppaADC)
{
	InitADCManager(nHzSampling, nHzSignal, gppaADC);
	CalculateSampleTime(nHzSampling);
	switch (nADCMode)
	{
		case ST_nCH_1ADC:
			memcpy(mgrADC.m_szText, "nCH_1ADC", MAX_JSON_KEY);
			mgrADC.m_adcBufferSize *= gppaADC.m_nPins;
			mgrADC.m_nChannelsPerADCModule = gppaADC.m_nPins;
			ADC12bConfig(mgrADC.m_nPortADC[nADCMode % 10], gppaADC, true);
			break;
		case ST_1CH_3ADC:
			memcpy(mgrADC.m_szText, "1CH_3ADC", MAX_JSON_KEY);
			Nu3InterleavedFastestADCConfig(mgrADC.m_nPortADC[nADCMode % 10], (GPortPinArray){1, gppaADC.m_gpp[0]});
			break;
		case ST_nCH_nADC:
			memcpy(mgrADC.m_szText, "nCH_nADC", MAX_JSON_KEY);
			mgrADC.m_nADCModules = gppaADC.m_nPins;
			mgrADC.m_adcTransferSize /= mgrADC.m_nADCModules;
			for (int i = 0; i < mgrADC.m_nADCModules; i++)
				ADC12bConfig(mgrADC.m_nPortADC[i], (GPortPinArray){1, gppaADC.m_gpp[i]}, false);
			break;
		case ST_3nCH_3ADC:
			memcpy(mgrADC.m_szText, "3nCH_3ADC", MAX_JSON_KEY);
			mgrADC.m_nADCModules = 3;
			mgrADC.m_adcTransferSize /= mgrADC.m_nADCModules;
			mgrADC.m_nChannelsPerADCModule = gppaADC.m_nPins / mgrADC.m_nADCModules;
			for (int i = 0; i < mgrADC.m_nADCModules; i++)
			{
				GPortPinArray gppa;
				gppa.m_nPins = mgrADC.m_nChannelsPerADCModule;
				for (int j = 0; j < mgrADC.m_nChannelsPerADCModule; j++)
					gppa.m_gpp[j] = gppaADC.m_gpp[mgrADC.m_nChannelsPerADCModule * i + j];
				ADC12bConfig(mgrADC.m_nPortADC[i], gppa, true);
			}
			break;
	}
	if (nOperationMode & OPMODE_USART)
		JsonWriteIntoJsonPair("ADC", mgrADC.m_szText); 
}


///////////////////////////
// JSON

bool bJDownloadADCInProcess = false;

void JDownloadADC()
{
	if ((nOperationMode & OPMODE_USART) == 0)
		return;
	if (bJDownloadADCInProcess)
		return;
	if (
		!mgrADC.m_bADCOutput[0] &&
		!mgrADC.m_bADCOutput[1] &&
		!mgrADC.m_bADCOutput[2])
		return;
	bJDownloadADCInProcess = true;
	int nSizeTransferred = 0;
	char* szKeys[] = {"ADC1", "ADC2", "ADC3"};
	JsonWriteOpen();
	if (mgrADC.m_nFIFOStart == 0)
	{
		JsonWriteIntoJsonPair("Chunk", "ADC"); 
		JsonWriteIntIntoJsonPair("FS", mgrADC.m_nHzSampling);
		JsonWriteIntIntoJsonPair("FX", mgrADC.m_nHzSignal);
		JsonWriteIntoJsonPair("MODE", mgrADC.m_szText); 
		JsonWriteIntIntoJsonPair("CH", mgrADC.m_nChannelsPerADCModule); 
	}		
	for (int i = 0; i < MAX_ADC; i++)
	{
		char szText[MAX_JSON_VALUE];
		memset(szText, 0, MAX_JSON_VALUE);
		if (!mgrADC.m_bADCOutput[i])
			continue;
		int k = 0;
		for (int j = mgrADC.m_nFIFOStart; 
				j < mgrADC.m_adcBufferSize / mgrADC.m_nDataDivision && k < mgrADC.m_adcTransferSize; 
				j++, k++)
		{
			strcat(szText, k == 0? "[": ",");				
			char szPin[MAX_JSON_VALUE];
			memset(szPin, 0, MAX_JSON_VALUE);
			sprintf(szPin, "%X", ADCSNAPSHOT[i][j]);
			strcat(szText, szPin);
		}
		strcat(szText, "]");
		JsonWriteIntoJsonPair(szKeys[i], szText);
		nSizeTransferred = _imax(nSizeTransferred, k);
	}
	mgrADC.m_nFIFOStart += nSizeTransferred;
	if (mgrADC.m_nFIFOStart >= mgrADC.m_adcBufferSize / mgrADC.m_nDataDivision)
	{
		JsonWriteIntoJsonPair("Chunk", "End");  
		JsonWriteClose();
		mgrADC.m_bFIFOADC = false;
	}
	else
		JsonWriteClose();
	nJsonWriteEnd = strlen(szJsonWriteLeft);
	bJDownloadADCInProcess = false;
}

void NuADCSnapShot(int nADC)
{
	mgrADC.m_bADCOutput[nADC] = true;
	memmove(ADCSNAPSHOT[nADC], mgrADC.m_dmaMemory0Base[nADC], mgrADC.m_adcBufferSize);
}

void JPrintADC(DWORD param[])
{		
	bProtectedSessionInProcess = true;
	NuADC_Cmd(DISABLE);
	for (int i = 0; i < mgrADC.m_nADCModules; i++)
		NuADCSnapShot(i);
	NuADC_Cmd(ENABLE);
	bProtectedSessionInProcess = false;
	mgrADC.m_nFIFOStart = 0;
	mgrADC.m_bFIFOADC = true;
	JsonWriteInit();
}

