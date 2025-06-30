#ifndef __NUSTM32_ADC_H
#define __NUSTM32_ADC_H
#include "NuSTM32_BASIC.h"
#include "stm32f4xx_adc.h"
#define MAX_ADC			3
#define ST_ADC_1		0x31
#define ST_ADC_2		0x32
#define ST_ADC_3		0x33

#define ST_nCH_1ADC		310
#define ST_1CH_3ADC		130
#define ST_nCH_nADC		330
#define ST_3nCH_3ADC	930
#define ST_ADC_MODE_5	5
#define ST_ADC_MODE_6	6

typedef struct
{
	ADC_TypeDef* ADCx;
	uint32_t ADCx_DR_ADDRESS;
	uint16_t nDMAChannel;
	uint16_t nStream;
	uint32_t ADCx_CDR_ADDRESS;
} STM32407G_ADC;

typedef struct
{
	uint16_t phaseA;
	uint16_t phaseB;
	uint16_t phaseC;	
} ThreePhaseData;


////////////////////
// ADC
#define ADC_1MHZ		0
#define ADC_SCAN		1
#define ADC_5_6MHZ	2

#define ADCMODE_SCAN				0
#define ADCMODE_FIFO				1
#define ADCMODE_CONTINUOUS	2

typedef struct{
	bool m_bFIFOADC;
	uint32_t m_nHzSampling;	
	uint32_t m_nHzSignal;	
	uint32_t m_nDataDivision;	
	uint16_t m_nADCModules;
	uint16_t m_nChannelsPerADCModule;
	uint16_t m_adcSampleTime;
	uint32_t m_adcPrescaler;	
	uint32_t m_adcTransferSize;
	uint32_t m_adcBufferSize;
	uint32_t m_nFIFOStart;	
	bool m_bADCOutput[MAX_ADC];
	uint16_t* m_dmaMemory0Base[MAX_ADC];
	uint16_t m_nPortADC[MAX_ADC];	
	char m_szText[MAX_JSON_KEY];
	GPortPinArray m_gppa;
} NuADCtrl;

extern NuADCtrl mgrADC;
void NuOPModeEnableADC(uint32_t nADCMode, int nHzSampling, int nHzSignal, GPortPinArray gppaADC);
void JDownloadADC(void);
void JPrintADC(DWORD param[]);

///////////////////////////////////
// Comprime Reconstruction
void JADCCoprimeReconstruction(DWORD param[]);
	void CoprimeReconstruction(int fs, int fx);

#endif


