#ifdef _USE_DAC_
#include "Nu_STM32.h"
#include "math.h"
#define ST_DAC_1		0x34
#define ST_DAC_2		0x35
#define DHR12R1_OFFSET             ((uint32_t)0x00000008)/* DHR registers offsets */
#define DHR12R2_OFFSET             ((uint32_t)0x00000014)
#define SINETABLE_SIZE	2000
uint32_t sineTable[SINETABLE_SIZE];

typedef struct
{
	uint8_t m_nDAChannel;
	double (*channel1)();
	double (*channel2)();

} _NuDACCtrl;

typedef struct
{
	uint32_t DHR;
	uint16_t nDMAChannel;
	uint32_t nStream;
} STM32407G_DAC;

_NuDACCtrl NuDACCtrl;

void NuEnableDACGPIO(uint32_t nDAChannel)
{
	switch (nDAChannel)
	{
		case DAC_Channel_1:
			NuEnableAnalogPP(A4);
			break;
		case DAC_Channel_2:
			NuEnableAnalogPP(A5);
			break;
		case DAC_Channel_All:
			NuEnableAnalogPPA((GPortPinArray) {2, A4, A5});
		break;
	} 
}

void DACInitConfig(uint32_t nDAChannel, uint32_t dac_Trigger)
{
	DAC_InitTypeDef DAC_InitStructure;
  DAC_InitStructure.DAC_Trigger = dac_Trigger; // 
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
 	switch (nDAChannel)
	{
		case DAC_Channel_1:
		case DAC_Channel_2:
			DAC_Init(nDAChannel, &DAC_InitStructure);
			DAC_Cmd(nDAChannel, ENABLE);
			break;
		case DAC_Channel_All:
			DAC_Init(DAC_Channel_1, &DAC_InitStructure);
			DAC_Init(DAC_Channel_2, &DAC_InitStructure);
			DAC_Cmd(DAC_Channel_1, ENABLE);
			DAC_Cmd(DAC_Channel_2, ENABLE);
			break;
	}
}

void NuEnableDAC(uint32_t nDAChannel, uint32_t uClockCycle)
{	// DACs configuration: double interleaved with 6cycles delay to reach 5Msps  
	NuEnableTIMTRO(ST_TIMER_6, uClockCycle, TIM_CounterMode_Up, TIM_CKD_DIV1, TIM_OPMode_Repetitive);
	DACInitConfig(nDAChannel, DAC_Trigger_T6_TRGO);
	NuEnableTIM_NVIC(ST_TIMER_6, NVIC_PriorityGroup_0, 2);
 	switch (nDAChannel)
	{
		case DAC_Channel_1:
		case DAC_Channel_2:		
			DAC_DMACmd(nDAChannel, ENABLE);
			break;
		case DAC_Channel_All:
			DAC_DMACmd(DAC_Channel_1, ENABLE);
			DAC_DMACmd(DAC_Channel_2, ENABLE);
			break;
	}
}

STM32407G_DAC GetDAC(uint8_t nDAC)
{
	uint32_t DHR12R1 = (uint32_t) DAC + DHR12R1_OFFSET;
	uint32_t DHR12R2 = (uint32_t) DAC + DHR12R2_OFFSET;
	STM32407G_DAC dacs[] = {{DHR12R1, 7, 5}, {DHR12R2, 7, 6}};
	return dacs[nDAC];
}

void NuDACCtrlConfig(uint32_t nDACChannel, uint32_t uClockCycle)
{
	NuEnableDACGPIO(nDACChannel);
	NuEnableRCCClock(ST_DAC_1);
	STM32407G_DAC dac1 = GetDAC(0);
	STM32407G_DAC dac2 = GetDAC(1);
	switch (nDACChannel)
	{
		case DAC_Channel_1:
			NuEnableDMA(ST_DMA_1, dac1.nDMAChannel, dac1.nStream, dac1.DHR, NULL, 0, DMA_DIR_MemoryToPeripheral);
			break;
		case DAC_Channel_2:
			NuEnableDMA(ST_DMA_1, dac2.nDMAChannel, dac2.nStream, dac2.DHR, NULL, 0, DMA_DIR_MemoryToPeripheral);
			break;
		case DAC_Channel_All:
			NuEnableDMA(ST_DMA_1, dac1.nDMAChannel, dac1.nStream, dac1.DHR, NULL, 0, DMA_DIR_MemoryToPeripheral);
			NuEnableDMA(ST_DMA_1, dac2.nDMAChannel, dac2.nStream, dac2.DHR, NULL, 0, DMA_DIR_MemoryToPeripheral);
			break;
	}
	NuEnableDAC(nDACChannel, uClockCycle);
}

uint16_t sinegenerator(double x)
{
	double sinx = 0;
	double dSum = x;
	double x2 = x * x;
	for (int i = 1; i < 10; i += 2)
	{
		sinx += dSum;
		dSum *= -x2 / ((i + 2) * (i + 1));
	}
	return 0x800 * sinx + 0x7FF;
}

void NuOPModeEnableDAC(uint32_t nDACChannel, uint32_t uClockCycle, double (*channel1)(), double (*channel2)())
{
	for (int i = 0; i < SINETABLE_SIZE; i++)
	{
		if (i <= SINETABLE_SIZE / 4)
			sineTable[i] = sinegenerator((double) PI2 * i / SINETABLE_SIZE);
		else if (i > SINETABLE_SIZE / 2)
			sineTable[i] = 0xFFD - sineTable[SINETABLE_SIZE - i];
		else
			sineTable[i] = sineTable[SINETABLE_SIZE / 2 - i];
	}
	DAC_DeInit(); 
	NuDACCtrl.m_nDAChannel = nDACChannel;
	NuDACCtrl.channel1 = channel1;
	NuDACCtrl.channel2 = channel2;
	NuDACCtrlConfig(nDACChannel, uClockCycle);
	if (nOperationMode & OPMODE_USART)
	{
		switch (NuDACCtrl.m_nDAChannel)
		{
			case DAC_Channel_1:
				JsonWriteIntoJsonPair("DAC1", "ON");
				break;
			case DAC_Channel_2:
				JsonWriteIntoJsonPair("DAC2", "ON");
				break;
			case DAC_Channel_All:
				JsonWriteIntoJsonPair("DAC1_DAC2", "ON");
				break;
		}
	}
}

void DACCallBack(void)
{
	if (!(nOperationMode & OPMODE_DAC))
		return;
	switch (NuDACCtrl.m_nDAChannel)
	{
		case DAC_Channel_1:
			DAC_SetChannel1Data(DAC_Align_12b_R, NuDACCtrl.channel1());
			break;
		case DAC_Channel_2:
			DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)(0x800 * NuDACCtrl.channel2() + 0x7FF));
			break;
		case DAC_Channel_All:
			DAC_SetChannel1Data(DAC_Align_12b_R, NuDACCtrl.channel1());
			DAC_SetChannel2Data(DAC_Align_12b_R, NuDACCtrl.channel2());
			break;
	}
}
////////////////////////////////
//

int nSinCnt = 0;
double DACChannel1(void)
{
	return sineTable[(nSinCnt++ / 2) % SINETABLE_SIZE];
}

double DACChannel2(void)
{
	return 1;
}

void JSetDAC12(DWORD param[])
{
	char szText[MAX_JSON_VALUE];
	char szParam[MAX_JSON_VALUE];
	memset(szText, 0, MAX_JSON_VALUE);
	memcpy(szText, (void*) param[0], MAX_JSON_VALUE);
	strsplit(szText, szParam, "[");
	GetCharString(szText, ",", szParam);
	if (strFind(szParam, "OFF") == 0)
	{
		DAC_DeInit();
		return;
	}
	uint32_t uClockCycle = Getint32(szText, "]");
	nOperationMode |= OPMODE_DAC;
	NuOPModeEnableDAC(DAC_Channel_All, uClockCycle, DACChannel1, DACChannel2);
}

#endif