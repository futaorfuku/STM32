#include "NuSTM32_ADC.h"

typedef struct
{
	double m_dChannel[4];
} CChannelData;

typedef struct 
{
	CChannelData m_data[10000];
	int m_nDataCount;
} CArrayChannelData;

CArrayChannelData m_adcrerOscilloscope;
CArrayChannelData m_adcreX;
CArrayChannelData m_adcreR;
int m_adcreN;
int m_adcreM;
int m_adcreH[2000];
int m_adcreIH[2000];
int m_adcrefs; // [kHz]
int m_adcrefx; // [kHz]
double m_adcreTimeSpan_us;

void InitChannelData()
{
	m_adcrerOscilloscope.m_nDataCount = 0;
	m_adcreX.m_nDataCount = 0;
	m_adcreR.m_nDataCount = 0;
}

void AddChannelData(CArrayChannelData* pArray, CChannelData data)
{
	pArray->m_data[pArray->m_nDataCount++] = data;
}

void PrintChannelData(CArrayChannelData* pArray)
{
	for (int i = 0; i < pArray->m_nDataCount; i++)
	{
		CChannelData cData = pArray->m_data[i];
		printf("%f,%f,%f,%f\n", 
			cData.m_dChannel[0],
			cData.m_dChannel[1],
			cData.m_dChannel[2],
			cData.m_dChannel[3]);
	}
}

void Reconstruction(CArrayChannelData* pArrayReconstruct, CArrayChannelData* pArrayX)
{
	for (int i = 0; i < m_adcreN; i++)
		AddChannelData(pArrayReconstruct, pArrayX->m_data[m_adcreIH[i]]);
}

double Remainder(double a, double b)
{
	if (a < b)
		return a;
	return Remainder(a - b, b);
}

void DataSampling(CArrayChannelData* pArrayX, CArrayChannelData* pArrayO)
{
	double dPeriodSize = (int)(1e7 / m_adcrefx / m_adcreTimeSpan_us);
	double dSampleJump = 1e7 / m_adcrefs / m_adcreTimeSpan_us;
	for (int i = 0; i < m_adcreN; i++)
		AddChannelData(pArrayX, pArrayO->m_data[(int)(Remainder(i * dSampleJump, dPeriodSize))]);
}



void ReducedFraction(int A, int B, int* pN, int* pM)
{
	int C = GCD(A, B);
	*pN = A / C;
	*pM = B / C;
}

void PermutationTable(int m_adcreN, int m_adcreM, int m_adcreH[], int m_adcreIH[])
{
	for (int i = 0; i < m_adcreN; i++)
	{
		int m = (i * m_adcreM) % m_adcreN;
		m_adcreH[i] = m;
		m_adcreIH[m] = i;
	}
}

void ADCReconstructionInit(int fs, int fx, int nTimeSpan_us)
{
	InitChannelData();
	m_adcrefs = fs;
	m_adcrefx = fx;
	m_adcreTimeSpan_us = nTimeSpan_us;
}


void ADCReconstructionRun()
{
	ReducedFraction(m_adcrefs, m_adcrefx, &m_adcreN, &m_adcreM);
	PermutationTable(m_adcreN, m_adcreM, m_adcreH, m_adcreIH);
	DataSampling(&m_adcreX, &m_adcrerOscilloscope);
	Reconstruction(&m_adcreR, &m_adcreX);
	PrintChannelData(&m_adcreR);
}


uint16_t ADCCoprimeReconstruction[FIFOSIZE] = {0};

void CRDataDownload(CArrayChannelData* pArrayX, int nChannel, uint16_t Buffer[])
{
	for (int i = 0; i < m_adcreN; i++)
	{
		CChannelData cData;
		cData.m_dChannel[nChannel] = Buffer[i];
		AddChannelData(pArrayX, cData);
	}
}

void CRDataUpload(CArrayChannelData* pArrayX, int nChannel, uint16_t Buffer[])
{
	for (int i = 0; i < pArrayX->m_nDataCount; i++)
		Buffer[i] = pArrayX->m_data[i].m_dChannel[nChannel];
}

void CoprimeReconstruction(int fs, int fx)
{
	int nChannel = 0;
	ADCReconstructionInit(fs, fx, 0);
	ReducedFraction(m_adcrefs, m_adcrefx, &m_adcreN, &m_adcreM);
	PermutationTable(m_adcreN, m_adcreM, m_adcreH, m_adcreIH);
	CRDataDownload(&m_adcreX, nChannel, ADCFIFORingBuffer);
	Reconstruction(&m_adcreR, &m_adcreX);
	CRDataUpload(&m_adcreX, nChannel, ADCCoprimeReconstruction);
}

