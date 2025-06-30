#include "NuSTM32_USART.h"
#include "NuSTM32_JSON.h"
#define RXBUFFERSIZE  MAX_JSON_VALUE

typedef struct
{
	USART_TypeDef* COMx;
	GPortPin m_gppTx;
	GPortPin m_gppRx;
	uint8_t  nAF;
	IRQn_Type nIRQ;
} STM32407G_USART;

typedef struct 
{
	USART_TypeDef* COMx;
	uint8_t szBufferRx[RXBUFFERSIZE];
	int ubNbrOfDataToRead;
	int uhRxCount; 
} _NuUSART;

_NuUSART* NupUSART;
#define USART_CHANNELS	3
_NuUSART usartd[USART_CHANNELS];

STM32407G_USART NuGetUSARTPin(uint16_t nPort)
{
	STM32407G_USART USARTPin[] = {
		{USART1, A9, A10, GPIO_AF_USART1, USART1_IRQn}, 
		{USART2, A2, A3, GPIO_AF_USART2, USART2_IRQn}, 
		{USART3, B10, B11, GPIO_AF_USART3, USART3_IRQn},
		{USART1, B6, B7, GPIO_AF_USART1, USART1_IRQn}, 
		{USART2, D5, D6, GPIO_AF_USART2, USART2_IRQn}, 
		{USART3, C10, C11, GPIO_AF_USART3, USART3_IRQn},
		{USART1, D8, D9, GPIO_AF_USART1, USART1_IRQn}, 
	};
	return USARTPin[nPort - ST_USART_1];
}

_NuUSART* GetUSARTData(uint16_t nPort)
{
	return &usartd[(nPort - ST_USART_1) % USART_CHANNELS];
}

void USARTDataInit(uint16_t nPort)
{
	_NuUSART* pUSART = GetUSARTData(nPort);
	STM32407G_USART usartpin = NuGetUSARTPin(nPort);
	pUSART->COMx = usartpin.COMx;
	pUSART->ubNbrOfDataToRead = RXBUFFERSIZE;
	pUSART->uhRxCount = 0; 
	memset(pUSART->szBufferRx, 0, RXBUFFERSIZE);
}

void USARTPinIOConfig(uint16_t nPort)
{
	STM32407G_USART usartpin = NuGetUSARTPin(nPort);
	NuEnableGPIOAFPP(usartpin.m_gppTx, usartpin.nAF, GPIO_High_Speed, GPIO_PuPd_UP);
	NuEnableGPIOAFPP(usartpin.m_gppRx, usartpin.nAF, GPIO_High_Speed, GPIO_PuPd_UP);
}

void USARTInitConfig(uint16_t nPort)
{
	_NuUSART* pUSART = GetUSARTData(nPort);
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// Hardware flow control disabled (RTS and CTS signals)
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;// Receive and transmit enabled
  USART_Init(pUSART->COMx, &USART_InitStructure); // USART configuration  
}

void USARTConfig(uint16_t nPort)
{
	STM32407G_USART usartpin = NuGetUSARTPin(nPort);
	NuEnableRCCClock(nPort);
	USARTDataInit(nPort);
	USARTPinIOConfig(nPort);
	USARTInitConfig(nPort);
  USART_ITConfig(usartpin.COMx, USART_IT_RXNE, ENABLE); // Enable the usart.COMx Transmit interrupt: this interrupt is generated when the usart.COMx transmit data register is empty
	NuNVICConfig(NVIC_PriorityGroup_0, NuGetUSARTPin(nPort).nIRQ, 0, 1, ENABLE);
  USART_Cmd(usartpin.COMx, ENABLE); // Enable USART
	USART_ClearFlag(usartpin.COMx, USART_FLAG_TC);
}

int fputc(int ch, FILE* f)
{
	DWORD nAddress = (DWORD) f;
	_NuUSART* pUSART = GetUSARTData((uint16_t) nAddress);
	while (USART_GetFlagStatus(pUSART->COMx, USART_FLAG_TXE) == RESET);	
	USART_SendData(pUSART->COMx, (uint16_t) ch);
	return ch;
}

void NuUSARTPrint(char szText[])
{
	int nLen = strlen(szText);
	for (int i = 0; i < nLen; i++)
	{
		while (USART_GetFlagStatus(NupUSART->COMx, USART_FLAG_TXE) == RESET);	
		USART_SendData(NupUSART->COMx, szText[i]);
	}
}

void JsonReceiver(_NuUSART* pUSART, char ch)
{
	if (ch == '{')
		return;
	if (ch == '}')
	{
		JsonRead((char*) pUSART->szBufferRx, pUSART->uhRxCount);
		JsonWrite(USART_FILE);
		pUSART->uhRxCount = 0;
		return;
	}
	pUSART->szBufferRx[pUSART->uhRxCount++] = ch;// Read one byte from the receive data register
	if (pUSART->uhRxCount >= pUSART->ubNbrOfDataToRead) // Disable the pUSART->COMx Receive interrupt
	{
		JsonReadStorage((char*) pUSART->szBufferRx, pUSART->uhRxCount);
		pUSART->uhRxCount = 0;
	}
}

void USARTReceiver(uint16_t nPort)
{
	_NuUSART* pUSART = GetUSARTData(nPort);
	if (USART_GetITStatus(pUSART->COMx, USART_IT_RXNE) != SET)
		return;
	char ch = USART_ReceiveData(pUSART->COMx);
	JsonReceiver(pUSART, ch);
}

void USART1_IRQHandler(void)
{
	USARTReceiver(ST_USART_1);
}

void USART2_IRQHandler(void)
{
	USARTReceiver(ST_USART_2);
}

void USART3_IRQHandler(void)
{
	USARTReceiver(ST_USART_3);
}

void NuOPModeEnableUSART(uint16_t nPort)
{
	USARTConfig(nPort);
	NupUSART = GetUSARTData(nPort);
	JsonWriteIntoJsonPair("USART", "ON"); 
}
