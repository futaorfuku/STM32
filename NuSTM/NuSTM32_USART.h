#ifndef __NUSTM32_USART_H
#define __NUSTM32_USART_H
#include "NuSTM32_BASIC.h"
#define ST_USART_1	0x60
#define ST_USART_2	0x61
#define ST_USART_3	0x62
#define ST_USART_1A	0x60
#define ST_USART_2A	0x61
#define ST_USART_3A	0x62
#define ST_USART_1B	0x63
#define ST_USART_2B	0x64
#define ST_USART_3B	0x65
#define ST_USART_1C	0x66
#define USART_PORT	ST_USART_1B
#define USART_FILE (FILE*) USART_PORT 

void USARTConfig(uint16_t nPort);
void NuInitUSARTData(void);
void NuUSARTSendData(uint16_t nPort, const char* szText, int nLen);
void NuUSARTReceiveData(uint16_t nPort);
void NuOPModeEnableUSART(uint16_t nPort);
void NuUSARTPrint(char szText[]);

#endif


