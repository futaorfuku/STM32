#ifndef __NUSTM32_GPIO_H
#define __NUSTM32_GPIO_H
#include "Nu_STM32Def.h"
#include "stm32f4xx_gpio.h"
#define MAX_PORT_PINS	8


typedef enum
{ 
	none = 0,
  A0, A1,	A2,	A3,	A4,	A5,	A6,	A7,	A8,	A9,	A10, A11,	A12, A13,	A14, A15,
  B0, B1,	B2,	B3,	B4,	B5,	B6,	B7,	B8,	B9,	B10, B11,	B12, B13,	B14, B15,
  C0, C1,	C2,	C3,	C4,	C5,	C6,	C7,	C8,	C9,	C10, C11,	C12, C13,	C14, C15,
  D0, D1,	D2,	D3,	D4,	D5,	D6,	D7,	D8,	D9,	D10, D11,	D12, D13,	D14, D15,
  E0, E1,	E2,	E3,	E4,	E5,	E6,	E7,	E8,	E9,	E10, E11,	E12, E13,	E14, E15,
} GPortPin;

typedef struct{
	uint16_t m_nPinID;		
	uint16_t m_nPort;	
	uint16_t m_GPIO;	
	GPIO_TypeDef* m_typedef;
} PortPin;

typedef struct{
	uint8_t m_nPins;	
	PortPin m_pp[MAX_PORT_PINS];
} PortPinArray;

typedef struct{
	uint8_t m_nPins;	
	GPortPin m_gpp[MAX_PORT_PINS];
} GPortPinArray;

#ifdef CODE_F103

typedef enum
{ 
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;

typedef enum
{ 
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;

#define GPIOH               NULL
#define GPIOI               NULL
#define GPIOJ               NULL
#define GPIOK               NULL

#endif


////////////////////
// Fundamental
PortPin PP(GPortPin pin);
bool ProcessBlocking(bool* pbHandle);
void ProcessUnblocking(bool* pbHandle);
void NuEnableAFPPA(GPortPinArray ppa, uint8_t AF_TIMx);
void NuEnableAnalogPPA(GPortPinArray ppa);
void NuEnableAnalogPP(GPortPin gpp);
void NuEnableInputPPA(GPortPinArray array);
void NuEnableInputPP(GPortPin gpp);
void NuEnableOutputPPA(GPortPinArray ppa);
void NuEnableOutputPP(GPortPin pp);
void NuEnableGPIOAFPP(GPortPin gpp, uint8_t gpio_AF, GPIOSpeed_TypeDef gpio_Speed, GPIOPuPd_TypeDef gpio_PuPd);
void NuEnableGPIOAFPPA(GPortPinArray ppa, uint8_t gpio_AF, GPIOSpeed_TypeDef gpio_Speed, GPIOPuPd_TypeDef gpio_PuPd);
uint16_t NuGetBit(GPortPin gpp);
void NuSetBit(GPortPin gpp, uint16_t nValue);

#endif /*__NUSTM32_GPIO_H*/


