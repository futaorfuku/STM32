#include "NuSTM32_GPIO.h"
#include "NuSTM32_JSON.h"

GPIO_TypeDef* GetGPIO_TypeDef(uint32_t nPort)
{
	GPIO_TypeDef* GPIOxs[] = {0, GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK};
	return GPIOxs[nPort];
}

void ReportGPIOUse(GPortPin gpp)
{
	char szText[MAX_JSON_VALUE];
	memset(szText, 0, MAX_JSON_VALUE);
	PortPin pp = PP(gpp);
	sprintf(szText, "%c%d", pp.m_nPort - 1 + 'A', pp.m_nPinID);
	JsonWriteIntoJsonPair("GPIO", szText);
}

void NuEnableGPIO(GPortPin gpp, GPIOMode_TypeDef gpio_Mode, 
	GPIOSpeed_TypeDef gpio_Speed, GPIOOType_TypeDef gpio_OType, GPIOPuPd_TypeDef gpio_PuPd)
{
	PortPin pp = PP(gpp);
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = pp.m_GPIO;
  GPIO_InitStructure.GPIO_Mode = gpio_Mode;  
	GPIO_InitStructure.GPIO_Speed = gpio_Speed;
#ifndef CODE_F103
  GPIO_InitStructure.GPIO_OType = gpio_OType;
  GPIO_InitStructure.GPIO_PuPd = gpio_PuPd;
#endif
  NuEnableRCCClock(pp.m_nPort); 
	GPIO_Init(pp.m_typedef, &GPIO_InitStructure);
	ReportGPIOUse(gpp);
}

void NuEnableOutputPPA(GPortPinArray ppa)
{
	for (int i = 0; i < ppa.m_nPins; i++)
			NuEnableOutputPP(ppa.m_gpp[i]);
}

void NuEnableOutputPP(GPortPin gpp)
{
	NuEnableGPIO(gpp, GPIO_Mode_OUT, GPIO_High_Speed, GPIO_OType_PP, GPIO_PuPd_NOPULL);
}

void NuEnableInputPPA(GPortPinArray ppa)
{
	for (int i = 0; i < ppa.m_nPins; i++)
		NuEnableInputPP(ppa.m_gpp[i]);
}

void NuEnableInputPP(GPortPin gpp)
{
	NuEnableGPIO(gpp, GPIO_Mode_IN, GPIO_High_Speed,	GPIO_OType_PP, GPIO_PuPd_DOWN);
}

void NuEnableAnalogPPA(GPortPinArray ppa)
{
	for (int i = 0; i < ppa.m_nPins; i++)
		NuEnableAnalogPP(ppa.m_gpp[i]);
}
 
void NuEnableAnalogPP(GPortPin gpp)
{
	NuEnableGPIO(gpp, GPIO_Mode_AN, GPIO_High_Speed, GPIO_OType_PP, GPIO_PuPd_NOPULL);
}

void NuEnableGPIOAFPP(GPortPin gpp, uint8_t gpio_AF, GPIOSpeed_TypeDef gpio_Speed, GPIOPuPd_TypeDef gpio_PuPd)
{
	PortPin pp = PP(gpp);
	NuEnableGPIO(gpp, GPIO_Mode_AF, gpio_Speed, GPIO_OType_PP, gpio_PuPd);
	GPIO_PinAFConfig(pp.m_typedef, pp.m_nPinID, gpio_AF);
}

void NuEnableGPIOAFPPA(GPortPinArray ppa, uint8_t gpio_AF, GPIOSpeed_TypeDef gpio_Speed, GPIOPuPd_TypeDef gpio_PuPd)
{
	for (int i =  0; i < ppa.m_nPins; i++)
		NuEnableGPIOAFPP(ppa.m_gpp[i], gpio_AF, gpio_Speed, gpio_PuPd);
}

void NuEnableAFPP(GPortPin gpp, uint8_t AF_TIMx) 
{
	NuEnableGPIOAFPP(gpp, AF_TIMx, GPIO_Medium_Speed, GPIO_PuPd_DOWN); 
}

void NuEnableAFPPA(GPortPinArray ppa, uint8_t AF_TIMx)
{
	for (int i = 0; i < ppa.m_nPins; i++)
		NuEnableAFPP(ppa.m_gpp[i], AF_TIMx); 
}

uint16_t NuGetBit(GPortPin gpp)
{
	PortPin pp = PP(gpp);
	if (pp.m_nPort > ST_REGST ||	!pp.m_nPort)
		return 0;
	if (pp.m_nPort == ST_REGST)
		return GPIO_ReadInputData(pp.m_typedef);
	return GPIO_ReadInputDataBit(pp.m_typedef, pp.m_GPIO);
}

void NuSetBit(GPortPin gpp, uint16_t nValue)
{
	PortPin pp = PP(gpp);
	if (pp.m_nPort > ST_REGST ||	!pp.m_nPort)
		return;
	if (pp.m_nPort == ST_REGST)
		GPIO_Write(pp.m_typedef, nValue);
	else
		GPIO_WriteBit(pp.m_typedef, pp.m_GPIO, nValue? Bit_SET: Bit_RESET);
}

PortPin PP(GPortPin gpp)
{
	if (gpp == 0)
		return (PortPin){0, 0};
	gpp--;
	PortPin pp;
	pp.m_nPort = gpp / 16 + 1;
	pp.m_typedef = GetGPIO_TypeDef(pp.m_nPort);
	pp.m_nPinID =	gpp % 16;
	pp.m_GPIO = 0x1 << pp.m_nPinID;
	return pp;
}