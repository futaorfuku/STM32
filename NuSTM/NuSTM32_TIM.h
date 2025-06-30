#ifndef __NUSTM32_TIM_H
#define __NUSTM32_TIM_H
#include "NuSTM32_GPIO.h"
#ifdef CODE_F103
#include "stm32f10x_tim.h"
#else
#include "stm32f4xx_tim.h"
#endif

#define _kHz * 1000
#define _MHz * 1000000

// TIM
TIM_TypeDef* GetTIM_TypeDef(uint32_t nPort);
uint32_t NuTimeBaseHzConfig(TIM_TimeBaseInitTypeDef* pTIM_TimeBaseStructure, uint32_t uHz, uint16_t tim_CounterMode);
void NuTimeOCInitStructure(TIM_OCInitTypeDef* pTIM_OCInitStructure, uint16_t tim_OCMode, bool bStateEnable, 
	uint16_t tim_OCIdleState,
	uint16_t tim_OCNIdleState);
void NuTIMBDTRInit(TIM_TypeDef* TIMx, uint16_t tim_DeadTime);
void Clock(GPortPin pin);
uint64_t CyclesToNanoSecond(uint32_t nClockCycles);
void NuTimeReport(char szText[MAX_JSON_VALUE]);
void NuEnableTIM(uint16_t nPort, uint32_t uCycles, uint16_t TIM_CounterMode, uint16_t TIM_ClockDivision, uint8_t TIM_RepetitionCounter);
void NuEnableTIMTRO(uint16_t nPort, uint32_t uCycles, uint16_t TIM_CounterMode,
  uint16_t TIM_ClockDivision, uint8_t TIM_RepetitionCounter);
void NuEnableIRQTIM(uint16_t nPort, uint32_t uCycles);
void NuEnableTIM_NVIC(uint16_t nPort, uint32_t nvic_PriorityGroup, uint8_t uSubPriority);
#endif /* __NUSTM32_TIM_H */


