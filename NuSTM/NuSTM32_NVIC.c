#include "NuSTM32_BASIC.h"
#include "NuSTM32_CAN.h"

//////////////////////////////
// NVIC

void NuNVICConfig(uint32_t nvic_PriorityGroup, 
	uint8_t nvic_IRQChannel, uint8_t nvic_IRQChannelPreemptionPriority, 
	uint8_t nvic_IRQChannelSubPriority, FunctionalState nvic_IRQChannelCmd) 
{
  if (nvic_PriorityGroup)
		NVIC_PriorityGroupConfig(nvic_PriorityGroup);
  NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = nvic_IRQChannel; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = nvic_IRQChannelPreemptionPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = nvic_IRQChannelSubPriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = nvic_IRQChannelCmd;
  NVIC_Init(&NVIC_InitStructure);
}
#ifdef _USE_CAN_
void NuEnableCAN_NVIC(uint16_t nPort, uint32_t PriorityGroup, uint8_t uSubPriority)
{
	uint8_t CAN_IRQs[] = {
  CAN1_TX_IRQn ,
  CAN1_RX0_IRQn,
  CAN1_RX1_IRQn,
  CAN1_SCE_IRQn,
  CAN2_TX_IRQn ,
  CAN2_RX0_IRQn,
  CAN2_RX1_IRQn,
  CAN2_SCE_IRQn,
		0, 
		0};
	NuNVICConfig(PriorityGroup, CAN_IRQs[nPort - ST_CAN_1TX], 0, 0, ENABLE);
}
#endif
void NuEnableTIM_NVIC(uint16_t nPort, uint32_t nvic_PriorityGroup, uint8_t uSubPriority)
{
	uint8_t TIM_IRQs[] = {
		0, 
		TIM2_IRQn, 
		TIM3_IRQn, 
		TIM4_IRQn, 
		TIM5_IRQn, 
		TIM6_DAC_IRQn, 
		TIM7_IRQn, 
		0, 
		0, 
		0, 
		0, 
		0, 
		0, 
		0};
	NuNVICConfig(nvic_PriorityGroup, TIM_IRQs[nPort - ST_TIMER_1], 0, uSubPriority, ENABLE);
}

