#include "Nu_STM32.h"
#include "stdio.h"
void PLC_FastLoop(void);
void PLC_NormalLoop(void);

bool ProcessBlocking(bool* pbHandle)
{
	if (*pbHandle)
		return false;
	*pbHandle = true;
	return true;
}

void ProcessUnblocking(bool* pbHandle)
{
	*pbHandle = false;
}

void TIM1_CC_IRQHandler()
{ 
  if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) 
  {
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
	}
  if (TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET) 
  {
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
	}
  if (TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET) 
  {
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
	}
  if (TIM_GetITStatus(TIM1, TIM_IT_CC4) == SET) 
  {
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
	}
}

uint16_t NotTIM_IT_Update = ~TIM_IT_Update;
uint64_t nSystemClockCnt = 0;

void TIM2_IRQHandler()
{
	if (TIM2->SR & TIM_IT_Update)
	{
		nSystemClockCnt++;
		TIM2->SR = NotTIM_IT_Update;
	}
}

void TIM4_IRQHandler()
{ // DeviceLoop
	if (TIM4->SR & TIM_IT_Update)
	{
		if (mgrPWM.m_bSnapShot)
			NuPWMManagerSnapShotSave();	
		TIM4->SR = NotTIM_IT_Update;
	}
}

void TIM5_IRQHandler()
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != SET)
		return;
	PLC_FastLoop();
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}

void TIM5_CC1_IRQHandler()
{
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != SET)
		return;;
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
}

void TIM6_DAC_IRQHandler()
{
	if (TIM6->SR & TIM_IT_Update)
	{
#ifdef _USE_DAC_
		DACCallBack();
#endif
		TIM6->SR = NotTIM_IT_Update;
	}
}

void TIM7_IRQHandler()
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != SET)
		return;
	PLC_NormalLoop();
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
}

void TIM8_CC_IRQHandler()
{ 
  if (TIM_GetITStatus(TIM8, TIM_IT_CC1) == SET) 
  {
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);
	}
  if (TIM_GetITStatus(TIM8, TIM_IT_CC2) == SET) 
  {
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);
	}
  if (TIM_GetITStatus(TIM8, TIM_IT_CC3) == SET) 
  {
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);
	}
  if (TIM_GetITStatus(TIM8, TIM_IT_CC4) == SET) 
  {
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);
	}
}


