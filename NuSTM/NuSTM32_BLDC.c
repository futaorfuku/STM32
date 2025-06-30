#include "NuSTM32_BLDC.h"
#include "math.h"

void* fcnTIMCompare[3] = {TIM_SetCompare1, TIM_SetCompare2, TIM_SetCompare3};
void (*TIM_SetCompare)(TIM_TypeDef* TIMx, uint32_t Compare);

double dDutyH = 0;
double dDutyL = 0;

void BLDCBridgeControl(TIM_TypeDef* TIMx, uint16_t tim_Channel, uint8_t bridgeOP, 
	DWORD nMode, uint32_t nFullDutyPulse, bool bForward)
{	
	uint16_t nEnable = TIM_CCx_Enable;
	switch (nMode)
	{
		case MODE_COSYNC:
			if (bridgeOP & BRIDGE_UP)
				TIM_SelectOCxM(TIMx, tim_Channel, TIM_OCMode_PWM1);
			else if (bridgeOP & BRIDGE_DN) 
				TIM_SelectOCxM(TIMx, tim_Channel, TIM_OCMode_PWM2);	 
			else
				nEnable = TIM_CCx_Disable;
			TIM_CCxCmd(TIMx, tim_Channel, nEnable);
			TIM_CCxNCmd(TIMx, tim_Channel, nEnable? TIM_CCxN_Enable: TIM_CCxN_Disable);
			break;
		case MODE_BISYNC:
			if (bridgeOP & BRIDGE_UP)
				TIM_SelectOCxM(TIMx, tim_Channel, TIM_OCMode_PWM1);
			else if (bridgeOP & BRIDGE_DN) 
				TIM_SelectOCxM(TIMx, tim_Channel, TIM_OCMode_PWM2);	 
			else
				nEnable = TIM_CCx_Disable;
			TIM_CCxCmd(TIMx, tim_Channel, nEnable);
			TIM_CCxNCmd(TIMx, tim_Channel, nEnable? TIM_CCxN_Enable: TIM_CCxN_Disable);
			break;
		case MODE_UNISYNC:
			if (bridgeOP)
			{
				TIM_SelectOCxM(TIMx, tim_Channel, TIM_OCMode_PWM1);
				TIM_SetCompare = fcnTIMCompare[tim_Channel / 4];			
			}
			else
				nEnable = TIM_CCx_Disable;
			if (bForward)
			{
				if (bridgeOP & BRIDGE_UP)
					TIM_SetCompare(TIMx, (uint16_t)(dDutyH * 8400));
				if (bridgeOP & BRIDGE_DN)
					TIM_SetCompare(TIMx, 0);
				TIM_CCxCmd(TIMx, tim_Channel, nEnable);
				TIM_CCxNCmd(TIMx, tim_Channel, nEnable? TIM_CCxN_Enable: TIM_CCxN_Disable);
			}
			else
			{
				if (bridgeOP & BRIDGE_UP)
					TIM_SetCompare(TIMx, 0);
				else if (bridgeOP & BRIDGE_DN)
					TIM_SetCompare(TIMx,(uint16_t)(dDutyL * 8400));
				TIM_CCxCmd(TIMx, tim_Channel, nEnable? TIM_CCx_Enable: TIM_CCx_Disable);
				TIM_CCxNCmd(TIMx, tim_Channel, nEnable? TIM_CCxN_Enable: TIM_CCxN_Disable);
			}
			break;
	}
}

ThreePhaseData BridgeControl6Step[8] = {
	{BRIDGE_NO, BRIDGE_NO, BRIDGE_NO}, // Step X, {0, 0, 0}
	{BRIDGE_UP, BRIDGE_DN, BRIDGE_NO}, // Step 1, {A1, B0, A1}	{1, 0, 1}
	{BRIDGE_UP, BRIDGE_NO, BRIDGE_DN}, // Step 2, {1, 0, 0}
	{BRIDGE_NO, BRIDGE_UP, BRIDGE_DN}, // Step 3, {1, 1, 0}
	{BRIDGE_DN, BRIDGE_UP, BRIDGE_NO}, // Step 4, {0, 1, 0}
	{BRIDGE_DN, BRIDGE_NO, BRIDGE_UP}, // Step 5, {0, 1, 1}
	{BRIDGE_NO, BRIDGE_DN, BRIDGE_UP}, // Step 6, {0, 0, 1}
	{BRIDGE_NO, BRIDGE_NO, BRIDGE_NO}  // Step X, {1, 1, 1} 
	};

void BLDC6StepControl(TIM_TypeDef* TIMx, int nBLDCStep, DWORD nMode, uint32_t nFullDutyPulse, bool bForward)
{
	if (!nMode)
		return;
	ThreePhaseData bc = BridgeControl6Step[nBLDCStep];
	BLDCBridgeControl(TIMx, TIM_Channel_1, bc.phaseA, nMode, nFullDutyPulse, bForward);
	BLDCBridgeControl(TIMx, TIM_Channel_2, bc.phaseB, nMode, nFullDutyPulse, bForward);
	BLDCBridgeControl(TIMx, TIM_Channel_3, bc.phaseC, nMode, nFullDutyPulse, bForward);
}

void NuBLDCSetDutyPulse(TIM_TypeDef* TIMx, double dDutyH, double dDutyL, uint16_t nPeriodPulse, int nBLDCStep)
{
	ThreePhaseData bc = BridgeControl6Step[nBLDCStep];
	if (bc.phaseA == BRIDGE_UP)
		TIM_SetCompare1(TIMx, dDutyH * nPeriodPulse);
	else if (bc.phaseA == BRIDGE_DN)
		TIM_SetCompare1(TIMx, dDutyL * nPeriodPulse);
	if (bc.phaseB == BRIDGE_UP)
		TIM_SetCompare2(TIMx, dDutyH * nPeriodPulse);
	else if (bc.phaseB == BRIDGE_DN)
		TIM_SetCompare2(TIMx, dDutyL * nPeriodPulse);
	if (bc.phaseC == BRIDGE_UP)
		TIM_SetCompare3(TIMx, dDutyH * nPeriodPulse);
	else if (bc.phaseC == BRIDGE_DN)
		TIM_SetCompare3(TIMx, dDutyL * nPeriodPulse);
}

//////////////////////////////////////

uint16_t NuGetBLDCPins()
{
	return
		(0x0400 & GPIOA->IDR) |
		(0xE000 & GPIOB->IDR) |
		(0x0B00 & GPIOE->IDR);
}

//////////////////////////////////////////////


void MotorPWMModuleInit(CBLDCMotorDrive* pDrive)
{
	mgrPWM.m_nCallFunction = PWM_CALL_HALF_BLDC;
	uint32_t nPortTimer = pDrive->m_pwmModule.m_nPortTimer;
	int nPWMHz = pDrive->m_pwmModule.m_nPWMHz;
	NuEnableRCCClock(nPortTimer);
	NuEnableTIMCC_NVIC(nPortTimer - ST_TIMER_1 + ST_PWM_1);
	uint8_t GPIO_AF_TIMx = nPortTimer == ST_TIMER_1? GPIO_AF_TIM1: GPIO_AF_TIM8;
	NuEnableGPIOAFPPA(pDrive->m_pwmModule.m_ppaPWM,	GPIO_AF_TIMx, GPIO_High_Speed, GPIO_PuPd_NOPULL);	
	pDrive->m_duty.m_TIM = GetTIM_TypeDef(nPortTimer);
	pDrive->m_nPWMTimerPeriod = NuPWMChannelConfig(nPortTimer, nPWMHz, 3, 0);
	NuPWMOutputEnable(nPortTimer);
}

void BLDCMotorInit(CBLDCMotorDrive* pDrive, int nIndex, bool bMotorReverse, 
	DWORD nPWMMode, GPortPinArray hsensor, GPortPin direction, PWMModule pwmModule, GPortPin gppEnable, 
	PIDController pidControl)
{
	for (int i = 0; i < MAX_STATES; i++)
	{
		pDrive->m_nBLDCStep[i] = 0; 
		pDrive->m_nStepCount[i] = 0;
	}
	pDrive->m_nPWMMode = nPWMMode;
	pDrive->m_pwmModule = pwmModule;
	if (nPWMMode !=  MODE_EXTERN_DRIVER)
		MotorPWMModuleInit(pDrive);
	pDrive->m_nMotorForwardDirection = bMotorReverse? -1: 1;
	pDrive->m_nStepChangeCountDown = 0;
	pDrive->bRotateForward = true;
	pDrive->m_bCommandForward = false;
	pDrive->m_hall = hsensor;
	pDrive->m_direction = direction; 
	pDrive->m_nMillisecondCntPreviousLowpass = nMilliSecondCnt;	
	pDrive->m_nMillisecondCntPreviousInstantaneous = nMilliSecondCnt;	
	pDrive->m_duty.m_nCompare = nIndex;
	NuEnableInputPPA(hsensor);
	NuEnableOutputPP(direction);	
	NuEnableOutputPP(gppEnable);
	pDrive->m_gppEnable = gppEnable;
	SetPIDParameter(&pDrive->m_ctrl, pidControl);
}

//#define BLDC_SIMU
int nSimuStep = 0;

int HallSum(GPortPinArray* pSensor)
{
#ifdef BLDC_SIMU
	int nSumSimu[6] = {5, 4, 6, 2, 1, 3};
	nSimuStep++;
	if (nSimuStep >= 6000)
		 nSimuStep = 0;
	return nSumSimu[nSimuStep / 1000];
#else
	return NuGetBit(pSensor->m_gpp[0]) * 4 + NuGetBit(pSensor->m_gpp[1]) * 2 + NuGetBit(pSensor->m_gpp[2]);
#endif
}

bool StepChangeDetected(CBLDCMotorDrive* pDrive)
{
	int nSum = HallSum(&pDrive->m_hall);
	if (nSum == 0 || nSum == 7)
		return false;	
	int nStepID[8] = {0, 6, 4, 5, 2, 1, 3, 0}; // Step Encoding
	pDrive->m_nBLDCStep[CURRENT_STATE] = nStepID[nSum];
	if (pDrive->m_bCommandForward && 
		!pDrive->m_bInitStart)
	{
		if (pDrive->m_nBLDCStep[CURRENT_STATE] == 1)
			pDrive->m_nBLDCStep[CURRENT_STATE] = 6;
		else
			pDrive->m_nBLDCStep[CURRENT_STATE] -= 1;
	}
	int nStepJump = (pDrive->m_nBLDCStep[CURRENT_STATE] - pDrive->m_nBLDCStep[PREVIOUS_STATE] + 6) % 6;
	if (nStepJump == 1)
		pDrive->bRotateForward = true;
	else if (nStepJump == 5)
		pDrive->bRotateForward = false;
	else if (!pDrive->m_bInitStart)
		pDrive->bRotateForward = true;
	else	
		return false;
	pDrive->m_bInitStart = true;
	return true;
}

int GetStepIncrement(CBLDCMotorDrive* pDrive)
{
	return pDrive->bRotateForward? -1: 1;
}

double GetInstantaneousDirectionalSpeed(CBLDCMotorDrive* pDrive)
{
	double dMinutes = (nMilliSecondCnt - pDrive->m_nMillisecondCntPreviousInstantaneous) / 60000.;
	pDrive->m_nMillisecondCntPreviousInstantaneous = nMilliSecondCnt;
	pDrive->m_dRPMInstantaneous = (double) GetStepIncrement(pDrive) / (3 * POLE_NUMBER) / dMinutes;
}

double GetLowpassDirectionalSpeed(CBLDCMotorDrive* pDrive)
{
	double dMinutes = (nMilliSecondCnt - pDrive->m_nMillisecondCntPreviousLowpass) / 60000.;
	pDrive->m_nMillisecondCntPreviousLowpass = nMilliSecondCnt;
	pDrive->m_dRPMLowpass = pDrive->m_dRPMLowpass * 0.9 + 0.1 * (double) 
		(pDrive->m_nStepCount[CURRENT_STATE] - pDrive->m_nStepCount[PREVIOUS_STATE])
		/ (3 * POLE_NUMBER) / dMinutes;		
	pDrive->m_nStepCount[PREVIOUS_STATE] = pDrive->m_nStepCount[CURRENT_STATE];
	return pDrive->m_dRPMLowpass;
}

void GetHallSensorState(CBLDCMotorDrive* pDrive)
{
	if (!StepChangeDetected(pDrive))		
		return;
	GetInstantaneousDirectionalSpeed(pDrive);
	pDrive->m_nBLDCStep[PREVIOUS_STATE] = pDrive->m_nBLDCStep[CURRENT_STATE];
	if (pDrive->m_bCommandForward)
	{	
		pDrive->m_nStepChangeCountDown = 
		60 / pDrive->m_dRPMInstantaneous  / (3 * POLE_NUMBER) / 3 * FASTLOOP_HZ * 0.9;
	}
	else
		pDrive->m_nStepChangeCountDown = 0;
}

void DoChangeStepControl(CBLDCMotorDrive* pDrive)
{
	BLDC6StepControl(pDrive->m_duty.m_TIM, pDrive->m_nBLDCStep[CURRENT_STATE], pDrive->m_nPWMMode,
		pDrive->m_nPWMTimerPeriod, pDrive->m_bCommandForward);	
	pDrive->m_nStepCount[CURRENT_STATE] += GetStepIncrement(pDrive);
}

void ExternalDriverSetDirectionAndDuty(CBLDCMotorDrive* pDrive, double du)
{
	TIM_SetCompare = fcnTIMCompare[pDrive->m_duty.m_nCompare - 1];		
	uint16_t nABSU = (uint16_t) _fmin(du >= 0? du: -du, pDrive->m_nPWMTimerPeriod);
	NuSetBit(pDrive->m_direction, pDrive->m_bCommandForward);
	TIM_SetCompare(pDrive->m_duty.m_TIM, nABSU);
}

void MotorSetDirectionAndDuty(CBLDCMotorDrive* pDrive, double du)
{
	double dRatio = _fmax(-0.2, _fmin(0.2, (du /pDrive->m_nPWMTimerPeriod)));
	//double dRatio = 0.03;
	pDrive->m_bCommandForward = (dRatio > 0);
	double dAlpha = (_fabs(dRatio));
	double dBeta = 0.5 * dAlpha * (1 - dRatio);
#ifdef BLDC_SIMU
	dRatio = nSimuStep / 3000. - 1; 
#endif
	switch (pDrive->m_nPWMMode)
	{
		case MODE_BISYNC:
			dDutyH = dDutyL = (dRatio + 1) / 2;
			break;
		case MODE_COSYNC:
			if (dRatio >= 0)
			{
				dDutyH = dRatio + dBeta;
				dDutyL = 1 - dBeta;				
			}
			else
			{
				dDutyL = 1 - dBeta;	
				dDutyH = dRatio + dBeta; 
			}
			break;
		case MODE_UNISYNC:
			if (dRatio >= 0)
			{
				dDutyH = dRatio;
				dDutyL = 0;
			}
			else
			{
				dDutyL = -dRatio;
				dDutyH = 0;
			}				
			break;
	}
	NuBLDCSetDutyPulse(pDrive->m_duty.m_TIM, dDutyH, dDutyL, 
		pDrive->m_nPWMTimerPeriod, pDrive->m_nBLDCStep[CURRENT_STATE]);
}

void ResetOutput(CBLDCMotorDrive* pDrive)
{
	switch (pDrive->m_nPWMMode)
	{
		case MODE_EXTERN_DRIVER:
			TIM_SetCompare = fcnTIMCompare[pDrive->m_duty.m_nCompare - 1];		
			TIM_SetCompare(pDrive->m_duty.m_TIM, 0);
			break;
		case MODE_BISYNC:
		case MODE_UNISYNC:
		case MODE_COSYNC:
			for (int j = 0; j < 3; j++)
			{
				TIM_SetCompare = fcnTIMCompare[j];		
				TIM_SetCompare(pDrive->m_duty.m_TIM, 0);
			} 
			break;
	}
}


