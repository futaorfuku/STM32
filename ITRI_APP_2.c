#include "ITRI_APP.h"

#define WHEEL_LEFT	0
#define WHEEL_RIGHT	1
#define MODE_MANUAL	0
#define MODE_AUTO		1
#define MODE_NUMBER_INDEX_ONE		1
#define MODE_NUMBER_INDEX_TWO		2
#define MODE_NUMBER_INDEX_THREE	3
#define POLE_NUMBER		  		  30
#define CIRCLE_DIAMETER			  87.7761

CITRICar ITRICar;

void MotorPWMModuleInit(CHubMotor* pMotor)
{
	uint32_t nPortTimer = pMotor->m_pwmModule.m_nPortTimer;
	int nPWMHz = pMotor->m_pwmModule.m_nPWMHz;
	NuEnableRCCClock(nPortTimer);
	NuEnableTIMCC_NVIC(nPortTimer - ST_TIMER_1 + ST_PWM_1);
	switch (nPortTimer)
	{
		case ST_TIMER_1:
			NuEnableTIM1GPIOAF(pMotor->m_pwmModule.m_portGPIO.m_nPort, pMotor->m_pwmModule.m_portGPIO.m_nPinID);	// TIM1_CH1, TIM1_CH2, TIM1_CH3
		break;
		case ST_TIMER_8:
			NuEnableTIM1GPIOAF(pMotor->m_pwmModule.m_portGPIO.m_nPort, pMotor->m_pwmModule.m_portGPIO.m_nPinID);	// TIM8_CH1, TIM8_CH2, TIM8_CH3
		break;
	}
	pMotor->m_duty.m_TIM = GetTIM_TypeDef(nPortTimer);
	NuPWMChannelConfig(nPortTimer, nPWMHz, 3, 0, &ITRICar.m_nPWMTimerPeriod);
	NuPWMOutputEnable(nPortTimer);
}

void HubMotorInit(CHubMotor* pMotor, int nIndex, bool bMotorReverse, 
	DWORD nPWMMode, HallSensor hsensor, PortPin direction, PWMModule pwmModule,
	double KP, double KI, double KD, double SamplingTime)
{
	for (int i = 0; i < MAX_STATES; i++)
	{
		pMotor->m_nBLDCStep[i] = 0;
		pMotor->m_nStepCount[i] = 0;
	}
	pMotor->m_nPWMMode = nPWMMode;
	pMotor->m_pwmModule = pwmModule;
	if (nPWMMode !=  MODE_EXTERN_DRIVER)
		MotorPWMModuleInit(pMotor);
	pMotor->m_nMotorForwardDirection = bMotorReverse? -1: 1;
	pMotor->bRotateForward = true;
	pMotor->m_bCommandForward = true;
	pMotor->m_dTotalMileage = 0;
	pMotor->m_hall = hsensor;
	pMotor->m_direction = direction;
	pMotor->m_dMeterPerStep = 1. / (3 * POLE_NUMBER) * CIRCLE_DIAMETER;
	pMotor->m_nFastLoopCntPrevious = nFastLoopCnt;	
	pMotor->m_duty.m_nCompare = nIndex;
	NuEnableInputGPIO(hsensor.m_nPort, hsensor.m_HallA | hsensor.m_HallB | hsensor.m_HallC);
	NuEnableOutputGPIO(direction.m_nPort, direction.m_nPinID);
	SetPIDParameter(&pMotor->m_ctrl, KP, KI, KD, SamplingTime);
}

int HallSum(HallSensor* pSensor)
{
	int nSum = 0;
	uint32_t IDRE = GetGPIO_TypeDef(pSensor->m_nPort)->IDR;
	if (IDRE & pSensor->m_HallA)
		nSum += 4;
	if (IDRE & pSensor->m_HallB)
		nSum += 2;
	if (IDRE & pSensor->m_HallC)
		nSum += 1;
	return nSum;
}

bool StepChangeDetected(CHubMotor* pMotor)
{
	int nSum = HallSum(&pMotor->m_hall);
	if (nSum == 0 || nSum == 7)
		return false;	
	int nStepID[8] = {0, 4, 6, 5, 2, 3, 1, 0}; // Step Encoding
	BLDC6StepControl(pMotor->m_duty.m_TIM, &BridgeControl6Step[nSum - 1], 
		pMotor->m_nPWMMode, ITRICar.m_nPWMTimerPeriod, pMotor->m_bCommandForward);
	pMotor->m_nBLDCStep[CURRENT_STATE] = nStepID[nSum];	
	int nStepJump = (pMotor->m_nBLDCStep[CURRENT_STATE] - pMotor->m_nBLDCStep[PREVIOUS_STATE] + 6) % 6;
	if (nStepJump == 1)
		pMotor->bRotateForward = true;
	else if (nStepJump == 5)
		pMotor->bRotateForward = false;
	else
		return false;
	return true;
}

void GetHallSensorState(CHubMotor* pMotor)
{
	if (StepChangeDetected(pMotor))
	{
		pMotor->m_nStepCount[CURRENT_STATE] += pMotor->bRotateForward? 1: -1;
		pMotor->m_nBLDCStep[PREVIOUS_STATE] = pMotor->m_nBLDCStep[CURRENT_STATE];
	}
}

void MotorCalculateMileage(CHubMotor* pMotor)
{
	pMotor->m_dTotalMileage = pMotor->m_dMeterPerStep * pMotor->m_nStepCount[CURRENT_STATE];
}

void InitPIDControllers()
{
	for (int i = 0; i < HUB_MOTORS; i++)
		InitPIDController(&ITRICar.HubMotor[i].m_ctrl);
}

////////////////////////////////////////////////

#define ADVALUE_THROTTLE_RANGE			1600
#define ADVALUE_THROTTLE_THRESHOLD	1500
#define ADVALUE_STEERING_MIDDLE  		2000 
#define ADVALUE_STEERING_RANGE			10000 // corresponding to pure spin
#define ADVALUE_STEERING_THRESHOLD	200
#define RPM_RANGE										50
#define AXEL_LENGTH									48.8 // [cm]

double GetDirectionalSpeed(CHubMotor* pMotor)
{
	double dMinutes = 1. / 60 * (nFastLoopCnt - pMotor->m_nFastLoopCntPrevious) / FASTLOOP_HZ;
	pMotor->m_nFastLoopCntPrevious = nFastLoopCnt;
	double dRPM = (double) (pMotor->m_nStepCount[CURRENT_STATE] - pMotor->m_nStepCount[PREVIOUS_STATE]) / (3 * POLE_NUMBER) / dMinutes ;
	pMotor->m_nStepCount[PREVIOUS_STATE] = pMotor->m_nStepCount[CURRENT_STATE];
	return dRPM;
}

void ExternalDriverSetDirectionAndDuty(CHubMotor* pMotor, double du)
{
	void (*TIM_SetCompare)(TIM_TypeDef* TIMx, uint32_t Compare);
	void* fcnCompare[3] = {TIM_SetCompare1, TIM_SetCompare2, TIM_SetCompare3};
	TIM_SetCompare = fcnCompare[pMotor->m_duty.m_nCompare - 1];		
	uint16_t nABSU = (uint16_t) min(fabs(du), ITRICar.m_nPWMTimerPeriod);
	GPIO_WriteBit(GetGPIO_TypeDef(pMotor->m_direction.m_nPort), pMotor->m_direction.m_nPinID, 
		pMotor->m_bCommandForward ? Bit_RESET: Bit_SET);
	TIM_SetCompare(pMotor->m_duty.m_TIM, nABSU);
}

void MotorSetDirectionAndDuty(CHubMotor* pMotor, double du)
{
	pMotor->m_bCommandForward = du > 0;
	double dDuty =  max(-1, min(1, (du /ITRICar.m_nPWMTimerPeriod)));
	switch (pMotor->m_nPWMMode)
	{
		case MODE_EXTERN_DRIVER:
			ExternalDriverSetDirectionAndDuty(pMotor, du);
			break;
		case MODE_BISYNC:
			NuBLDCSetDutyPulse(pMotor->m_duty.m_TIM, (dDuty + 1) * ITRICar.m_nPWMTimerPeriod / 2);
			break;
		case MODE_UNISYNC:
			NuBLDCSetDutyPulse(pMotor->m_duty.m_TIM, dDuty * ITRICar.m_nPWMTimerPeriod);
			break;
	}
}

void ResetMotorOutput()
{
	for (int i = 0; i < 2; i++)
		MotorSetDirectionAndDuty(&ITRICar.HubMotor[i], 0);
}

void MotorDriver(CHubMotor* pMotor, double dSpeedLinear, double dSpeedRotate)
{
	double dSpeedCmd = pMotor->m_nMotorForwardDirection * 
		(dSpeedLinear + pMotor->m_nMotorForwardDirection * dSpeedRotate);
	MotorSetDirectionAndDuty(pMotor, 
		PIDFeedback(&pMotor->m_ctrl, dSpeedCmd, GetDirectionalSpeed(pMotor)));
}

void MotorRun(void)
{
	double dSpeedThrottle = ITRICar.m_dLPFValue[ADC_STEERING] > ADVALUE_THROTTLE_THRESHOLD? 
		(ITRICar.m_dLPFValue[ADC_STEERING] - ADVALUE_THROTTLE_THRESHOLD) * ITRICar.dADValueToSpeedCmd: 0;
	double dRotateRatio = !ITRICar.m_bUseSteering? 0: (fabs(ITRICar.m_dLPFValue[ADC_STEERING] - ADVALUE_STEERING_MIDDLE) >= ADVALUE_STEERING_THRESHOLD? 
		(ITRICar.m_dLPFValue[ADC_STEERING] - ADVALUE_STEERING_MIDDLE) / ADVALUE_STEERING_RANGE: 0);
	for (int i = 0; i < 2; i++)
	{
		MotorDriver(&ITRICar.HubMotor[i],
			dSpeedThrottle * (1 - fabs(dRotateRatio)), dSpeedThrottle * dRotateRatio);
	}
}

void ReadControlMode()
{
	ITRICar.m_nWorkMode[CURRENT_STATE] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
	ITRICar.m_nWorkMode[CURRENT_STATE] = MODE_AUTO;
	if (ITRICar.m_nWorkMode[PREVIOUS_STATE] != ITRICar.m_nWorkMode[CURRENT_STATE])
		InitPIDControllers();
}

void ControlPinInit(void)
{
	NuEnableInputGPIO(ST_E_PORT, GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4);
	ReadControlMode();
}

void ReadHandleThreeMode(void)
{
	ITRICar.nReadModeNumberPin[0] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3);
	ITRICar.nReadModeNumberPin[1] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
}

////////////////////////////////////////
#define AD_VALUE_SIZE	20
uint16_t AD_Value[AD_VALUE_SIZE];

void ITRICarADCInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	NuEnableDMA(ST_DMA_2, DMA_Channel_0, 0, (uint32_t) &ADC1->DR, (uint32_t) AD_Value, AD_VALUE_SIZE, DMA_DIR_PeripheralToMemory);
	NuEnableAnalogGPIO(ST_A_PORT, GPIO_Pin_2|GPIO_Pin_3);
	NuADCCommonConfigEx(ADC_Mode_Independent, ADC_TwoSamplingDelay_5Cycles, ADC_DMAAccessMode_Disabled,
		ADC_Prescaler_Div4);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_480Cycles);
	ADCInitConfig(&ADC_InitStructure, ADC_Resolution_12b, 2, ENABLE);
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);
}

void CarPWMDisablePinInit(PortPin p)
{
	ITRICar.m_pwmDisable = p;
	NuEnableOutputGPIO(p.m_nPort, p.m_nPinID);
	GPIO_SetBits(GetGPIO_TypeDef(ITRICar.m_pwmDisable.m_nPort), ITRICar.m_pwmDisable.m_nPinID);
}

void TwinMotorPWMInit(uint32_t nPortTimer, int nPWMHz)
{
	NuEnableRCCClock(nPortTimer);
	NuEnableTIMCC_NVIC(nPortTimer - ST_TIMER_1 + ST_PWM_1);
	NuEnableTIM1GPIOAF(ST_A_PORT, GPIO_Pin_8); 	// TIM1_CH1
	NuEnableTIM1GPIOAF(ST_E_PORT, GPIO_Pin_11 | GPIO_Pin_12); // TIM1_CH2, 3N why??
	NuPWMChannelConfig(nPortTimer, nPWMHz, 2, 0, &ITRICar.m_nPWMTimerPeriod);
	NuPWMOutputEnable(nPortTimer);
	for (int i = 0; i < 2; i++)
		ITRICar.HubMotor[i].m_duty.m_TIM = GetTIM_TypeDef(nPortTimer);
}

void ITRICarInit(bool bUseSteering, DWORD nPWMMode, int nPWMHz)
{
	ITRICar.m_bUseSteering = bUseSteering;	
	ITRICar.m_nWorkMode[CURRENT_STATE] = ITRICar.m_nWorkMode[PREVIOUS_STATE] = MODE_MANUAL;
	for (int i = 0; i < ADC_CHANNELS; i++)
		ITRICar.m_dLPFValue[i] = 0;	
	switch (nPWMMode)
	{
		case MODE_EXTERN_DRIVER:
			TwinMotorPWMInit(ST_TIMER_1, nPWMHz);
			break;
	}
	ControlPinInit();
	ITRICarADCInit();
	ITRICar.dADValueToSpeedCmd = (double) RPM_RANGE /	ADVALUE_THROTTLE_RANGE;
	CarPWMDisablePinInit((PortPin) {ST_B_PORT, GPIO_Pin_2}); // why?
	HubMotorInit(&ITRICar.HubMotor[WHEEL_LEFT], 1,
		true, nPWMMode,
		(HallSensor) {ST_E_PORT, GPIO_Pin_8, GPIO_Pin_12, GPIO_Pin_10}, 
		(PortPin) {ST_B_PORT, GPIO_Pin_0}, 
		(PWMModule) {ST_TIMER_1, nPWMHz, (PortPin) {ST_A_PORT, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10}},
		1, 0.0001, 0, 0.05);
	HubMotorInit(&ITRICar.HubMotor[WHEEL_RIGHT], 2,
		false, nPWMMode,
		(HallSensor) {ST_E_PORT, GPIO_Pin_7, GPIO_Pin_9, GPIO_Pin_13},
		(PortPin) {ST_B_PORT, GPIO_Pin_1}, 
		(PWMModule) {ST_TIMER_8, nPWMHz, (PortPin) {ST_C_PORT, GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8}},
		1, 0.0001, 0, 0.05);
}

void ITRICarADCLowPassFilter()
{
	int nChannels = ITRICar.m_bUseSteering? 2: 1;
	for (int j = 0; j < nChannels; j++)
	{
		for (int i = j; i < AD_VALUE_SIZE; i += 6)
		ITRICar.m_dLPFValue[j] += (AD_Value[i] - ITRICar.m_dLPFValue[j]) * 0.2;
	}
}

void MotorBrake(bool bStop)
{
	if (bStop)
		ResetMotorOutput();
	GPIO_WriteBit(GetGPIO_TypeDef(ITRICar.m_pwmDisable.m_nPort), ITRICar.m_pwmDisable.m_nPinID, bStop? Bit_SET: Bit_RESET);
}

///////////////////////////////////////////////
void ITRICarMain(void)
{
	ITRICarInit(
		true /*use steering*/, 
		MODE_EXTERN_DRIVER,
		10000 /*Hz*/); 
	ResetMotorOutput();
	JsonWriteIntoJsonPair("ITRICar", "ON"); 
}

bool ITRICarNormalLoopInProcess = false;
bool ITRICarFastLoopInProcess = false;
bool bReportInit = false;

void ITRICarReport(void)
{
	if (!bReportInit)
	{
		fprintf(USART_FILE, "{\"OpenCSVFile\" = \"D:/test123.csv\"}"); 
		fprintf(USART_FILE, "{\"SetKeys\" = \"Time,Left Wheel,Right Wheel,Throttle,Steering\"}"); 
		bReportInit = true;
	}
	char szText[MAX_JSON_VALUE];
	memset(szText, 0, MAX_JSON_VALUE);
	int nSeconds = nFastLoopCnt / FASTLOOP_HZ;
	sprintf(szText, "%02d:%02d:%02d", nSeconds / 3600, nSeconds / 60 % 60, nSeconds % 60);
	fprintf(USART_FILE, "{\"%s\"=\"%s,%d,%d,%.2f,%.2f\"}",
		ITRICar.m_nWorkMode[CURRENT_STATE] == MODE_MANUAL? "SetValues": "SetBeepValues",
		szText,
		ITRICar.HubMotor[WHEEL_LEFT].m_nStepCount[CURRENT_STATE],
		ITRICar.HubMotor[WHEEL_RIGHT].m_nStepCount[CURRENT_STATE],
		ITRICar.m_dLPFValue[ADC_THROTTLE],
		ITRICar.m_dLPFValue[ADC_STEERING]);
}

void ITRICarNormalLoop(void)
{ 
	if (!ProcessBlocking(&ITRICarNormalLoopInProcess))
		return;
	ReadControlMode();
	switch(ITRICar.m_nWorkMode[CURRENT_STATE])
	{
		case MODE_MANUAL:
			MotorBrake(true);
			break;
		case MODE_AUTO:
			MotorBrake(false);
			ITRICarADCLowPassFilter();
			MotorRun(); 
			break;
	}
	for (int i = 0; i < 2; i++)
		MotorCalculateMileage(&ITRICar.HubMotor[i]);
	ITRICar.m_nWorkMode[PREVIOUS_STATE] = ITRICar.m_nWorkMode[CURRENT_STATE];
	ProcessUnblocking(&ITRICarNormalLoopInProcess);
}

void ITRICarFastLoop(void)
{
	if (!ProcessBlocking(&ITRICarFastLoopInProcess))
		return;
	for (int i = 0; i < 2; i++)
		GetHallSensorState(&ITRICar.HubMotor[i]);
	ProcessUnblocking(&ITRICarFastLoopInProcess);
}

