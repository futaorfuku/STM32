#include "ITRI_APP.h"

#define WHEEL_LEFT	0
#define WHEEL_RIGHT	1
#define MODE_MANUAL	0
#define MODE_AUTO		1
#define BRAKE_RATE 	0.8
#define ZERO_FREQ_TICK_NUMBER  	4000
#define READ_FREQ_TICK_NUMBER  	10000
#define MODE_NUMBER_INDEX_ONE		1
#define MODE_NUMBER_INDEX_TWO		2
#define MODE_NUMBER_INDEX_THREE	3
#define POLE_NUMBER		  		  30
#define CIRCLE_DIAMETER			  87.7761

CITRICar ITRICar;

void PWM_2ChannelConfig(uint32_t uHz, 
	uint32_t uDuty1, uint32_t uDeadTime1, uint32_t uDuty2, uint32_t uDeadTime2)
{
	PWMOCInit pwm;
	uint32_t TimerPeriod = NuTimeBaseHzConfig(&pwm.TIM_TimeBaseStructure, uHz, TIM_CounterMode_CenterAligned1);;
	uint32_t Duty[2] = {
		1000 - uDuty1 + uDeadTime1 / 2, uDuty2 - uDeadTime2 / 2};	
  for (int i = 0; i < 2; i++)
		pwm.ChannelPulse[i] = (uint16_t) (((uint32_t) Duty[i] * (TimerPeriod - 1)) / 1000);
	pwm.TIMx = TIM1;
	NuTimeOCInitStructure(&pwm.TIM_OCInitStructure, TIM_OCMode_PWM1, TIM_OCIdleState_Reset, TIM_OCNIdleState_Reset);
}

void TwinMotorPWMInit(int nPWMHz)
{
	NuEnableTIMCC_NVIC(ST_PWM_1);
	NuEnableGPIOAF(ST_A_PORT, GPIO_Pin_8, GPIO_AF_TIM1, 
		GPIO_Medium_Speed, GPIO_OType_PP, GPIO_PuPd_NOPULL); // TIM1 Channel 1
	NuEnableGPIOAF(ST_E_PORT, GPIO_Pin_11, GPIO_AF_TIM1, 
		GPIO_Medium_Speed, GPIO_OType_PP, GPIO_PuPd_NOPULL); // TIM1 Channel 2
	PWM_2ChannelConfig(nPWMHz, 0, 0, 0, 0);
	// Brake
	NuTIMBDTRInit(TIM1, 1);
	NuEnableGPIO(ST_B_PORT, GPIO_Pin_2, GPIO_Mode_OUT, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	GPIO_SetBits(GPIOB, GPIO_Pin_2);
	// Init
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void InitPIDController(PIDController* pid)
{
	pid->ErrorAccumulation = 0;
	pid->PerviousError = 0;
	pid->U = 0;
}

void SetPIDParameter(PIDController* pid, double KP, double KI, double KD, double SamplingTime)
{
	pid->KP = KP;
	pid->KI = KI;
	pid->KD = KD;
	pid->SamplingTime = SamplingTime;
}

double PIDFeedback(PIDController *pid, double Target, double Estimate)
{
	double dError = Target - Estimate;
	pid->U = pid->KP * dError + pid->KI * pid->ErrorAccumulation + pid->KD * (dError - pid->PerviousError) / pid->SamplingTime;
	pid->ErrorAccumulation += dError * pid->SamplingTime;
	pid->PerviousError = dError;
	return pid->U;
}

void HallSensorInit(void)
{
	NuEnableGPIO(ST_E_PORT, GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_13,
		GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_DOWN);
}

void HubMotorInit(CHubMotor* pMotor, bool bMotorReverse, 
	uint16_t nPinID1, uint16_t nPinID2, uint16_t nPinID3,
	double KP, double KI, double KD, double SamplingTime)
{
	pMotor->m_bMotorReverse = bMotorReverse;
	pMotor->nStepHz = 1;
	pMotor->dFeedback_U = 0;
	pMotor->nCurrentStep = 0;
	pMotor->nPreviousStep = 0;
	pMotor->bRotateForward = true;
	pMotor->nStepChanges = 0;
	pMotor->dTotalMilage = 0;
	pMotor->m_nPinID[0] = nPinID1;
	pMotor->m_nPinID[1] = nPinID2;
	pMotor->m_nPinID[2] = nPinID3;
	pMotor->dMeterPerStep = 1. / (3 * POLE_NUMBER) * CIRCLE_DIAMETER;
	pMotor->m_nFastLoopCntPrevious = nFastLoopCnt;
	SetPIDParameter(&pMotor->m_ctrl, KP, KI, KD, SamplingTime);
}

bool StepChangeDetected(CHubMotor* pMotor, int nSum)
{
	if (nSum == 0 || nSum == 7)
		return false;	
	int nStepID[8] = {0, 4, 6, 5, 2, 3, 1, 0}; // Step Encoding
	pMotor->nCurrentStep = nStepID[nSum];	
	int nStepJump = (pMotor->nCurrentStep - pMotor->nPreviousStep + 6) % 6;
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
	int nSum = 0;
	int nMultiple[3] = {1, 2, 4};
	uint32_t IDRE = GPIOE->IDR;
	for (int i = 0; i < HALL_SENSOR_BITS; i++)
	{
		if (IDRE & pMotor->m_nPinID[i])
			nSum += nMultiple[i];
 	}
	if (!StepChangeDetected(pMotor, nSum))
		return;
	pMotor->nStepHz = nFastLoopCnt - pMotor->m_nFastLoopCntPrevious;
	pMotor->nStepChanges += pMotor->bRotateForward? 1: -1;
	pMotor->m_nFastLoopCntPrevious = nFastLoopCnt;
	pMotor->nPreviousStep = pMotor->nCurrentStep;
}

void DriverDirectionPinInit(void)
{
	NuEnableGPIO(ST_B_PORT, GPIO_Pin_0|GPIO_Pin_1,
		GPIO_Mode_OUT, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
}

void GetDirectionalSpeed(CHubMotor* pMotor)
{
	pMotor->dRPM = 4. * FASTLOOP_HZ / pMotor->nStepHz * (pMotor->bRotateForward? 1: -1);
}

void MotorBrake(bool bStop)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_2, bStop? Bit_SET: Bit_RESET);
}

void InitPIDControllers()
{
	for (int i = 0; i < HUB_MOTORS; i++)
			InitPIDController(&ITRICar.HubMotor[i].m_ctrl);
}

////////////////////////////////////////////////
#define MOTOR_RUN_AD_VALUE	   780
#define MAX_GRIP_AD_VALUE	   1960
#define MIN_GRIP_AD_VALUE	   738
#define MAX_RPM		60
#define MIN_RPM		15
#define MOTOR_MIDDLE_VOLT_AD_VALUE   	 	2000 
#define STRAIGHT_AD_VALUE_RANGE	  		  200
#define AXEL_LENGTH	48.8
#define MAX_ICR			200
#define MIN_ICR			80
#define MOTOR_ROTATE_MAX_LEFT_AD_VALUE		-400
#define MOTOR_ROTATE_MAX_RIGHT_AD_VALUE		600

void MotorRotate(double dRotateADValue)
{
	double dICR; // Instant Center of Rotation
	double dADValue = dRotateADValue - MOTOR_MIDDLE_VOLT_AD_VALUE;
	if (fabs(dADValue) < STRAIGHT_AD_VALUE_RANGE)
		 return;
	double dMaxADValue = dADValue > 0? MOTOR_ROTATE_MAX_RIGHT_AD_VALUE: MOTOR_ROTATE_MAX_LEFT_AD_VALUE;
	dICR = MIN_ICR + (double)(MAX_ICR - MIN_ICR) * (dMaxADValue - dADValue) / (dMaxADValue - STRAIGHT_AD_VALUE_RANGE);
	double dAAA = (dICR - AXEL_LENGTH) / dICR;
	if (dADValue > 0)
		ITRICar.HubMotor[WHEEL_RIGHT].m_dSpeedCmd = -ITRICar.HubMotor[WHEEL_LEFT].m_dSpeedCmd * dAAA;
	else
		ITRICar.HubMotor[WHEEL_LEFT].m_dSpeedCmd = -ITRICar.HubMotor[WHEEL_RIGHT].m_dSpeedCmd * dAAA;
}

void DriverRun(uint8_t nWheelID)
{
	switch (nWheelID)
	{
		case WHEEL_LEFT:
			GPIO_WriteBit(GPIOB, GPIO_Pin_0, ITRICar.HubMotor[nWheelID].dFeedback_U > 0? Bit_RESET: Bit_SET);
			break;
		case WHEEL_RIGHT:
			GPIO_WriteBit(GPIOB, GPIO_Pin_1, ITRICar.HubMotor[nWheelID].dFeedback_U > 0? Bit_RESET: Bit_SET);
			break;
	}
}

void ITRICarMotorDriver(void)
{
	for (int i = 0; i < 2; i++)
	{
		GetDirectionalSpeed(&ITRICar.HubMotor[i]);
		ITRICar.HubMotor[i].dFeedback_U = PIDFeedback(&ITRICar.HubMotor[i].m_ctrl, ITRICar.HubMotor[i].m_dSpeedCmd, ITRICar.HubMotor[i].dRPM);
		DriverRun(i);
		ITRICar.HubMotor[i].dFeedback_U = fabs(ITRICar.HubMotor[i].dFeedback_U);
	}	
	TIM_SetCompare1(TIM1, (uint16_t) (11.408 + ITRICar.HubMotor[WHEEL_LEFT].dFeedback_U) / 5.5423 * 10);
	TIM_SetCompare2(TIM1, (uint16_t) (11.352 + ITRICar.HubMotor[WHEEL_RIGHT].dFeedback_U) / 5.4904 * 10);
}

void MotorRun(void)
{
	double dSpeedLinear = (ITRICar.dLPFValueThrottle - MOTOR_RUN_AD_VALUE) * ITRICar.dADValueToSpeedCmd;
	bool bPositiveSpeed = dSpeedLinear > 0;
	MotorBrake(!bPositiveSpeed);	
	if (!bPositiveSpeed)
		return;
	for (int i = 0; i < 2; i++)
		ITRICar.HubMotor[i].m_dSpeedCmd = (ITRICar.HubMotor[i].m_bMotorReverse? -1: 1) * dSpeedLinear;
	MotorRotate(ITRICar.dLPFValueSteering);
	ITRICarMotorDriver();
}

void MotorBrakes(void)
{
	for (int i = 0; i < HUB_MOTORS; i++)
	{
		ITRICar.HubMotor[i].m_dSpeedCmd = sign(ITRICar.HubMotor[i].m_dSpeedCmd) *
			sqr(sqrt(fabs(ITRICar.HubMotor[i].m_dSpeedCmd)) * BRAKE_RATE);
	}
}

void ReadControlMode()
{
	ITRICar.nWorkMode = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
}

void ControlPinInit(void)
{
	NuEnableGPIO(ST_E_PORT, GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4,
		GPIO_Mode_IN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_DOWN);
	ReadControlMode();
}

void ReadHandleThreeMode(void)
{
	ITRICar.nReadModeNumberPin[0] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3);
	ITRICar.nReadModeNumberPin[1] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
	if (!ITRICar.nReadModeNumberPin[0])
	{
		if (ITRICar.nReadModeNumberPin[1])
			ITRICar.nModeNumber = MODE_NUMBER_INDEX_ONE;
		ITRICar.nModeNumber = MODE_NUMBER_INDEX_TWO;
	}
	if (ITRICar.nReadModeNumberPin[0])
	{
		if (!ITRICar.nReadModeNumberPin[1])
			ITRICar.nModeNumber = MODE_NUMBER_INDEX_THREE;
	}
}

////////////////////////////////////////
#define AD_VALUE_SIZE	200
uint16_t AD_Value[AD_VALUE_SIZE];

void ITRICarADCInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	NuEnableDMA(ST_DMA_2, DMA_Channel_0, 0, (uint32_t) &ADC1->DR, (uint32_t) AD_Value, AD_VALUE_SIZE, DMA_DIR_PeripheralToMemory);
	NuEnableGPIO(ST_A_PORT, GPIO_Pin_2|GPIO_Pin_3,
		GPIO_Mode_AN, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
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

void ITRICarInit()
{
	ITRICar.nWorkMode = ITRICar.nPreviousWorkMode = MODE_MANUAL;
	TwinMotorPWMInit(10000);
	HallSensorInit();
	DriverDirectionPinInit();
	ControlPinInit();
	ITRICarADCInit();
	ITRICar.dLPFValueThrottle = 0;
	ITRICar.dLPFValueSteering = 0;
	ITRICar.dADValueToSpeedCmd = (MAX_RPM - MIN_RPM) /(MAX_GRIP_AD_VALUE  - MOTOR_RUN_AD_VALUE) + MIN_RPM;
	HubMotorInit(&ITRICar.HubMotor[WHEEL_LEFT], true, 
		GPIO_Pin_12, GPIO_Pin_10, GPIO_Pin_8 /*U*/, 1.185, 1, 0.00005, 0.05);
	HubMotorInit(&ITRICar.HubMotor[WHEEL_RIGHT], false, 
		GPIO_Pin_13, GPIO_Pin_9, GPIO_Pin_7 /*U*/, 1.18, 1, 0.0001, 0.05);
	InitPIDControllers();
}

void ITRICarADCLowPassFilter()
{
	for (int i = 0; i < AD_VALUE_SIZE; i += 5)
		ITRICar.dLPFValueThrottle += (AD_Value[i] - ITRICar.dLPFValueThrottle) * 0.3;
	for (int i = 1; i < AD_VALUE_SIZE; i += 5)
		ITRICar.dLPFValueSteering += (AD_Value[i] - ITRICar.dLPFValueSteering) * 0.3;
}

///////////////////////////////////////////////

bool bReportInit = false;
void ITRICarReport(void)
{
	if (!bReportInit)
	{
		fprintf(USART_FILE, "{\"OpenCSVFile\" = \"D:/test123.txt\"}"); 
		fprintf(USART_FILE, "{\"SetKey1\" = \"Left Wheel\"}"); 
		fprintf(USART_FILE, "{\"SetKey2\" = \"Right Wheel\"}"); 
		fprintf(USART_FILE, "{\"SetKey3\" = \"SpeedCmd\"}"); 
		bReportInit = true;
	}
	fprintf(USART_FILE, "{\"SetValue1\" = \"%.2f\"}", ITRICar.HubMotor[WHEEL_RIGHT].m_dSpeedCmd);
	fprintf(USART_FILE, "{\"SetValue2\" = \"%.2f\"}", 1.);
	fprintf(USART_FILE, "{\"SetValue3\" = \"%.2f\"}", 2.);
}

bool ITRICarNormalLoopInProcess = false;
void ITRICarNormalLoop(void)
{ 
	if (ITRICarNormalLoopInProcess)
		return;
	ITRICarNormalLoopInProcess = true;
	ReadControlMode();
	switch(ITRICar.nWorkMode)
	{
		case MODE_MANUAL:
			MotorBrake(true);
			break;
		case MODE_AUTO:
			ITRICarADCLowPassFilter();
			if (ITRICar.nPreviousWorkMode != ITRICar.nWorkMode)
				InitPIDControllers();
			MotorRun();
			break;
	}
	for (int i = 0; i < 2; i++)
		ITRICar.HubMotor[i].dTotalMilage = ITRICar.HubMotor[i].dMeterPerStep * ITRICar.HubMotor[i].nStepChanges;
	ITRICar.nPreviousWorkMode = ITRICar.nWorkMode;
	ITRICarNormalLoopInProcess = false;
}

bool ITRICarFastLoopInProcess = false;
void ITRICarFastLoop(void)
{
	if (ITRICarFastLoopInProcess)
		return;
	ITRICarFastLoopInProcess = true;
	for (int i = 0; i < 2; i++)
		GetHallSensorState(&ITRICar.HubMotor[i]);
	ITRICarFastLoopInProcess = false;
}

void ITRICarMain(void)
{
	ITRICarInit();
	JsonWriteIntoJsonPair("ITRICar", "ON"); 
	fprintf(USART_FILE, "{\"CloseCSVFile\" = \"\"}"); 
}
