#ifndef __NU_BLDC_H
#define __NU_BLDC_H
#include "Nu_STM32.h"
#include "NuSTM32_PID.h"

#define POLE_NUMBER			30

#define MODE_EXTERN_DRIVER 	0
#define MODE_BIASYNC 		1
#define MODE_BISYNC 		2
#define MODE_UNISYNC  		3
#define MODE_COSYNC  		4

#define BRIDGE_NO	0
#define BRIDGE_UP	0x1
#define BRIDGE_DN	0x2

#define MAX_HIST_RECORD	3
#define K_0	2
#define K_1	1
#define K_2	0

void NuBLDCSetDutyPulse(TIM_TypeDef* TIMx, double dDutyH, double dDutyL, uint16_t nPeriodPulse, int nBLDCStep);
void BLDC6StepControl(TIM_TypeDef* TIMx, int nBLDCStep, DWORD nMode, uint32_t nDutyPulse, bool bForward);
void BLDC6StepTIMConfig(void);

//////////////////////////////////
#define MAX_STATES			2
#define CURRENT_STATE		1
#define PREVIOUS_STATE		0	
#define HALL_SENSOR_BITS	3
#define HALL_SENSOR_A		2
#define HALL_SENSOR_B		1
#define HALL_SENSOR_C		0


typedef struct{
	TIM_TypeDef* m_TIM;	
	uint16_t m_nCompare;
} MotorDuty;

typedef struct{
	uint32_t m_nPortTimer;
	int  m_nPWMHz;
	GPortPinArray m_ppaPWM;
} PWMModule;

typedef struct 
{
	uint8_t m_nBLDCStep[MAX_STATES];	
	int m_nStepCount[MAX_STATES];
	int m_nMotorForwardDirection;
	int m_nStepChangeCountDown;
	bool bRotateForward;
	bool m_bCommandForward;
	uint32_t m_nMillisecondCntPreviousInstantaneous;
	uint32_t m_nMillisecondCntPreviousLowpass;
	uint32_t m_nPWMTimerPeriod;
	DWORD m_nPWMMode;
	GPortPinArray m_hall;
	GPortPin m_direction;
	PWMModule m_pwmModule;
	MotorDuty m_duty;
	PIDController m_ctrl;
	GPortPin m_gppEnable;
	bool m_bInitStart;
	double m_dRPMLowpass;
	double m_dRPMInstantaneous;
} CBLDCMotorDrive;

void GetHallSensorState(CBLDCMotorDrive* pDrive);
void BLDCMotorInit(CBLDCMotorDrive* pDrive, int nIndex, bool bMotorReverse, 
	DWORD nPWMMode, GPortPinArray hsensor, GPortPin direction, PWMModule pwmModule, GPortPin gppEnable, 
	PIDController pidControl);
void ResetOutput(CBLDCMotorDrive* pDrive);
void MotorSetDirectionAndDuty(CBLDCMotorDrive* pDrive, double du);
void DoChangeStepControl(CBLDCMotorDrive* pDrive);
double GetLowpassDirectionalSpeed(CBLDCMotorDrive* pDrive);
double GetInstantaneousDirectionalSpeed(CBLDCMotorDrive* pDrive);
bool StepChangeDetected(CBLDCMotorDrive* pDrive);
#endif

