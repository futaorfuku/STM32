#ifndef __ITRI_APP_H
#define __ITRI_APP_H
#include "main.h"
#include <math.h>

#define HUB_MOTORS		2
#define ADC_CHANNELS	3
#define ADC_UVW_BEMF	0
#define ADC_THROTTLE	1
#define ADC_STEERING	2
typedef struct
{
	int nPoleNumber;
} CHubMotor;


typedef struct
{
	CBLDCMotorDrive m_drive;
	CHubMotor m_motor;
	double m_dTotalMileage;
	double m_dMeterPerStep;
	double m_dPIDValue;
} HubMotorWheel;

typedef struct
{
	uint8_t m_nWorkMode[MAX_STATES];
	uint8_t nModeNumber;
	uint8_t nReadModeNumberPin[2];
	bool m_bUseSteering;
	uint32_t m_nPIDFastLoopSkip;
	double m_dLPFValue[ADC_CHANNELS];
	double m_dLPFValueMean[ADC_CHANNELS];
	double dADValueToSpeedCmd;
	double m_dSpeedThrottle;
	double m_dRotateRatio;
	HubMotorWheel m_wheel[HUB_MOTORS];
	GPortPin m_pwmDisable;
	GPortPinArray m_ppaControl;
} CITRICar;

void ITRICarNormalLoop(void);
void ITRICarReport(void);
void ITRICarFastLoop(uint64_t nFastLoopCount);
void ITRICarMain(void);
void JITRIApp(DWORD param[]);
#endif /* __ITRI_APP_H */

