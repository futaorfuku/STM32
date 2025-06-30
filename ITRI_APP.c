#include "ITRI_APP.h"

#define sqr(x) ((x) * (x))
#define WHEEL_LEFT					0
#define WHEEL_RIGHT					1
#define MODE_MANUAL					0
#define MODE_AUTO					1
#define MODE_NUMBER_INDEX_ONE		1
#define MODE_NUMBER_INDEX_TWO		2
#define MODE_NUMBER_INDEX_THREE		3
#define CIRCLE_DIAMETER				84.82
#define MATH_PI 					3.14159265
#define ADVALUE_THROTTLE_RPM_SCALE	100
#define ADVALUE_THROTTLE_THRESHOLD	2600
#define ADVALUE_STEERING_THRESHOLD	100
#define RPM_RANGE					30
#define AXEL_LENGTH					46.0 // [cm]
#define MAXIMUM_RPM 				30
#define MINIMUM_RPM 				6
#define WHEEL_DIAMETER				27.0
#define SECOND_TO_MINUTE			60
#define RECORD_SIZE					400
#define ALLOWED_DISTANCE_ERROR      2 // [cm]
#define ALLOWED_ANGLE_ERROR         0.015 // [radians]
#define SAMPLING_FREQUENCY          5
#define MOTION_OSCILLATION          1
#define NUM_OSCILLATIONS            2
 
typedef struct
{
	float x;
	float y;
	float theta;
} MyPointPlane;

bool bMotionReverse = 0;
int dCurrentOscillation = 0;
MyPointPlane m_ptOutput = {0, 0, 0};
MyPointPlane m_ptDesired = {0, 0, 0};
// Goals
int TotalNumberofGoals = 34;
int CurrentGoalPoseIndex = -1;
int CurrentIMRPoseIndex = 0;

CITRICar ITRICar;
int car;
double dSpeedCmd;
float dRPM_Left = 0.0;
float dRPM_Right = 0.0;
/////////////////////////////////////////////////////////// Rohit
float fLeftWheelVelocity = 0;
float fRightWheelVelocity = 0;
float PreviouLVal = 0;
float PreviouRVal = 0;
float SamplingTime = 0;
bool TakeControl = false;
bool bPositionControl = false;
bool bOrientationControl = false;
bool GoalReached = false;
float dTime = 0.0;
PIDController VCLeftWheel = {30, 0.5, 3.0, 50};
PIDController VCRightWheel = {30, 0.5, 3.0, 50};

int PrintCounter = 0;
#define MAX_POINTS 	250

MyPointPlane m_ptPathData[MAX_POINTS]=
{
{10.00,160.00,0.00},
{11.88,160.00,0.00},
{17.50,160.00,0.00},
{25.00,160.00,0.00},
{32.50,160.00,0.00},
{40.00,160.00,0.00},
{47.50,160.00,0.00},
{55.00,160.00,0.00},
{62.50,160.00,0.00},
{70.00,160.00,0.00},
{77.50,160.00,0.00},
{85.00,160.00,0.00},
{92.50,160.00,0.00},
{100.00,160.00,0.00},
{107.50,160.00,0.00},
{115.00,160.00,0.00},
{122.50,160.00,0.00},
{130.00,160.00,0.00},
{137.50,160.00,0.00},
{145.00,160.00,0.00},
{152.50,160.00,0.00},
{160.00,160.00,0.00},
{167.50,160.00,0.00},
{175.00,160.00,0.00},
{182.50,160.00,0.00},
{190.00,160.00,0.00},
{197.50,160.00,0.00},
{205.00,160.00,0.00},
{212.50,160.00,0.00},
{220.00,160.00,0.00},
{227.50,160.00,0.00},
{235.00,160.00,0.00},
{242.50,160.00,0.00},
{250.00,160.00,0.00},
{257.50,160.00,0.00},
{265.00,160.00,0.00},
{272.50,160.00,0.00},
{280.00,160.00,0.00},
{287.50,160.00,0.00},
{295.00,160.00,0.00},
{302.50,160.00,0.00},
{310.00,160.00,0.00},
{317.50,160.00,0.00},
{325.00,160.00,0.00},
{332.50,160.00,0.00},
{340.00,160.00,0.00},
{347.50,160.00,0.00},
{355.00,160.00,0.00},
{362.50,160.00,0.00},
{370.00,160.00,0.00},
{377.50,160.00,0.00},
{385.00,160.00,0.00},
{392.50,160.00,0.00},
{400.00,160.00,0.00},
{407.50,160.00,0.00},
{415.00,160.00,0.00},
{422.50,160.00,0.00},
{430.00,160.00,0.00},
{437.50,160.00,0.00},
{445.00,160.00,0.00},
{452.50,160.00,0.00},
{460.00,160.00,0.00},
{467.50,160.00,0.00},
{475.00,160.00,0.00},
{482.50,160.00,0.00},
{490.00,160.00,0.00},
{497.50,160.00,0.00},
{504.79,160.00,0.00},
{509.17,160.00,0.00}
};


/////////////////////////////////////////////////////////// Rohit
void ChangeGoalTextDataIndex(int pIndex)
{
	CurrentGoalPoseIndex = pIndex;
	m_ptDesired = m_ptPathData[CurrentGoalPoseIndex];
}

MyPointPlane Difference(MyPointPlane pt1, MyPointPlane pt2)
{
	MyPointPlane pt = {pt1.x - pt2.x, pt1.y - pt2.y, pt1.theta - pt2.theta};
	return pt;
}

float Mod(MyPointPlane Vec)
{
	float ModValue = sqr(Vec.x) + sqr(Vec.y);
	return sqrt(ModValue);
}

float DotXY(MyPointPlane pt1, MyPointPlane pt2)
{
	return pt1.x * pt2.x +  pt1.y * pt2.y;
}

float CrossProductXY(MyPointPlane pt1, MyPointPlane pt2)
{
	return pt1.x * pt2.y -  pt1.y * pt2.x;
}

float DifferenceNorm2(MyPointPlane pt1, MyPointPlane pt2)
{
	MyPointPlane pt = Difference(pt1, pt2);
	return sqr(pt.x) + sqr(pt.y);
}

float FindAngleBetweenVectorsGivenPoints(MyPointPlane B, MyPointPlane D, bool* RightWheelMore)
{
	MyPointPlane Vec1 = Difference(B, m_ptOutput);
	MyPointPlane Vec2 = Difference(D, m_ptOutput);
	float dXY = DotXY(Vec1, Vec2);
	float angle_between_lines = acos(dXY/ (Mod(Vec1) * Mod(Vec2))); 
	float determinant = CrossProductXY(Vec1, Vec2);
	if (determinant == 0)
	{
		if (angle_between_lines == 0 ||	fabs(angle_between_lines - PI) <= 1e-3)
		{
			*RightWheelMore = true;
			return angle_between_lines == 0 ? 0 : angle_between_lines;
		}	
		return angle_between_lines;	
	}	
	*RightWheelMore = determinant > 0 == angle_between_lines < PI;
	return *RightWheelMore == determinant > 0 ? angle_between_lines : 2 * PI - angle_between_lines;
}

float Get_eTheta()
{
	float theta_m = atan2(m_ptDesired.y - m_ptOutput.y, m_ptDesired.x - m_ptOutput.x);
	return theta_m - m_ptOutput.theta;
}

bool GetIntersectionPointBetweenLines(MyPointPlane LineA, MyPointPlane LineB, MyPointPlane* ResultPoint) // Line Equation : Ax + By + C = 0
{
	float determinant = LineA.x * LineB.y - LineB.x * LineA.y;
	if (determinant == 0)
		return false;
	ResultPoint->x = (LineB.theta * LineA.y - LineA.theta * LineB.y) / determinant;
	ResultPoint->y = (LineB.x * LineA.theta - LineA.x * LineB.theta) / determinant;
	return true;
}

bool GetPerpendicularLineEquationGivenTwoPoints(MyPointPlane* FirstPoint, MyPointPlane* SecondPoint, MyPointPlane* ResultLine)
{
	if (fabs(SecondPoint->y - FirstPoint->y) < 1e-4)
	{
		ResultLine->x = 1; 
		ResultLine->y = 0; 
		ResultLine->theta = -FirstPoint->x; 
		return true;
	}
	else
	{
		float slope = (FirstPoint->x - SecondPoint->x) / (SecondPoint->y - FirstPoint->y) ;
		ResultLine->x = -slope; // A
		ResultLine->y = 1; // B
		ResultLine->theta = -1*(FirstPoint->y - slope * FirstPoint->x); // C
		return true;
	}
	return false;
}

bool GetLineEquationGivenTwoPoints(MyPointPlane* FirstPoint, MyPointPlane* SecondPoint, MyPointPlane* ResultLine)
{
	if (fabs(SecondPoint->x - FirstPoint->x) < 1e-4)
	{
		ResultLine->x = 1; 
		ResultLine->y = 0; 
		ResultLine->theta = -FirstPoint->x; 
		return true;
	}
	else
	{
		float slope = (SecondPoint->y - FirstPoint->y) / (SecondPoint->x - FirstPoint->x);
		ResultLine->x = -slope; // A
		ResultLine->y = 1; // B
		ResultLine->theta = -1*(FirstPoint->y - slope * FirstPoint->x); // C
		return true;
	}
	return false;
}

MyPointPlane MoveForward(MyPointPlane pt, float dDistance)
{
	return (MyPointPlane) {
		pt.x + dDistance * cos(pt.theta),
		pt.y + dDistance * sin(pt.theta), 
		0};
}

float DetermineOrientationNeeded()
{
	bool RightWheelMore = false;
	float DegreeAngle = FindAngleBetweenVectorsGivenPoints(MoveForward(m_ptOutput, 10), 
		m_ptDesired, &RightWheelMore);
	if (!RightWheelMore)
		DegreeAngle = -1 * DegreeAngle;
	return DegreeAngle;
}

void NormalizeRPM()
{
	float CalculatedRPM_Left = dRPM_Left;
	float CalculatedRPM_Right = dRPM_Right;
	float Ratio = CalculatedRPM_Right / CalculatedRPM_Left;	
	if (CalculatedRPM_Left != 0)
		dRPM_Left = CalculatedRPM_Left / fabs(CalculatedRPM_Left) * _fmax(MINIMUM_RPM, _fmin(fabs(CalculatedRPM_Left), MAXIMUM_RPM));
	if (CalculatedRPM_Right != 0)
		dRPM_Right = CalculatedRPM_Right / fabs(CalculatedRPM_Right) * _fmax(MINIMUM_RPM, _fmin(fabs(CalculatedRPM_Right), MAXIMUM_RPM));
	if (_fmax(_fabs(CalculatedRPM_Right), _fabs(CalculatedRPM_Left)) > MAXIMUM_RPM)
	{
		if (_fabs(CalculatedRPM_Right) > _fabs(CalculatedRPM_Left))
		{
			dRPM_Right = CalculatedRPM_Right < 0 ? -MAXIMUM_RPM : MAXIMUM_RPM;
			dRPM_Left = dRPM_Right / Ratio;
		}
		if (_fabs(CalculatedRPM_Right) < _fabs(CalculatedRPM_Left))
		{
			dRPM_Left = CalculatedRPM_Left < 0 ? -MAXIMUM_RPM : MAXIMUM_RPM;
			dRPM_Right = dRPM_Left * Ratio;
		}
	}	
}

float VelocityToRPM(float pVelocity)
{
	return (pVelocity) * SECOND_TO_MINUTE / (WHEEL_DIAMETER * MATH_PI);
}

float RPMToVelocity(float pRPM)
{
	return (pRPM) * (WHEEL_DIAMETER * MATH_PI) / SECOND_TO_MINUTE;
}

int GetCurrentGoalPoseIndex()
{
	int PositionIndex = 0;
	float MinDistance = 100000000;
	float fDistance = 0;
	if (!bMotionReverse)
	{
		for (int i = _fmax(0, CurrentIMRPoseIndex - 1); i < _fmin(TotalNumberofGoals, CurrentIMRPoseIndex + 5); i++)
		{
			fDistance = sqrt(DifferenceNorm2(m_ptOutput, m_ptPathData[i]));
			if (MinDistance == 100000000 || fDistance <= MinDistance)
			{
				MinDistance = fDistance;
				PositionIndex = i;
			}
		}
	}
	else
	{
		for (int i = _fmin(TotalNumberofGoals - 1, CurrentIMRPoseIndex + 1); i > _fmax(-1, CurrentIMRPoseIndex - 5); i--)
		{
			fDistance = sqrt(DifferenceNorm2(m_ptOutput, m_ptPathData[i]));
			if (MinDistance == 100000000 || fDistance <= MinDistance)
			{
				MinDistance = fDistance;
				PositionIndex = i;
			}
		}
	}	
	return PositionIndex;
}
void GetGoalPoseIndex()
{
	CurrentIMRPoseIndex = GetCurrentGoalPoseIndex();
}

void GetCurrentPoseIndexAndSetGoalIndex()
{
	GetGoalPoseIndex();
	if (!bMotionReverse)
		ChangeGoalTextDataIndex(_fmin(CurrentIMRPoseIndex + 2, TotalNumberofGoals-1));
	else
		ChangeGoalTextDataIndex(_fmax(CurrentIMRPoseIndex - 2, 0));
}

void transposeMatrix3(const float A[][2], float result[][3], int n, int m) 
{
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			result[j][i] = A[i][j];
		}
	}
}
void multiplyMatrices32(const float A[][3], const float B[][2], float result[][2], int n, int m, int k) 
{
	for (int i = 0; i < n; ++i) 
	{
		for (int j = 0; j < m; ++j) 
		{
			result[i][j] = 0.0;
			for (int l = 0; l < k; ++l) 
			{
				result[i][j] += A[i][l] * B[l][j];
			}
		}
	}
}
void multiplyMatrixVector31(const float A[][3], const float x[], float result[], int n, int m) 
{
	for (int i = 0; i < n; ++i) 
	{
		result[i] = 0.0;
		for (int j = 0; j < m; ++j) 
		{
			result[i] += A[i][j] * x[j];
		}
	}
}
void multiplyMatrixVector21(const float A[][2], const float x[], float result[], int n, int m) 
{
	for (int i = 0; i < n; ++i) 
	{
		result[i] = 0.0;
		for (int j = 0; j < m; ++j) 
		{
			result[i] += A[i][j] * x[j];
		}
	}
}
bool inverse2x2(const float A[][2], float invA[][2]) 
{
	float a = A[0][0];
	float b = A[0][1];
	float c = A[1][0];
	float d = A[1][1];

	float det = a * d - b * c;
	if (det == 0) {
		return false;
	}

	float invDet = 1.0 / det;
	invA[0][0] = d * invDet;
	invA[0][1] = -b * invDet;
	invA[1][0] = -c * invDet;
	invA[1][1] = a * invDet;
	return true;
}

float m_DeltaA[3][2] = {0};
float m_Deltab[3] = {0};
void LeastSquareSolution()
{
	float a = cos(m_ptOutput.theta) * SamplingTime / 2;
	float b = sin(m_ptOutput.theta) * SamplingTime / 2;
	float c = SamplingTime / AXEL_LENGTH;
	m_DeltaA[0][0] = a;
	m_DeltaA[0][1] = a;
	m_DeltaA[1][0] = b;
	m_DeltaA[1][1] = b;
	m_DeltaA[2][0] = -c;
	m_DeltaA[2][1] = c;

	float KP_Position = 0.4;
	float KP_Angle = 0.4;
	m_Deltab[0] = (m_ptPathData[CurrentGoalPoseIndex].x - m_ptOutput.x)* KP_Position;
	m_Deltab[1] = (m_ptPathData[CurrentGoalPoseIndex].y - m_ptOutput.y)* KP_Position;
	m_Deltab[2] = (m_ptPathData[CurrentGoalPoseIndex].theta - m_ptOutput.theta)* KP_Angle;
	// Perform Least squares
	float At[2][3];
	transposeMatrix3(m_DeltaA, At, 3, 2);		
	float AtA[2][2];
	multiplyMatrices32(At, m_DeltaA, AtA, 2, 2, 3);		
	float Atb[2];
	multiplyMatrixVector31(At, m_Deltab, Atb, 2, 3);
	float invAtA[2][2];
	if (inverse2x2(AtA, invAtA))
	{
		float x[2];
		multiplyMatrixVector21(invAtA, Atb, x, 2, 2);		
		dRPM_Left = VelocityToRPM(x[0]);
		dRPM_Right = VelocityToRPM(x[1]);
		return;
	}
	dRPM_Left = 0;
	dRPM_Right = 0;		
}

bool CheckGoalReached()
{
	if (!bMotionReverse)
	{
		if (CurrentGoalPoseIndex == TotalNumberofGoals - 1)
		{
			GoalReached = true;	
			return true;
		}	
		return false;
	}
	if (CurrentGoalPoseIndex == 0)
	{
		GoalReached = true;	
		return true;
	}	
	return false;
}

void MBITPositionControl()
{
	if (!bPositionControl)
		return;
	if (CheckGoalReached())
	{
		bPositionControl = false;
		bOrientationControl = false;
		dRPM_Left = 0;
		dRPM_Right = 0;
		return;
	}		
	LeastSquareSolution();
	NormalizeRPM();
}

void InitMotionParameters(bool Reverse)
{
	if (!Reverse)
	{
		CurrentIMRPoseIndex = 0;
		m_ptOutput = m_ptPathData[0];
		ChangeGoalTextDataIndex(CurrentIMRPoseIndex+1);
	}
	else
	{
		CurrentIMRPoseIndex = TotalNumberofGoals - 1;
		m_ptOutput = m_ptPathData[CurrentIMRPoseIndex];
		ChangeGoalTextDataIndex(CurrentIMRPoseIndex-1);
	}
	bPositionControl = true;
	GoalReached = false;
	dCurrentOscillation += 1;
}

void UpdateIMRPose()
{
	SamplingTime = (float)1/SAMPLING_FREQUENCY;
	fLeftWheelVelocity =  (((double)ITRICar.m_wheel[WHEEL_LEFT].m_drive.m_nStepCount[CURRENT_STATE] - 
	PreviouLVal) * SAMPLING_FREQUENCY * MATH_PI * (WHEEL_DIAMETER / 2) / 45);				
	fRightWheelVelocity = -1 * ((double)ITRICar.m_wheel[WHEEL_RIGHT].m_drive.m_nStepCount[CURRENT_STATE] - 
	PreviouRVal) * SAMPLING_FREQUENCY * MATH_PI * (WHEEL_DIAMETER / 2) / 45;			
	m_ptOutput.x += (fLeftWheelVelocity + fRightWheelVelocity) / 2 * cos(m_ptOutput.theta) * SamplingTime;				
	m_ptOutput.y += (fLeftWheelVelocity + fRightWheelVelocity) / 2 * sin(m_ptOutput.theta) * SamplingTime;				
	m_ptOutput.theta += (fRightWheelVelocity - fLeftWheelVelocity) / AXEL_LENGTH * SamplingTime ;	
	if (m_ptOutput.theta >= 2 * MATH_PI)
		m_ptOutput.theta = fabs(m_ptOutput.theta - 2 * MATH_PI);
	else if (m_ptOutput.theta <= -2 * MATH_PI)
		m_ptOutput.theta = fabs(m_ptOutput.theta + 2 * MATH_PI);
	PreviouLVal = ITRICar.m_wheel[WHEEL_LEFT].m_drive.m_nStepCount[CURRENT_STATE];
	PreviouRVal = ITRICar.m_wheel[WHEEL_RIGHT].m_drive.m_nStepCount[CURRENT_STATE];
	dTime += SamplingTime;
}


////////////////////////////////////////////////
void ResetWheel(HubMotorWheel* pWheel)
{
	pWheel->m_dTotalMileage = 0;
	pWheel->m_dMeterPerStep = 1. / (3 * POLE_NUMBER) * CIRCLE_DIAMETER;
	pWheel->m_motor.nPoleNumber = POLE_NUMBER;
    pWheel->m_drive.m_dRPMLowpass = 0;
    pWheel->m_drive.m_dRPMInstantaneous = 0;
}

void WheelControl(HubMotorWheel* pWheel, double dSpeedLinear, double dSpeedRotate)
{
	if (car == 0)
		dSpeedCmd = dRPM_Left;
	else
		dSpeedCmd = -dRPM_Right;
	dSpeedCmd = _fmin(MAXIMUM_RPM, _fmax(-MAXIMUM_RPM, dSpeedCmd));
	GetLowpassDirectionalSpeed(&pWheel->m_drive);
	MotorSetDirectionAndDuty(&pWheel->m_drive, 
		PIDFeedback(&pWheel->m_drive.m_ctrl, dSpeedCmd, pWheel->m_drive.m_dRPMLowpass));
}

/////////////////////////////
void InitPIDControllers()
{
	for (int i = 0; i < HUB_MOTORS; i++)
		InitPIDController(&ITRICar.m_wheel[i].m_drive.m_ctrl);
}

void ResetMotorOutput()
{
	for (int i = 0; i < 2; i++)
		ResetOutput(&ITRICar.m_wheel[i].m_drive);
}

void MotorSpeedControl(uint64_t nFastLoopCount)
{
	for (int i = 1; i >= 0; i--)
	{
		car = i;
		WheelControl(&ITRICar.m_wheel[i],
			ITRICar.m_dSpeedThrottle * (1 - _fabs(ITRICar.m_dRotateRatio)), ITRICar.m_dSpeedThrottle * ITRICar.m_dRotateRatio);
		/*if (nFastLoopCount % 5000 == 0 &&
			ITRICar.m_wheel[i].m_drive.m_dRPMLowpass < 1 && 
			ITRICar.m_wheel[i].m_drive.m_bCommandForward)
		{
			ITRICar.m_wheel[i].m_drive.m_bCommandForward = false;
			StepChangeDetected(&ITRICar.m_wheel[i].m_drive);
			CBLDCMotorDrive *pDrive = &ITRICar.m_wheel[i].m_drive;
			BLDC6StepControl(pDrive->m_duty.m_TIM, 
			(pDrive->m_nBLDCStep[CURRENT_STATE] + 1 > 6) ? 1 : pDrive->m_nBLDCStep[CURRENT_STATE] + 1, 
			pDrive->m_nPWMMode,
			pDrive->m_nPWMTimerPeriod, pDrive->m_bCommandForward);
		}*/
	}
}

void ReadDIOControl()
{
	ITRICar.m_nWorkMode[CURRENT_STATE] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
	ITRICar.m_nWorkMode[CURRENT_STATE] = MODE_AUTO;
	if (ITRICar.m_nWorkMode[PREVIOUS_STATE] != ITRICar.m_nWorkMode[CURRENT_STATE])
		InitPIDControllers();
}

void ControlPinInit(GPortPinArray ppa)
{
	ITRICar.m_ppaControl = ppa;
	NuEnableInputPPA(ppa); 
	ReadDIOControl();
}

void ReadHandleThreeMode(void)
{
	for (int i = 0; i < ITRICar.m_ppaControl.m_nPins; i++)
		ITRICar.nReadModeNumberPin[i] = NuGetBit(ITRICar.m_ppaControl.m_gpp[i]);
}

////////////////////////////////////////
int AD_VALUE_SIZE	= ADC_CHANNELS * 10;

void CarPWMDisablePinInit(GPortPin gpp)
{
	PortPin pp = PP(gpp);
	ITRICar.m_pwmDisable = gpp;
	NuEnableOutputPP(gpp);
	GPIO_SetBits(pp.m_typedef, pp.m_GPIO);
}



void ITRICarInit(bool bUseSteering, DWORD nPWMMode, int nPWMHz)
{
	mgrPWM.m_nHzSignal = nPWMHz;
	mgrPWM.m_nModePWM = nPWMMode;
	mgrPWM.m_nCallFunction = PWM_CALL_HALF_BLDC;
	ITRICar.m_bUseSteering = bUseSteering;	
	ITRICar.m_dSpeedThrottle = 0;
	ITRICar.m_dRotateRatio = 0;
	ITRICar.m_nWorkMode[CURRENT_STATE] = ITRICar.m_nWorkMode[PREVIOUS_STATE] = MODE_MANUAL;
	for (int i = 0; i < ADC_CHANNELS; i++)
	{
		ITRICar.m_dLPFValue[i] = 0;
		ResetWheel(&ITRICar.m_wheel[i]);
	}
	ITRICar.dADValueToSpeedCmd = (double) RPM_RANGE / ADVALUE_THROTTLE_RPM_SCALE;
	ITRICar.m_wheel[WHEEL_LEFT].m_drive.m_bInitStart = false;
	ITRICar.m_wheel[WHEEL_RIGHT].m_drive.m_bInitStart = false;
	CarPWMDisablePinInit(B2);
	CarPWMDisablePinInit(C0);
	GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_0, Bit_RESET);
	BLDCMotorInit(&ITRICar.m_wheel[WHEEL_LEFT].m_drive, 1,
		true, nPWMMode,
		(GPortPinArray) {3, E8, E10, E12},  
		B3, 
		(PWMModule) {ST_TIMER_1, nPWMHz, (GPortPinArray) {7, A8, E11, A10, B12, A7, B14, B15}}, none,
		VCLeftWheel);		
	BLDCMotorInit(&ITRICar.m_wheel[WHEEL_RIGHT].m_drive, 2,
		true, nPWMMode,			
		(GPortPinArray) {3, E7, E9, E15},
		B4, 
		(PWMModule) {ST_TIMER_8, nPWMHz, (GPortPinArray) {7, C6, C7, C8, A6, A5, B0, B1}}, none,
		VCRightWheel);
	ITRICar.m_nPIDFastLoopSkip = FASTLOOP_HZ / ITRICar.m_wheel[WHEEL_LEFT].m_drive.m_ctrl.m_nHzPID;
	InitMotionParameters(bMotionReverse);	
}

void ReadAIOControl()
{
			
}

bool MotorBrake(bool bStop)
{
	if (bStop)
		ResetMotorOutput();
	return bStop;
}

///////////////////////////////////////////////
void ITRICarMain(void)
{
	ITRICarInit(false /*use steering*/, MODE_BISYNC, 20000 /*Hz*/); 
	ResetMotorOutput();
	JsonWriteIntoJsonPair("ITRICar", "ON"); 
}

#define RECORD_ARRAY_SIZE 80000
uint16_t RecordArrayCount = 1;
int PrintfCount = 0;

void ITRICarReportWrite(bool bReadWrite)
{
	char szText[MAX_JSON_VALUE];		
	NuTimeReport(szText);
	fprintf(USART_FILE,"{\"SetBeepValues\"=\"%s,%f,%f,%f,%f,%f,%f,%f,%f\"}",
		szText,
		fLeftWheelVelocity,
		fRightWheelVelocity,
		m_ptOutput.x,
		m_ptOutput.y,
		dTime,
		m_ptDesired.x,
		m_ptDesired.y,
		m_ptDesired.theta);	
}

void ITRICarReport(void)
{
	if (!m_bSystemReportInited)
	{
		fprintf(USART_FILE, "{\"OpenCSVFile\" = \"D:/test123.csv\"}"); 
		fprintf(USART_FILE, "{\"SetKeys\" = \"Time,WheelLeft,WheelRight,PositionX,PositionY,PositionTheta,GoalX,GoalY,GoalTheta\"}"); 
		m_bSystemReportInited = true;
	}
	ITRICarReportWrite(true);
	if (RecordArrayCount == RECORD_ARRAY_SIZE || GoalReached)
		ITRICarReportWrite(true);
	if (PrintfCount == RECORD_ARRAY_SIZE)
		ITRICarReportWrite(false);	
}

void MotorCalculateMileage(HubMotorWheel* pWheel)
{
	pWheel->m_dTotalMileage = pWheel->m_dMeterPerStep * pWheel->m_drive.m_nStepCount[CURRENT_STATE];
}

void ITRICarNormalLoop(void)
{ 
	ReadDIOControl();
	if (!MotorBrake(ITRICar.m_nWorkMode[CURRENT_STATE] == MODE_MANUAL))
		ReadAIOControl();
	for (int i = 0; i < 2; i++)
		MotorCalculateMileage(&ITRICar.m_wheel[i]);
	ITRICar.m_nWorkMode[PREVIOUS_STATE] = ITRICar.m_nWorkMode[CURRENT_STATE];
}

void MotionBuiltInTest()
{
	PrintCounter++;
	UpdateIMRPose();
	dRPM_Left = 60;
	dRPM_Right = -60;
	return;
	if (TakeControl)		
	{
		GetCurrentPoseIndexAndSetGoalIndex();
		MBITPositionControl();	
		if (MOTION_OSCILLATION)
		{
			if (GoalReached && (dCurrentOscillation / 2 < NUM_OSCILLATIONS))
			{
				bMotionReverse = bMotionReverse == 1 ? 0 : 1;
				InitMotionParameters(bMotionReverse);
			}				
		}		
	}
	else
	{
		dRPM_Left = 10;
		dRPM_Right = -10;
	}	
	//dRPM_Left = 5;
	//dRPM_Right = 6;
	if (dTime > 0.5)
	{
		//TakeControl = true;	
		//InitMotionParameters(bMotionReverse);
	}		
		
}

void MotionBuiltInTestNextPosition()
{
	if (RecordArrayCount > RECORD_ARRAY_SIZE)
	{
		dRPM_Left = 0;
		dRPM_Right = 0;
	}
	RecordArrayCount++;
}

void ITRICarFastLoop(uint64_t nFastLoopCount)
{
	for (int i = 1; i >= 0; i--)
	{
		GetHallSensorState(&ITRICar.m_wheel[i].m_drive);
		if (ITRICar.m_wheel[i].m_drive.m_nStepChangeCountDown >= 0)
		{
			if (ITRICar.m_wheel[i].m_drive.m_nStepChangeCountDown == 0)
				DoChangeStepControl(&ITRICar.m_wheel[i].m_drive);
			ITRICar.m_wheel[i].m_drive.m_nStepChangeCountDown--;
		}
	}
	if (ITRICar.m_nWorkMode[CURRENT_STATE] == MODE_AUTO &&
		nFastLoopCount % ITRICar.m_nPIDFastLoopSkip == 0)
		MotorSpeedControl(nFastLoopCount); 
//	if (nFastLoopCount % 100 == 0 &&
//		RecordArrayCount < RECORD_ARRAY_SIZE)
//		MotionBuiltInTestNextPosition();
//	if (nFastLoopCount % 2000 == 0 &&
//		RecordArrayCount < RECORD_ARRAY_SIZE) 
//		MotionBuiltInTest();
}

void JITRIApp(DWORD param[])
{
	/*char szText[MAX_JSON_VALUE];
	JsonPartition(param, szText);
	m_ptDesired.x = Bound(1000, 0, Getdouble(szText, ","));
	m_ptDesired.y = Bound(1000, 0, Getdouble(szText, ","));
	m_ptDesired.theta = Bound(1000, 0, Getdouble(szText, "]"));
	m_bSystemReportInited = false;*/
}
