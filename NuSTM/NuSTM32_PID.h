#ifndef __NUSTM32_PID_H
#define __NUSTM32_PID_H

typedef struct 
{
	double m_dKP;
	double m_dKI;
	double m_dKD;
	int m_nHzPID;
	double m_dErrorIntegration;
	double m_dError;
	double m_dOutput;
} PIDController;

void InitPIDController(PIDController* pid);
void SetPIDParameter(PIDController* pPID, PIDController pid);
double PIDFeedback(PIDController *pid, double dCommand, double dFeedback);
double PIDFeedbackForm2(PIDController *pid, double dError, double dErrorDot);
#endif


