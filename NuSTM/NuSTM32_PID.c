#include "NuSTM32_PID.h"
#include "NuSTM32_C++.h"
void InitPIDController(PIDController* pPID)
{
	pPID->m_dErrorIntegration = 0;
	pPID->m_dError = 0;
	pPID->m_dOutput = 0;
}

void SetPIDParameter(PIDController* pPID, PIDController pid)
{
	*pPID = pid;
	InitPIDController(pPID);
}

double PIDFeedback(PIDController *pid, double dCommand, double dFeedback)
{
	return PIDFeedbackForm2(pid, dCommand - dFeedback, dCommand - dFeedback - pid->m_dError);
}

double PIDFeedbackForm2(PIDController *pid, double dError, double dErrorDot)
{
	pid->m_dOutput = pid->m_dKP * dError + pid->m_dKI * pid->m_dErrorIntegration + 
			pid->m_dKD * dErrorDot;
	pid->m_dErrorIntegration = _fmax(-1000000., _fmin(1000000., pid->m_dErrorIntegration + dError));
	pid->m_dError = dError;	
	return pid->m_dOutput;
}
