#include "Nu_STM32.h"
#include <string.h>

uint64_t nMilliSecondCnt = 0;
bool bProtectedSessionInProcess = true;

void STInit(int nFastLoopHz, int nNormalLoopHz)
{
	if (SysTick_Config(SystemCoreClock / 1000))
		while (1);
	NuEnableInputPP(A0); // Push Button
	NuEnableInputPP(C4); // VDD Power On
	NuEnableOutputPPA((GPortPinArray) {4, D12, D13, D14, D15}); // LED
	NuInit(nFastLoopHz,  nNormalLoopHz);
	JsonInit();
	JsonWriteInit();
	JsonWriteOpen();
}
