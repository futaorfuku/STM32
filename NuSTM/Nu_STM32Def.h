#ifndef __NU_STM32DEF_H
#define __NU_STM32DEF_H

#ifdef CODE_F103
#include "stm32f10x.h"
#else
#include "stm32f4xx.h"
#endif

#define sign(x) ((x) > 0? 1: -1)
#define PI 3.14159265359
#define PI2 6.2831853
#define true 		1
#define false 	0
#define bool 		uint8_t
#define NULL 		0
#define DWORD 	uint32_t
#define FIFOSIZE	5600

#define ST_REGST 	0x10
#define ST_TIMER_1 	0x11
#define ST_TIMER_2 	0x12
#define ST_TIMER_3 	0x13
#define ST_TIMER_4 	0x14
#define ST_TIMER_5 	0x15
#define ST_TIMER_6 	0x16
#define ST_TIMER_7 	0x17
#define ST_TIMER_8 	0x18
#define ST_TIMER_9 	0x19
#define ST_TIMER_10 0x1A
#define ST_TIMER_11 0x1B
#define ST_TIMER_12 0x1C
#define ST_TIMER_13 0x1D
#define ST_TIMER_14 0x1E
#define ST_DMA_1		0x21
#define ST_DMA_2		0x22

#define PWM_CALL_HALF_BRIDGE	1
#define PWM_CALL_HALF_BLDC		2

#define TIM1_DOWN_CLOCKING	2
#define TIM1_CLOCK 	(SystemCoreClock /	TIM1_DOWN_CLOCKING)

#define MAX_JSON_STRING	2400
#define MAX_JSON_KEY		30
#define MAX_JSON_VALUE	420

#define NORMALLOOP_HZ		100
#define FASTLOOP_HZ			(10 _kHz)
#endif

