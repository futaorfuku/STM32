#ifdef _USE_CAN_
#ifndef __NUSTM32_CAN_H
#define __NUSTM32_CAN_H
#include "NuSTM32_BASIC.h"
#define ST_CAN_1		0x50
#define ST_CAN_2		0x51
#define ST_CAN_1TX	0x52
#define ST_CAN_1RX0	0x53
#define ST_CAN_1RX1	0x54
#define ST_CAN_1SCE 0x55
#define ST_CAN_2TX	0x56
#define ST_CAN_2RX0	0x57
#define ST_CAN_2RX1	0x58
#define ST_CAN_2SCE 0x59

#define CAN_NODE_A							(uint16_t)0x101
#define CAN_NODE_B							(uint16_t)0x102
#define CAN_NODE_C							(uint16_t)0x103

typedef struct{
	uint16_t m_nNodeID;
	uint16_t m_nPort;
} _NuCANNode;

void CANConfig(uint16_t nPort);
void CANPolling(uint8_t nCANPort);
bool CANLoopbackBIT(uint8_t nCANPort);
void NuEnableCAN_NVIC(uint16_t nPort, uint32_t PriorityGroup, uint8_t uSubPriority);
bool CANDataSend(uint8_t nCANPort);
void NuOPModeEnableCAN(uint16_t nCANPort, uint32_t nNodeID);
void CAN1Send(uint16_t nRxNodeID, char* pnData);
void CAN2Send(uint16_t nRxNodeID, char* pnData);
void JCANTx(DWORD param[]);
void JSetCAN(DWORD param[]);
#endif

#endif

