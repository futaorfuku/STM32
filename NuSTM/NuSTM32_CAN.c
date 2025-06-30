#ifdef _USE_CAN_
#include "NuSTM32_CAN.h"
#include "NuSTM32_JSON.h"
#include "string.h"
#define MAX_CAN_CHANNELS	2
#define CAN1_SN	0
#define CAN2_SN	1

_NuCANNode NuCANNode[MAX_CAN_CHANNELS];
CanRxMsg m_canRxMsg[MAX_CAN_CHANNELS];

CAN_TypeDef* GetCANTypeDef(uint8_t nCANPort)
{
  CAN_TypeDef* CANxs[] = {CAN1, CAN2};
	return CANxs[nCANPort - ST_CAN_1];
}	

void CANSendMessage(uint16_t nCANPort, uint8_t* pnMessage, uint8_t nLength, uint16_t nRxNodeID)
{
	CAN_TypeDef* CANx = GetCANTypeDef(nCANPort);
	CanTxMsg canTxMsg;
	canTxMsg.StdId = nRxNodeID;
	canTxMsg.RTR = CAN_RTR_DATA;
	canTxMsg.IDE = CAN_ID_STD;
	canTxMsg.DLC = nLength;
	memcpy(canTxMsg.Data, pnMessage, nLength);
	uint8_t nTransmitMailbox = CAN_Transmit(CANx, &canTxMsg);
	while (CAN_TransmitStatus(CANx, nTransmitMailbox) == CAN_TxStatus_Failed);
}

void CAN1Send(uint16_t nRxNodeID, char* pnData)
{
	if (NuCANNode[CAN1_SN].m_nNodeID == nRxNodeID)
		printf("{Loopback NodeID Sent}");
	else
		CANSendMessage(ST_CAN_1, (uint8_t*) pnData, strlen(pnData), nRxNodeID);
}

void CAN2Send(uint16_t nRxNodeID, char* pnData)
{
	if (NuCANNode[CAN1_SN].m_nNodeID == nRxNodeID)
		printf("{Loopback NodeID Sent}");
	else
		CANSendMessage(ST_CAN_2, (uint8_t*) pnData, strlen(pnData), nRxNodeID);
}
	
void CANRxMessageInit(CanRxMsg *RxMessage)
{
  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  memset(RxMessage->Data, 0, sizeof(uint8_t) * 8);
}

void CANRxIRQHandler(uint8_t nCANsn)
{
	CAN_TypeDef* CANx = GetCANTypeDef(NuCANNode[nCANsn].m_nPort);
	CANRxMessageInit(&m_canRxMsg[nCANsn]);
	CAN_Receive(CANx, CAN_FIFO0, &m_canRxMsg[nCANsn]);
	printf("{CAN%d = %s}", nCANsn, m_canRxMsg[nCANsn].Data);	
}

bool IsCANTransmitted(uint8_t nCANPort)
{
	CAN_TypeDef* CANx = GetCANTypeDef(nCANPort);
	return
		(CAN_GetFlagStatus(CANx, CAN_FLAG_RQCP0) == RESET) &&
		(CAN_GetFlagStatus(CANx, CAN_FLAG_RQCP1) == RESET) &&
		(CAN_GetFlagStatus(CANx, CAN_FLAG_RQCP2) == RESET);
}

void NuEnableCANGPIO(uint8_t nCANPort)
{ // http://wiki.csie.ncku.edu.tw/embedded/CAN
	switch (nCANPort)
	{
		case ST_CAN_1:
			NuEnableGPIOAFPPA((GPortPinArray) {2, D0, D1}, GPIO_AF_CAN1, GPIO_Fast_Speed, GPIO_PuPd_UP);
			break;
		case ST_CAN_2:
			NuEnableGPIOAFPPA((GPortPinArray) {2, B12, B13}, GPIO_AF_CAN2, GPIO_Fast_Speed, GPIO_PuPd_UP);
			break;
	}
}

void CANClock(uint8_t nCANPort)
{
	switch (nCANPort)
	{
		case ST_CAN_1:
			CAN_DeInit(CAN1);
			NuEnableRCCClock(ST_CAN_1);
			break;
		case ST_CAN_2:
			CAN_DeInit(CAN2);
			NuEnableRCCClock(ST_CAN_2);
			break;
	}
}

void CANInitConfig(uint8_t nCANPort, uint8_t can_Mode, uint16_t can_Prescaler)
{
	CAN_TypeDef* CANx = GetCANTypeDef(nCANPort);
	CAN_InitTypeDef caninitStructure;
	caninitStructure.CAN_TTCM = DISABLE;
  caninitStructure.CAN_ABOM = DISABLE;
  caninitStructure.CAN_AWUM = DISABLE;
  caninitStructure.CAN_NART = DISABLE;
  caninitStructure.CAN_RFLM = DISABLE;
  caninitStructure.CAN_TXFP = DISABLE;
  caninitStructure.CAN_Mode = can_Mode;
	caninitStructure.CAN_SJW = CAN_SJW_1tq;
	caninitStructure.CAN_BS1 = CAN_BS1_3tq;
	caninitStructure.CAN_BS2 = CAN_BS2_2tq;	
  caninitStructure.CAN_Prescaler = can_Prescaler;
  CAN_Init(CANx, &caninitStructure);
}

void CANFFilterInitConfig(uint8_t nCANPort, uint8_t can_FilterMode)
{
	CAN_FilterInitTypeDef canFilterInitStructure;
	canFilterInitStructure.CAN_FilterNumber = nCANPort == ST_CAN_1? 0: 14; 
  canFilterInitStructure.CAN_FilterMode = can_FilterMode;
  canFilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	uint16_t nNodeID = (uint16_t) NuCANNode[nCANPort - ST_CAN_1].m_nNodeID;
  canFilterInitStructure.CAN_FilterIdHigh = nNodeID << 5;
  canFilterInitStructure.CAN_FilterIdLow = 0x0000;
  canFilterInitStructure.CAN_FilterMaskIdHigh = 0xFFE0;
  canFilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  canFilterInitStructure.CAN_FilterFIFOAssignment = 0;
  canFilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&canFilterInitStructure);
}

void CANConfig(uint16_t nCANPort)
{
 	CAN_TypeDef* CANx = GetCANTypeDef(nCANPort);
	NuEnableCANGPIO(nCANPort);
	CANClock(nCANPort);
	CANInitConfig(nCANPort, CAN_Mode_Normal, 8 - 1);
	CANFFilterInitConfig(nCANPort, CAN_FilterMode_IdMask);
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
	switch (nCANPort)
	{ 
		case ST_CAN_1:
			NuEnableCAN_NVIC(ST_CAN_1RX0, NVIC_PriorityGroup_0, 2);
		break;
		case ST_CAN_2:
			NuEnableCAN_NVIC(ST_CAN_2RX0, NVIC_PriorityGroup_0, 2);
		break;
	}
}

void NuOPModeEnableCAN(uint16_t nCANPort, uint32_t nNodeID)
{
	int nCANsn = nCANPort - ST_CAN_1;  
	char* szCANs[] = {"CAN1", "CAN2"};
	NuCANNode[nCANsn].m_nPort = nCANPort;
	NuCANNode[nCANsn].m_nNodeID = nNodeID;
	CANConfig(nCANPort);
	if (nOperationMode & OPMODE_USART)
	{
		char szNodeID[MAX_JSON_VALUE];
		memset(szNodeID, 0, MAX_JSON_VALUE);
		sprintf(szNodeID, "0x%X", nNodeID);
		JsonWriteIntoJsonPair(szCANs[nCANsn], szNodeID); 
	}
}

void CAN1_RX0_IRQHandler(void)
{
	CANRxIRQHandler(CAN1_SN);
}

void CAN2_RX0_IRQHandler(void)
{
	CANRxIRQHandler(CAN2_SN);
}

///////////////////////////////////////////
void JSetCAN(DWORD param[])
{
	char szText[MAX_JSON_VALUE];
	char szParam[MAX_JSON_VALUE];
	memset(szText, 0, MAX_JSON_VALUE);
	memcpy(szText, (void*) param[0], MAX_JSON_VALUE);
	strsplit(szText, szParam, "[");
	uint32_t nPort = Getint32(szText, ",");
	GetCharString(szText, ",", szParam);
	if (strFind(szParam, "OFF") == 0)
	{
		if (nPort == 1)
		{
			CAN_DeInit(CAN1);
			nOperationMode = nOperationMode & ~OPMODE_CAN1;
		}
		else
		{
			CAN_DeInit(CAN2);
			nOperationMode = nOperationMode & ~OPMODE_CAN2;
		}
		return;
	}
	uint32_t uAddress = Getint32(szText, "]");
	if (nPort == 1)
	{
		nPort = ST_CAN_1;
		nOperationMode |= OPMODE_CAN1;
	}
	else
	{
		nPort = ST_CAN_2;
		nOperationMode |= OPMODE_CAN2;
	}
	NuOPModeEnableCAN(nPort, uAddress);
}

void JCANTx(DWORD param[])
{
	char szText[MAX_JSON_VALUE];
	char szParam[MAX_JSON_VALUE];
	memset(szText, 0, MAX_JSON_VALUE);
	memcpy(szText, (void*) param[0], MAX_JSON_VALUE);
	strsplit(szText, szParam, "[");
	uint32_t nPort = Getint32(szText, ",");
	uint32_t nAddress = Getint32(szText, ",");
	GetCharString(szText, ",", szParam);
	if (nPort == 1)
	{
		CAN1Send(nAddress, szParam);
	}
	else
	{
		CAN2Send(nAddress, szParam);
	}
}
#endif