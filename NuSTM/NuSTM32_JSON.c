#include "Nu_STM32.h"
#include "ITRI_APP.h"
#include "ELAN_APP.h"
char szJsonReadRight[MAX_JSON_STRING];
int nJsonReadRight = 0;
char szJsonWriteLeft[MAX_JSON_STRING];
int nJsonWriteLeft = 0;
int nJsonWriteStart = 0;
int nJsonWriteEnd = 0;
int nJsonStringIndex = 0;

void JSetBit(DWORD param[]);
void JGetBit(JsonPair* pJP, DWORD param[]);
void JRealTime(JsonPair* pJP, DWORD param[]);
void JSystemReset(DWORD param[]);

void JWriteData(DWORD param[])
{
	if (nOperationMode & OPMODE_ITRI_APP)
		JITRIApp(param);
	/*
	else if (nOperationMode & OPMODE_ELAN_APP)
		JELANApp(param);
	*/
}

JsonCommand JasonCommands[MAX_JASON_ACTIONS] =
{
	{"TIME", PARAMVALUE_IS_STRING, NULL, JRealTime, 1, {0}},
#ifdef _USE_DAC_
	{"DAC12", PARAMVALUE_IS_STRING, JSetDAC12, NULL, 1, {PARAMVALUE}},
#endif
	{"CHARGING", PARAMVALUE_IS_BOOL, JSetBit, JGetBit, 1, {PARAMVALUE}},
  {"P_PWM", PARAMVALUE_IS_STRING, JPrintPWM, NULL, 1, {PARAMVALUE}},	
	{"SYSTEM", PARAMVALUE_IS_BOOL, JSystemReset, NULL, 1, {0}},
	{"P_ADC", 		PARAMVALUE_IS_STRING, JPrintADC, NULL, 1, {PARAMVALUE}},
	{"WriteData", PARAMVALUE_IS_STRING, JWriteData, NULL, 1, {PARAMVALUE}},
#ifdef _USE_CAN_
	{"CAN", PARAMVALUE_IS_STRING, JSetCAN, NULL, 1, {PARAMVALUE}},
	{"CANTx", PARAMVALUE_IS_STRING, JCANTx, NULL, 1, {PARAMVALUE}},
#endif
	};


void JSetBit(DWORD param[])
{
	NuSetBit(D13, param[0]);
}

void JsonPairInit(JsonPair* pJP)
{
	memset(pJP->szKey, 0, MAX_JSON_KEY);
	memset(pJP->szValue, 0, MAX_JSON_VALUE);
}

void JRealTime(JsonPair* pJP, DWORD param[])
{
	char szText[MAX_JSON_VALUE];
	NuTimeReport(szText);
	JsonWriteIntoJsonPair(pJP->szKey, szText);
}

void JSystemReset(DWORD param[])
{
	if (!NuGetBit(C4))
		NVIC_SystemReset();
	else
	{
		m_bSystemReportInited = false;
		fprintf(USART_FILE, "{\"P\"=\"%d\"}", 1);
	}
}

void JsonEcodeIntArray(int iArray[], int nLen, char szText[])
{
	memset(szText, 0, MAX_JSON_VALUE);
	strcat(szText, "[");
	for (int i = 0; i < nLen; i++)
	{
		char szPin[MAX_JSON_VALUE];
		memset(szPin, 0, MAX_JSON_VALUE);
		if (strlen(szText) > 1)
				strcat(szText, ",");	
		sprintf(szPin, "%d", iArray[i]);
		strcat(szText, szPin);
	}
	strcat(szText, "]");
}

bool JMoreToPop()
{
	if (nJsonWriteEnd == 0)
		return false;
	if (nJsonWriteStart < nJsonWriteEnd)
		return true;
	JsonWriteInit();
	nJsonWriteStart = 0;
	nJsonWriteEnd = 0;
	return false;
}

bool JPopWrite(void)
{
	if (!JMoreToPop())
		return false;
	fputc((char) szJsonWriteLeft[nJsonWriteStart++], USART_FILE);
	return JMoreToPop();
}

bool JPopWriteBufferEmpty(int nCount)
{
	if ((nOperationMode & OPMODE_USART) == 0)
		return true;
	for (int i = 0; i < nCount; i++)
	{
		if (!JPopWrite())
			return true;
	}
	return false;
}

void GetCharString(char szText[], const char* szSymbol, char szParam[])
{
	strsplit(szText, szParam, szSymbol);
}

int32_t Getint32(char szText[], const char* szSymbol)
{
	char szParam[MAX_JSON_VALUE];
	GetCharString(szText,  szSymbol, szParam);
	return atoi(szParam);
}

double Getdouble(char szText[], const char* szSymbol)
{
	char szParam[MAX_JSON_VALUE];
	GetCharString(szText,  szSymbol, szParam);
	return atof(szParam);
}

////////////////////////////////////
//

DWORD JsonValueToDWORD(JsonPair* pJP, uint8_t nValueDataType)
{
	DWORD data = 0;
	switch (nValueDataType)
	{
		case PARAMVALUE_IS_STRING:
			data = (DWORD) pJP->szValue;
			break;
		case PARAMVALUE_IS_INT:
			data = atoi(pJP->szValue);
			break;
		case PARAMVALUE_IS_FLOAT:
			{
				float f = atof(pJP->szValue);
				data = *(DWORD*)&f;
			}
			break;
		case PARAMVALUE_IS_BOOL:
			data = strcmp(pJP->szValue, "TRUE") == 0 || strcmp(pJP->szValue, "ON") == 0? 1: 0;
	}
	return data;
}

DWORD JParam(JsonCommand* pJC, int nParam, JsonPair* pJP)
{
	DWORD data = pJC->m_param[nParam];
	if (data != PARAMVALUE)
		return data;
	return JsonValueToDWORD(pJP, pJC->nValueDataType);
}

void DWORDToJsonValue(DWORD data, uint8_t nValueDataType, JsonPair* pJP)
{
	memset(pJP->szValue, 0, MAX_JSON_VALUE);
	switch (nValueDataType)
	{
		case PARAMVALUE_IS_STRING:
			memcpy(pJP->szValue, (void*) data, MAX_JSON_VALUE);
			break;
		case PARAMVALUE_IS_INT:
			sprintf(pJP->szValue, "%d", data);
			break;
		case PARAMVALUE_IS_FLOAT:
			sprintf(pJP->szValue, "%f", *(float*)&data);
			break;
		case PARAMVALUE_IS_BOOL:
			sprintf(pJP->szValue, "%s", data == 0? "OFF": "ON");
			break;
	}
}

void LoadJasonParam(JsonCommand* pJC, DWORD param[], JsonPair* pJP)
{
	for (int i = 0; i < pJC->nParams; i++)
		param[i] = JParam(pJC, i, pJP);
}

void JsonWriteStorage(char sz[], int nLen)
{
	memcpy(&szJsonWriteLeft[nJsonWriteLeft], sz, nLen);
	nJsonWriteLeft += nLen;
}

void JsonWriteJasonPair(JsonPair* pJP)
{
	if (nJsonWriteLeft > 1 &&
		szJsonWriteLeft[nJsonWriteLeft - 1] != '{')
		JsonWriteStorage(",", 1);
	sprintf(&szJsonWriteLeft[nJsonWriteLeft], "\"%s\"=\"%s\"",	pJP->szKey, pJP->szValue);
	nJsonWriteLeft = strlen(szJsonWriteLeft);
}

void JsonAction(JsonPair* pJP)
{
	DWORD param[MAX_PARAMS] = {0};
	for (int i = 0; i < MAX_JASON_ACTIONS; i++)
	{
		JsonCommand* pJC = &JasonCommands[i];
		LoadJasonParam(pJC, param, pJP);
		bool bQuery = strcmp(pJP->szValue, JSON_QUERY_STRING) == 0;
		if (strcmp(pJP->szKey, pJC->szKey) == 0)
		{
			if (bQuery)
			{
				JasonCommands[i].QueryFunctionID(pJP, param);
			}
			else
				JasonCommands[i].ActionFunctionID(param);
		}
	}
	JsonPairInit(pJP);
}

void JsonReadInit()
{
	memset(szJsonReadRight, 0, MAX_JSON_STRING);
	nJsonReadRight = 0;
}

void JsonWriteInit()
{
	memset(szJsonWriteLeft, 0, MAX_JSON_STRING);
	nJsonWriteLeft = 0;
}

void JsonWriteOpen()
{
	szJsonWriteLeft[nJsonWriteLeft++] = '{';
}

void JsonWriteClose()
{
	szJsonWriteLeft[nJsonWriteLeft++] = '}';
}

void JGetBit(JsonPair* pJP, DWORD param[])
{
	DWORDToJsonValue(NuGetBit(D13), PARAMVALUE_IS_BOOL, pJP);
	JsonWriteJasonPair(pJP);
}

void JsonInit()
{
	JsonReadInit();
	JsonWriteInit();
}

void JsonParser()
{
	JsonPair JP;
	JsonPairInit(&JP);
	bool bDoubleQuoteOpen = false;
	bool bKey = true;
	int nCharCount = 0;
	for (int i = 0; i < nJsonReadRight; i++)
	{
		char ch = szJsonReadRight[i];
		if (ch == '\"')
		{
			if (!bDoubleQuoteOpen)
				bDoubleQuoteOpen = true;
			else
			{
				bDoubleQuoteOpen = false;
				if (!bKey)
				{
					JsonAction(&JP);
					JsonPairInit(&JP);
				}
				nCharCount = 0;
				bKey = !bKey;
			}
		}
		else if (bDoubleQuoteOpen)
		{
			if (bKey)
				JP.szKey[nCharCount++] = ch;
			else
				JP.szValue[nCharCount++] = ch;
		}
	}
	JsonReadInit();
}

void JsonReadStorage(char sz[], int nLen)
{
	memcpy(&szJsonReadRight[nJsonReadRight], sz, nLen);
	nJsonReadRight += nLen;
}

void JsonRead(char sz[], int nLen)
{
	JsonWriteInit();
	JsonWriteOpen();
	JsonReadStorage(sz, nLen);
	JsonParser();
	JsonWriteClose();
}

void JsonWriteIntIntoJsonPair(char szKey[], int nValue)
{
	char szText[MAX_JSON_VALUE];
	memset(szText, 0, MAX_JSON_VALUE);
	sprintf(szText, "%d", nValue);
	JsonWriteIntoJsonPair(szKey, szText);
}

void JsonWriteIntoJsonPair(char szKey[], char szValue[])
{
	if ((nOperationMode & OPMODE_USART) == 0)
		return;
	JsonPair JP;
	JsonPairInit(&JP);
	memcpy(JP.szKey, szKey, MAX_JSON_VALUE);
	memcpy(JP.szValue, szValue, MAX_JSON_VALUE);
	JsonWriteJasonPair(&JP);
}

void JsonWriteIntoJsonPairObject(char szKey[], char szValue[])
{
	JsonPair JP;
	JsonPairInit(&JP);
	memcpy(JP.szKey, szKey, MAX_JSON_KEY);
	memcpy(JP.szValue, szValue, MAX_JSON_VALUE);
	JsonWriteJasonPair(&JP);
}

void JsonWrite(FILE* ouf)
{
	if (strlen(szJsonWriteLeft) <= 2)
		return;
	NuUSARTPrint(szJsonWriteLeft);
	JsonWriteInit();
}

void JsonPrint(void)
{
	JsonWrite(USART_FILE);
}

void JsonPartition(DWORD param[], char szText[MAX_JSON_VALUE])
{
	char szParam[MAX_JSON_VALUE];
	memset(szText, 0, MAX_JSON_VALUE);
	memcpy(szText, (void*) param[0], MAX_JSON_VALUE);
	strsplit(szText, szParam, "[");
}
