#ifndef __NU_JSON_H
#define __NU_JSON_H
#include "NuSTM32_BASIC.h"
#include "NuSTM32_C++.h"

////////////////////
// Json
extern int nJsonWriteStart;
extern int nJsonWriteEnd;
double Getdouble(char szText[], const char* szSymbol);
int32_t Getint32(char szText[], const char* szSymbol);
void GetCharString(char szText[], const char* szSymbol, char szParam[]);
#define JSON_APP

#ifdef JSON_APP
extern char szJsonWriteLeft[MAX_JSON_STRING];
void JsonInit(void);
void JsonWriteInit(void);
void JsonRead(char sz[], int nLen);
void JsonReadStorage(char sz[], int nLen);
void GetCharString(char szText[], const char* szSymbol, char szParam[]);
void JsonWriteOpen(void);
void JsonWriteClose(void);
void JsonWriteIntoJsonPair(char szKey[], char szValue[]);
void JsonEcodeIntArray(int iArray[], int nLen, char szText[]);
void JsonWrite(FILE* ouf);
bool JPopWriteBufferEmpty(int nCount);
bool JPopWrite(void);
void JsonPrint(void);
void JsonWriteIntIntoJsonPair(char szKey[], int nValue);
void JsonPartition(DWORD param[], char szText[MAX_JSON_VALUE]);
#endif

///////////////////////////////
//
#define MAX_STRING_IN_QUEUE	 	10
#define PARAMVALUE_IS_STRING	1
#define PARAMVALUE_IS_INT			2
#define PARAMVALUE_IS_BOOL		3
#define PARAMVALUE_IS_FLOAT		4
#define JSON_QUERY_STRING 		"?"
#define MAX_PARAMS						10
#define PARAMVALUE						101010101
#define MAX_JASON_ACTIONS			10

typedef struct
{
	char szKey[MAX_JSON_KEY];
	char szValue[MAX_JSON_VALUE];
} JsonPair;

typedef struct{
	char szKey[MAX_JSON_KEY];
	uint8_t nValueDataType;
	void (*ActionFunctionID)(DWORD param[]);
	void (*QueryFunctionID)(JsonPair* pJP, DWORD param[]);
	uint8_t nParams;
	DWORD m_param[MAX_PARAMS];
} JsonCommand;


#endif
