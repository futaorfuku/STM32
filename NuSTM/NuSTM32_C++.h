#ifndef _NuSTM32_CPLUSPLUS_
#define _NuSTM32_CPLUSPLUS_
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

int strFind(char szText[], const char* sz);
void strsplit(char szText[], char szParam[], const char* szSymbol);
void strtrim(char szText[]);
int GCD(int A, int B);
int Bound(int imax, int imin, int ivalue);;
double _fabs(double a);
double _fmax(double x, double y);
double _fmin(double x, double y);
int _iabs(int a);
int _imax(int x, int y);
int _imin(int x, int y);
#endif
