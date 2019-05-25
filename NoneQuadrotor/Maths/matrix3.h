#ifndef __MATRIX3__H__
#define __MATRIX3__H__

#include "mathtool.h"

void Matrix3_Add(float* a,float* b,float* c);
void Matrix3_Sub(float* a,float* b,float* c);
void Matrix3_Mul(float* a,float* b,float* c);
void Matrix3_Copy(float* a, float* b);
void Matrix3_Tran(float* a, float* b);
void Matrix3_Det(float* a,float* b);

#endif
