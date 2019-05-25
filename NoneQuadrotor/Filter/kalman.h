#ifndef __KALMAN__H__
#define __KALMAN__H__

#include "stdlib.h"

typedef struct {
    float X_last; //��һʱ�̵����Ž��
    float X_mid;  //��ǰʱ�̵�Ԥ����
    float X_now;  //��ǰʱ�̵����Ž��
    float P_mid;  //��ǰʱ��Ԥ������Э����
    float P_now;  //��ǰʱ�����Ž����Э����
    float P_last; //��һʱ�����Ž����Э����
    float kg;     //kalman����
    float A;      //ϵͳ����
    float Q;
    float R;
    float H;
}kalman;

void kalmanCreate(kalman *p,float T_Q,float T_R);
float KalmanFilter(kalman* p,float dat);
#endif