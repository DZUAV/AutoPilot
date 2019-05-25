#ifndef __DRV_SBUS_H_
#define __DRV_SBUS_H_	 
#include "sys.h" 
#include "mathtool.h"

#define SBUS_BSIZE    25
#define SBUS_CHANNELS 16
#define SBUS_MIN 880.0f
#define SBUS_MAX 2156.0f
#define SBUS_SCALE (2048.0f / (SBUS_MAX - SBUS_MIN))  

extern int sbus_channel[12];


void Sbus_Init(void);
void Sbus_Parser(void);
int sbus_get_data(int ch_num);
void HAL_SYSTICK_Callback(void);
void timer_cb(void);
#endif

















