#ifndef __DRV_PPM_H_
#define __DRV_PPM_H_	 
#include "sys.h" 
#include "mathtool.h"

extern uint16_t ppm_rx[10];            // ���յ�ppm����



void PPM_Init(void);
void TIM4_Init(uint32_t arr,uint16_t psc);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void PPM_Signal_Analysis(void);



#endif

















