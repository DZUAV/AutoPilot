#ifndef __DRV_PWM_H_
#define __DRV_PWM_H_	 
#include "sys.h" 
#include "mathtool.h"




void TIM1_PWM_Init(uint16_t arr,uint16_t psc,uint16_t Pulse );
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void TIM_SetTIM1Compare1(uint32_t compare);
void TIM_SetTIM1Compare2(uint32_t compare);
void TIM_SetTIM1Compare3(uint32_t compare);
void TIM_SetTIM1Compare4(uint32_t compare);
void Set_Motor_PwmOut_Value(void);
#endif

















