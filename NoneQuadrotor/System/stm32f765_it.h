#ifndef __STM32F765_IT__H__
#define __STM32F765_IT__H__
#include "stm32f7xx.h" 

          


extern volatile uint32_t SysTickUptime;




void SysTick_Handler(void);
void USART2_IRQHandler(void);







#endif

