#ifndef __DRV_LED_H_
#define __DRV_LED_H_	 
#include "sys.h" 

//LED�˿ڶ���
#define LED0(x)			GPIO_Pin_Set(GPIOB,PIN1,x)		// DS0
#define LED1(x)			GPIO_Pin_Set(GPIOC,PIN6,x)		// DS1 
#define LED2(x)			GPIO_Pin_Set(GPIOC,PIN7,x)		// DS1 
void LED_Init(void);	//��ʼ��		 


#endif

















