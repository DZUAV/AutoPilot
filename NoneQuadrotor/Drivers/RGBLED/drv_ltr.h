#ifndef __DRV_LTR_H_
#define __DRV_LTR_H_	 
#include "sys.h" 

//LED端口定义
#define LTRRGB_LED_RED(x)			GPIO_Pin_Set(GPIOH,PIN10,x)		// DS0
#define LTRRGB_LED_GREEN(x)			GPIO_Pin_Set(GPIOH,PIN11,x)		// DS1 
#define LTRRGB_LED_BLUE(x)			GPIO_Pin_Set(GPIOH,PIN12,x)		// DS1 
enum
{
	 Ltr_PWMLed_Color_RED,     //红色
	 Ltr_PWMLed_Color_GREEN,   //绿色
	 Ltr_PWMLed_Color_BLUE,    //蓝色
	 Ltr_PWMLed_Color_YELLOW,  //黄色 :红+绿
	 Ltr_PWMLed_Color_PURPLE,  //紫色 :红+蓝
	 Ltr_PWMLed_Color_CYAN,    //青色 :蓝+绿
	 Ltr_PWMLed_Color_ALLOFF,  //关闭
};

void LTRrgb_LED_Init(void);
void LTRrgb_hw_set_rgb(uint8_t rgb_color);
void set_rgbled_red_on(void);
void set_rgbled_red_off(void);
void set_rgbled_green_on(void);
void set_rgbled_green_off(void);
void set_rgbled_blue_on(void);
void set_rgbled_blue_off(void);










#endif

















