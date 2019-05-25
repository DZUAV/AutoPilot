#ifndef __DRV_LTR_H_
#define __DRV_LTR_H_	 
#include "sys.h" 

//LED�˿ڶ���
#define LTRRGB_LED_RED(x)			GPIO_Pin_Set(GPIOH,PIN10,x)		// DS0
#define LTRRGB_LED_GREEN(x)			GPIO_Pin_Set(GPIOH,PIN11,x)		// DS1 
#define LTRRGB_LED_BLUE(x)			GPIO_Pin_Set(GPIOH,PIN12,x)		// DS1 
enum
{
	 Ltr_PWMLed_Color_RED,     //��ɫ
	 Ltr_PWMLed_Color_GREEN,   //��ɫ
	 Ltr_PWMLed_Color_BLUE,    //��ɫ
	 Ltr_PWMLed_Color_YELLOW,  //��ɫ :��+��
	 Ltr_PWMLed_Color_PURPLE,  //��ɫ :��+��
	 Ltr_PWMLed_Color_CYAN,    //��ɫ :��+��
	 Ltr_PWMLed_Color_ALLOFF,  //�ر�
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

















