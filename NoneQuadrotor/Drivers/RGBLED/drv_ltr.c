/**********************************************************************************************************
*文件说明：RGBLED驱动文件配置函数
*实现功能：配置点亮rgbled
*修改日期：2018-11-26
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/
#include "drv_ltr.h" 
#include "copter.h"

/**********************************************************************************************************
*函数原型: void Ltrrgb_LED_Init(void)
*函数功能: RGBLED IO初始化
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void LTRrgb_LED_Init(void)
{    	 
  RCC->AHB1ENR|=1<<7;	//使能PORTB时钟 
	
	GPIO_Set(GPIOH,PIN10|PIN11|PIN12,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PB1设置
	LTRRGB_LED_RED(1);
	LTRRGB_LED_GREEN(1);
	LTRRGB_LED_BLUE(1);
}

/**********************************************************************************************************
*函数原型: void LTRrgb_hw_set_rgb(uint8_t rgb_color)
*函数功能: 设定颜色
*输入参数: rgb_color：颜色值
*返回数据: none
*修改日期: 2018-11-26
*备注信息：
**********************************************************************************************************/
void LTRrgb_hw_set_rgb(uint8_t rgb_color)
{
	 switch (rgb_color)
	 {
		 case Ltr_PWMLed_Color_RED:   //红
			 set_rgbled_red_on();
			 set_rgbled_green_off();
			 set_rgbled_blue_off();
			   break;
		 case Ltr_PWMLed_Color_GREEN: //绿
			 set_rgbled_red_off();
			 set_rgbled_green_on();
			 set_rgbled_blue_off();
		       break;
		 case Ltr_PWMLed_Color_BLUE:  //蓝
			 set_rgbled_red_off();
			 set_rgbled_green_off();
			 set_rgbled_blue_on();
			   break;
		 case Ltr_PWMLed_Color_YELLOW://黄
			 set_rgbled_red_on();
			 set_rgbled_green_on();
			 set_rgbled_blue_off();
		       break;
		 case Ltr_PWMLed_Color_PURPLE://紫
			 set_rgbled_red_on();
			 set_rgbled_green_off();
			 set_rgbled_blue_on();
			   break;
		 case Ltr_PWMLed_Color_CYAN: //青色
			 set_rgbled_red_off();
			 set_rgbled_green_on();
			 set_rgbled_blue_on();
		       break;
		 case Ltr_PWMLed_Color_ALLOFF: //关闭
				 set_rgbled_red_off();
				 set_rgbled_green_off();
				 set_rgbled_blue_off();
			       break;
		 default:
			 break;
	 }


}
/**********************************************************************************************************
*函数原型: void set_rgbled_red_on(void)
*函数功能: red打开
*输入参数: none
*返回数据: none
*修改日期: 2018-11-26
*备注信息：
**********************************************************************************************************/
void set_rgbled_red_on(void)
{
  LTRRGB_LED_RED(0);
}
/**********************************************************************************************************
*函数原型: void set_rgbled_red_off(void)
*函数功能: green打开
*输入参数: none
*返回数据: none
*修改日期: 2018-11-26
*备注信息：
**********************************************************************************************************/
void set_rgbled_red_off(void)
{
  LTRRGB_LED_RED(1);
}
/**********************************************************************************************************
*函数原型: void set_rgbled_green_on(void)
*函数功能: green打开
*输入参数: none
*返回数据: none
*修改日期: 2018-11-26
*备注信息：
**********************************************************************************************************/
void set_rgbled_green_on(void)
{
 LTRRGB_LED_GREEN(0);
}
/**********************************************************************************************************
*函数原型: void set_rgbled_green_off(void)
*函数功能: green关闭
*输入参数: none
*返回数据: none
*修改日期: 2018-11-26
*备注信息：
**********************************************************************************************************/

void set_rgbled_green_off(void)
{
  LTRRGB_LED_GREEN(1);
}
/**********************************************************************************************************
*函数原型: void set_rgbled_blue_on(void)
*函数功能: 蓝灯打开
*输入参数: none
*返回数据: none
*修改日期: 2018-11-26
*备注信息：
**********************************************************************************************************/
void set_rgbled_blue_on(void)
{

 LTRRGB_LED_BLUE(0);

}
/**********************************************************************************************************
*函数原型: void set_rgbled_blue_off(void)
*函数功能: 蓝灯关闭
*输入参数: none
*返回数据: none
*修改日期: 2018-11-26
*备注信息：
**********************************************************************************************************/
void set_rgbled_blue_off(void)
{
  LTRRGB_LED_BLUE(1);
}
/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/

