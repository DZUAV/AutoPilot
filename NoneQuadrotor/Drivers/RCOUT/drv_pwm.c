/**********************************************************************************************************
*文件说明：NoneQuadrotor UAV飞行控制代码
*实现功能：实现无人机精准控制
*修改日期：2018-11-17
*修改作者：crystal cup
*修改备注：
		             PE9   TIM1_CH1 TIM1 PWM(4) GPIO(53)
					 PE11 TIM1_CH2 TIM1 PWM(3) GPIO(52)
					 PA10 TIM1_CH3 TIM1 PWM(2) GPIO(51)
					 PE14 TIM1_CH4 TIM1 PWM(1) GPIO(50)
					 PD13 TIM4_CH2 TIM4 PWM(5) GPIO(54)
		       PD14 TIM4_CH3 TIM4 PWM(6) GPIO(55)
**********************************************************************************************************/
#include "drv_pwm.h"



TIM_HandleTypeDef TIM1_Handler;         //定时器1 PWM句柄 

TIM_OC_InitTypeDef TIM1_CH1Handler;     //定时器1通道1句柄
TIM_OC_InitTypeDef TIM1_CH2Handler;     //定时器1通道2句柄
TIM_OC_InitTypeDef TIM1_CH3Handler;     //定时器1通道3句柄
TIM_OC_InitTypeDef TIM1_CH4Handler;     //定时器1通道4句柄


/**********************************************************************************************************
*函数原型: void TIM1_PWM_Init(uint16_t arr,uint16_t psc,uint16_t Pulse )
*函数功能: 板层初始化配置
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：psc:19  216000000/20=10800000  psc10800000/480=22500
*          TIM1_PWM_Init(22499,19,9000); 
*          计数频率：10800000Hz
*          PWM频率=216000000/(19+1)/(22499+1)=480Hz
*          占空比：Pulse=40%
**********************************************************************************************************/
void TIM1_PWM_Init(uint16_t arr,uint16_t psc,uint16_t Pulse )
{ 
    TIM1_Handler.Instance=TIM1;            //定时器3
    TIM1_Handler.Init.Prescaler=psc;       //定时器分频
    TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM1_Handler.Init.Period=arr;          //自动重装载值
    TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM1_Handler);       //初始化PWM
	
    //TIM1_CH1
    TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM1_CH1Handler.Pulse=Pulse;          //设置比较值,此值用来确定占空比，
                                            //默认比较值为自动重装载值的一半,即占空比为50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为低 
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_1);//配置TIM3通道1
	  TIM1_Handler.Instance->BDTR|=(1<<15);
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1);//开启PWM通道4
	  //TIM1_CH2
	  TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM1_CH1Handler.Pulse=Pulse;          //设置比较值,此值用来确定占空比，
                                            //默认比较值为自动重装载值的一半,即占空比为50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为低 
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_2);//配置TIM1通道1
		  TIM1_Handler.Instance->BDTR|=(1<<15);
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_2);//开启PWM通道4
	  //TIM1_CH3
	  TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM1_CH1Handler.Pulse=Pulse;          //设置比较值,此值用来确定占空比，
                                            //默认比较值为自动重装载值的一半,即占空比为50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为低 
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_3);//配置TIM3通道1
		  TIM1_Handler.Instance->BDTR|=(1<<15);
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_3);//开启PWM通道4
	  //TIM1_CH4
	  TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM1_CH1Handler.Pulse=Pulse;          //设置比较值,此值用来确定占空比，
                                            //默认比较值为自动重装载值的一半,即占空比为50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为低 
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_4);//配置TIM3通道1
	TIM1_Handler.Instance->BDTR|=(1<<15);
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_4);//开启PWM通道4
	   
	
}



/**********************************************************************************************************
*函数原型: void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
*函数功能: 定时器底层驱动，时钟使能，引脚配置
*输入参数: htim:定时器句柄
*返回数据: none
*修改日期: 2018-11-17
*备注信息：此函数会被HAL_TIM_PWM_Init()调用
*          PE9  TIM1_CH1 TIM1 PWM(4) GPIO(53)
					 PE11 TIM1_CH2 TIM1 PWM(3) GPIO(52)
					 PA10 TIM1_CH3 TIM1 PWM(2) GPIO(51)
					 PE14 TIM1_CH4 TIM1 PWM(1) GPIO(50)
**********************************************************************************************************/

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM1_CLK_ENABLE();			//使能定时器1
	__HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();			//开启GPIOE时钟
	
    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_14;        //PE9,PE11,PE14
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	  //复用推完输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate=GPIO_AF1_TIM1;	  //PB1复用为TIM3_CH4
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	
	  
	GPIO_Initure.Pin=GPIO_PIN_10;        //PA10
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	  //复用推完输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate=GPIO_AF1_TIM1;	  //PA10复用为TIM1_CH3
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
}


/**********************************************************************************************************
*函数原型: void TIM_SetTIM1Compare1(u32 compare)
*函数功能: 设置TIM通道1的占空比
*输入参数: compare:比较值
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void TIM_SetTIM1Compare1(uint32_t compare)
{
	TIM1->CCR1=compare; 
}

/**********************************************************************************************************
*函数原型: void TIM_SetTIM1Compare2(u32 compare)
*函数功能: 设置TIM通道2的占空比
*输入参数: compare:比较值
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void TIM_SetTIM1Compare2(uint32_t compare)
{
	TIM1->CCR2=compare; 
}

/**********************************************************************************************************
*函数原型: void TIM_SetTIM1Compare3(u32 compare)
*函数功能: 设置TIM通道3的占空比
*输入参数: compare:比较值
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void TIM_SetTIM1Compare3(uint32_t compare)
{
	TIM1->CCR3=compare; 
}

/**********************************************************************************************************
*函数原型: void TIM_SetTIM1Compare4(u32 compare)
*函数功能: 设置TIM通道4的占空比
*输入参数: compare:比较值
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void TIM_SetTIM1Compare4(uint32_t compare)
{
	TIM1->CCR4=compare; 
}

void Set_Motor_PwmOut_Value(void)
{
	TIM_SetTIM1Compare1(9000);
	TIM_SetTIM1Compare2(9000);
  TIM_SetTIM1Compare3(9000);
	TIM_SetTIM1Compare4(9000);

	
}

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/