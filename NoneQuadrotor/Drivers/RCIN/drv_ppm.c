/**********************************************************************************************************
*文件说明：PPM驱动文件配置函数
*实现功能：配置ppm
*修改日期：2018-12-6
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/

#include "drv_ppm.h"
#include "copter.h"

TIM_HandleTypeDef TIM4_Handler;  //定时器5句柄

uint16_t ppm_rx[10];            // 接收到ppm数据

/**********************************************************************************************************
*函数原型: void PPM_Init(void)
*函数功能: PPM初始化
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：1MHz的计数频率,20ms中断一次
**********************************************************************************************************/

void PPM_Init(void)
{
  TIM4_Init(0xffff,108-1); //这里配置20ms
}


/**********************************************************************************************************
*函数原型: void TIM4_Init(uint32_t arr,uint16_t psc)
*函数功能: TIM4初始化
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：TIM4时钟是108M
**********************************************************************************************************/
void TIM4_Init(uint32_t arr,uint16_t psc)
{ 
	  //配置TIM4_CH2
    TIM_IC_InitTypeDef TIM4_CH2Config;  
	  TIM_IC_InitTypeDef TIM4_CH3Config;  
	
    TIM4_Handler.Instance=TIM4;                          //通用定时器4
    TIM4_Handler.Init.Prescaler=psc;                     //分频
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM4_Handler.Init.Period=arr;                        //自动装载值
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&TIM4_Handler);
    
    TIM4_CH2Config.ICPolarity=TIM_ICPOLARITY_BOTHEDGE;    //上升沿捕获
    TIM4_CH2Config.ICSelection=TIM_ICSELECTION_DIRECTTI;//映射到TI1上
    TIM4_CH2Config.ICPrescaler=TIM_ICPSC_DIV1;          //配置输入分频，不分频
    TIM4_CH2Config.ICFilter=0;                          //配置输入滤波器，不滤波
    HAL_TIM_IC_ConfigChannel(&TIM4_Handler,&TIM4_CH2Config,TIM_CHANNEL_2);//配置TIM4通道1
    HAL_TIM_IC_Start_IT(&TIM4_Handler,TIM_CHANNEL_2);   //开始捕获TIM4的通道2
    __HAL_TIM_ENABLE_IT(&TIM4_Handler,TIM_IT_UPDATE);   //使能更新中断
	
	  //配置TIM4_CH3
	
    TIM4_Handler.Instance=TIM4;                          //通用定时器4
    TIM4_Handler.Init.Prescaler=psc;                     //分频
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM4_Handler.Init.Period=arr;                        //自动装载值
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&TIM4_Handler);                      //调用HAL_TIM_IC_MspInit
    
    TIM4_CH3Config.ICPolarity=TIM_ICPOLARITY_BOTHEDGE;    //上升沿捕获
    TIM4_CH3Config.ICSelection=TIM_ICSELECTION_DIRECTTI;//映射到TI1上
    TIM4_CH3Config.ICPrescaler=TIM_ICPSC_DIV1;          //配置输入分频，不分频
    TIM4_CH3Config.ICFilter=0;                          //配置输入滤波器，不滤波
    HAL_TIM_IC_ConfigChannel(&TIM4_Handler,&TIM4_CH3Config,TIM_CHANNEL_3);//配置TIM4通道1
    HAL_TIM_IC_Start_IT(&TIM4_Handler,TIM_CHANNEL_3);   //开始捕获TIM4的通道3
    __HAL_TIM_ENABLE_IT(&TIM4_Handler,TIM_IT_UPDATE);   //使能更新中断
	
}

/**********************************************************************************************************
*函数原型: void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
*函数功能: 
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-12-6
*备注信息：
**********************************************************************************************************/

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM4_CLK_ENABLE();            //使能TIM4时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();			      //开启GPIOD时钟
	
    GPIO_Initure.Pin=GPIO_PIN_13|GPIO_PIN_14;            //PD13,PD14
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    GPIO_Initure.Alternate=GPIO_AF2_TIM4;   //PA0复用为TIM4通道1
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM4_IRQn,2,0);    //设置中断优先级，抢占优先级2，子优先级0
    HAL_NVIC_EnableIRQ(TIM4_IRQn);          //开启ITM4中断   
}

/**********************************************************************************************************
*函数原型: void TIM4_IRQHandler(void)
*函数功能: 定时器4中断服务函数
*输入参数: none
*返回数据: none
*修改日期: 2018-12-7
*备注信息：解析PPM信号
**********************************************************************************************************/
uint8_t  TIM4CH2_CAPTURE_STA=0,ppm_rx_sta=0,ppm_rx_num=0;	//输入捕获状态		
uint16_t	TIM4CH2_CAPTURE_VAL;	//输入捕获值
void TIM4_IRQHandler(void)
{	
 	if((TIM4CH2_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (((TIM4->SR &(0x0001))!=RESET)&&((TIM4->DIER &(0x0001))!=RESET))
		 
		{	    
			if(TIM4CH2_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM4CH2_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM4CH2_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM4CH2_CAPTURE_VAL=0XFFFF;
				}else 
				{
				 TIM4CH2_CAPTURE_STA++;
				}
			}	 
		}
	if (((TIM4->SR &(0x0002))!=RESET)&&((TIM4->DIER &(0x0002))!=RESET))//捕获1发生捕获事件
		{	
			if(TIM4CH2_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM4CH2_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
				TIM4CH2_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&TIM4_Handler,TIM_CHANNEL_2);
				TIM_RESET_CAPTUREPOLARITY(&TIM4_Handler,TIM_CHANNEL_2);   //一定要先清除原来的设置！！
        TIM_SET_CAPTUREPOLARITY(&TIM4_Handler,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//配置TIM4通道2上升沿捕获
		   		
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM4CH2_CAPTURE_STA=0;			//清空
				TIM4CH2_CAPTURE_VAL=0;
				TIM4CH2_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				__HAL_TIM_SET_COUNTER(&TIM4_Handler,0);
	 			

				TIM_RESET_CAPTUREPOLARITY(&TIM4_Handler,TIM_CHANNEL_2);   //一定要先清除原来的设置！！
				TIM_SET_CAPTUREPOLARITY(&TIM4_Handler,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);//定时器4通道2设置为下降沿捕获
				__HAL_TIM_ENABLE(&TIM4_Handler);//使能定时器4

			}		    
		}			     	    					   
 	}
	//开始处理解析PPM数据
	PPM_Signal_Analysis();
	TIM4->SR = (uint16_t)~((0x0001)|(0x0002)); //清除中断标志位
}

/**********************************************************************************************************
*函数原型: void PPM_Signal_Analysis(void)
*函数功能: PPM数据解析
*输入参数: none
*返回数据: none
*修改日期: 2018-12-8
*备注信息：解析PPM信号
**********************************************************************************************************/
void PPM_Signal_Analysis(void)
{

		if(TIM4CH2_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
		{

			if(ppm_rx_sta==1) 
      {
				ppm_rx[ppm_rx_num+1]=TIM4CH2_CAPTURE_VAL;
				ppm_rx_num++;
			
			}
			if(4>TIM4CH2_CAPTURE_STA&0X3F>0||TIM4CH2_CAPTURE_VAL>3000)
			{
			   ppm_rx_sta++;//低电平时间大于3000us为起始帧
			}
			if(ppm_rx_sta==2) 
      {
				ppm_rx_sta=0;
				ppm_rx[0]=1;
				ppm_rx_num=0;
			}
			
			TIM4CH2_CAPTURE_STA=0;//开启下一次捕获
			
		}
}


/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/

