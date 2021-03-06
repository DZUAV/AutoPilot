/**********************************************************************************************************
*文件说明: 任务调度代码
*实现功能：实现无人机精准控制
*修改日期：2018-11-17
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/
#include "scheduler.h"
#include "copter.h"



float MS5611_BUFFER[3]={0,0,0};

/**********************************************************************************************************
*函数原型:void Loop_Schedule(void)
*函数功能:所有的任务调度都在这里进行
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void Loop_Schedule(void)
{
  
	if( LoopTime.check_flag == 1 )
	{
     //1ms
		if(LoopTime.cnt_1ms>=1)
		{
			LoopTime.cnt_1ms=0;
		    Loop_Task_1ms();
		}
		//2ms
	   if(LoopTime.cnt_2ms>=2)
		{
			LoopTime.cnt_2ms=0;
			Loop_Task_2ms();
		}

		//5ms
		 if(LoopTime.cnt_5ms>=5)
		{
			LoopTime.cnt_5ms=0;
			Loop_Task_5ms();
		}
		//10ms
		 if(LoopTime.cnt_10ms>=10)
		{
			LoopTime.cnt_10ms=0;
			Loop_Task_10ms();
		}
			//20ms
		 if(LoopTime.cnt_20ms>=20)
		{
			LoopTime.cnt_20ms=0;
			Loop_Task_20ms();
		}
			//25ms
		 if(LoopTime.cnt_25ms>=25)
		{
			LoopTime.cnt_25ms=0;
			Loop_Task_25ms();
		}
		
		
			//50ms
		 if(LoopTime.cnt_50ms>=50)
		{
			LoopTime.cnt_50ms=0;
			Loop_Task_50ms();
		}
		if(LoopTime.cnt_80ms>=80)
		{
			LoopTime.cnt_80ms=0;
			Loop_Task_80ms();
		}
		if(LoopTime.cnt_90ms>=90)
		{
			LoopTime.cnt_90ms=0;
			Loop_Task_90ms();
		}
		//100ms
		 if(LoopTime.cnt_100ms>=100)
		{
			LoopTime.cnt_100ms=0;
			Loop_Task_100ms();
		}
	  LoopTime.check_flag = 0;		//
	}
  
}



/**********************************************************************************************************
*函数原型: void  Loop_Task_1ms(void)
*函数功能: 1ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void  Loop_Task_1ms(void)
{


  Communicate_DT_Data_Exchange();

}

/**********************************************************************************************************
*函数原型:void  Loop_Task_2ms(void)
*函数功能:2ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/
uint8_t BMI055_acc_data[6]={0};
uint8_t BMI055_gyro_data[6]={0};
void  Loop_Task_2ms(void)
{
  ICM_20602_Read_Data();
  ICM_20689_Read_Data();
  BMI055_Read_Data(BMI055_acc_data,BMI055_gyro_data);
}



/**********************************************************************************************************
*函数原型: void  Loop_Task_5ms(void)
*函数功能: 5ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void  Loop_Task_5ms(void)
{
//  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);

//  Set_Motor_PwmOut_Value();

}

/**********************************************************************************************************
*函数原型:void  Loop_Task_10ms(void)
*函数功能:10ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/
static uint8_t JISUAN=0;
void  Loop_Task_10ms(void)
{
	
	JISUAN++;
	if(JISUAN==100)
	{
	  LED2(0);
	}
	else if((JISUAN>100)&&(JISUAN<=200))
	{
	     LED2(1);
		if(JISUAN==200)
		{
		  JISUAN=0;
		}
	}
	MS5611_Update();
//   MS5611_GetData(&MS5611_BUFFER[0],&MS5611_BUFFER[1],&MS5611_BUFFER[2]);


}

/**********************************************************************************************************
*函数原型:void  Loop_Task_20ms(void)
*函数功能:20ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void  Loop_Task_20ms(void)
{
        IST8310_Update();
 		LTRRGB_LED_RED(1);
	    LTRRGB_LED_GREEN(0);
	    LTRRGB_LED_BLUE(1);


}

/**********************************************************************************************************
*函数原型:void  Loop_Task_25ms(void)
*函数功能:25ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/
void  Loop_Task_25ms(void)
{



}


/**********************************************************************************************************
*函数原型:void  Loop_Task_50ms(void)
*函数功能:50ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/
void  Loop_Task_50ms(void)
{



}

/**********************************************************************************************************
*函数原型:void  Loop_Task_80ms(void)
*函数功能:80ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void  Loop_Task_80ms(void)
{




}

/**********************************************************************************************************
*函数原型:void  Loop_Task_90ms(void)
*函数功能:90ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/
void  Loop_Task_90ms(void)
{



}



/**********************************************************************************************************
*函数原型:void  Loop_Task_100ms(void)
*函数功能:100ms任务
*输入参数: None
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/
void  Loop_Task_100ms(void)
{



}




/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/





