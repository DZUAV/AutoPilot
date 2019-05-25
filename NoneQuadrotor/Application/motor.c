/**********************************************************************************************************
*文件说明：遥控器输入接口
*实现功能：统一遥控器输入接口
*修改日期：2018-11-17
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/
#include "motor.h"
#include "copter.h"
Motors_flags motors_flags;



/**********************************************************************************************************
*函数原型: bool motor_armed(void)
*函数功能: 获取电机解锁信息
*输入参数: none
*返回数据: motors_flags.armed：判断电机是否解锁
*修改日期: 2018-12-16
*备注信息：
**********************************************************************************************************/
bool motor_armed(void)
{

 return motors_flags.armed;

}



/**********************************************************************************************************
*函数原型: void arm_motors_check(void)
*函数功能: 电机解锁检查
*输入参数: none
*返回数据: none
*修改日期: 2018-12-16
*备注信息：
**********************************************************************************************************/

void arm_motors_check(void)
{
	  int16_t roll_in=0;
	  int16_t pitch_in=0;
	  int16_t yaw_in=0;
    static int16_t arming_counter;  //解锁计数
     //如果油门值太低,直接返回
     if (get_control_in(radio_channel_throttle) > 0)
    {
        arming_counter = 0;
        return;
    }
		
   //获取横滚,俯仰,偏航通道的控制量
	   roll_in = get_control_in(radio_channel_roll);
	   pitch_in = get_control_in(radio_channel_pitch);
	   yaw_in =get_control_in(radio_channel_yaw) ;
		if((yaw_in > 4000)&&(roll_in<-4000)&&(pitch_in>4000))
		{

    	// increase the arming counter to a maximum of 1 beyond the auto trim counter
			if (arming_counter <= AUTO_TRIM_DELAY)
			{
					arming_counter++;
			}
		  if (arming_counter == ARM_DELAY && !motor_armed()) //延迟2s,电机确实没有解锁
			{
			
			
			}
		
		}
		
}



/**********************************************************************************************************
*函数原型: void arm_motors_check(void)
*函数功能: 电机解锁检查
*输入参数: none
*返回数据: none
*修改日期: 2018-12-16
*备注信息：
**********************************************************************************************************/








