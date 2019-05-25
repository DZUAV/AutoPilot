/**********************************************************************************************************
*文件说明：遥控器输入接口
*实现功能：统一遥控器输入接口
*修改日期：2018-11-17
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/
#include "radio.h"
#include "copter.h"


RADIODATA_t sbusData;
enum ChannelType type_in;
enum ChannelMap radio_channel_map;  //遥控器通道映射
//主要配置是美国手,支持1,2,3,4通道设置：横滚,俯仰,油门,偏航通道
Radio_input_value radio_channel_roll;     //1通道
Radio_input_value radio_channel_pitch;    //2通道
Radio_input_value radio_channel_throttle; //3通道
Radio_input_value radio_channel_yaw;      //4通道
Radio_input_value radio_channel5;         //5通道
Radio_input_value radio_channel6;         //6通道
Radio_input_value radio_channel7;         //7通道
Radio_input_value radio_channel8;         //8通道
/**********************************************************************************************************
*函数原型: void Radio_init_in(void)
*函数功能: 设置遥控通道的范围
*输入参数: none
*返回数据: none
*修改日期: 2018-12-13
*备注信息：
**********************************************************************************************************/
void Radio_init_in(void)
{
	  Sbus_Init();  //初始化Sbus遥控器
	
	 //设定1--8通道的量程范围
   Radio_set_angle(radio_channel_roll,ROLL_PITCH_YAW_INPUT_MAX); //设置横滚通道的最大值是4500
   Radio_set_angle(radio_channel_pitch,ROLL_PITCH_YAW_INPUT_MAX);//设置俯仰通道的最大值是4500
	 Radio_set_angle(radio_channel_yaw,ROLL_PITCH_YAW_INPUT_MAX);  //设置偏航通道的最大值是4500
   Radio_set_range(radio_channel_throttle,1000);                 //设置油门通道的最大值是1000
   Radio_set_range(radio_channel5,1000);                         //设置5通道的最大值是1000
	 Radio_set_range(radio_channel6,1000);                         //设置6通道的最大值是1000
   Radio_set_range(radio_channel7,1000);                         //设置7通道的最大值是1000
	 Radio_set_range(radio_channel8,1000);                         //设置8通道的最大值是1000
	 //设置1--8通道的死区范围
	 Radio_set_default_dead_zone(radio_channel_roll,20);
	 Radio_set_default_dead_zone(radio_channel_pitch,20);
	 Radio_set_default_dead_zone(radio_channel_throttle,30);
	 Radio_set_default_dead_zone(radio_channel_yaw,20);
	 Radio_set_default_dead_zone(radio_channel5,0);
	 Radio_set_default_dead_zone(radio_channel6,0);
	 Radio_set_default_dead_zone(radio_channel7,0);
	 Radio_set_default_dead_zone(radio_channel8,0);
}



/**********************************************************************************************************
*函数原型: void Radio_Update(void)
*函数功能: 更新遥控器控制数据
*输入参数: none
*返回数据: none
*修改日期: 2018-12-13
*备注信息：
**********************************************************************************************************/

void Radio_Update(void)
{
  //需要的遥控器数据
  sbusData.channel_value[0]=sbus_get_data(RADIO_CH_ROLL);
  sbusData.channel_value[1]=sbus_get_data(RADIO_CH_PITCH);
	sbusData.channel_value[2]=sbus_get_data(RADIO_CH_THROTTLE);
  sbusData.channel_value[3]=sbus_get_data(RADIO_CH_YAW);
  sbusData.channel_value[4]=sbus_get_data(CH_5);
  sbusData.channel_value[5]=sbus_get_data(CH_6);
	sbusData.channel_value[6]=sbus_get_data(CH_7);
  sbusData.channel_value[7]=sbus_get_data(CH_8);
	sbusData.channel_value[8]=sbus_get_data(CH_9);
	sbusData.channel_value[9]=sbus_get_data(CH_10);
  sbusData.channel_value[10]=sbus_get_data(CH_11);
  sbusData.channel_value[11]=sbus_get_data(CH_12);
	
	//控制输入量
	radio_channel_roll.radio_in=sbus_get_data(RADIO_CH_ROLL);
  radio_channel_pitch.radio_in=sbus_get_data(RADIO_CH_PITCH);
	radio_channel_throttle.radio_in=sbus_get_data(RADIO_CH_THROTTLE);
  radio_channel_yaw.radio_in=sbus_get_data(RADIO_CH_YAW);
  radio_channel5.radio_in=sbus_get_data(CH_5);
  radio_channel6.radio_in=sbus_get_data(CH_6);
	radio_channel7.radio_in=sbus_get_data(CH_7);
  radio_channel8.radio_in=sbus_get_data(CH_8);
	//设置横滚控制输入量
	if(radio_channel_roll.type_in==RC_CHANNEL_TYPE_ANGLE)
	{
	 radio_channel_roll.control_in=Radio_pwm_to_angle(radio_channel_roll);
	
	}
	else if(radio_channel_roll.type_in==RC_CHANNEL_TYPE_RANGE)
	{
	 radio_channel_roll.control_in=Radio_pwm_to_range(radio_channel_roll);
	}

  	//设置俯仰控制输入量
	if(radio_channel_pitch.type_in==RC_CHANNEL_TYPE_ANGLE)
	{
	 radio_channel_pitch.control_in=Radio_pwm_to_angle(radio_channel_pitch);
	
	}
	else if(radio_channel_pitch.type_in==RC_CHANNEL_TYPE_RANGE)
	{
	 radio_channel_pitch.control_in=Radio_pwm_to_range(radio_channel_pitch);
	}

}


/**********************************************************************************************************
*函数原型: int16_t get_control_in(Radio_input_value radio_value)
*函数功能: 返回控制输入值
*输入参数: radio_value：遥控器控制输入值
*返回数据: control_in:哪个控制通道的输入值
*修改日期: 2018-12-13
*备注信息：
**********************************************************************************************************/
int16_t get_control_in(Radio_input_value radio_value)
{
  return radio_value.control_in;
}

/**********************************************************************************************************
*函数原型: void set_control_in(Radio_input_value radio_value,int16_t val)
*函数功能: 设定遥控器的控制输入值
*输入参数: radio_value：遥控器控制输入值
*返回数据: val:设定哪个控制通道的输入值的大小
*修改日期: 2018-12-17
*备注信息：
**********************************************************************************************************/
void set_control_in(Radio_input_value radio_value,int16_t val)
{
  radio_value.control_in=val;
}



/**********************************************************************************************************
*函数原型: void Radio_set_default_dead_zone(Radio_input_value radio_value,int16_t dzone)
*函数功能: 设置遥控通道范围
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-12-13
*备注信息：
**********************************************************************************************************/
void Radio_set_default_dead_zone(Radio_input_value radio_value,int16_t dzone)
{
  radio_value.dead_zone=dzone; 
}



/**********************************************************************************************************
*函数原型: void Radio_set_range(Radio_input_value radio_value,uint16_t high)
*函数功能: 设置遥控通道范围
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-12-13
*备注信息：
**********************************************************************************************************/
void Radio_set_range(Radio_input_value radio_value,uint16_t high)
{
  radio_value.type_in=RC_CHANNEL_TYPE_RANGE; //设定范围
  radio_value.high_in=high;
}

/**********************************************************************************************************
*函数原型: void Radio_set_angle(Radio_input_value radio_value,uint16_t angle)
*函数功能: 设置遥控通道角度范围
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-12-13
*备注信息：
**********************************************************************************************************/
void Radio_set_angle(Radio_input_value radio_value,uint16_t angle)
{
  radio_value.type_in=RC_CHANNEL_TYPE_ANGLE; //设定角度
  radio_value.high_in=angle;
}


/**********************************************************************************************************
*函数原型: int16_t Radio_pwm_to_range(Radio_input_value radio_value)
*函数功能: 遥控器PWM值转换成范围值
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-12-13
*备注信息：
**********************************************************************************************************/
int16_t Radio_pwm_to_range(Radio_input_value radio_value)
{

   return Radio_pwm_to_range_dz(radio_value,radio_value.dead_zone);

}

/**********************************************************************************************************
*函数原型: int16_t Radio_pwm_to_angle(Radio_input_value radio_value)
*函数功能: 遥控器PWM值转换成角度值
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-12-13
*备注信息：
**********************************************************************************************************/
int16_t Radio_pwm_to_angle(Radio_input_value radio_value)
{
 return Radio_pwm_to_angle_dz(radio_value,radio_value.dead_zone);
}


/**************************************************************************************************************************************************
*函数原型: int16_t Radio_pwm_to_range_dz(Radio_input_value radio_value,uint16_t _dead_zone)
*函数功能: 使用指定的死区将脉冲调制值转换为配置范围内的值
*输入参数: radio_value哪个遥控器通道值, _dead_zone:死区值
*返回数据: none
*修改日期: 2018-12-13
*备注信息：convert a pulse width modulation value to a value in the configured range, using the specified deadzone
**************************************************************************************************************************************************/
int16_t Radio_pwm_to_range_dz(Radio_input_value radio_value,uint16_t _dead_zone)
{
	  int16_t r_in;
    int16_t radio_trim_low;
    r_in = ConstrainInt16(radio_value.radio_in, radio_value.radio_min, radio_value.radio_max);//修改最大值,最小值

    if (radio_value.reversed) 
    {
	    r_in = radio_value.radio_max - (r_in - radio_value.radio_min); //修改最大值,最小值
    }

     radio_trim_low  = radio_value.radio_min + _dead_zone;

    if (r_in > radio_trim_low) 
    {
        return (((int32_t)(radio_value.high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_value.radio_max - radio_trim_low));
    }
    return 0;

}

/**************************************************************************************************************************************************
*函数原型: int16_t Radio_pwm_to_angle_dz(Radio_input_value radio_value,uint16_t _dead_zone)
*函数功能: pwm值转换成角度值
*输入参数: radio_value哪个遥控器通道值, _dead_zone:死区值
*返回数据: none
*修改日期: 2018-12-13
*备注信息：
**************************************************************************************************************************************************/
int16_t Radio_pwm_to_angle_dz(Radio_input_value radio_value,uint16_t _dead_zone)
{

  return Radio_pwm_to_angle_dz_trim( radio_value,_dead_zone,radio_value.radio_trim);


}

/******************************************************************************************************************************************************************************
*函数原型: int16_t Radio_pwm_to_angle_dz_trim(Radio_input_value radio_value,uint16_t _dead_zone, uint16_t _trim)
*函数功能: 从当前遥控器输入值处理后,返回一个角度值,单位是cm deg(范围是：-4500 +4500)；函数中使用了死区处理
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-12-13
*备注信息：return an "angle in centidegrees" (normally -4500 to 4500) from the current radio_in value using the specified dead_zone
******************************************************************************************************************************************************************************/
int16_t Radio_pwm_to_angle_dz_trim(Radio_input_value radio_value,uint16_t _dead_zone, uint16_t _trim)
{
    int16_t radio_trim_high = _trim + _dead_zone; //中立点加上死区上的值,得到一个死区的上边界值
    int16_t radio_trim_low  = _trim - _dead_zone; //中立点加上死区的下值,得到一个死区的下边界值

    int16_t reverse_mul = (radio_value.reversed?-1:1); //是否反转遥控器的输入值
    if (radio_value.radio_in > radio_trim_high && radio_value.radio_max != radio_trim_high) //大于最大上边界，并且不等于上边界(1*4500*(1800-1540)/(2000-1540))
		{
        return reverse_mul * ((int32_t)radio_value.high_in * (int32_t)(radio_value.radio_in - radio_trim_high)) / (int32_t)(radio_value.radio_max  - radio_trim_high);
    } 
		else if (radio_value.radio_in < radio_trim_low && radio_trim_low != radio_value.radio_min) //小于最小边界值,并且不等于最小边界值
		{
        return reverse_mul * ((int32_t)radio_value.high_in * (int32_t)(radio_value.radio_in - radio_trim_low)) / (int32_t)(radio_trim_low - radio_value.radio_min);
    } 
		else //否则就是反汇0,也就是中间值
		{
        return 0;
    }
}




/**********************************************************************************************************
*函数原型: float Radio_norm_input(Radio_input_value radio_value)
*函数功能: 归一化遥控器输入
*输入参数: radio_value:设定哪个通道
*返回数据: none
*修改日期: 2018-12-15
*备注信息：
**********************************************************************************************************/
float Radio_norm_input(Radio_input_value radio_value)
{
    float ret;
    int16_t reverse_mul = (radio_value.reversed?-1:1);
    if (radio_value.radio_in < radio_value.radio_trim) 
		{
        if (radio_value.radio_min >= radio_value.radio_trim) 
				{
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_value.radio_in - radio_value.radio_trim) / (float)(radio_value.radio_trim - radio_value.radio_min);
    } else 
		{
        if (radio_value.radio_max <= radio_value.radio_trim) 
				{
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_value.radio_in - radio_value.radio_trim) / (float)(radio_value.radio_max  - radio_value.radio_trim);
    }
    return ConstrainFloat(ret, -1.0f, 1.0f);

}


/**********************************************************************************************************
*函数原型: float Radio_norm_input_dz(Radio_input_value radio_value)
*函数功能: 归一化遥控器输入
*输入参数: radio_value:设定哪个通道
*返回数据: none
*修改日期: 2018-12-15
*备注信息：
**********************************************************************************************************/
float Radio_norm_input_dz(Radio_input_value radio_value)
{

    int16_t dz_min = radio_value.radio_trim - radio_value.dead_zone;
    int16_t dz_max = radio_value.radio_trim + radio_value.dead_zone;
    float ret;
    int16_t reverse_mul = (radio_value.reversed?-1:1);
    if (radio_value.radio_in < dz_min && dz_min > radio_value.radio_min) 
		{
        ret = reverse_mul * (float)(radio_value.radio_in - dz_min) / (float)(dz_min - radio_value.radio_min);
    } else if (radio_value.radio_in > dz_max && radio_value.radio_max > dz_max) 
		{
        ret = reverse_mul * (float)(radio_value.radio_in - dz_max) / (float)(radio_value.radio_max  - dz_max);
    } else 
		{
        ret = 0;
    }
    return ConstrainFloat(ret, -1.0f, 1.0f);

}

/*****************************************************************************************************************************
*函数原型: float Radio_percent_input(Radio_input_value radio_value)
*函数功能: 获取遥控器输入的百分比
*输入参数: radio_value:设定哪个通道
*返回数据: none
*修改日期: 2018-12-15
*备注信息：get percentage input from 0 to 100. This ignores the trim value.
*****************************************************************************************************************************/
float Radio_percent_input(Radio_input_value radio_value)
{
	   uint8_t ret ;
     if (radio_value.radio_in <= radio_value.radio_min) 
		{
        return radio_value.reversed?100:0;
    }
    if (radio_value.radio_in >= radio_value.radio_max) 
		{
        return radio_value.reversed?0:100;
    }
    ret = 100.0f * (radio_value.radio_in - radio_value.radio_min) / (float)(radio_value.radio_max - radio_value.radio_min);
    if (radio_value.reversed) 
		{
        ret = 100 - ret;
    }
    return ret;

}




/***********************************************************************************************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************************************************************************************/