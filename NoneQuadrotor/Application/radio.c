/**********************************************************************************************************
*�ļ�˵����ң��������ӿ�
*ʵ�ֹ��ܣ�ͳһң��������ӿ�
*�޸����ڣ�2018-11-17
*�޸����ߣ�crystal cup
*�޸ı�ע��
**********************************************************************************************************/
#include "radio.h"
#include "copter.h"


RADIODATA_t sbusData;
enum ChannelType type_in;
enum ChannelMap radio_channel_map;  //ң����ͨ��ӳ��
//��Ҫ������������,֧��1,2,3,4ͨ�����ã����,����,����,ƫ��ͨ��
Radio_input_value radio_channel_roll;     //1ͨ��
Radio_input_value radio_channel_pitch;    //2ͨ��
Radio_input_value radio_channel_throttle; //3ͨ��
Radio_input_value radio_channel_yaw;      //4ͨ��
Radio_input_value radio_channel5;         //5ͨ��
Radio_input_value radio_channel6;         //6ͨ��
Radio_input_value radio_channel7;         //7ͨ��
Radio_input_value radio_channel8;         //8ͨ��
/**********************************************************************************************************
*����ԭ��: void Radio_init_in(void)
*��������: ����ң��ͨ���ķ�Χ
*�������: none
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��
**********************************************************************************************************/
void Radio_init_in(void)
{
	  Sbus_Init();  //��ʼ��Sbusң����
	
	 //�趨1--8ͨ�������̷�Χ
   Radio_set_angle(radio_channel_roll,ROLL_PITCH_YAW_INPUT_MAX); //���ú��ͨ�������ֵ��4500
   Radio_set_angle(radio_channel_pitch,ROLL_PITCH_YAW_INPUT_MAX);//���ø���ͨ�������ֵ��4500
	 Radio_set_angle(radio_channel_yaw,ROLL_PITCH_YAW_INPUT_MAX);  //����ƫ��ͨ�������ֵ��4500
   Radio_set_range(radio_channel_throttle,1000);                 //��������ͨ�������ֵ��1000
   Radio_set_range(radio_channel5,1000);                         //����5ͨ�������ֵ��1000
	 Radio_set_range(radio_channel6,1000);                         //����6ͨ�������ֵ��1000
   Radio_set_range(radio_channel7,1000);                         //����7ͨ�������ֵ��1000
	 Radio_set_range(radio_channel8,1000);                         //����8ͨ�������ֵ��1000
	 //����1--8ͨ����������Χ
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
*����ԭ��: void Radio_Update(void)
*��������: ����ң������������
*�������: none
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��
**********************************************************************************************************/

void Radio_Update(void)
{
  //��Ҫ��ң��������
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
	
	//����������
	radio_channel_roll.radio_in=sbus_get_data(RADIO_CH_ROLL);
  radio_channel_pitch.radio_in=sbus_get_data(RADIO_CH_PITCH);
	radio_channel_throttle.radio_in=sbus_get_data(RADIO_CH_THROTTLE);
  radio_channel_yaw.radio_in=sbus_get_data(RADIO_CH_YAW);
  radio_channel5.radio_in=sbus_get_data(CH_5);
  radio_channel6.radio_in=sbus_get_data(CH_6);
	radio_channel7.radio_in=sbus_get_data(CH_7);
  radio_channel8.radio_in=sbus_get_data(CH_8);
	//���ú������������
	if(radio_channel_roll.type_in==RC_CHANNEL_TYPE_ANGLE)
	{
	 radio_channel_roll.control_in=Radio_pwm_to_angle(radio_channel_roll);
	
	}
	else if(radio_channel_roll.type_in==RC_CHANNEL_TYPE_RANGE)
	{
	 radio_channel_roll.control_in=Radio_pwm_to_range(radio_channel_roll);
	}

  	//���ø�������������
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
*����ԭ��: int16_t get_control_in(Radio_input_value radio_value)
*��������: ���ؿ�������ֵ
*�������: radio_value��ң������������ֵ
*��������: control_in:�ĸ�����ͨ��������ֵ
*�޸�����: 2018-12-13
*��ע��Ϣ��
**********************************************************************************************************/
int16_t get_control_in(Radio_input_value radio_value)
{
  return radio_value.control_in;
}

/**********************************************************************************************************
*����ԭ��: void set_control_in(Radio_input_value radio_value,int16_t val)
*��������: �趨ң�����Ŀ�������ֵ
*�������: radio_value��ң������������ֵ
*��������: val:�趨�ĸ�����ͨ��������ֵ�Ĵ�С
*�޸�����: 2018-12-17
*��ע��Ϣ��
**********************************************************************************************************/
void set_control_in(Radio_input_value radio_value,int16_t val)
{
  radio_value.control_in=val;
}



/**********************************************************************************************************
*����ԭ��: void Radio_set_default_dead_zone(Radio_input_value radio_value,int16_t dzone)
*��������: ����ң��ͨ����Χ
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��
**********************************************************************************************************/
void Radio_set_default_dead_zone(Radio_input_value radio_value,int16_t dzone)
{
  radio_value.dead_zone=dzone; 
}



/**********************************************************************************************************
*����ԭ��: void Radio_set_range(Radio_input_value radio_value,uint16_t high)
*��������: ����ң��ͨ����Χ
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��
**********************************************************************************************************/
void Radio_set_range(Radio_input_value radio_value,uint16_t high)
{
  radio_value.type_in=RC_CHANNEL_TYPE_RANGE; //�趨��Χ
  radio_value.high_in=high;
}

/**********************************************************************************************************
*����ԭ��: void Radio_set_angle(Radio_input_value radio_value,uint16_t angle)
*��������: ����ң��ͨ���Ƕȷ�Χ
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��
**********************************************************************************************************/
void Radio_set_angle(Radio_input_value radio_value,uint16_t angle)
{
  radio_value.type_in=RC_CHANNEL_TYPE_ANGLE; //�趨�Ƕ�
  radio_value.high_in=angle;
}


/**********************************************************************************************************
*����ԭ��: int16_t Radio_pwm_to_range(Radio_input_value radio_value)
*��������: ң����PWMֵת���ɷ�Χֵ
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��
**********************************************************************************************************/
int16_t Radio_pwm_to_range(Radio_input_value radio_value)
{

   return Radio_pwm_to_range_dz(radio_value,radio_value.dead_zone);

}

/**********************************************************************************************************
*����ԭ��: int16_t Radio_pwm_to_angle(Radio_input_value radio_value)
*��������: ң����PWMֵת���ɽǶ�ֵ
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��
**********************************************************************************************************/
int16_t Radio_pwm_to_angle(Radio_input_value radio_value)
{
 return Radio_pwm_to_angle_dz(radio_value,radio_value.dead_zone);
}


/**************************************************************************************************************************************************
*����ԭ��: int16_t Radio_pwm_to_range_dz(Radio_input_value radio_value,uint16_t _dead_zone)
*��������: ʹ��ָ�����������������ֵת��Ϊ���÷�Χ�ڵ�ֵ
*�������: radio_value�ĸ�ң����ͨ��ֵ, _dead_zone:����ֵ
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��convert a pulse width modulation value to a value in the configured range, using the specified deadzone
**************************************************************************************************************************************************/
int16_t Radio_pwm_to_range_dz(Radio_input_value radio_value,uint16_t _dead_zone)
{
	  int16_t r_in;
    int16_t radio_trim_low;
    r_in = ConstrainInt16(radio_value.radio_in, radio_value.radio_min, radio_value.radio_max);//�޸����ֵ,��Сֵ

    if (radio_value.reversed) 
    {
	    r_in = radio_value.radio_max - (r_in - radio_value.radio_min); //�޸����ֵ,��Сֵ
    }

     radio_trim_low  = radio_value.radio_min + _dead_zone;

    if (r_in > radio_trim_low) 
    {
        return (((int32_t)(radio_value.high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_value.radio_max - radio_trim_low));
    }
    return 0;

}

/**************************************************************************************************************************************************
*����ԭ��: int16_t Radio_pwm_to_angle_dz(Radio_input_value radio_value,uint16_t _dead_zone)
*��������: pwmֵת���ɽǶ�ֵ
*�������: radio_value�ĸ�ң����ͨ��ֵ, _dead_zone:����ֵ
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��
**************************************************************************************************************************************************/
int16_t Radio_pwm_to_angle_dz(Radio_input_value radio_value,uint16_t _dead_zone)
{

  return Radio_pwm_to_angle_dz_trim( radio_value,_dead_zone,radio_value.radio_trim);


}

/******************************************************************************************************************************************************************************
*����ԭ��: int16_t Radio_pwm_to_angle_dz_trim(Radio_input_value radio_value,uint16_t _dead_zone, uint16_t _trim)
*��������: �ӵ�ǰң��������ֵ�����,����һ���Ƕ�ֵ,��λ��cm deg(��Χ�ǣ�-4500 +4500)��������ʹ������������
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-13
*��ע��Ϣ��return an "angle in centidegrees" (normally -4500 to 4500) from the current radio_in value using the specified dead_zone
******************************************************************************************************************************************************************************/
int16_t Radio_pwm_to_angle_dz_trim(Radio_input_value radio_value,uint16_t _dead_zone, uint16_t _trim)
{
    int16_t radio_trim_high = _trim + _dead_zone; //��������������ϵ�ֵ,�õ�һ���������ϱ߽�ֵ
    int16_t radio_trim_low  = _trim - _dead_zone; //�����������������ֵ,�õ�һ���������±߽�ֵ

    int16_t reverse_mul = (radio_value.reversed?-1:1); //�Ƿ�תң����������ֵ
    if (radio_value.radio_in > radio_trim_high && radio_value.radio_max != radio_trim_high) //��������ϱ߽磬���Ҳ������ϱ߽�(1*4500*(1800-1540)/(2000-1540))
		{
        return reverse_mul * ((int32_t)radio_value.high_in * (int32_t)(radio_value.radio_in - radio_trim_high)) / (int32_t)(radio_value.radio_max  - radio_trim_high);
    } 
		else if (radio_value.radio_in < radio_trim_low && radio_trim_low != radio_value.radio_min) //С����С�߽�ֵ,���Ҳ�������С�߽�ֵ
		{
        return reverse_mul * ((int32_t)radio_value.high_in * (int32_t)(radio_value.radio_in - radio_trim_low)) / (int32_t)(radio_trim_low - radio_value.radio_min);
    } 
		else //������Ƿ���0,Ҳ�����м�ֵ
		{
        return 0;
    }
}




/**********************************************************************************************************
*����ԭ��: float Radio_norm_input(Radio_input_value radio_value)
*��������: ��һ��ң��������
*�������: radio_value:�趨�ĸ�ͨ��
*��������: none
*�޸�����: 2018-12-15
*��ע��Ϣ��
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
*����ԭ��: float Radio_norm_input_dz(Radio_input_value radio_value)
*��������: ��һ��ң��������
*�������: radio_value:�趨�ĸ�ͨ��
*��������: none
*�޸�����: 2018-12-15
*��ע��Ϣ��
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
*����ԭ��: float Radio_percent_input(Radio_input_value radio_value)
*��������: ��ȡң��������İٷֱ�
*�������: radio_value:�趨�ĸ�ͨ��
*��������: none
*�޸�����: 2018-12-15
*��ע��Ϣ��get percentage input from 0 to 100. This ignores the trim value.
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