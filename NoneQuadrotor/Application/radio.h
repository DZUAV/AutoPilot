#ifndef __RADIO__H__
#define __RADIO__H__
#include "sys.h"


# define ROLL_PITCH_YAW_INPUT_MAX      4500        // roll, pitch and yaw input range
#define CH_1 1
#define CH_2 2
#define CH_3 3
#define CH_4 4
#define CH_5 5
#define CH_6 6
#define CH_7 7
#define CH_8 8
#define CH_9 9
#define CH_10 10
#define CH_11 11
#define CH_12 12
#define CH_NONE 255





enum ChannelMap   //ң����ͨ������
{
  RADIO_CH_ROLL=1,
	RADIO_CH_PITCH,
	RADIO_CH_THROTTLE,
	RADIO_CH_YAW,

};
extern enum ChannelMap radio_channel_map;
enum ChannelType   //ң����ͨ������
{
    RC_CHANNEL_TYPE_ANGLE = 0,
    RC_CHANNEL_TYPE_RANGE = 1,
};
extern enum ChannelType type_in;
typedef struct
{
    int channel_value[12];

} RADIODATA_t;
extern RADIODATA_t sbusData;

typedef struct
{
	//PWM��������
  int16_t radio_in;
	//ֵ�γɴ�pwm��һ��������ϵ��
  int16_t    control_in;
	//ң����������Сֵ
  int16_t    radio_min;
	//ң���������м�ֵ
  int16_t    radio_trim;
	//ң�����������ֵ
  int16_t    radio_max;
	//������,Ҳ�����������ֵ
	int16_t     high_in;
	//��������
	int16_t     dead_zone;
	//�Ƿ����ң������ֵ��ת
	int8_t   reversed ;
	//����ң��������
	uint8_t  type_in;
	

}Radio_input_value;

extern Radio_input_value radio_channel_roll;     //1ͨ��
extern Radio_input_value radio_channel_pitch;    //2ͨ��
extern Radio_input_value radio_channel_throttle; //3ͨ��
extern Radio_input_value radio_channel_yaw;      //4ͨ��
extern Radio_input_value radio_channel5;         //5ͨ��
extern Radio_input_value radio_channel6;         //6ͨ��
extern Radio_input_value radio_channel7;         //7ͨ��
extern Radio_input_value radio_channel8;         //8ͨ��

void Radio_init_in(void);
int16_t Radio_pwm_to_range(Radio_input_value radio_value);
int16_t Radio_pwm_to_angle(Radio_input_value radio_value);
int16_t Radio_pwm_to_range_dz(Radio_input_value radio_value,uint16_t _dead_zone);
int16_t Radio_pwm_to_angle_dz(Radio_input_value radio_value,uint16_t _dead_zone);
int16_t Radio_pwm_to_angle_dz_trim(Radio_input_value radio_value,uint16_t _dead_zone, uint16_t _trim);
void Radio_set_range(Radio_input_value radio_value,uint16_t high);
void Radio_set_angle(Radio_input_value radio_value,uint16_t angle);
void Radio_set_default_dead_zone(Radio_input_value radio_value,int16_t dzone);
float Radio_norm_input(Radio_input_value radio_value);
float Radio_norm_input_dz(Radio_input_value radio_value);
int16_t get_control_in(Radio_input_value radio_value);
void set_control_in(Radio_input_value radio_value,int16_t val);
#endif
