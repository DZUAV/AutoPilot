#ifndef __SENSOR__H__
#define __SENSOR__H__

#include "sys.h"
#include "vector3.h"

	
#define IMU1_SENSOR  1
#define IMU2_SENSOR  1
#define IMU3_SENSOR  1

typedef struct 
{
	//��һ��IMU����---icm20602
	Vector3f_t acc1;
	Vector3f_t gyro1;
	Vector3f_t mag1;
	//�ڶ���IMU����---icm20689
  Vector3f_t acc2;
	Vector3f_t gyro2;
	Vector3f_t mag2;
	//������IMU����---bmi055
	Vector3f_t acc3;
	Vector3f_t gyro3;
	Vector3f_t mag3;
	
	//��ѹ������
  float   baro;
	//λ����Ϣ
	Vector3f_t position;
	
} IMU_SENSOR_t;


extern IMU_SENSOR_t imu_sensor;






#endif

