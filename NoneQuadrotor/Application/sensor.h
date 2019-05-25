#ifndef __SENSOR__H__
#define __SENSOR__H__

#include "sys.h"
#include "vector3.h"

	
#define IMU1_SENSOR  1
#define IMU2_SENSOR  1
#define IMU3_SENSOR  1

typedef struct 
{
	//第一组IMU数据---icm20602
	Vector3f_t acc1;
	Vector3f_t gyro1;
	Vector3f_t mag1;
	//第二组IMU数据---icm20689
  Vector3f_t acc2;
	Vector3f_t gyro2;
	Vector3f_t mag2;
	//第三组IMU数据---bmi055
	Vector3f_t acc3;
	Vector3f_t gyro3;
	Vector3f_t mag3;
	
	//气压计数据
  float   baro;
	//位置信息
	Vector3f_t position;
	
} IMU_SENSOR_t;


extern IMU_SENSOR_t imu_sensor;






#endif

