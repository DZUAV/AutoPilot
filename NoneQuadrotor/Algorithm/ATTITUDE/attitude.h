#ifndef __ATTITUDE__H__
#define __ATTITUDE__H__

#include "sys.h"
#include "vector3.h"

#define Kp 0.6f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.1f                	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	?????
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define ABS(x) ( (x)>0?(x):-(x) )
typedef struct 
{
	Vector3f_t err;
	Vector3f_t err_tmp;
	Vector3f_t err_lpf;
	Vector3f_t err_Int;
	Vector3f_t g;
	
}ref_t;


void IMUupdate(float sample_half_T,Vector3f_t acc_data, Vector3f_t gyro_data,Vector3f_t mag_data,Vector3f_t angle) ;

#endif

