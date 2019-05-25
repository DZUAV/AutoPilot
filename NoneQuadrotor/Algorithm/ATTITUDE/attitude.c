/**********************************************************************************************************
*文件说明：姿态解锁文件
*实现功能：实现无人机姿态解锁
*修改日期：2018-12-1
*修改作者：crystal cup
*修改备注：应用程序入口
**********************************************************************************************************/
#include "attitude.h"
#include "copter.h"

ref_t 	ref;

/*==========================================================================================================================================*/
/*==========================================================================================================================================*
**函数原型: void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
**函数功能: 数据融合与姿态解算
**输    入: sample_half_T:一半的采样时间
               acc_data : 加速度数据
               gyro_data: 三轴陀螺仪数据
							 mag_data :  三轴地磁数据
**输    出: angle：姿态角
**备    注: 
**=======================================================================================================================================*/
/*======================================================================================================================================*/
void IMUupdate(float sample_half_T,Vector3f_t acc_data, Vector3f_t gyro_data,Vector3f_t mag_data,Vector3f_t angle) 
{		
		float ref_err_lpf_hz;
	  float mag_norm ,mag_norm_xyz ;
		static float yaw_correct;
		float mag_norm_tmp;
		static Vector3f_t mag_tmp;
		static float yaw_mag;
	  Vector3f_t reference_v;
	  Vector3f_t mag_sim_3d;
	  float ref_q[4] = {1,0,0,0};
	  float norm_acc,norm_q;
    float norm_acc_lpf;
	  mag_norm_tmp = 20 *(6.28f *sample_half_T);	  //20*6.28*1.7=213
		mag_norm_xyz = my_sqrt(	mag_data.x *mag_data.x+ mag_data.y*mag_data.y + mag_data.z*mag_data.z);
	
		if( mag_norm_xyz != 0) //磁力计进行低通滤波
	 {
		mag_tmp.x += mag_norm_tmp *( (float)	mag_data.x /( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( (float)	mag_data.y/( mag_norm_xyz ) - mag_tmp.y);	
		mag_tmp.z += mag_norm_tmp *( (float)	mag_data.z/( mag_norm_xyz ) - mag_tmp.z);	
	 }
	
	 simple_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d);
	
	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);
	
	if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		yaw_mag = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f;
		
	}
	

	//=============================================================================
  //计算等效重力
	reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	
	


	//计算加速度向量的模
	norm_acc = my_sqrt(acc_data.x*acc_data.x + acc_data.y*acc_data.y + acc_data.z*acc_data.z);   
	norm_acc_lpf +=  NORM_ACC_LPF_HZ *(6.28f *sample_half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001


	if(ABS(acc_data.x )<4400 && ABS(acc_data.y )<4400 && ABS(acc_data.z )<4400 )
	{	
		//把加速度的三维向量转换成单位向量
		acc_data.x  = acc_data.x / norm_acc;//4096.0f;
		acc_data.y  = acc_data.y / norm_acc;//4096.0f;
		acc_data.z  = acc_data.z / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* 叉乘得到误差 */
			ref.err_tmp.x = acc_data.y *reference_v.z - acc_data.z *reference_v.y;
			ref.err_tmp.y = acc_data.z*reference_v.x - acc_data.x*reference_v.z;
	  
			
			/* 误差进行低通滤波 */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *sample_half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );

			
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//

		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;

	}
	/* 误差积分 */
	ref.err_Int.x += ref.err.x *Ki *2 *sample_half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *sample_half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *sample_half_T ;
	
	/* 积分限制幅度 */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	


	
	ref.g.x = (gyro_data.x - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gyro_data.y - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = (gyro_data.z - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;
	
	/* 用叉积误差来坐PI修正陀螺仪零偏和正交化 */

	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*sample_half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*sample_half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*sample_half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*sample_half_T;  

	/* 四元数归一化 */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;
	
	//得到姿态角度
	angle.x= fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	angle.y = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
    angle.z= yaw_mag;
}






