/**********************************************************************************************************
*文件说明：quaternion.c
*实现功能：四元数相关运算，不同坐标系和旋转方向定义，公式的具体形式有所区别
*修改日期：2018-10-31
*修改作者：cihang_uav
*备注信息:
**********************************************************************************************************/
#include "quaternion.h"



/**********************************************************************************************************
*函数原型: void EulerAngleToQuaternion(Vector3f_t angle, float q[4])
*函数功能: 欧拉角转四元数
*输入形参: angle：欧拉角；q[4]：四元数
*返回数据: none
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/
void EulerAngleToQuaternion(Vector3f_t angle, float q[4])
{
    /*方法1*/
    float sinx = sinf(angle.x / 2);
    float cosx = cosf(angle.x / 2);
    float siny = sinf(angle.y / 2);
    float cosy = cosf(angle.y / 2);
    float sinz = sinf(angle.z / 2);
    float cosz = cosf(angle.z / 2);
    
    q[0] = cosx * cosy * cosz - sinx * siny * sinz;
    q[1] = -sinx * cosy * cosz - cosx * siny * sinz;
    q[2] = -cosx * siny * cosz + sinx * cosy * sinz;
    q[3] = cosx * cosy * sinz + sinx * siny * cosz;
    
    /*方法2*/
//    float dcM[9];
//    EulerAngleToDCM(angle, dcM);
//    
//    q[0] = 0.5f * sqrtf(1 + dcM[0] + dcM[4] + dcM[8]);
//    q[1] = (dcM[7] - dcM[5]) / (4 * q[0]);
//    q[2] = (dcM[2] - dcM[6]) / (4 * q[0]);
//    q[3] = (dcM[3] - dcM[1]) / (4 * q[0]);
}


/**********************************************************************************************************
*函数原型: void QuaternionToDCM(float q[4], float dcM[9])
*函数功能: 四元数转方向余弦矩阵(表示从参考系到机体系)
*输入形参: q[4]:四元数 dcM[9]:方向余弦矩阵
*返回数据: none
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/
void QuaternionToDCM(float q[4], float dcM[9])
{
    float q0q0 = q[0] * q [0];
    float q0q1 = q[0] * q [1]; 
    float q0q2 = q[0] * q [2]; 
    float q0q3 = q[0] * q [3];
    float q1q1 = q[1] * q [1];   
    float q1q2 = q[1] * q [2];  
    float q1q3 = q[1] * q [3];  
    float q2q2 = q[2] * q [2];
    float q2q3 = q[2] * q [3];
    float q3q3 = q[3] * q [3];
    
    dcM[0] = q0q0 + q1q1 - q2q2 - q3q3;
    dcM[1] = 2 * (q1q2 + q0q3);
    dcM[2] = 2 * (q0q2 + q1q3);
    dcM[3] = 2 * (q1q2 - q0q3);
    dcM[4] = q0q0 - q1q1 + q2q2 - q3q3;
    dcM[5] = 2 * (q2q3 - q0q1);
    dcM[6] = 2 * (q1q3 - q0q2);
    dcM[7] = 2 * (q0q1 + q2q3);
    dcM[8] = q0q0 - q1q1 - q2q2 + q3q3; 
}

/**********************************************************************************************************
*函数原型: void QuaternionToDCM_T(float q[4], float dcM[9])
*函数功能: 四元数转方向余弦矩阵(表示从机体系到参考系)
*输入形参: q[4]:四元数 dcM[9]:方向余弦矩阵
*返回数据: none
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/

void QuaternionToDCM_T(float q[4], float dcM[9])
{
    float q0q0 = q[0] * q [0];
    float q0q1 = q[0] * q [1]; 
    float q0q2 = q[0] * q [2]; 
    float q0q3 = q[0] * q [3];
    float q1q1 = q[1] * q [1];   
    float q1q2 = q[1] * q [2];  
    float q1q3 = q[1] * q [3];  
    float q2q2 = q[2] * q [2];
    float q2q3 = q[2] * q [3];
    float q3q3 = q[3] * q [3];
     
    dcM[0] = q0q0 + q1q1 - q2q2 - q3q3;
    dcM[1] = 2 * (q1q2 - q0q3);
    dcM[2] = 2 * (q1q3 - q0q2);
    dcM[3] = 2 * (q1q2 + q0q3);
    dcM[4] = q0q0 - q1q1 + q2q2 - q3q3;
    dcM[5] = 2 * (q0q1 + q2q3);
    dcM[6] = 2 * (q0q2 + q1q3);
    dcM[7] = 2 * (q2q3 - q0q1);
    dcM[8] = q0q0 - q1q1 - q2q2 + q3q3;
}

/**********************************************************************************************************
*函数原型: Vector3f_t QuaternionRotateToEarthFrame(float q[4], Vector3f_t vector)
*函数功能: 使用四元数旋转一个向量（机体系到导航系）
*输入形参: q[4]:四元数 vector:方向向量
*返回数据: none
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/

Vector3f_t QuaternionRotateToEarthFrame(float q[4], Vector3f_t vector)
{
    static float dcM[9];
    QuaternionToDCM_T(q, dcM);
    return Matrix3MulVector3(dcM, vector);
}


/**********************************************************************************************************
*函数原型: Vector3f_t QuaternionRotateToBodyFrame(float q[4], Vector3f_t vector)
*函数功能: 使用四元数旋转一个向量（导航系到机体系）
*输入形参: q[4]:四元数 vector:方向向量
*返回数据: none
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/

Vector3f_t QuaternionRotateToBodyFrame(float q[4], Vector3f_t vector)
{
    static float dcM[9];
    QuaternionToDCM(q, dcM);
    return Matrix3MulVector3(dcM, vector);
}


/**********************************************************************************************************
*函数原型: void QuaternionToEulerAngle(float q[4], Vector3f_t* angle)
*函数功能: 四元数转欧拉角
*输入形参: q[4]:四元数 angle:欧拉角
*返回数据: none
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/
void QuaternionToEulerAngle(float q[4], Vector3f_t* angle)
{
    static float dcM[9];
    
    QuaternionToDCM(q, dcM);

	angle->x = -asinf(-dcM[7]);           
	angle->y = atan2f(-dcM[6], dcM[8]); 
    angle->z = -atan2f(dcM[3], dcM[4]);
}


/**********************************************************************************************************
*函数原型: void QuaternionNormalize(float q[4])
*函数功能: 四元数归一化
*输入形参: q[4]:四元数 
*返回数据: none
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/

void QuaternionNormalize(float q[4])
{
    float qMag = Pythagorous4(q[0], q[1], q[2], q[3]);
    
    q[0] /= qMag;
    q[1] /= qMag;
    q[2] /= qMag;
    q[3] /= qMag;
}



/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/





























