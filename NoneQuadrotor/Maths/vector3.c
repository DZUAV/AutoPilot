/**********************************************************************************************************
*文件说明：vector3文件实现
*实现功能：3维向量结构体的定义与相关函数
*修改日期：2018-10-31
*修改作者：
*修改备注：
**********************************************************************************************************/

#include "vector3.h"




/**********************************************************************************************************
*函数原型：void Vector3f_Normalize(Vector3f_t* vector)
*函数功能：三维向量的归一化
*输入形参：vector：获取输入向量
*返回数据：none
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
void Vector3f_Normalize(Vector3f_t* vector)
{
    float mag = Pythagorous3(vector->x, vector->y, vector->z);
    
    vector->x /= mag;
    vector->y /= mag;
    vector->z /= mag;
}


/**********************************************************************************************************
*函数原型：Vector3f_t Vector3iTo3f(Vector3i_t vector)
*函数功能：三维向量类型转换，转换成浮点类型
*输入形参：vector：获取输入向量
*返回数据：v_tmp
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
Vector3f_t Vector3iTo3f(Vector3i_t vector)
{
	Vector3f_t v_tmp;
	v_tmp.x = (float)vector.x;
	v_tmp.y = (float)vector.y;	
	v_tmp.z = (float)vector.z;	
	return v_tmp;
}

/**********************************************************************************************************
*函数原型：Vector3i_t Vector3fTo3i(Vector3f_t vector)
*函数功能：三维向量类型转换,浮点转换成整形
*输入形参：vector：获取输入向量
*返回数据：v_tmp
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
Vector3i_t Vector3fTo3i(Vector3f_t vector)
{
	Vector3i_t v_tmp;
	v_tmp.x = (int16_t)vector.x;
	v_tmp.y = (int16_t)vector.y;	
	v_tmp.z = (int16_t)vector.z;	
	return v_tmp;
}

/**********************************************************************************************************
*函数原型：Vector3f_t Vector3f_Add(Vector3f_t v1, Vector3f_t v2)
*函数功能：三维向量加法
*输入形参：v1,v2
*返回数据：v_tmp
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
Vector3f_t Vector3f_Add(Vector3f_t v1, Vector3f_t v2)
{
    Vector3f_t v_tmp;
    
    v_tmp.x = v1.x + v2.x;
    v_tmp.y = v1.y + v2.y;
    v_tmp.z = v1.z + v2.z;
    
    return v_tmp;
}

/**********************************************************************************************************
*函数原型：Vector3f_t Vector3f_Sub(Vector3f_t v1, Vector3f_t v2)
*函数功能：三维向量减法
*输入形参：vector：获取输入向量
*返回数据：v1,v2
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
Vector3f_t Vector3f_Sub(Vector3f_t v1, Vector3f_t v2)
{
    Vector3f_t v_tmp;
    
    v_tmp.x = v1.x - v2.x;
    v_tmp.y = v1.y - v2.y;
    v_tmp.z = v1.z - v2.z;
    
    return v_tmp;
}

/**********************************************************************************************************
*函数原型：Vector3f_t VectorCrossProduct(Vector3f_t a, Vector3f_t b)
*函数功能：三维向量叉乘运算
*输入形参：a,b
*返回数据：c
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
Vector3f_t VectorCrossProduct(Vector3f_t a, Vector3f_t b)
{
	Vector3f_t c;
	
	c.x = a.y * b.z - b.y * a.z;
	c.y = a.z * b.x - b.z * a.x;
	c.z = a.x * b.y - b.x * a.y;
	
	return c;
}

/**********************************************************************************************************
*函数原型：Vector3f_t Matrix3MulVector3(float* m, Vector3f_t vector)
*函数功能：三维矩阵与三维向量相乘
*输入形参：m：三维矩阵；vector：三维向量
*返回数据：v_tmp
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
Vector3f_t Matrix3MulVector3(float* m, Vector3f_t vector)
{
    Vector3f_t v_tmp;
    
    v_tmp.x = m[0] * vector.x + m[1] * vector.y + m[2] * vector.z;
    v_tmp.y = m[3] * vector.x + m[4] * vector.y + m[5] * vector.z;
    v_tmp.z = m[6] * vector.x + m[7] * vector.y + m[8] * vector.z;
    
    return v_tmp;
}

/**********************************************************************************************************
*函数原型：void EulerAngleToDCM(Vector3f_t angle, float* dcM)
*函数功能：欧拉角转换成方向余弦(参考系到机体坐标系)
*输入形参：angle：输入欧拉角；dcM：矩阵
*返回数据：none
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
void EulerAngleToDCM(Vector3f_t angle, float* dcM)
{
    Vector3f_t cos, sin;
    
    cos.x = cosf(angle.x);
    cos.y = cosf(angle.y);
    cos.z = cosf(angle.z);   
    sin.x = sinf(angle.x);
    sin.y = sinf(angle.y);
    sin.z = sinf(angle.z);    

    dcM[0] = cos.y * cos.z; 
    dcM[1] = sin.z * cos.x + sin.x * sin.y * cos.z; 
    dcM[2] = -sin.x * sin.z + sin.y * cos.x * cos.z; 
    dcM[3] = -sin.z * cos.y;
    dcM[4] = cos.x * cos.z - sin.x * sin.y * sin.z;
    dcM[5] = -sin.x * cos.z - sin.y * sin.z * cos.x; 
    dcM[6] = -sin.y;
    dcM[7] = sin.x * cos.y;
    dcM[8] = cos.x * cos.y;
}

/**********************************************************************************************************
*函数原型：void EulerAngleToDCM_T(Vector3f_t angle, float* dcM)
*函数功能：欧拉角转换成方向余弦(机体坐标系到参考坐标系)
*输入形参：angle：输入欧拉角；dcM：矩阵
*返回数据：none
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
void EulerAngleToDCM_T(Vector3f_t angle, float* dcM)
{
    Vector3f_t cos, sin;
    
    cos.x = cosf(angle.x);
    cos.y = cosf(angle.y);
    cos.z = cosf(angle.z);   
    sin.x = sinf(angle.x);
    sin.y = sinf(angle.y);
    sin.z = sinf(angle.z);    

    dcM[0] = cos.y * cos.z; 
    dcM[1] = -sin.z * cos.y;
    dcM[2] = -sin.y;
    dcM[3] = sin.z * cos.x + sin.x * sin.y * cos.z; 
    dcM[4] = cos.x * cos.z - sin.x * sin.y * sin.z;
    dcM[5] = sin.x * cos.y;
    dcM[6] = -sin.x * sin.z + sin.y * cos.x * cos.z; 
    dcM[7] = -sin.x * cos.z - sin.y * sin.z * cos.x;
    dcM[8] = cos.x * cos.y;
}

/**********************************************************************************************************
*函数原型：Vector3f_t VectorRotateToBodyFrame(Vector3f_t vector, Vector3f_t deltaAngle)
*函数功能：三维向量旋转到机体坐标系
*输入形参：vector：三维向量；deltaAngle：角度变化量
*返回数据：
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
Vector3f_t VectorRotateToBodyFrame(Vector3f_t vector, Vector3f_t deltaAngle)
{
    float dcMat[9];

    //欧拉角转换为方向余弦矩阵
    EulerAngleToDCM(deltaAngle, dcMat);

    //方向余弦矩阵乘以向量,得到旋转后的新向量
    return Matrix3MulVector3(dcMat, vector);
}

/**********************************************************************************************************
*函数原型：Vector3f_t VectorRotateToEarthFrame(Vector3f_t vector, Vector3f_t deltaAngle)
*函数功能：三维向量旋转到参考坐标系
*输入形参：vector：三维向量；deltaAngle：角度变化量
*返回数据：
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
Vector3f_t VectorRotateToEarthFrame(Vector3f_t vector, Vector3f_t deltaAngle)
{
    float dcMat[9];

    //欧拉角转换为方向余弦矩阵
    EulerAngleToDCM_T(deltaAngle, dcMat);

    //方向余弦矩阵乘以向量,得到旋转后的新向量
    return Matrix3MulVector3(dcMat, vector);
}

/**********************************************************************************************************
*函数原型：void AccVectorToRollPitchAngle(Vector3f_t* angle, Vector3f_t vector)
*函数功能：根据重力加速度向量在机体坐标系上的投影计算横滚和俯仰角度
*输入形参：angle：角度；vector：三维向量
*返回数据：angle
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
void AccVectorToRollPitchAngle(Vector3f_t* angle, Vector3f_t vector)
{
    //向量归一化
    Vector3f_Normalize(&vector);
    
	angle->x = -SafeArcsin(vector.y);       //横滚
	angle->y = atan2f(vector.x, vector.z);  //俯仰
}

/**********************************************************************************************************
*函数原型：void MagVectorToYawAngle(Vector3f_t* angle, Vector3f_t vector)
*函数功能：根据地磁场向量在机体坐标系上的投影计算偏航角
*输入形参：vector：获取输入向量
*返回数据：v_tmp
*修改日期：2018-10-31
*修改备注：
**********************************************************************************************************/
void MagVectorToYawAngle(Vector3f_t* angle, Vector3f_t vector)
{
	angle->z = -atan2f(vector.y, vector.x);     //偏航角
}





/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/



