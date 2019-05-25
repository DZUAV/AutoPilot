/**********************************************************************************************************
*文件说明：lowPassFilter.c
*实现功能：低通滤波器
*修改日期：2018-10-31
*修改作者：cihang_uav
*备注信息:
**********************************************************************************************************/
#include "lowpassfilter.h"



/**********************************************************************************************************
*函数原型: void LowPassFilter1st(Vector3f_t* data, Vector3f_t newData, float coff)
*函数功能: 一阶低通滤波器
*输入形参: data:数据指针;newData:新数据; coff:滤波系数(新数据权重）
*返回数据: none
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/
void LowPassFilter1st(Vector3f_t* data, Vector3f_t newData, float coff)
{
    data->x = data->x * (1 - coff) + newData.x * coff;
    data->y = data->y * (1 - coff) + newData.y * coff;
    data->z = data->z * (1 - coff) + newData.z * coff;
}


/**********************************************************************************************************
*函数原型: void LowPassFilter2ndFactorCal(float deltaT, float Fcut, LPF2ndData_t* lpf_data)
*函数功能: 二阶IIR低通滤波器系数计算，该滤波器由二阶模拟低通滤波电路数字化推导而来
*输入形参: deltaT:滤波器运行频率; Fcut:截止频率; lpf_data:滤波器结构体指针
*返回数据: none
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/

void LowPassFilter2ndFactorCal(float deltaT, float Fcut, LPF2ndData_t* lpf_data)
{
	float a = 1 / (2 * M_PI * Fcut * deltaT);
	lpf_data->b0 = 1 / (a*a + 3*a + 1);
	lpf_data->a1 = (2*a*a + 3*a) / (a*a + 3*a + 1);
	lpf_data->a2 = (a*a) / (a*a + 3*a + 1);
}

/**********************************************************************************************************
*函数原型: Vector3f_t LowPassFilter2nd(LPF2ndData_t* lpf_2nd, Vector3f_t rawData)
*函数功能: 二阶IIR低通滤波器过程实现
*输入形参: lpf_2nd:滤波器结构体指针;  rawData:原始数据
*返回数据: lpf_2nd_data:滤波后数据
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/

Vector3f_t LowPassFilter2nd(LPF2ndData_t* lpf_2nd, Vector3f_t rawData)
{
	Vector3f_t lpf_2nd_data;

	lpf_2nd_data.x = rawData.x * lpf_2nd->b0 + lpf_2nd->lastout.x * lpf_2nd->a1 - lpf_2nd->preout.x * lpf_2nd->a2;
	lpf_2nd_data.y = rawData.y * lpf_2nd->b0 + lpf_2nd->lastout.y * lpf_2nd->a1 - lpf_2nd->preout.y * lpf_2nd->a2;
	lpf_2nd_data.z = rawData.z * lpf_2nd->b0 + lpf_2nd->lastout.z * lpf_2nd->a1 - lpf_2nd->preout.z * lpf_2nd->a2;

	lpf_2nd->preout.x = lpf_2nd->lastout.x;
	lpf_2nd->preout.y = lpf_2nd->lastout.y;
	lpf_2nd->preout.z = lpf_2nd->lastout.z;

	lpf_2nd->lastout.x = lpf_2nd_data.x;
	lpf_2nd->lastout.y = lpf_2nd_data.y;
	lpf_2nd->lastout.z = lpf_2nd_data.z;

	return lpf_2nd_data;
}

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/




