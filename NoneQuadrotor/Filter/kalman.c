/**********************************************************************************************************
*文件说明: kalamn滤波器文件
*实现功能：kalamn滤波器设计
*修改日期：2018-11-27
*修改作者：crystal cup
*修改备注： 一维卡尔曼滤波器的具体实现。实现过程完全与硬件无关，可直接调用，任意移植。   *
 *          使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器，           *
 *          每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波。  
 *           MPU6050的卡尔曼滤波器参考参数 Q：10 R：400           
**********************************************************************************************************/
  
#include "kalman.h"


/**********************************************************************************************************
*函数原型: void kalmanCreate(kalman *p,float T_Q,float T_R)
*函数功能: main函数入口
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-27
*备注信息：
* @name   kalmanCreate
* @brief  创建一个卡尔曼滤波器
* @param  p:  滤波器
*         T_Q:系统噪声协方差
*         T_R:测量噪声协方差
*         
* @retval none
***********************************************************************************************************/

void kalmanCreate(kalman *p,float T_Q,float T_R)
{
    p->X_last = 0.f;
    p->P_last = 1.f;
    p->Q = T_Q;
    p->R = T_R;
}




/**********************************************************************************************************
*函数原型: float KalmanFilter(kalman* p,float dat)
*函数功能: 卡尔曼滤波器
*输入参数: p:  滤波器 ,dat:待滤波数据
*返回数据: 滤波后的数据
*修改日期: 2018-11-27
*备注信息：

***********************************************************************************************************/
float KalmanFilter(kalman* p,float dat)
{
    p->X_mid = p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->P_last + p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
	
    p->kg = p->P_mid / (p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1 - p->kg) * p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;
}


/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/
