/**********************************************************************************************************
*文件说明：PID控制算法实现
*实现功能：PID控制
*修改日期：2018-11-30
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/
#include "pid.h"
#include "copter.h"


/**********************************************************************************************************
*函数原型: float PID_GetP(PID_t* pid, float error)
*函数功能: 获取P控制
*输入参数: pid：控制量,  error:控制误差
*返回数据: none
*修改日期: 2018-11-30
*备注信息：
**********************************************************************************************************/

float PID_GetP(PID_t* pid, float error)
{
    return (float)error * pid->kP;
}

/**********************************************************************************************************
*函数原型: float PID_GetI(PID_t* pid, float error, float dt)
*函数功能: 获取I控制
*输入参数: pid：控制量,  error:控制误差 dt:控制时间
*返回数据: none
*修改日期: 2018-11-30
*备注信息：
**********************************************************************************************************/
float PID_GetI(PID_t* pid, float error, float dt)
{
    if((pid->kI != 0) && (dt != 0))
    {
        pid->integrator += ((float)error * pid->kI) * dt;
        pid->integrator = ConstrainFloat(pid->integrator, -pid->imax, +pid->imax);
        return pid->integrator;
    }
    return 0;
}

/**********************************************************************************************************
*函数原型: void PID_ResetI(PID_t* pid)
*函数功能: 复位I
*输入参数: pid：控制量
*返回数据: none
*修改日期: 2018-11-30
*备注信息：
**********************************************************************************************************/
void PID_ResetI(PID_t* pid)
{
    pid->integrator = 0;
}
/**********************************************************************************************************
*函数原型: float PID_GetD(PID_t* pid, float error, float dt)
*函数功能: 获取D控制
*输入参数: pid：控制量,  error:控制误差 dt:控制时间
*返回数据: none
*修改日期: 2018-11-30
*备注信息：
**********************************************************************************************************/
float PID_GetD(PID_t* pid, float error, float dt)
{
    if ((pid->kD != 0) && (dt != 0))
    {
        float derivative;

        derivative = (error - pid->lastError) / dt;

        derivative = pid->lastDerivative + (dt / ( pid->dFilter + dt)) * (derivative - pid->lastDerivative);

        pid->lastError	= error;
        pid->lastDerivative = derivative;

        return pid->kD * derivative;
    }
    return 0;
}

/**********************************************************************************************************
*函数原型: float PID_GetPI(PID_t* pid, float error, float dt)
*函数功能: 获取PI控制
*输入参数: pid：控制量,  error:控制误差 dt:控制时间
*返回数据: none
*修改日期: 2018-11-30
*备注信息：
**********************************************************************************************************/
float PID_GetPI(PID_t* pid, float error, float dt)
{
    return PID_GetP(pid, error) + PID_GetI(pid, error, dt);
}

/**********************************************************************************************************
*函数原型: float PID_GetPID(PID_t* pid, float error, float dt)
*函数功能: 获取PID控制
*输入参数: pid：控制量,  error:控制误差 dt:控制时间
*返回数据: none
*修改日期: 2018-11-30
*备注信息：
**********************************************************************************************************/
float PID_GetPID(PID_t* pid, float error, float dt)
{
    return PID_GetP(pid, error) + PID_GetI(pid, error, dt) + PID_GetD(pid, error, dt);
}

/**********************************************************************************************************
*函数原型: void PID_SetParam(PID_t* pid, float p, float i, float d, float imaxval, float dCutFreq)
*函数功能: 设置PID参数
*输入参数: pid：控制量,  error:控制误差
*返回数据: none
*修改日期: 2018-11-30
*备注信息：
**********************************************************************************************************/
void PID_SetParam(PID_t* pid, float p, float i, float d, float imaxval, float dCutFreq)
{
    pid->kP = p;
    pid->kI = i;
    pid->kD = d;
    pid->imax = imaxval;
    if(dCutFreq != 0)
        pid->dFilter = 1 / (2 * M_PI * dCutFreq);
    else
        pid->dFilter = 0;
}

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/
