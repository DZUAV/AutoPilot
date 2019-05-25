/**********************************************************************************************************
*文件说明：rotation.c
*实现功能：方向旋转
*修改日期：
*修改作者：
*备注信息
**********************************************************************************************************/
#include "rotation.h"



/**********************************************************************************************************
*函数原型: void RotateVector3f(enum Rotation rot, Vector3f_t* v)
*函数功能: 向量方向变换
*输入形参: rot:旋转方式；v：旋转后矢量
*返回数据: 矩阵c
*修改日期：2018-10-31
*备注信息
**********************************************************************************************************/
void RotateVector3f(enum Rotation rot, Vector3f_t* v)
{
	float tmp;

    switch (rot) 
    {
        case ROTATION_NONE:
        case ROTATION_MAX:
            return;

        case ROTATION_YAW_45: 
					  {
                tmp  = HALF_SQRT_2 * (v->x - v->y);
                v->y = HALF_SQRT_2 * (v->x + v->y);
                v->x = tmp;
                return;
            }

        case ROTATION_YAW_90: 
					  {
                tmp = v->x; v->x = -v->y; v->y = tmp;
                return;
            }

        case ROTATION_YAW_135: 
					  {
                tmp  = -HALF_SQRT_2 * (v->x + v->y);
                v->y =  HALF_SQRT_2 * (v->x - v->y);
                v->x = tmp;
                return;
            }

        case ROTATION_YAW_180:
            v->x = -v->x; v->y = -v->y;
            return;

        case ROTATION_YAW_225: 
					  {
                tmp  = HALF_SQRT_2 * (v->y - v->x);
                v->y = -HALF_SQRT_2 * (v->x + v->y);
                v->x = tmp;
                return;
            }

        case ROTATION_YAW_270: 
					  {
                tmp = v->x; v->x = v->y; v->y = -tmp;
                return;
            }

        case ROTATION_YAW_315: 
					 {
                tmp  = HALF_SQRT_2 * (v->x + v->y);
                v->y = HALF_SQRT_2 * (v->y - v->x);
                v->x = tmp;
                return;
            }

        case ROTATION_ROLL_180: 
					  {
                v->y = -v->y; v->z = -v->z;
                return;
            }

        case ROTATION_ROLL_180_YAW_45: 
					  {
                tmp  = HALF_SQRT_2 * (v->x + v->y);
                v->y = HALF_SQRT_2 * (v->x - v->y);
                v->x = tmp; v->z = -v->z;
                return;
            }

        case ROTATION_ROLL_180_YAW_90: 
					  {
                tmp = v->x; v->x = v->y; v->y = tmp; v->z = -v->z;
                return;
            }

        case ROTATION_ROLL_180_YAW_135: 
					  {
                tmp = HALF_SQRT_2 * (v->y - v->x);
                v->y   = HALF_SQRT_2 * (v->y + v->x);
                v->x = tmp; v->z = -v->z;
                return;
            }

        case ROTATION_PITCH_180: 
					  {
                v->x = -v->x; v->z = -v->z;
                return;
            }

        case ROTATION_ROLL_180_YAW_225: 
					  {
                tmp = -HALF_SQRT_2 * (v->x + v->y);
                v->y   =  HALF_SQRT_2 * (v->y - v->x);
                v->x = tmp; v->z = -v->z;
                return;
            }

        case ROTATION_ROLL_180_YAW_270: 
					  {
                tmp = v->x; v->x = -v->y; v->y = -tmp; v->z = -v->z;
                return;
            }

        case ROTATION_ROLL_180_YAW_315: 
					  {
                tmp  =  HALF_SQRT_2 * (v->x - v->y);
                v->y = -HALF_SQRT_2 * (v->x + v->y);
                v->x = tmp; v->z = -v->z;
                return;
            }

        case ROTATION_ROLL_90: 
					  {
                tmp = v->z; v->z = v->y; v->y = -tmp;
                return;
            }

        case ROTATION_ROLL_90_YAW_45: 
					  {
                tmp  = v->z; v->z = v->y; v->y = -tmp;
                tmp  = HALF_SQRT_2 * (v->x - v->y);
                v->y = HALF_SQRT_2 * (v->x + v->y);
                v->x = tmp;
                return;
            }

        case ROTATION_ROLL_90_YAW_90: 
					  {
                tmp = v->z; v->z = v->y; v->y = -tmp;
                tmp = v->x; v->x = -v->y; v->y = tmp;
                return;
            }

        case ROTATION_ROLL_90_YAW_135: 
					  {
                tmp  = v->z; v->z = v->y; v->y = -tmp;
                tmp  = -HALF_SQRT_2 * (v->x + v->y);
                v->y =  HALF_SQRT_2 * (v->x - v->y);
                v->x = tmp;
                return;
            }

        case ROTATION_ROLL_270: 
					  {
                tmp = v->z; v->z = -v->y; v->y = tmp;
                return;
            }

        case ROTATION_ROLL_270_YAW_45: 
					  {
                tmp  = v->z; v->z = -v->y; v->y = tmp;
                tmp  = HALF_SQRT_2 * (v->x - v->y);
                v->y = HALF_SQRT_2 * (v->x + v->y);
                v->x = tmp;
                return;
            }

        case ROTATION_ROLL_270_YAW_90: 
					  {
                tmp = v->z; v->z = -v->y; v->y = tmp;
                tmp = v->x; v->x = -v->y; v->y = tmp;
                return;
            }

        case ROTATION_ROLL_270_YAW_135: 
					  {
                tmp  = v->z; v->z = -v->y; v->y = tmp;
                tmp  = -HALF_SQRT_2 * (v->x + v->y);
                v->y =  HALF_SQRT_2 * (v->x - v->y);
                v->x = tmp;
                return;
            }

        case ROTATION_ROLL_270_YAW_270: 
					  {
                tmp = v->z; v->z = -v->y; v->y = tmp;
                tmp = v->x; v->x = v->y; v->y = -tmp;
                return;
            }

        case ROTATION_PITCH_90: 
					  {
                tmp = v->z; v->z = -v->x; v->x = tmp;
                return;
            }

        case ROTATION_PITCH_270: 
					  {
                tmp = v->z; v->z = v->x; v->x = -tmp;
                return;
            }

        case ROTATION_ROLL_180_PITCH_270: 
					  {
                tmp = v->z; v->z = v->x; v->x = tmp;
                v->y = -v->y;
                return;
            }

        case ROTATION_PITCH_90_YAW_180: 
					  {
                tmp = v->x; v->x = v->z; v->z = tmp;
                v->y = -v->y;
                return;
            }

        case ROTATION_PITCH_90_ROLL_90: 
					  {
                tmp = v->x; v->x = v->y;
                v->y = -v->z; v->z = -tmp;
                return;
            }

        case ROTATION_YAW_293_PITCH_68_ROLL_90: 
					  {
                float tmpx = v->x;
                float tmpy = v->y;
                float tmpz = v->z;
                v->x =  0.143039f * tmpx +  0.368776f * tmpy + -0.918446f * tmpz;
                v->y = -0.332133f * tmpx + -0.856289f * tmpy + -0.395546f * tmpz;
                v->z = -0.932324f * tmpx +  0.361625f * tmpy +  0.000000f * tmpz;
                return;
            }

        case ROTATION_PITCH_90_ROLL_270: 
					  {
                tmp = v->x; v->x = -v->y;
                v->y = v->z; v->z = -tmp;
                return;
            }
    }
}

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/










