#ifndef __MATHTYPE__H__
#define __MATHTYPE__H__

#include "vector3.h"

#define REAL   float
#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256
#define MY_PPPIII   3.14159f
#define MY_PPPIII_HALF   1.570796f

#define ABS(x) ( (x)>0?(x):-(x) )

float my_abs(float f);
REAL fast_atan2(REAL y, REAL x);

float invSqrt(float x);
float my_sqrt(float number);
float my_float_sqrt(float a);

double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deathzoom(float x,float zoom);
float my_deathzoom_2(float x,float zoom);
float To_180_degrees(float x);
float my_pow_2_curve(float in,float a,float max);
void simple_3d_trans(Vector3f_t *ref, Vector3f_t *in, Vector3f_t *out);
#endif
