#ifndef __ADRC__H__
#define __ADRC__H__

#include "sys.h"

typedef struct
{
	  float _kp;
	  float _kd;
	  float _beta0, _beta1, _beta2;
	  float _r0; 
	  float _b01; 
	  float  _alfa1, _alfa2; 
	  float _tao;
	  float _h0; 


	  float _h1;
	  float _v1, _v2; 
	  float _z[3]; 

	  float _e0,_e1, _e2;
	  float _disturb, _disturb_u;
	  float _u;
	  uint8_t  _level;


}ADRC_t;


extern ADRC_t adrc_t;




float adrc_control(float v,float y,float u,float MAX);
float adcr_constrain(float val, float min, float max);
void adrc_td(float in);
float adrc_fsg(float x,float d);
float adrc_fhan(float x1, float x2, float r, float h);
float adrc_fal(float e,float alfa,float delta);
float adrc_eso(float v,float y,float u,float T,float MAX);
float adrc_nsl(float v,float y,float u,float T,float MAX);
int8_t adrc_sign(float input);






#endif
