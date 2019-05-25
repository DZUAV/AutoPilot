/**********************************************************************************************************
*�ļ�˵����ADRC�����㷨ʵ��
*ʵ�ֹ��ܣ�ADRC����
*�޸����ڣ�2018-12-8
*�޸����ߣ�crystal cup
*�޸ı�ע��
**********************************************************************************************************/
#include "copter.h"
#include "adrc.h"
ADRC_t adrc_t;


/******************************************************************************************************************************
*����ԭ��: float adrc_control(float v,float y,float u,float MAX)
*��������: adrc���ƺ���
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ��
******************************************************************************************************************************/
float adrc_control(float v,float y,float u,float MAX)
{
	adrc_td(v);                            //�������΢����TD
	adrc_eso(v, y, u, adrc_t._h0, MAX);    //��չ״̬�۲���ESO
	adrc_nsl(v, y, u, adrc_t._h0, MAX);   //����������
	return adrc_t._u;
}


/**********************************************************************************************************
*����ԭ��: float adcr_constrain(float val, float min, float max)
*��������: ���Ʒ���
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ��
**********************************************************************************************************/

float adcr_constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

/******************************************************************************************************************************
*����ԭ��: float adcr_constrain(float val, float min, float max)
*��������: ���Ʒ���
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ��
******************************************************************************************************************************/
void adrc_td(float in)
{
	float v1_pre = adrc_t._v1;
	adrc_t._v1 += adrc_t._h0 * adrc_t._v2;                                        //td_x1=v1; v1(t+1) = v1(t) + T * v2(t)
	adrc_t._v2 += adrc_t._h0 * adrc_fhan(v1_pre - in, adrc_t._v2, adrc_t._r0, adrc_t._h1);           //td_x2=v2 v2(t+1) = t2(t) + T fst2(v1(t) - in, v2(t), r, h)
}
/******************************************************************************************************************************
*����ԭ��: float adrc_fsg(float x,float d)
*��������: ���Ʒ���
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ��
******************************************************************************************************************************/
float adrc_fsg(float x,float d)
{
    float value;
    value=(adrc_sign(x+d)-adrc_sign(x-d))*0.5f;
	  return(value);
}


/******************************************************************************************************************************
*����ԭ��: float adrc_fhan(float x1, float x2, float r, float h)
*��������: ���Ʒ���
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ��
******************************************************************************************************************************/
float adrc_fhan(float x1, float x2, float r, float h)
{

	float y=0;
	float a0=0,a1,a2;
	float a=0;
	float fhan=0;
	float d=0;
	float sa;

	d = r * h * h;                                             //����d=r*h,d=h*d
	a0 = h * x2;                                                //a0=h*x2
	y = x1 + a0;                                                //y=x1+a0
	a1 = sqrtf(d*(d + 8.0 * fabsf(y)));                        //a1=sqrtf(d*(d+8*(|y|)))
	a2 = a0 + adrc_sign(y) * (a1-d) * 0.5f;                     //a2=a0+sign(y)*(a1-d)/2

	a = (a0 + y - a2) * adrc_fsg(y,d) + a2;                          //a=(a0+y-a2)*fsg(y,d)+a2
	sa = adrc_fsg(a,d);                                              //fsg(a,d)=(sign(a+d)-sign(a-d))/2
	fhan = -r * (a / d - adrc_sign(a)) * sa - r * adrc_sign(a); //fhan=-r*fsg(a,d)-r*sign(a)(1-fsg(a,d))
	return(fhan);
}
/******************************************************************************************************************************
*����ԭ��: float adrc_fal(float e,float alfa,float delta)
*��������: ָ������fal(e,0.5,0.01)
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ��ԭ�㸽���������Զ��������ݴκ���
******************************************************************************************************************************/
float adrc_fal(float e,float alfa,float delta)
{
	float fal = 0.0f;
	float s=0;
	s=adrc_fsg(e,delta);
	fal=e*s/(powf(delta,1-alfa))+powf(fabs(e),alfa)*adrc_sign(e)*(1-s);
	return(fal);
}
/******************************************************************************************************************************
*����ԭ��: float adrc_eso(float v,float y,float u,float T,float MAX)
*��������: ��չ״̬�۲���
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ��
******************************************************************************************************************************/
float adrc_eso(float v,float y,float u,float T,float MAX)
{
	float e=0,fe,fe1;
	e = adrc_t._z[0] - y;
	fe = adrc_fal(e, 0.5f, adrc_t._h0);
	fe1=adrc_fal(e, 0.25f, adrc_t._h0);
	adrc_t._z[0] += adrc_t._h0 * (adrc_t._z[1] - adrc_t._beta0*e);
	adrc_t._z[1] += adrc_t._h0 * (adrc_t._z[2] - adrc_t._beta1 * fe + adrc_t._b01 *  u);
	adrc_t._z[2] += -adrc_t._h0 * adrc_t._beta2 * fe1;
	return adrc_t._disturb=adcr_constrain(adrc_t._z[2] , -MAX, MAX);//constrain_float

}

/******************************************************************************************************************************
*����ԭ��: float adrc_nsl(float v,float y,float u,float T,float MAX)
*��������: ����������
*�������: 
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ��
******************************************************************************************************************************/
float adrc_nsl(float v,float y,float u,float T,float MAX)
{
	adrc_t._e0+=adrc_t._e1*adrc_t._h0;      //e0=e1??
	adrc_t._e1 = adrc_t._v1 - adrc_t._z[0];
	adrc_t._e2 = adrc_t._v2 - adrc_t._z[1];
	adrc_t._u = adrc_t._kp * adrc_fal(adrc_t._e1, adrc_t._alfa1, adrc_t._tao) + adrc_t._kd * adrc_fal(adrc_t._e2,adrc_t._alfa2,adrc_t._tao); //0<_alfa1<1<_alfa2
	adrc_t._disturb_u = adrc_t._z[2] /adrc_t._b01;
	adrc_t._u-=adrc_t._disturb_u;
	adrc_t._u=adcr_constrain(adrc_t._u, -MAX, MAX);
	return (adrc_t._u);

}
/******************************************************************************************************************************
*����ԭ��: float adrc_nsl(float v,float y,float u,float T,float MAX)
*��������: ����������
*�������: 
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ��
******************************************************************************************************************************/
int8_t adrc_sign(float input)  //adrc����
{
		//int8_t output
		if(input > 1e-6)
		{
			return 1;
		}
		else if(input < -1e-6)
		{
			return -1;
		}
		else
			return 0;
}



/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/