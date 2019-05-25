/**********************************************************************************************************
*�ļ�˵����NoneQuadrotor UAV���п��ƴ���
*ʵ�ֹ��ܣ�ʵ�����˻���׼����
*�޸����ڣ�2018-11-17
*�޸����ߣ�crystal cup
*�޸ı�ע��
		             PE9   TIM1_CH1 TIM1 PWM(4) GPIO(53)
					 PE11 TIM1_CH2 TIM1 PWM(3) GPIO(52)
					 PA10 TIM1_CH3 TIM1 PWM(2) GPIO(51)
					 PE14 TIM1_CH4 TIM1 PWM(1) GPIO(50)
					 PD13 TIM4_CH2 TIM4 PWM(5) GPIO(54)
		       PD14 TIM4_CH3 TIM4 PWM(6) GPIO(55)
**********************************************************************************************************/
#include "drv_pwm.h"



TIM_HandleTypeDef TIM1_Handler;         //��ʱ��1 PWM��� 

TIM_OC_InitTypeDef TIM1_CH1Handler;     //��ʱ��1ͨ��1���
TIM_OC_InitTypeDef TIM1_CH2Handler;     //��ʱ��1ͨ��2���
TIM_OC_InitTypeDef TIM1_CH3Handler;     //��ʱ��1ͨ��3���
TIM_OC_InitTypeDef TIM1_CH4Handler;     //��ʱ��1ͨ��4���


/**********************************************************************************************************
*����ԭ��: void TIM1_PWM_Init(uint16_t arr,uint16_t psc,uint16_t Pulse )
*��������: ����ʼ������
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��psc:19  216000000/20=10800000  psc10800000/480=22500
*          TIM1_PWM_Init(22499,19,9000); 
*          ����Ƶ�ʣ�10800000Hz
*          PWMƵ��=216000000/(19+1)/(22499+1)=480Hz
*          ռ�ձȣ�Pulse=40%
**********************************************************************************************************/
void TIM1_PWM_Init(uint16_t arr,uint16_t psc,uint16_t Pulse )
{ 
    TIM1_Handler.Instance=TIM1;            //��ʱ��3
    TIM1_Handler.Init.Prescaler=psc;       //��ʱ����Ƶ
    TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
    TIM1_Handler.Init.Period=arr;          //�Զ���װ��ֵ
    TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM1_Handler);       //��ʼ��PWM
	
    //TIM1_CH1
    TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM1_CH1Handler.Pulse=Pulse;          //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�
                                            //Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_1);//����TIM3ͨ��1
	  TIM1_Handler.Instance->BDTR|=(1<<15);
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1);//����PWMͨ��4
	  //TIM1_CH2
	  TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM1_CH1Handler.Pulse=Pulse;          //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�
                                            //Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_2);//����TIM1ͨ��1
		  TIM1_Handler.Instance->BDTR|=(1<<15);
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_2);//����PWMͨ��4
	  //TIM1_CH3
	  TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM1_CH1Handler.Pulse=Pulse;          //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�
                                            //Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_3);//����TIM3ͨ��1
		  TIM1_Handler.Instance->BDTR|=(1<<15);
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_3);//����PWMͨ��4
	  //TIM1_CH4
	  TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM1_CH1Handler.Pulse=Pulse;          //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�
                                            //Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_4);//����TIM3ͨ��1
	TIM1_Handler.Instance->BDTR|=(1<<15);
    HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_4);//����PWMͨ��4
	   
	
}



/**********************************************************************************************************
*����ԭ��: void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
*��������: ��ʱ���ײ�������ʱ��ʹ�ܣ���������
*�������: htim:��ʱ�����
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ���˺����ᱻHAL_TIM_PWM_Init()����
*          PE9  TIM1_CH1 TIM1 PWM(4) GPIO(53)
					 PE11 TIM1_CH2 TIM1 PWM(3) GPIO(52)
					 PA10 TIM1_CH3 TIM1 PWM(2) GPIO(51)
					 PE14 TIM1_CH4 TIM1 PWM(1) GPIO(50)
**********************************************************************************************************/

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM1_CLK_ENABLE();			//ʹ�ܶ�ʱ��1
	__HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
    __HAL_RCC_GPIOE_CLK_ENABLE();			//����GPIOEʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_14;        //PE9,PE11,PE14
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	  //�����������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
	GPIO_Initure.Alternate=GPIO_AF1_TIM1;	  //PB1����ΪTIM3_CH4
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	
	  
	GPIO_Initure.Pin=GPIO_PIN_10;        //PA10
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	  //�����������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
	GPIO_Initure.Alternate=GPIO_AF1_TIM1;	  //PA10����ΪTIM1_CH3
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
}


/**********************************************************************************************************
*����ԭ��: void TIM_SetTIM1Compare1(u32 compare)
*��������: ����TIMͨ��1��ռ�ձ�
*�������: compare:�Ƚ�ֵ
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void TIM_SetTIM1Compare1(uint32_t compare)
{
	TIM1->CCR1=compare; 
}

/**********************************************************************************************************
*����ԭ��: void TIM_SetTIM1Compare2(u32 compare)
*��������: ����TIMͨ��2��ռ�ձ�
*�������: compare:�Ƚ�ֵ
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void TIM_SetTIM1Compare2(uint32_t compare)
{
	TIM1->CCR2=compare; 
}

/**********************************************************************************************************
*����ԭ��: void TIM_SetTIM1Compare3(u32 compare)
*��������: ����TIMͨ��3��ռ�ձ�
*�������: compare:�Ƚ�ֵ
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void TIM_SetTIM1Compare3(uint32_t compare)
{
	TIM1->CCR3=compare; 
}

/**********************************************************************************************************
*����ԭ��: void TIM_SetTIM1Compare4(u32 compare)
*��������: ����TIMͨ��4��ռ�ձ�
*�������: compare:�Ƚ�ֵ
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void TIM_SetTIM1Compare4(uint32_t compare)
{
	TIM1->CCR4=compare; 
}

void Set_Motor_PwmOut_Value(void)
{
	TIM_SetTIM1Compare1(9000);
	TIM_SetTIM1Compare2(9000);
  TIM_SetTIM1Compare3(9000);
	TIM_SetTIM1Compare4(9000);

	
}

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/