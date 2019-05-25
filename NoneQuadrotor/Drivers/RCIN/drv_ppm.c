/**********************************************************************************************************
*�ļ�˵����PPM�����ļ����ú���
*ʵ�ֹ��ܣ�����ppm
*�޸����ڣ�2018-12-6
*�޸����ߣ�crystal cup
*�޸ı�ע��
**********************************************************************************************************/

#include "drv_ppm.h"
#include "copter.h"

TIM_HandleTypeDef TIM4_Handler;  //��ʱ��5���

uint16_t ppm_rx[10];            // ���յ�ppm����

/**********************************************************************************************************
*����ԭ��: void PPM_Init(void)
*��������: PPM��ʼ��
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��1MHz�ļ���Ƶ��,20ms�ж�һ��
**********************************************************************************************************/

void PPM_Init(void)
{
  TIM4_Init(0xffff,108-1); //��������20ms
}


/**********************************************************************************************************
*����ԭ��: void TIM4_Init(uint32_t arr,uint16_t psc)
*��������: TIM4��ʼ��
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��TIM4ʱ����108M
**********************************************************************************************************/
void TIM4_Init(uint32_t arr,uint16_t psc)
{ 
	  //����TIM4_CH2
    TIM_IC_InitTypeDef TIM4_CH2Config;  
	  TIM_IC_InitTypeDef TIM4_CH3Config;  
	
    TIM4_Handler.Instance=TIM4;                          //ͨ�ö�ʱ��4
    TIM4_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM4_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&TIM4_Handler);
    
    TIM4_CH2Config.ICPolarity=TIM_ICPOLARITY_BOTHEDGE;    //�����ز���
    TIM4_CH2Config.ICSelection=TIM_ICSELECTION_DIRECTTI;//ӳ�䵽TI1��
    TIM4_CH2Config.ICPrescaler=TIM_ICPSC_DIV1;          //���������Ƶ������Ƶ
    TIM4_CH2Config.ICFilter=0;                          //���������˲��������˲�
    HAL_TIM_IC_ConfigChannel(&TIM4_Handler,&TIM4_CH2Config,TIM_CHANNEL_2);//����TIM4ͨ��1
    HAL_TIM_IC_Start_IT(&TIM4_Handler,TIM_CHANNEL_2);   //��ʼ����TIM4��ͨ��2
    __HAL_TIM_ENABLE_IT(&TIM4_Handler,TIM_IT_UPDATE);   //ʹ�ܸ����ж�
	
	  //����TIM4_CH3
	
    TIM4_Handler.Instance=TIM4;                          //ͨ�ö�ʱ��4
    TIM4_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM4_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&TIM4_Handler);                      //����HAL_TIM_IC_MspInit
    
    TIM4_CH3Config.ICPolarity=TIM_ICPOLARITY_BOTHEDGE;    //�����ز���
    TIM4_CH3Config.ICSelection=TIM_ICSELECTION_DIRECTTI;//ӳ�䵽TI1��
    TIM4_CH3Config.ICPrescaler=TIM_ICPSC_DIV1;          //���������Ƶ������Ƶ
    TIM4_CH3Config.ICFilter=0;                          //���������˲��������˲�
    HAL_TIM_IC_ConfigChannel(&TIM4_Handler,&TIM4_CH3Config,TIM_CHANNEL_3);//����TIM4ͨ��1
    HAL_TIM_IC_Start_IT(&TIM4_Handler,TIM_CHANNEL_3);   //��ʼ����TIM4��ͨ��3
    __HAL_TIM_ENABLE_IT(&TIM4_Handler,TIM_IT_UPDATE);   //ʹ�ܸ����ж�
	
}

/**********************************************************************************************************
*����ԭ��: void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
*��������: 
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-6
*��ע��Ϣ��
**********************************************************************************************************/

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM4_CLK_ENABLE();            //ʹ��TIM4ʱ��
    __HAL_RCC_GPIOD_CLK_ENABLE();			      //����GPIODʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_13|GPIO_PIN_14;            //PD13,PD14
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //�����������
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    GPIO_Initure.Alternate=GPIO_AF2_TIM4;   //PA0����ΪTIM4ͨ��1
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM4_IRQn,2,0);    //�����ж����ȼ�����ռ���ȼ�2�������ȼ�0
    HAL_NVIC_EnableIRQ(TIM4_IRQn);          //����ITM4�ж�   
}

/**********************************************************************************************************
*����ԭ��: void TIM4_IRQHandler(void)
*��������: ��ʱ��4�жϷ�����
*�������: none
*��������: none
*�޸�����: 2018-12-7
*��ע��Ϣ������PPM�ź�
**********************************************************************************************************/
uint8_t  TIM4CH2_CAPTURE_STA=0,ppm_rx_sta=0,ppm_rx_num=0;	//���벶��״̬		
uint16_t	TIM4CH2_CAPTURE_VAL;	//���벶��ֵ
void TIM4_IRQHandler(void)
{	
 	if((TIM4CH2_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{	  
		if (((TIM4->SR &(0x0001))!=RESET)&&((TIM4->DIER &(0x0001))!=RESET))
		 
		{	    
			if(TIM4CH2_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM4CH2_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM4CH2_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM4CH2_CAPTURE_VAL=0XFFFF;
				}else 
				{
				 TIM4CH2_CAPTURE_STA++;
				}
			}	 
		}
	if (((TIM4->SR &(0x0002))!=RESET)&&((TIM4->DIER &(0x0002))!=RESET))//����1���������¼�
		{	
			if(TIM4CH2_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM4CH2_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM4CH2_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&TIM4_Handler,TIM_CHANNEL_2);
				TIM_RESET_CAPTUREPOLARITY(&TIM4_Handler,TIM_CHANNEL_2);   //һ��Ҫ�����ԭ�������ã���
        TIM_SET_CAPTUREPOLARITY(&TIM4_Handler,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//����TIM4ͨ��2�����ز���
		   		
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM4CH2_CAPTURE_STA=0;			//���
				TIM4CH2_CAPTURE_VAL=0;
				TIM4CH2_CAPTURE_STA|=0X40;		//��ǲ�����������
				__HAL_TIM_SET_COUNTER(&TIM4_Handler,0);
	 			

				TIM_RESET_CAPTUREPOLARITY(&TIM4_Handler,TIM_CHANNEL_2);   //һ��Ҫ�����ԭ�������ã���
				TIM_SET_CAPTUREPOLARITY(&TIM4_Handler,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);//��ʱ��4ͨ��2����Ϊ�½��ز���
				__HAL_TIM_ENABLE(&TIM4_Handler);//ʹ�ܶ�ʱ��4

			}		    
		}			     	    					   
 	}
	//��ʼ�������PPM����
	PPM_Signal_Analysis();
	TIM4->SR = (uint16_t)~((0x0001)|(0x0002)); //����жϱ�־λ
}

/**********************************************************************************************************
*����ԭ��: void PPM_Signal_Analysis(void)
*��������: PPM���ݽ���
*�������: none
*��������: none
*�޸�����: 2018-12-8
*��ע��Ϣ������PPM�ź�
**********************************************************************************************************/
void PPM_Signal_Analysis(void)
{

		if(TIM4CH2_CAPTURE_STA&0X80)//�ɹ�������һ��������
		{

			if(ppm_rx_sta==1) 
      {
				ppm_rx[ppm_rx_num+1]=TIM4CH2_CAPTURE_VAL;
				ppm_rx_num++;
			
			}
			if(4>TIM4CH2_CAPTURE_STA&0X3F>0||TIM4CH2_CAPTURE_VAL>3000)
			{
			   ppm_rx_sta++;//�͵�ƽʱ�����3000usΪ��ʼ֡
			}
			if(ppm_rx_sta==2) 
      {
				ppm_rx_sta=0;
				ppm_rx[0]=1;
				ppm_rx_num=0;
			}
			
			TIM4CH2_CAPTURE_STA=0;//������һ�β���
			
		}
}


/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/

