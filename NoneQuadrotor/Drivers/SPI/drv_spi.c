/**********************************************************************************************************
*�ļ�˵����SPI�����ļ�
*ʵ�ֹ��ܣ�����SPI
*�޸����ڣ�2018-10-31
*�޸����ߣ�cihuang_uav
*�޸ı�ע��
*��ע��Ϣ��SPI1���ţ�
										PA6:  SPI1_MISO
										PD7:  SPI1_MOSI
										PG11: SPI1_SCK
										
										PF2:  SPI1_CS1_ICM20689
										PF3:  SPI1_CS2_ICM20602
										PF4:  SPI1_CS3_BMI055_GYRO
										PB4:  SPI1_DRDY1_ICM20689
										PB14: SPI1_DRDY2_BMI055_GYRO
										PB15: SPI1_DRDY3_BMI055_ACC
										PC5:  SPI1_DRDY4_ICM20602
										PC13: SPI1_DRDY5_BMI055_GYRO
										PD10: SPI1_DRDY6_BMI055_ACC
										
*		       SPI4���ţ�	PE13---SPI4-MISO		
*                     PE2---SPI4-SCK			
*                     PE6---SPI4-MOSI			
*                     PF10---SPI4_CS_MS5611	
**********************************************************************************************************/

#include "drv_spi.h"

#include "copter.h"
#include "stm32f7xx_hal_spi.h"

SPI_HandleTypeDef SPI1_Handler;  //SPI1���; 
SPI_HandleTypeDef SPI2_Handler;  //SPI2���; 
SPI_HandleTypeDef SPI4_Handler;  //SPI4���; 







/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : void SPI1_Init_CS(void)
**��    �� : SPI1������ʼ��
**��    �� : None
**��    �� : None
**��    ע : 
**====================================================================================================*/
/*====================================================================================================*/
void SPI1_Init_CS(void)
{
	RCC->AHB1ENR|=1<<5;	//ʹ��PORTFʱ�� 
	//SPI1_CS1
	GPIO_Set(SPI1_CS1_ICM20689_GPIO,SPI1_CS1_ICM20689_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF2����

	//SPI1_CS2
	GPIO_Set(SPI1_CS2_ICM20602_GPIO,SPI1_CS2_ICM20602_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF2����
	//SPI1_CS3
	GPIO_Set(SPI1_CS3_BMI055_GYRO_GPIO,SPI1_CS3_BMI055_GYRO_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF4����
	
   //SPI1_CS4
	GPIO_Set(SPI1_CS4_BMI055_ACC_GPIO,SPI1_CS4_BMI055_ACC_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PG10����
	//Ƭѡ��ѡ��
	GPIO_Pin_Set(SPI1_CS1_ICM20689_GPIO,SPI1_CS1_ICM20689_PIN,1);
	GPIO_Pin_Set(SPI1_CS2_ICM20602_GPIO,SPI1_CS2_ICM20602_PIN,1);
	GPIO_Pin_Set(SPI1_CS3_BMI055_GYRO_GPIO,SPI1_CS3_BMI055_GYRO_PIN,1);
	GPIO_Pin_Set(SPI1_CS4_BMI055_ACC_GPIO,SPI1_CS4_BMI055_ACC_PIN,1);
}

/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : void SPI2_Init_CS(void)
**��    �� : SPI1������ʼ��
**��    �� : None
**��    �� : None
**��    ע : PF5 SPI2_CS 
**====================================================================================================*/
/*====================================================================================================*/
void SPI2_Init_CS(void)
{
	RCC->AHB1ENR|=1<<5;	//ʹ��PORTFʱ�� 

	//SPI2_CS:FM25V01G
	GPIO_Set(SPI2_CS1_FM25V01G_GPIO,SPI2_CS1_FM25V01G_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF5����
	//Ƭѡ��ѡ��
	GPIO_Pin_Set(SPI2_CS1_FM25V01G_GPIO,SPI2_CS1_FM25V01G_PIN,1);
}


/**********************************************************************************************************
*����ԭ��: void SPI4_Init_CS(void)
*��������: spi4��GPIO��ʼ��Ƭѡ����
*�������: none
*��������: none
*�޸�����: 2018-11-26
*�޸�����: 
*		       SPI4���ţ�	PE13---SPI4-MISO		
*                           PE2---SPI4-SCK			
*                           PE6---SPI4-MOSI			
*                           PF10---SPI4_CS_MS5611	
**********************************************************************************************************/
void SPI4_Init_CS(void)
{
	RCC->AHB1ENR|=1<<5;	//ʹ��PORTFʱ�� 
	//SPI4_CS1
	GPIO_Set(SPI4_CS1_MS5611_GPIO,SPI4_CS1_MS5611_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF10����

	//Ƭѡ��ѡ��
	GPIO_Pin_Set(SPI4_CS1_MS5611_GPIO,SPI4_CS1_MS5611_PIN,1);

}

/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : void SPI1_Init(void)
**��    �� : SPI1������ʼ��
**��    �� : None
**��    �� : None
**��    ע :   
**====================================================================================================*/
/*====================================================================================================*/
void SPI1_Init(void)
{

    RCC->APB2ENR|=1<<12;        //ʹ��SPI1ʱ��
    SPI1_Handler.Instance=SPI1;                         //SP1
    SPI1_Handler.Init.Mode=SPI_MODE_MASTER;             //����SPI����ģʽ������Ϊ��ģʽ
    SPI1_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ
    SPI1_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI1_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH;    //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI1_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;         //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI1_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI1_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI1_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI1_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //�ر�TIģʽ
    SPI1_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//�ر�Ӳ��CRCУ��
    SPI1_Handler.Init.CRCPolynomial=7;                  //CRCֵ����Ķ���ʽ
	
    HAL_SPI_Init(&SPI1_Handler);//��ʼ��
    SPI1->CR1|=1<<6; 		//SPI�豸ʹ��	 
    SPI1_ReadWriteByte(0Xff);                           //��������
    SPI1_SetSpeed(SPI_SPEED_256);    //����Ϊ45Mʱ��,����ģʽ
	  SPI1_Init_CS();
}

/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : void SPI2_Init(void)
**��    �� : SPI2������ʼ��
**��    �� : None
**��    �� : None
**��    ע : PI1 SPI2_SCK SPI2
             PI2 SPI2_MISO SPI2
             PI3 SPI2_MOSI SPI2   
**====================================================================================================*/
/*====================================================================================================*/
void SPI2_Init(void)
{

    RCC->APB1ENR|=1<<14;        //ʹ��SPI2ʱ��
    SPI2_Handler.Instance=SPI2;                         //SP1
    SPI2_Handler.Init.Mode=SPI_MODE_MASTER;             //����SPI����ģʽ������Ϊ��ģʽ
    SPI2_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ
    SPI2_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI2_Handler.Init.CLKPolarity=SPI_POLARITY_LOW;    //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI2_Handler.Init.CLKPhase=SPI_PHASE_1EDGE;         //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI2_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI2_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI2_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI2_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //�ر�TIģʽ
    SPI2_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//�ر�Ӳ��CRCУ��
    SPI2_Handler.Init.CRCPolynomial=7;                  //CRCֵ����Ķ���ʽ
	
    HAL_SPI_Init(&SPI2_Handler);//��ʼ��
    SPI2->CR1|=1<<6; 		//SPI�豸ʹ��	 
    SPI2_ReadWriteByte(0Xff);                           //��������
    SPI2_SetSpeed(SPI_SPEED_256);    //����Ϊ45Mʱ��,����ģʽ
	  SPI2_Init_CS();
}
/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : void SPI4_Init(void)
**��    �� : SPI4������ʼ��
**��    �� : None
**��    �� : None
**��    ע :   
**====================================================================================================*/
/*====================================================================================================*/
void SPI4_Init(void)
{
 
    RCC->APB2ENR|=1<<13;        //ʹ��SPI4ʱ��
    SPI4_Handler.Instance=SPI4;                         //SP4
    SPI4_Handler.Init.Mode=SPI_MODE_MASTER;             //����SPI����ģʽ������Ϊ��ģʽ
    SPI4_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ
    SPI4_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI4_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH;    //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI4_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;         //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI4_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI4_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI4_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI4_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //�ر�TIģʽ
    SPI4_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//�ر�Ӳ��CRCУ��
    SPI4_Handler.Init.CRCPolynomial=7;                  //CRCֵ����Ķ���ʽ
	
    HAL_SPI_Init(&SPI4_Handler);//��ʼ��
    SPI4->CR1|=1<<6; 		      //SPI4�豸ʹ��	 
    SPI4_ReadWriteByte(0Xff);                           //��������
    SPI4_SetSpeed(SPI_SPEED_256);    //����Ϊ45Mʱ��,����ģʽ
	  SPI4_Init_CS();
}

/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
**��    �� : SPI1������ʼ��
**��    �� : None
**��    �� : None
**��    ע : PE13---SPI4-MISO			
*            PE6---SPI4-MOSI	
*            PE2---SPI4-SCK		
*            PI1 SPI2_SCK SPI2
             PI2 SPI2_MISO SPI2
             PI3 SPI2_MOSI SPI2   
**====================================================================================================*/
/*====================================================================================================*/

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_Initure;
		//////////////////SPI1/////////////////////////////////////////////
		RCC->APB2ENR|=1<<12;        //ʹ��SPI1ʱ��
		RCC->AHB1ENR|=1<<0;	        //ʹ��PORTAʱ�� 
		RCC->AHB1ENR|=1<<3;	        //ʹ��PORTDʱ�� 
		RCC->AHB1ENR|=1<<6;	        //ʹ��PORTGʱ�� 

		//PA6---SPI1_MISO 
		GPIO_Set(GPIOA,PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PA6����
		 GPIO_AF_Set(GPIOA,6,5);	 //PA6,AF5
		//PD7---SPI1_MOSI
		GPIO_Set(GPIOD,PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PD7����
		GPIO_AF_Set(GPIOD,7,5);	 //PD7,AF5
		//PG11---SPI1_SCK
		GPIO_Set(GPIOG,PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PG11����
		GPIO_AF_Set(GPIOG,11,5); //PG11,AF5 
	
		//////////////////SPI2/////////////////////////////////////////////

		RCC->APB1ENR|=1<<14;         //ʹ��SPI2ʱ��
		RCC->AHB1ENR|=1<<8;	         //ʹ��PORTIʱ�� 

    
//      RCC->AHB1ENR|=1<<8;	                    //ʹ��PORTIʱ�� ;                   
//    __HAL_RCC_SPI2_CLK_ENABLE();                    //ʹ��SPI2ʱ��
//    
//    //PB13,14,15
//    GPIO_Initure.Pin=GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
//    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //�����������
//    GPIO_Initure.Pull=GPIO_PULLUP;                  //����
//    GPIO_Initure.Speed=GPIO_SPEED_FAST;             //����    
//    GPIO_Initure.Alternate=GPIO_AF5_SPI2;           //����ΪSPI2
//    HAL_GPIO_Init(GPIOI,&GPIO_Initure);             //��ʼ��
		//PI2---SPI2_MISO 
		GPIO_Set(GPIOI,PIN2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PA6����
		GPIO_AF_Set(GPIOI,2,5);	 //PI2,AF5
		//PI3---SPI2_MOSI
		GPIO_Set(GPIOI,PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PD7����
		GPIO_AF_Set(GPIOI,3,5);	 //PI3,AF5
		//PI1---SPI2_SCK
		GPIO_Set(GPIOI,PIN1,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PG11����
		GPIO_AF_Set(GPIOI,1,5); //PI1,AF5 

	
		//////////////////SPI4/////////////////////////////////////////////
		RCC->APB2ENR|=1<<13;        //ʹ��SPI4ʱ��
		RCC->AHB1ENR|=1<<4;	        //ʹ��PORTIʱ�� 

		//PE13---SPI4_MISO 
		GPIO_Set(GPIOE,PIN13,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PA6����
		GPIO_AF_Set(GPIOE,13,5);	 //PE13,AF5
		//PE6---SPI1_SCK
		GPIO_Set(GPIOE,PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PD7����
		GPIO_AF_Set(GPIOE,6,5);	 //PE6,AF5
		//PE2---SPI1_MOSI
		GPIO_Set(GPIOE,PIN2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PG11����
		GPIO_AF_Set(GPIOE,2,5); //PE2,AF5 
	
	
}




/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
**��    �� : SPI1������ʼ��
**��    �� : None
**��    �� : None
**��    ע : 
**====================================================================================================*/
/*====================================================================================================*/
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{

	SPI_BaudRatePrescaler&=0X07;			//���Ʒ�Χ
	SPI1->CR1&=0XFFC7; 
	SPI1->CR1|=SPI_BaudRatePrescaler<<3;	//����SPI1�ٶ�  
	SPI1->CR1|=1<<6; 		//SPI�豸ʹ��	 
   
}

/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : void SPI2_SetSpeed(uint8_t SPI_BaudRatePrescaler)
**��    �� : SPI1������ʼ��
**��    �� : None
**��    �� : None
**��    ע : 
**====================================================================================================*/
/*====================================================================================================*/
void SPI2_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{

	SPI_BaudRatePrescaler&=0X07;			//���Ʒ�Χ
	SPI2->CR1&=0XFFC7; 
	SPI2->CR1|=SPI_BaudRatePrescaler<<3;	//����SPI2�ٶ�  
	SPI2->CR1|=1<<6; 		//SPI�豸ʹ��	 
   
}


/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : void SPI4_SetSpeed(uint8_t SPI_BaudRatePrescaler)
**��    �� : SPI1������ʼ��
**��    �� : None
**��    �� : None
**��    ע : 
**====================================================================================================*/
/*====================================================================================================*/
void SPI4_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{

	SPI_BaudRatePrescaler&=0X07;			//���Ʒ�Χ
	SPI4->CR1&=0XFFC7; 
	SPI4->CR1|=SPI_BaudRatePrescaler<<3;	//����SPI1�ٶ�  
	SPI4->CR1|=1<<6; 		//SPI�豸ʹ��	 
   
}

/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : uint8_t SPI1_ReadWriteByte(uint8_t TxData)
**��    �� : SPI1������ȡ
**��    �� : None
**��    �� : None
**��    ע : 
**====================================================================================================*/
/*====================================================================================================*/
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
  uint8_t Rxdata;
  HAL_SPI_TransmitReceive(&SPI1_Handler,&TxData,&Rxdata,1, 1000);
  return Rxdata; 
}

/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : uint8_t SPI1_ReadWriteByte(uint8_t TxData)
**��    �� : SPI1������ȡ
**��    �� : None
**��    �� : None
**��    ע : 
**====================================================================================================*/
/*====================================================================================================*/
uint8_t SPI2_ReadWriteByte(uint8_t TxData)
{

  uint8_t Rxdata;
  HAL_SPI_TransmitReceive(&SPI2_Handler,&TxData,&Rxdata,1, 1000);
  return Rxdata; 
      	
}


/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : uint8_t SPI1_ReadWriteByte(uint8_t TxData)
**��    �� : SPI1������ȡ
**��    �� : None
**��    �� : None
**��    ע : 
**====================================================================================================*/
/*====================================================================================================*/
uint8_t SPI4_ReadWriteByte(uint8_t TxData)
{
//  uint8_t Rxdata;
//  HAL_SPI_TransmitReceive(&SPI4_Handler,&TxData,&Rxdata,1, 1000);
//  return Rxdata; 
	while((SPI4->SR&1<<1)==0);		//�ȴ��������� 
	SPI4->DR=TxData;	 	  		//����һ��byte  
	while((SPI4->SR&1<<0)==0);		//�ȴ�������һ��byte  
 	return SPI4->DR;          	

}

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/
