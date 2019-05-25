/**********************************************************************************************************
*�ļ�˵����NoneQuadrotor UAV���п��ƴ���
*ʵ�ֹ��ܣ�ʵ�����˻���׼����
*�޸����ڣ�2018-11-17
*�޸����ߣ�crystal cup
*�޸ı�ע��
**********************************************************************************************************/
#include "drv_iic.h"
#include "copter.h"


/**********************************************************************************************************
*����ԭ��:void IIC3_Init(void)
*��������: ����ʼ������
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/

void IIC3_Init(void)
{					     
	RCC->AHB1ENR|=1<<7;    //ʹ��PORTHʱ��	   	  
	GPIO_Set(GPIOH,PIN7|PIN8,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PH7/PH8���� 
	IIC3_SCL(1);
	IIC3_SDA(1);
}

/**********************************************************************************************************
*����ԭ��:void IIC3_Start(void)
*��������:����IIC��ʼ�ź�
*�������: 
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/

void IIC3_Start(void)
{
	IIC3_SDA_OUT();	//sda�����
	IIC3_SDA(1);	  	  
	IIC3_SCL(1);
	delay_us(4);
 	IIC3_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC3_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**********************************************************************************************************
*����ԭ��:void IIC3_Stop(void)
*��������:����IICֹͣ�ź�
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/

void IIC3_Stop(void)
{
	IIC3_SDA_OUT();//sda�����
	IIC3_SCL(0);
	IIC3_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC3_SCL(1); 
	delay_us(4);	
	IIC3_SDA(1);//����I2C���߽����ź�						   	
}

/**********************************************************************************************************
*����ԭ��:uint8_t IIC3_Wait_Ack(void)
*��������: �ȴ�Ӧ���źŵ���
*�������: pvParameters
*��������: ����ֵ��1������Ӧ��ʧ��  0������Ӧ��ɹ�
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/

uint8_t IIC3_Wait_Ack(void)
{
	u8 ucErrTime=0;
	IIC3_SDA_IN();      //SDA����Ϊ����  
	IIC3_SDA(1);delay_us(1);	   
	IIC3_SCL(1);delay_us(1);	 
	while(IIC3_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC3_Stop();
			return 1;
		}
	}
	IIC3_SCL(0);//ʱ�����0 	   
	return 0;  
} 

/**********************************************************************************************************
*����ԭ��:void IIC3_Ack(void)
*��������:����ACKӦ��
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/

void IIC3_Ack(void)
{
	IIC3_SCL(0);
	IIC3_SDA_OUT();
	IIC3_SDA(0);
	delay_us(2);
	IIC3_SCL(1);
	delay_us(2);
	IIC3_SCL(0);
}

/**********************************************************************************************************
*����ԭ��:void IIC3_NAck(void)
*��������:������ACKӦ��		
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/

void IIC3_NAck(void)
{
	IIC3_SCL(0);
	IIC3_SDA_OUT();
	IIC3_SDA(1);
	delay_us(2);
	IIC3_SCL(1);
	delay_us(2);
	IIC3_SCL(0);
}	

/**********************************************************************************************************
*����ԭ��:void IIC3_Send_Byte(u8 txd)
*��������:IIC����һ���ֽ�
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
          ���شӻ�����Ӧ��
            1����Ӧ��
            0����Ӧ��		
**********************************************************************************************************/
	  
void IIC3_Send_Byte(u8 txd)
{                        
    u8 t;   
	  IIC3_SDA_OUT(); 	    
    IIC3_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC3_SDA((txd&0x80)>>7);
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC3_SCL(1);
		delay_us(2); 
		IIC3_SCL(0);	
		delay_us(2);
    }	 
} 	

/**********************************************************************************************************
*����ԭ��:uint8_t IIC3_Read_Byte(unsigned char ack)
*��������: ����ʼ������
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ����1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK  
**********************************************************************************************************/

uint8_t IIC3_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC3_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC3_SCL(0); 
        delay_us(2);
		IIC3_SCL(1);
        receive<<=1;
        if(IIC3_READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC3_NAck();//����nACK
    else
        IIC3_Ack(); //����ACK   
    return receive;
}


/**********************************************************************************************************
*����ԭ��:uint8_t IIC3_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
*��������: ����ʼ������
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ����1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK  
**********************************************************************************************************/
uint8_t IIC3_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	IIC3_Start();
	IIC3_Send_Byte(SlaveAddress<<1);   
	if(IIC3_Wait_Ack())
	{
		IIC3_Stop();
		return 1;
	}
	IIC3_Send_Byte(REG_Address);       
	IIC3_Wait_Ack();	
	IIC3_Send_Byte(REG_data);
	IIC3_Wait_Ack();   
	IIC3_Stop(); 
	return 0;
}

/**********************************************************************************************************
*����ԭ��:uint8_t IIC3_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
*��������: ����ʼ������
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ����1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK  
**********************************************************************************************************/
uint8_t IIC3_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
{      		
	IIC3_Start();
	IIC3_Send_Byte(SlaveAddress<<1); 
	if(IIC3_Wait_Ack())
	{
		IIC3_Stop();
		return 1;
	}
	IIC3_Send_Byte(REG_Address);     
	IIC3_Wait_Ack();
	IIC3_Start();
	IIC3_Send_Byte(SlaveAddress<<1 | 0x01);
	IIC3_Wait_Ack();
	*REG_data= IIC3_Read_Byte(0);
	IIC3_Stop();
	return 0;
}	

/**********************************************************************************************************
*����ԭ��:uint8_t IIC3_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
*��������: ����ʼ������
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ����1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK  
**********************************************************************************************************/
uint8_t IIC3_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	IIC3_Start();
	IIC3_Send_Byte(SlaveAddress<<1); 
	if(IIC3_Wait_Ack())
	{
		IIC3_Stop();
		return 1;
	}
	IIC3_Send_Byte(REG_Address); 
	IIC3_Wait_Ack();
	while(len--) 
	{
		IIC3_Send_Byte(*buf++); 
		IIC3_Wait_Ack();
	}
	IIC3_Stop();
	return 0;
}
/**********************************************************************************************************
*����ԭ��:uint8_t IIC3_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
*��������: ����ʼ������
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ����1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK  
**********************************************************************************************************/
uint8_t IIC3_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	IIC3_Start();
	IIC3_Send_Byte(SlaveAddress<<1); 
	if(IIC3_Wait_Ack())
	{
		IIC3_Stop();
		return 1;
	}
	IIC3_Send_Byte(REG_Address); 
	IIC3_Wait_Ack();
	
	IIC3_Start();
	IIC3_Send_Byte(SlaveAddress<<1 | 0x01); 
	IIC3_Wait_Ack();
	while(len) 
	{
		if(len == 1)
		{
			*buf = IIC3_Read_Byte(0);
		}
		else
		{
			*buf = IIC3_Read_Byte(1);
		}
		buf++;
		len--;
	}
	IIC3_Stop();
	return 0;
}

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/

