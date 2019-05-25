/**********************************************************************************************************
*文件说明：NoneQuadrotor UAV飞行控制代码
*实现功能：实现无人机精准控制
*修改日期：2018-11-17
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/
#include "drv_iic.h"
#include "copter.h"


/**********************************************************************************************************
*函数原型:void IIC3_Init(void)
*函数功能: 板层初始化配置
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void IIC3_Init(void)
{					     
	RCC->AHB1ENR|=1<<7;    //使能PORTH时钟	   	  
	GPIO_Set(GPIOH,PIN7|PIN8,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PH7/PH8设置 
	IIC3_SCL(1);
	IIC3_SDA(1);
}

/**********************************************************************************************************
*函数原型:void IIC3_Start(void)
*函数功能:产生IIC起始信号
*输入参数: 
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void IIC3_Start(void)
{
	IIC3_SDA_OUT();	//sda线输出
	IIC3_SDA(1);	  	  
	IIC3_SCL(1);
	delay_us(4);
 	IIC3_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC3_SCL(0);//钳住I2C总线，准备发送或接收数据 
}

/**********************************************************************************************************
*函数原型:void IIC3_Stop(void)
*函数功能:产生IIC停止信号
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void IIC3_Stop(void)
{
	IIC3_SDA_OUT();//sda线输出
	IIC3_SCL(0);
	IIC3_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC3_SCL(1); 
	delay_us(4);	
	IIC3_SDA(1);//发送I2C总线结束信号						   	
}

/**********************************************************************************************************
*函数原型:uint8_t IIC3_Wait_Ack(void)
*函数功能: 等待应答信号到来
*输入参数: pvParameters
*返回数据: 返回值：1，接收应答失败  0，接收应答成功
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

uint8_t IIC3_Wait_Ack(void)
{
	u8 ucErrTime=0;
	IIC3_SDA_IN();      //SDA设置为输入  
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
	IIC3_SCL(0);//时钟输出0 	   
	return 0;  
} 

/**********************************************************************************************************
*函数原型:void IIC3_Ack(void)
*函数功能:产生ACK应答
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
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
*函数原型:void IIC3_NAck(void)
*函数功能:不产生ACK应答		
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
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
*函数原型:void IIC3_Send_Byte(u8 txd)
*函数功能:IIC发送一个字节
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
          返回从机有无应答
            1，有应答
            0，无应答		
**********************************************************************************************************/
	  
void IIC3_Send_Byte(u8 txd)
{                        
    u8 t;   
	  IIC3_SDA_OUT(); 	    
    IIC3_SCL(0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC3_SDA((txd&0x80)>>7);
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC3_SCL(1);
		delay_us(2); 
		IIC3_SCL(0);	
		delay_us(2);
    }	 
} 	

/**********************************************************************************************************
*函数原型:uint8_t IIC3_Read_Byte(unsigned char ack)
*函数功能: 板层初始化配置
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：读1个字节，ack=1时，发送ACK，ack=0，发送nACK  
**********************************************************************************************************/

uint8_t IIC3_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC3_SDA_IN();//SDA设置为输入
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
        IIC3_NAck();//发送nACK
    else
        IIC3_Ack(); //发送ACK   
    return receive;
}


/**********************************************************************************************************
*函数原型:uint8_t IIC3_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
*函数功能: 板层初始化配置
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：读1个字节，ack=1时，发送ACK，ack=0，发送nACK  
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
*函数原型:uint8_t IIC3_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
*函数功能: 板层初始化配置
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：读1个字节，ack=1时，发送ACK，ack=0，发送nACK  
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
*函数原型:uint8_t IIC3_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
*函数功能: 板层初始化配置
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：读1个字节，ack=1时，发送ACK，ack=0，发送nACK  
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
*函数原型:uint8_t IIC3_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
*函数功能: 板层初始化配置
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-11-17
*备注信息：读1个字节，ack=1时，发送ACK，ack=0，发送nACK  
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

