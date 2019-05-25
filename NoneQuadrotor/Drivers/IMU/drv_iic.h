#ifndef __DRV_IIC_H__
#define __DRV_IIC_H__
#include "sys.h"


//IO方向设置
#define IIC3_SDA_IN()  {GPIOH->MODER&=~(3<<(8*2));GPIOH->MODER|=0<<8*2;}	//PH7输入模式
#define IIC3_SDA_OUT() {GPIOH->MODER&=~(3<<(8*2));GPIOH->MODER|=1<<8*2;}    //PH7输出模式
//IO操作函数	 
#define IIC3_SCL(x)		GPIO_Pin_Set(GPIOH,PIN7,x)		//SCL
#define IIC3_SDA(x)		GPIO_Pin_Set(GPIOH,PIN8,x)		//SDA
#define IIC3_READ_SDA	GPIO_Pin_Get(GPIOH,PIN8)   		//读取SDA
 


void IIC3_Init(void);
void IIC3_Start(void);
void IIC3_Stop(void);
uint8_t IIC3_Wait_Ack(void);
void IIC3_Ack(void);
void IIC3_NAck(void);
void IIC_Send_Byte(u8 txd);
uint8_t IIC3_Read_Byte(unsigned char ack);



uint8_t IIC3_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
uint8_t IIC3_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
uint8_t IIC3_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
uint8_t IIC3_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);












#endif
