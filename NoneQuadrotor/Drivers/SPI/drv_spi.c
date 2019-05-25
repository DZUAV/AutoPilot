/**********************************************************************************************************
*文件说明：SPI驱动文件
*实现功能：配置SPI
*修改日期：2018-10-31
*修改作者：cihuang_uav
*修改备注：
*备注信息：SPI1引脚：
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
										
*		       SPI4引脚：	PE13---SPI4-MISO		
*                     PE2---SPI4-SCK			
*                     PE6---SPI4-MOSI			
*                     PF10---SPI4_CS_MS5611	
**********************************************************************************************************/

#include "drv_spi.h"

#include "copter.h"
#include "stm32f7xx_hal_spi.h"

SPI_HandleTypeDef SPI1_Handler;  //SPI1句柄; 
SPI_HandleTypeDef SPI2_Handler;  //SPI2句柄; 
SPI_HandleTypeDef SPI4_Handler;  //SPI4句柄; 







/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void SPI1_Init_CS(void)
**功    能 : SPI1函数初始化
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/
void SPI1_Init_CS(void)
{
	RCC->AHB1ENR|=1<<5;	//使能PORTF时钟 
	//SPI1_CS1
	GPIO_Set(SPI1_CS1_ICM20689_GPIO,SPI1_CS1_ICM20689_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF2设置

	//SPI1_CS2
	GPIO_Set(SPI1_CS2_ICM20602_GPIO,SPI1_CS2_ICM20602_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF2设置
	//SPI1_CS3
	GPIO_Set(SPI1_CS3_BMI055_GYRO_GPIO,SPI1_CS3_BMI055_GYRO_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF4设置
	
   //SPI1_CS4
	GPIO_Set(SPI1_CS4_BMI055_ACC_GPIO,SPI1_CS4_BMI055_ACC_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PG10设置
	//片选不选中
	GPIO_Pin_Set(SPI1_CS1_ICM20689_GPIO,SPI1_CS1_ICM20689_PIN,1);
	GPIO_Pin_Set(SPI1_CS2_ICM20602_GPIO,SPI1_CS2_ICM20602_PIN,1);
	GPIO_Pin_Set(SPI1_CS3_BMI055_GYRO_GPIO,SPI1_CS3_BMI055_GYRO_PIN,1);
	GPIO_Pin_Set(SPI1_CS4_BMI055_ACC_GPIO,SPI1_CS4_BMI055_ACC_PIN,1);
}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void SPI2_Init_CS(void)
**功    能 : SPI1函数初始化
**输    入 : None
**输    出 : None
**备    注 : PF5 SPI2_CS 
**====================================================================================================*/
/*====================================================================================================*/
void SPI2_Init_CS(void)
{
	RCC->AHB1ENR|=1<<5;	//使能PORTF时钟 

	//SPI2_CS:FM25V01G
	GPIO_Set(SPI2_CS1_FM25V01G_GPIO,SPI2_CS1_FM25V01G_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF5设置
	//片选不选中
	GPIO_Pin_Set(SPI2_CS1_FM25V01G_GPIO,SPI2_CS1_FM25V01G_PIN,1);
}


/**********************************************************************************************************
*函数原型: void SPI4_Init_CS(void)
*函数功能: spi4的GPIO初始化片选设置
*输入参数: none
*返回数据: none
*修改日期: 2018-11-26
*修改作者: 
*		       SPI4引脚：	PE13---SPI4-MISO		
*                           PE2---SPI4-SCK			
*                           PE6---SPI4-MOSI			
*                           PF10---SPI4_CS_MS5611	
**********************************************************************************************************/
void SPI4_Init_CS(void)
{
	RCC->AHB1ENR|=1<<5;	//使能PORTF时钟 
	//SPI4_CS1
	GPIO_Set(SPI4_CS1_MS5611_GPIO,SPI4_CS1_MS5611_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF10设置

	//片选不选中
	GPIO_Pin_Set(SPI4_CS1_MS5611_GPIO,SPI4_CS1_MS5611_PIN,1);

}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void SPI1_Init(void)
**功    能 : SPI1函数初始化
**输    入 : None
**输    出 : None
**备    注 :   
**====================================================================================================*/
/*====================================================================================================*/
void SPI1_Init(void)
{

    RCC->APB2ENR|=1<<12;        //使能SPI1时钟
    SPI1_Handler.Instance=SPI1;                         //SP1
    SPI1_Handler.Init.Mode=SPI_MODE_MASTER;             //设置SPI工作模式，设置为主模式
    SPI1_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //设置SPI单向或者双向的数据模式:SPI设置为双线模式
    SPI1_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //设置SPI的数据大小:SPI发送接收8位帧结构
    SPI1_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH;    //串行同步时钟的空闲状态为高电平
    SPI1_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;         //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI1_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI1_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//定义波特率预分频的值:波特率预分频值为256
    SPI1_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI1_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //关闭TI模式
    SPI1_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//关闭硬件CRC校验
    SPI1_Handler.Init.CRCPolynomial=7;                  //CRC值计算的多项式
	
    HAL_SPI_Init(&SPI1_Handler);//初始化
    SPI1->CR1|=1<<6; 		//SPI设备使能	 
    SPI1_ReadWriteByte(0Xff);                           //启动传输
    SPI1_SetSpeed(SPI_SPEED_256);    //设置为45M时钟,高速模式
	  SPI1_Init_CS();
}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void SPI2_Init(void)
**功    能 : SPI2函数初始化
**输    入 : None
**输    出 : None
**备    注 : PI1 SPI2_SCK SPI2
             PI2 SPI2_MISO SPI2
             PI3 SPI2_MOSI SPI2   
**====================================================================================================*/
/*====================================================================================================*/
void SPI2_Init(void)
{

    RCC->APB1ENR|=1<<14;        //使能SPI2时钟
    SPI2_Handler.Instance=SPI2;                         //SP1
    SPI2_Handler.Init.Mode=SPI_MODE_MASTER;             //设置SPI工作模式，设置为主模式
    SPI2_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //设置SPI单向或者双向的数据模式:SPI设置为双线模式
    SPI2_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //设置SPI的数据大小:SPI发送接收8位帧结构
    SPI2_Handler.Init.CLKPolarity=SPI_POLARITY_LOW;    //串行同步时钟的空闲状态为高电平
    SPI2_Handler.Init.CLKPhase=SPI_PHASE_1EDGE;         //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI2_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI2_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//定义波特率预分频的值:波特率预分频值为256
    SPI2_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI2_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //关闭TI模式
    SPI2_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//关闭硬件CRC校验
    SPI2_Handler.Init.CRCPolynomial=7;                  //CRC值计算的多项式
	
    HAL_SPI_Init(&SPI2_Handler);//初始化
    SPI2->CR1|=1<<6; 		//SPI设备使能	 
    SPI2_ReadWriteByte(0Xff);                           //启动传输
    SPI2_SetSpeed(SPI_SPEED_256);    //设置为45M时钟,高速模式
	  SPI2_Init_CS();
}
/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void SPI4_Init(void)
**功    能 : SPI4函数初始化
**输    入 : None
**输    出 : None
**备    注 :   
**====================================================================================================*/
/*====================================================================================================*/
void SPI4_Init(void)
{
 
    RCC->APB2ENR|=1<<13;        //使能SPI4时钟
    SPI4_Handler.Instance=SPI4;                         //SP4
    SPI4_Handler.Init.Mode=SPI_MODE_MASTER;             //设置SPI工作模式，设置为主模式
    SPI4_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //设置SPI单向或者双向的数据模式:SPI设置为双线模式
    SPI4_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //设置SPI的数据大小:SPI发送接收8位帧结构
    SPI4_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH;    //串行同步时钟的空闲状态为高电平
    SPI4_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;         //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI4_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI4_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//定义波特率预分频的值:波特率预分频值为256
    SPI4_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI4_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //关闭TI模式
    SPI4_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//关闭硬件CRC校验
    SPI4_Handler.Init.CRCPolynomial=7;                  //CRC值计算的多项式
	
    HAL_SPI_Init(&SPI4_Handler);//初始化
    SPI4->CR1|=1<<6; 		      //SPI4设备使能	 
    SPI4_ReadWriteByte(0Xff);                           //启动传输
    SPI4_SetSpeed(SPI_SPEED_256);    //设置为45M时钟,高速模式
	  SPI4_Init_CS();
}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
**功    能 : SPI1函数初始化
**输    入 : None
**输    出 : None
**备    注 : PE13---SPI4-MISO			
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
		RCC->APB2ENR|=1<<12;        //使能SPI1时钟
		RCC->AHB1ENR|=1<<0;	        //使能PORTA时钟 
		RCC->AHB1ENR|=1<<3;	        //使能PORTD时钟 
		RCC->AHB1ENR|=1<<6;	        //使能PORTG时钟 

		//PA6---SPI1_MISO 
		GPIO_Set(GPIOA,PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PA6设置
		 GPIO_AF_Set(GPIOA,6,5);	 //PA6,AF5
		//PD7---SPI1_MOSI
		GPIO_Set(GPIOD,PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PD7设置
		GPIO_AF_Set(GPIOD,7,5);	 //PD7,AF5
		//PG11---SPI1_SCK
		GPIO_Set(GPIOG,PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PG11设置
		GPIO_AF_Set(GPIOG,11,5); //PG11,AF5 
	
		//////////////////SPI2/////////////////////////////////////////////

		RCC->APB1ENR|=1<<14;         //使能SPI2时钟
		RCC->AHB1ENR|=1<<8;	         //使能PORTI时钟 

    
//      RCC->AHB1ENR|=1<<8;	                    //使能PORTI时钟 ;                   
//    __HAL_RCC_SPI2_CLK_ENABLE();                    //使能SPI2时钟
//    
//    //PB13,14,15
//    GPIO_Initure.Pin=GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
//    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //复用推挽输出
//    GPIO_Initure.Pull=GPIO_PULLUP;                  //上拉
//    GPIO_Initure.Speed=GPIO_SPEED_FAST;             //快速    
//    GPIO_Initure.Alternate=GPIO_AF5_SPI2;           //复用为SPI2
//    HAL_GPIO_Init(GPIOI,&GPIO_Initure);             //初始化
		//PI2---SPI2_MISO 
		GPIO_Set(GPIOI,PIN2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PA6设置
		GPIO_AF_Set(GPIOI,2,5);	 //PI2,AF5
		//PI3---SPI2_MOSI
		GPIO_Set(GPIOI,PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PD7设置
		GPIO_AF_Set(GPIOI,3,5);	 //PI3,AF5
		//PI1---SPI2_SCK
		GPIO_Set(GPIOI,PIN1,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PG11设置
		GPIO_AF_Set(GPIOI,1,5); //PI1,AF5 

	
		//////////////////SPI4/////////////////////////////////////////////
		RCC->APB2ENR|=1<<13;        //使能SPI4时钟
		RCC->AHB1ENR|=1<<4;	        //使能PORTI时钟 

		//PE13---SPI4_MISO 
		GPIO_Set(GPIOE,PIN13,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PA6设置
		GPIO_AF_Set(GPIOE,13,5);	 //PE13,AF5
		//PE6---SPI1_SCK
		GPIO_Set(GPIOE,PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PD7设置
		GPIO_AF_Set(GPIOE,6,5);	 //PE6,AF5
		//PE2---SPI1_MOSI
		GPIO_Set(GPIOE,PIN2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PG11设置
		GPIO_AF_Set(GPIOE,2,5); //PE2,AF5 
	
	
}




/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
**功    能 : SPI1函数初始化
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{

	SPI_BaudRatePrescaler&=0X07;			//限制范围
	SPI1->CR1&=0XFFC7; 
	SPI1->CR1|=SPI_BaudRatePrescaler<<3;	//设置SPI1速度  
	SPI1->CR1|=1<<6; 		//SPI设备使能	 
   
}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void SPI2_SetSpeed(uint8_t SPI_BaudRatePrescaler)
**功    能 : SPI1函数初始化
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/
void SPI2_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{

	SPI_BaudRatePrescaler&=0X07;			//限制范围
	SPI2->CR1&=0XFFC7; 
	SPI2->CR1|=SPI_BaudRatePrescaler<<3;	//设置SPI2速度  
	SPI2->CR1|=1<<6; 		//SPI设备使能	 
   
}


/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void SPI4_SetSpeed(uint8_t SPI_BaudRatePrescaler)
**功    能 : SPI1函数初始化
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/
void SPI4_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{

	SPI_BaudRatePrescaler&=0X07;			//限制范围
	SPI4->CR1&=0XFFC7; 
	SPI4->CR1|=SPI_BaudRatePrescaler<<3;	//设置SPI1速度  
	SPI4->CR1|=1<<6; 		//SPI设备使能	 
   
}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : uint8_t SPI1_ReadWriteByte(uint8_t TxData)
**功    能 : SPI1函数读取
**输    入 : None
**输    出 : None
**备    注 : 
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
**函数原型 : uint8_t SPI1_ReadWriteByte(uint8_t TxData)
**功    能 : SPI1函数读取
**输    入 : None
**输    出 : None
**备    注 : 
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
**函数原型 : uint8_t SPI1_ReadWriteByte(uint8_t TxData)
**功    能 : SPI1函数读取
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/
uint8_t SPI4_ReadWriteByte(uint8_t TxData)
{
//  uint8_t Rxdata;
//  HAL_SPI_TransmitReceive(&SPI4_Handler,&TxData,&Rxdata,1, 1000);
//  return Rxdata; 
	while((SPI4->SR&1<<1)==0);		//等待发送区空 
	SPI4->DR=TxData;	 	  		//发送一个byte  
	while((SPI4->SR&1<<0)==0);		//等待接收完一个byte  
 	return SPI4->DR;          	

}

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/
