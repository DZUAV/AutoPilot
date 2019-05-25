#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "sys.h"
#include "stm32f7xx_hal.h"
extern SPI_HandleTypeDef SPI1_Handler;  //SPI���; 
extern SPI_HandleTypeDef SPI4_Handler;  //SPI4���; 
extern SPI_HandleTypeDef SPI2_Handler;  //SPI2���; 
// SPI�����ٶ����� 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7

//#define   GPIO_PIN_RESET  0
//#define   GPIO_PIN_SET   1

//spi1 cs�˿�
#define SPI1_CS1_ICM20689_GPIO         GPIOF   
#define SPI1_CS2_ICM20602_GPIO         GPIOF   
#define SPI1_CS3_BMI055_GYRO_GPIO      GPIOF  
#define SPI1_CS4_BMI055_ACC_GPIO       GPIOG 
//spi2 cs�˿�
#define SPI2_CS1_FM25V01G_GPIO         GPIOF   


//spi4 cs�˿�
#define SPI4_CS1_MS5611_GPIO           GPIOF  


//spi1 cs����
#define SPI1_CS1_ICM20689_PIN         PIN2  
#define SPI1_CS2_ICM20602_PIN         PIN3   
#define SPI1_CS3_BMI055_GYRO_PIN      PIN4   
#define SPI1_CS4_BMI055_ACC_PIN       PIN10   


//spi2 cs1����
#define SPI2_CS1_FM25V01G_PIN         PIN5  

//spi1 cs����
#define SPI4_CS1_MS5611_PIN           PIN10   







/***************************************************************************************************************************************************************************
                                     �궨�忪ʼ
***************************************************************************************************************************************************************************/

/*************************************************************************************************************************************************************************
                                     ����������ʼ
*************************************************************************************************************************************************************************/
//spi1�ļ�����

void SPI1_Init(void);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler);
void SPI1_Init_CS(void);
uint8_t SPI1_ReadWriteByte(uint8_t TxData);

//spi2�ļ�����
void SPI2_Init(void);
void SPI2_SetSpeed(uint8_t SPI_BaudRatePrescaler);
uint8_t SPI2_ReadWriteByte(uint8_t TxData);


//spi4�ļ�����
void SPI4_Init_CS(void);
void SPI4_Init(void);
void SPI4_SetSpeed(uint8_t SPI_BaudRatePrescaler);
uint8_t SPI4_ReadWriteByte(uint8_t TxData);

void Spi_BaroSingleWrite(uint8_t reg, uint8_t value);
void Spi_BaroMultiRead(uint8_t reg,uint8_t *data, uint8_t length);
void SPI4_MultiWriteAndRead( uint8_t *out, uint8_t *in, int len);
/*************************************************************************************************************************************************************************
                                     ������������
*************************************************************************************************************************************************************************/



#endif
