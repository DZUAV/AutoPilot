#ifndef  __DRV_SENSOR__H__
#define  __DRV_SENSOR__H__
#include "mathtool.h"
#include "drv_spi.h"

//	PB4:  SPI1_DRDY1_ICM20689
//	PB14: SPI1_DRDY2_BMI055_GYRO
//	PB15: SPI1_DRDY3_BMI055_ACC
//	PC5:  SPI1_DRDY4_ICM20602
//	PC13: SPI1_DRDY5_BMI055_GYRO
//	PD10: SPI1_DRDY6_BMI055_ACC
enum 
{
	//加速度+陀螺仪
  IMU_MPU6000, 
	IMU_MPU6500,
	IMU_MPU6050,
	IMU_MPU9250,
	IMU_MPU9150,	
  IMU_ICM20602,
  IMU_ICM20689,
	IMU_BMI055,
	//陀螺仪
  IMU_L3GD20,
  //地磁
  IMU_IST8310,
  IMU_HMC5883L,
  IMU_HMC5983,
	IMU_LSM303D,
	//气压计
  IMU_MS5611,
  IMU_SPL_06_001,

}	;

enum SPI_BUS_NUM
{
  IMU_SPI1_BUS1,
  IMU_SPI2_BUS2,
	IMU_SPI3_BUS3,
	IMU_SPI4_BUS4,
  IMU_SPI5_BUS5,

}	;	

enum IIC_BUS_NUM
{
  IMU_IIC1_BUS1,
  IMU_IIC2_BUS2,
	IMU_IIC3_BUS3,
	IMU_IIC4_BUS4,
	IMU_NO_IIC_BUS,

}	;	
enum CAN_BUS_NUM
{
  IMU_CAN1_BUS1,
  IMU_CAN2_BUS2,
	IMU_CAN3_BUS3,
	IMU_CAN4_BUS4,

}	;	

enum IMU_BUS
{
 IIC_BUS,
 SPI_BUS,
 CAN_BUS,

};


typedef struct 
{
  
  uint8_t imu_ii_flag;
  uint8_t imu_spi_flag;
  uint8_t imu_can_flag;
}IMUBUS_Recog_flag;

typedef struct 
{
  
  uint8_t ii1_flag;
  uint8_t ii2_flag;
  uint8_t ii3_flag;
  uint8_t ii4_flag;
}IMU_IIC_FLAG;

typedef struct 
{
  
  uint8_t spi1_flag;
  uint8_t spi2_flag;
  uint8_t spi3_flag;
  uint8_t spi4_flag;
	uint8_t spi5_flag;
}IMU_SPI_FLAG;

typedef struct 
{
  
  uint8_t can1_flag;
  uint8_t can2_flag;
  uint8_t can3_flag;
  uint8_t can4_flag;
}IMU_CAN_FLAG;

typedef struct 
{
 uint8_t imu_mpu6000_flag;
 uint8_t imu_mpu6500_flag;
 uint8_t imu_mpu6050_flag;
 uint8_t imu_mpu9250_flag;
 uint8_t imu_mpu9150_flag;
 uint8_t imu_icm20602_flag;
 uint8_t imu_icm20689_flag;
 uint8_t imu_bmi055_flag;
 uint8_t imu_l3gd20_flag;
 uint8_t imu_ist8310_flag;
 uint8_t imu_hmc583l_flag;
 uint8_t imu_hmc5983_flag;
 uint8_t imu_lsm303d_flag;
 uint8_t imu_ms5611_flag;
 uint8_t imu_spl_06_001_flag;

}IMU_SENSOR_NUM_FLAG;


//设置ICM20602片选
#define  ICM20602_ON_CS()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_RESET) //PF3:SPI1_CS2_ICM20602
#define  ICM20602_OFF_CS()   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_SET)

//设置ICM20689片选
#define  ICM20689_ON_CS()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET) //PF2:SPI1_CS1_ICM20689
#define  ICM20689_OFF_CS()   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET)

//设置MPU6000片选
#define  MPU6000_ON_CS()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET) //PF10: SPI4_MS5611_CS
#define  MPU6000_OFF_CS()   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET)

//设置BMI055 GYRO片选
#define  BMI055_GYRO_ON_CS()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_RESET) //PF4:SPI1_CS3_BMI055_GYRO
#define  BMI055_GYRO_OFF_CS()   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_SET)

//设置BMI055 ACC片选
#define  BMI055_ACC_ON_CS()    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_10,GPIO_PIN_RESET)  //PG10:SPI1_CS4_BMI055_ACC
#define  BMI055_ACC_OFF_CS()   HAL_GPIO_WritePin(GPIOG,GPIO_PIN_10,GPIO_PIN_SET)

//设置MS5611片选
#define  MS5611_ON_CS()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET) //PF10: SPI4_MS5611_CS
#define  MS5611_OFF_CS()   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET)

//设置FM25V01G片选
#define  FM25V01G_ON_CS()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_RESET) //PF5: SPI2_fm25v01g_CS
#define  FM25V01G_OFF_CS()   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_SET)

uint8_t Sensor_Self_Delect(IMUBUS_Recog_flag imu_bus_flag,uint8_t imubus,IMU_IIC_FLAG imu_iic_flag,uint8_t iic_num,IMU_SPI_FLAG imu_spi_flag,
                           uint8_t spi_num,IMU_SENSOR_NUM_FLAG imu_sensor_num_flag,uint8_t imu_sensor_num);
#endif
