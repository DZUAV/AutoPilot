/**********************************************************************************************************
*文件说明：传感器配置驱动文件
*实现功能：配置传感器
*修改日期：2018-10-31
*修改作者：cihuang_uav
*修改备注：
**********************************************************************************************************/
#include "drv_sensor.h"
#include "stdint.h"



/********************************************************************************************************************************************************************************************
*函数原型: uint8_t Sensor_Self_Recognition(uint8_t chip_bus_type)
*函数功能: 传感器选择
*输入参数: imu_bus_flag:选择你想要的总线,imubus：总线号；
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
******************************************************************************************************************************************************************************************/
uint8_t Sensor_Self_Delect(IMUBUS_Recog_flag imu_bus_flag,uint8_t imubus,IMU_IIC_FLAG imu_iic_flag,uint8_t iic_num,IMU_SPI_FLAG imu_spi_flag,
                           uint8_t spi_num,IMU_SENSOR_NUM_FLAG imu_sensor_num_flag,uint8_t imu_sensor_num)
{
    switch(imubus)
	{
	
			case IIC_BUS:
	                  imu_bus_flag.imu_ii_flag=1;
	                  break;
			case SPI_BUS:
	                  imu_bus_flag.imu_spi_flag=1;
	                  break;
			case CAN_BUS:
	                  imu_bus_flag.imu_can_flag=1;
	                  break;
			default:
				          imu_bus_flag.imu_ii_flag=0;
			              imu_bus_flag.imu_spi_flag=0;
			              imu_bus_flag.imu_can_flag=0;
				            break;
	
	}
	if(imu_bus_flag.imu_ii_flag==1)
	{
			 switch(iic_num)
			{
			
					case IMU_IIC1_BUS1:
												imu_iic_flag.ii1_flag=1;
												break;
					case IMU_IIC2_BUS2:
												imu_iic_flag.ii2_flag=1;
												break;
					case IMU_IIC3_BUS3:
												imu_iic_flag.ii3_flag=1;
												break;
					case IMU_IIC4_BUS4:
												imu_iic_flag.ii4_flag=1;
												break;
					default:
                        imu_iic_flag.ii1_flag=0;
                        imu_iic_flag.ii2_flag=0;
                        imu_iic_flag.ii3_flag=0;
                        imu_iic_flag.ii4_flag=0;							 
				                 break;
			}
	
	}
	else if(imu_bus_flag.imu_spi_flag==1)
	{
	       switch(spi_num)
			{
			
					case IMU_SPI1_BUS1:
												imu_spi_flag.spi1_flag=1;
												break;
					case IMU_SPI2_BUS2:
												imu_spi_flag.spi2_flag=1;
												break;
					case IMU_SPI3_BUS3:
												imu_spi_flag.spi3_flag=1;
												break;
					case IMU_SPI4_BUS4:
												imu_spi_flag.spi4_flag=1;
												break;
					case IMU_SPI5_BUS5:
												imu_spi_flag.spi5_flag=1;
												break;
						   default:
                        imu_spi_flag.spi1_flag=0;
                        imu_spi_flag.spi2_flag=0;
                        imu_spi_flag.spi3_flag=0;
                        imu_spi_flag.spi4_flag=0;	
                        imu_spi_flag.spi5_flag=0;								 
				                 break;
			}
	
	}

	switch(imu_sensor_num)
	{
	   case IMU_MPU6000:
			               imu_sensor_num_flag.imu_mpu6000_flag=1;
			                 break;
		case IMU_MPU6500:
			               imu_sensor_num_flag.imu_mpu6500_flag=1;
			                 break;
		case IMU_MPU6050:
						         imu_sensor_num_flag.imu_mpu6050_flag=1;
			                 break;
		case IMU_MPU9250:
									   imu_sensor_num_flag.imu_mpu9250_flag=1;
			                 break;
		case IMU_MPU9150:
									   imu_sensor_num_flag.imu_mpu9150_flag=1;
			                 break;
		case IMU_ICM20602:
									   imu_sensor_num_flag.imu_icm20602_flag=1;
			                 break;
		case IMU_ICM20689:
									   imu_sensor_num_flag.imu_icm20689_flag=1;
			                 break;
		case IMU_BMI055:
									   imu_sensor_num_flag.imu_bmi055_flag=1;
			                 break;
		case IMU_L3GD20:
									   imu_sensor_num_flag.imu_l3gd20_flag=1;
			                 break;
		case IMU_IST8310:
									   imu_sensor_num_flag.imu_ist8310_flag=1;
			                 break;
		case IMU_HMC5883L:
									   imu_sensor_num_flag.imu_hmc583l_flag=1;
			                 break;
		case IMU_HMC5983:
									   imu_sensor_num_flag.imu_hmc5983_flag=1;
			                 break;
		case IMU_LSM303D:
									   imu_sensor_num_flag.imu_lsm303d_flag=1;
			                 break;
		case IMU_MS5611:
									   imu_sensor_num_flag.imu_ms5611_flag=1;
			                 break;
		case IMU_SPL_06_001:
									   imu_sensor_num_flag.imu_spl_06_001_flag=1;
                       break;
		default:

				              break;
	
	}

  return 1;


}



/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/