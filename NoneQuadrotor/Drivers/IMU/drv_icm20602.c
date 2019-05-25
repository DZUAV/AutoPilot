/**********************************************************************************************************
*文件说明：icm20602驱动文件
*实现功能：配置icm20602
*修改日期：2018-10-31
*修改作者：cihuang_uav
*修改备注：6自由度芯片,大小是3mm*3mm*0.75(16pin LGA)16位数据ADC
           LGA平面网格阵列封装
					 PGA:插针网格阵列封装
					 BGA:球栅网格阵列封装
					 Gyroerr: +-0.1%
					 Accerr 100ug/Hz
**********************************************************************************************************/
#include "drv_icm20602.h"
#include "copter.h"
#include "systick.h"

float sensor[9];

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : uint8_t ICM_20602_Init(void)
**功    能 : ICM_20602函数初始化
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/
uint8_t ICM_20602_Init(void)
{ 
	ICM_20602_WriteRegister(ICM20602_RA_PWR_MGMT_1,0X80);	//复位icm20602
	delay_ms(100);
	ICM_20602_WriteRegister(ICM20602_RA_SIGNAL_PATH_RESET,ICM20602_BIT_GYRO | ICM20602_BIT_ACC | ICM20602_BIT_TEMP); //Reset accel digital signal path. Reset temp digital signal path.
	delay_ms(100);
	ICM_20602_WriteRegister(ICM20602_RA_PWR_MGMT_1,0X00); //唤醒icm20602

    ICM_20602_WriteRegister(ICM20602_RA_USER_CTRL, 0x10); //USER控制模式
    delay_us(50);

    ICM_20602_WriteRegister(ICM20602_RA_PWR_MGMT_2, 0x00); //电源2管理,全部打开读写
    delay_us(50);

    //陀螺仪采样率0x00(1000Hz)   采样率 = 陀螺仪的输出率/ (1 + SMPLRT_DIV)
	  ICM_20602_WriteRegister(ICM20602_RA_SMPLRT_DIV, 0x00); //电源2管理,全部打开读写
    delay_us(50);

    //i2c旁路模式
    // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    ICM_20602_WriteRegister(ICM20602_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
    delay_us(50);

    //低通滤波
    ICM_20602_WriteRegister(ICM20602_RA_CONFIG, ICM20602_LPF_42HZ);
    delay_us(50);

    //陀螺仪自检及测量范围,典型值0x18(不自检,2000deg/s) (0x10 1000deg/s) (0x10 1000deg/s) (0x08 500deg/s)
		ICM_20602_WriteRegister(ICM20602_RA_GYRO_CONFIG,0x18);
    delay_us(50);

    //加速度自检及测量范围(典型值,+-8G)
    ICM_20602_WriteRegister(ICM20602_RA_ACCEL_CONFIG,2 << 3);
    delay_us(50);

    //加速度低通滤波
    
    ICM_20602_WriteRegister(ICM20602_RA_ACCEL_CONFIG2,ICM20602_LPF_42HZ);
    delay_ms(5);
	
	  return ICM_20602_SpiDetect();					//器件ID不正确,返回false


}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : void ICM_20602_Read_Data(void)
**功    能 : 读取
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/
void ICM_20602_Read_Data(void)
{
   uint8_t tmpRead[14] = {0};
   uint8_t reg=0x80|ICM20602_RA_ACCEL_XOUT_H;
   int16_t  acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z;
   ICM20602_ON_CS();
   HAL_SPI_Transmit(&SPI1_Handler,&reg,1,100);reg=0x0f;
   HAL_SPI_TransmitReceive(&SPI1_Handler,&reg,tmpRead,14,100);
   ICM20602_OFF_CS();
     
   acc_x = ((int16_t)((tmpRead[0]<<8) | (tmpRead[1]))); // Acc.X
   acc_y =  ((int16_t)((tmpRead[2]<<8) | (tmpRead[3]))); // Acc.Y
   acc_z =  ((int16_t)((tmpRead[4]<<8) | (tmpRead[5]))); // Acc.Z
   gyro_x= ((int16_t)((tmpRead[8]<<8) | (tmpRead[9]))); // Gyr.X
   gyro_y=  ((int16_t)((tmpRead[10]<<8) | (tmpRead[11])));// Gyr.Y
   gyro_z= ((int16_t)((tmpRead[12]<<8) | (tmpRead[13])));// Gyr.Z
  // acc_x/2048;
   sensor[0]=acc_x;
   sensor[1]=acc_y;
   sensor[2]=acc_z;
   sensor[3]=gyro_x;
   sensor[4]=gyro_y;
   sensor[5]=gyro_z;

}
/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : uint8_t ICM_20602_SpiDetect(void)
**功    能 : 配置写寄存器
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/
uint8_t ICM_20602_SpiDetect(void)
{
    uint8_t tmp;

	ICM_20602_ReadRegister(ICM20602_RA_WHO_AM_I, 1, &tmp);

	delay_ms(500);

    if (tmp == ICM20602_WHO_AM_I_CONST) 
	{
        return 1;
    }
	else
	{
	 
		
		return 0;
	}
    
}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : uint8_t ICM_20602_WriteRegister(uint8_t reg,uint8_t data)
**功    能 : 配置写寄存器
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/

uint8_t ICM_20602_WriteRegister(uint8_t reg,uint8_t data)
{
	 uint8_t status;
     ICM20602_ON_CS();
     status=HAL_SPI_Transmit(&SPI1_Handler,&reg,1,100); 
     HAL_SPI_Transmit(&SPI1_Handler,&data,1,100); 
	 ICM20602_OFF_CS();
     return (status);
}

/*====================================================================================================*/
/*====================================================================================================*
**函数原型 : uint8_t ICM_20602_ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
**功    能 : 配置读寄存器
**输    入 : None
**输    出 : None
**备    注 : 
**====================================================================================================*/
/*====================================================================================================*/
uint8_t ICM_20602_ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t reg_val;
	reg=reg|0x80;
	ICM20602_ON_CS();
	HAL_SPI_Transmit(&SPI1_Handler,&reg,1,100); 
	reg=0x0f;
	reg_val=HAL_SPI_TransmitReceive(&SPI1_Handler,&reg,data,length,100);
	ICM20602_OFF_CS();
    return reg_val;
   
}














///***********************************************************************************************************
//*                               NoneQuadrotor UAV file_end
//***********************************************************************************************************/




