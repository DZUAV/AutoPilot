/**********************************************************************************************************
*�ļ�˵����NoneQuadrotor UAV���п��ƴ���
*ʵ�ֹ��ܣ�ʵ�����˻���׼����
*�޸����ڣ�2018-11-17
*�޸����ߣ�crystal cup
*�޸ı�ע��Ӧ�ó������
**********************************************************************************************************/
#include "drv_mpu6000.h"
#include "copter.h"

IMUBUS_Recog_flag mpu6000_bus_flag;
IMU_IIC_FLAG mpu6000_iic_flag;
IMU_SPI_FLAG mpu6000_spi_flag;
IMU_SENSOR_NUM_FLAG mpu6000_sensor_num_flag;

void MPU6000_Self_Select(void)
{

  Sensor_Self_Delect(mpu6000_bus_flag,SPI_BUS,mpu6000_iic_flag,IMU_NO_IIC_BUS,mpu6000_spi_flag,IMU_SPI1_BUS1,mpu6000_sensor_num_flag,IMU_MPU6000);

}


/*==================================================================================================================*/
/*==================================================================================================================*
**����ԭ�� : void DEV_MPU6000::Mpu6000_Init(void)
**��    �� : mpu6000��ʼ��
**��    �� : None
**��    �� : None
**��    ע : 
**================================================================================================================*/
/*================================================================================================================*/
void MPU6000_Init(void)
{
	  
	  char id=0;  //�����ȡ��������ַ�ı���
  	SPI_MPU6000_WriteReg(MPU6000_PWR_MGMT1_REG,0X80);   		//��Դ����,��λMPU6000
		delay_ms(2);
	
	  SPI_MPU6000_WriteReg(MPU6000_PWR_MGMT1_REG,0X00);   		//����MPU6000
		delay_ms(2);
	
	  SPI_MPU6000_WriteReg(MPU6000_GYRO_CONFIG,0x03<<3);   		//����������������+-2000 deg/s
		delay_ms(2);
	
	  SPI_MPU6000_WriteReg(MPU6000_ACCEL_CONFIG,0x10);   		//����ACCEL���� -8g +8g 4096LSB/g
		delay_ms(2);

      SPI_MPU6000_WriteReg(MPU6000_SMPLRT_DIV,0x13);   		//���ֵ�ͨ�˲���50HZ
		delay_ms(2);
	  SPI_MPU6000_WriteReg(MPU6000_CONFIG,0x19); 
	  delay_ms(2);
	  SPI_MPU6000_WriteReg(MPU6000_INT_ENABLE,0x00);   	   //�ر������ж�
		delay_ms(2);
	
	  SPI_MPU6000_WriteReg(MPU6000_USER_CTRL_REG,0x00);   //i2c��ģʽ�ر�		
		delay_ms(2);

       SPI_MPU6000_WriteReg(MPU6000_FIFO_EN,0x00);        //�ر�FIFO	
		delay_ms(2);
		
		SPI_MPU6000_WriteReg(MPU6000_INT_PIN_CFG,0x80);   //INT���ŵ͵�ƽ��Ч
		delay_ms(2);
		
		SPI_MPU6000_WriteReg(MPU6000_USER_CTRL_REG,0x00);   //i2c��ģʽ�ر�		
		delay_ms(2);
        id=SPI_MPU6000_ReadReg(MPU6000_DEVICE_ID_REG);
		delay_ms(2);
		if(id==0x68) //����ID��ȷ
		{
		
			SPI_MPU6000_WriteReg(MPU6000_PWR_MGMT1_REG,0x01);   //����CLKSEL,PLL x��Ϊ�ο�
			delay_ms(2);
			
			SPI_MPU6000_WriteReg(MPU6000_PWR_MGMT2_REG,0x00);   //���ٶ��������Ƕ�����
			delay_ms(2);

			SPI_MPU6000_WriteReg(MPU6000_SMPLRT_DIV,0x13);   //�������ֵ�ͨ�˲���
			delay_ms(2);
			
			SPI_MPU6000_WriteReg(MPU6000_CONFIG,0x19);   
			delay_ms(2);
//		  Sensor_finish_flag.set_up_is_finish_mpu6000=1; //��ʼ����ɱ�־
		}
		else
		{
		
//		 Sensor_finish_flag.set_up_is_finish_mpu6000=0; //��ʼ��ʧ�ܱ�־
//			Mode_Colour_Led.setup_sensor_temp=2;
		}

}




/*==================================================================================================================*/
/*==================================================================================================================*
**����ԭ�� : void DEV_MPU6000::Mpu6000_Get_Initial_Data(void)
**��    �� : mpu6000��ʼ��
**��    �� : None
**��    �� : None
**��    ע : 
**================================================================================================================*/
/*================================================================================================================*/
void Mpu6000_Get_Initial_Data(void)
{
		uint8_t i;

		int16_t mpu6000_acc_temp[3];
		int16_t mpu6000_gyro_temp[3];
	  //ʹ�����߶�ȡ����
    MPU6000_ON_CS();
	  //���Ͷ�ȡ��������
	  SPI1_ReadWriteByte(MPU6000_ACCEL_XOUT_H| 0x80); 
	  //��ȡԭʼ����---acc--gyro--temp
	  for(i=0;i<14;i++)
	 {
//	   mpu6000_buf[i]=SPI1_ReadWriteByte(0xff);
	 }
//	 //��ȡԭʼ���ٶ�����
//	 Get_Sensor_Data.mpu6000_acc[0]=BYTE16(int16_t,mpu6000_buf[0],mpu6000_buf[1]);
//   Get_Sensor_Data.mpu6000_acc[1]=BYTE16(int16_t,mpu6000_buf[2],mpu6000_buf[3]);
//	 Get_Sensor_Data.mpu6000_acc[2]=BYTE16(int16_t,mpu6000_buf[4],mpu6000_buf[5]);
//	 //��ȡԭʼ����������
//	 Get_Sensor_Data.mpu6000_gyro[0]=BYTE16(int16_t,mpu6000_buf[8],mpu6000_buf[9]);
//   Get_Sensor_Data.mpu6000_gyro[1]=BYTE16(int16_t,mpu6000_buf[10],mpu6000_buf[11]);
//	 Get_Sensor_Data.mpu6000_gyro[2]=BYTE16(int16_t,mpu6000_buf[12],mpu6000_buf[13]);
//	 //ת������ϵ��������
//	 Sensor.Align_Sensor(Get_Sensor_Data.mpu6000_acc,mpu6000_acc_temp,6);
//	 Sensor.Align_Sensor(Get_Sensor_Data.mpu6000_gyro,mpu6000_gyro_temp,6);
//	 //��ȡ��������
//	 Amend_Get_Sensor_Data.amend_mpu6000_acc[0]=mpu6000_acc_temp[0];
//	 Amend_Get_Sensor_Data.amend_mpu6000_acc[1]=mpu6000_acc_temp[1];
//	 Amend_Get_Sensor_Data.amend_mpu6000_acc[2]=mpu6000_acc_temp[2];
//	 Amend_Get_Sensor_Data.amend_mpu6000_gyro[0]=mpu6000_gyro_temp[0];
//	 Amend_Get_Sensor_Data.amend_mpu6000_gyro[1]=mpu6000_gyro_temp[1];
//	 Amend_Get_Sensor_Data.amend_mpu6000_gyro[2]=mpu6000_gyro_temp[2];
	 
	 //��ʹ������
	 MPU6000_OFF_CS();
}


/**********************************************************************************************************
*����ԭ��:void Board_Init(void)
*��������: ����ʼ������
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/
uint8_t MPU6000_WriteReg(uint8_t reg,uint8_t dat)
{
	if(mpu6000_bus_flag.imu_spi_flag==1)
	{
	   SPI_MPU6000_WriteReg(reg,dat);
	}
	else if(mpu6000_bus_flag.imu_ii_flag==1)
	{
	
	
	}


}

/**********************************************************************************************************
*����ԭ��:uint8_t MPU6000_ReadReg(uint8_t reg,uint8_t dat)
*��������:mpu6000�Ĵ�����ȡ,��������ʱIIC��ȡ����,SPI��ȡ����֧��
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/
uint8_t MPU6000_ReadReg(uint8_t reg)
{
		if(mpu6000_bus_flag.imu_spi_flag==1)
	{
	  SPI_MPU6000_ReadReg(reg);
	}
	else if(mpu6000_bus_flag.imu_ii_flag==1)
	{
	
	 
	}
  

}









/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : uint8_t SPI_MPU_WriteReg(uint8_t reg,uint8_t dat)
**��    �� : SPI1д������оƬ
**��    �� : reg:MPU������+�Ĵ�����ַ��dat:��Ҫ��Ĵ���д�������
**��    �� : MPU��status�Ĵ�����״̬
**��    ע : 
**====================================================================================================*/
/*====================================================================================================*/
uint8_t SPI_MPU6000_WriteReg(uint8_t reg,uint8_t dat)
{
	if(mpu6000_spi_flag.spi1_flag==1)
	{
	   uint8_t status;
	/*�õ�CSN��ʹ��SPI����*/
    MPU6000_ON_CS();
				
	/*��������Ĵ����� */
	  status = SPI1_ReadWriteByte(reg); 
		 
	 /*��Ĵ���д������*/
    SPI1_ReadWriteByte(dat); 
	          
	/*CSN���ߣ����*/	   
  	 MPU6000_OFF_CS();
		
	/*����״̬�Ĵ�����ֵ*/
   	return(status);
	
	}
	else if(mpu6000_spi_flag.spi2_flag==1)
	{
		uint8_t status;
	/*�õ�CSN��ʹ��SPI����*/
    MPU6000_ON_CS();
				
	/*��������Ĵ����� */
	  status = SPI1_ReadWriteByte(reg); 
		 
	 /*��Ĵ���д������*/
    SPI1_ReadWriteByte(dat); 
	          
	/*CSN���ߣ����*/	   
  	 MPU6000_OFF_CS();
		
	/*����״̬�Ĵ�����ֵ*/
   	return(status);
	
	}
	
		else if(mpu6000_spi_flag.spi3_flag==1)
	{
		uint8_t status;
	/*�õ�CSN��ʹ��SPI����*/
    MPU6000_ON_CS();
				
	/*��������Ĵ����� */
	  status = SPI1_ReadWriteByte(reg); 
		 
	 /*��Ĵ���д������*/
    SPI1_ReadWriteByte(dat); 
	          
	/*CSN���ߣ����*/	   
  	 MPU6000_OFF_CS();
		
	/*����״̬�Ĵ�����ֵ*/
   	return(status);
	
	}
		else if(mpu6000_spi_flag.spi4_flag==1)
	{
		uint8_t status;
	/*�õ�CSN��ʹ��SPI����*/
    MPU6000_ON_CS();
				
	/*��������Ĵ����� */
	  status = SPI1_ReadWriteByte(reg); 
		 
	 /*��Ĵ���д������*/
    SPI1_ReadWriteByte(dat); 
	          
	/*CSN���ߣ����*/	   
  	 MPU6000_OFF_CS();
		
	/*����״̬�Ĵ�����ֵ*/
   	return(status);
	
	}
	
		else if(mpu6000_spi_flag.spi5_flag==1)
	{
		uint8_t status;
	/*�õ�CSN��ʹ��SPI����*/
    MPU6000_ON_CS();
				
	/*��������Ĵ����� */
	  status = SPI1_ReadWriteByte(reg); 
		 
	 /*��Ĵ���д������*/
    SPI1_ReadWriteByte(dat); 
	          
	/*CSN���ߣ����*/	   
  	 MPU6000_OFF_CS();
		
	/*����״̬�Ĵ�����ֵ*/
   	return(status);
	
	}

	
}

/*====================================================================================================*/
/*====================================================================================================*
**����ԭ�� : uint8_t SPI_MPU_ReadReg(uint8_t reg)
**��    �� : ���ڴ�MPU�ض��ļĴ�����������
**��    �� : reg:MPU������+�Ĵ�����ַ��
**��    �� : �Ĵ����е�����
**��    ע : 
**====================================================================================================*/
/*====================================================================================================*/

uint8_t SPI_MPU6000_ReadReg(uint8_t reg)
{
		if(mpu6000_spi_flag.spi1_flag==1)
	{
 	uint8_t reg_val;


	/*�õ�CSN��ʹ��SPI����*/
  MPU6000_ON_CS();
				
  	 /*���ͼĴ�����*/
	SPI1_ReadWriteByte(reg| 0x80); 

	 /*��ȡ�Ĵ�����ֵ */
	reg_val = SPI1_ReadWriteByte(0xff);
	            
   	/*CSN���ߣ����*/
	 MPU6000_OFF_CS();		
   	
	return reg_val;
	
	}
	else if(mpu6000_spi_flag.spi2_flag==1)
	{
 	uint8_t reg_val;


	/*�õ�CSN��ʹ��SPI����*/
  MPU6000_ON_CS();
				
  	 /*���ͼĴ�����*/
	SPI1_ReadWriteByte(reg| 0x80); 

	 /*��ȡ�Ĵ�����ֵ */
	reg_val = SPI1_ReadWriteByte(0xff);
	            
   	/*CSN���ߣ����*/
	 MPU6000_OFF_CS();		
   	
	return reg_val;
	
	}
	
		else if(mpu6000_spi_flag.spi3_flag==1)
	{
 	uint8_t reg_val;


	/*�õ�CSN��ʹ��SPI����*/
  MPU6000_ON_CS();
				
  	 /*���ͼĴ�����*/
	SPI1_ReadWriteByte(reg| 0x80); 

	 /*��ȡ�Ĵ�����ֵ */
	reg_val = SPI1_ReadWriteByte(0xff);
	            
   	/*CSN���ߣ����*/
	 MPU6000_OFF_CS();		
   	
	return reg_val;
	
	}
		else if(mpu6000_spi_flag.spi4_flag==1)
	{
 	uint8_t reg_val;


	/*�õ�CSN��ʹ��SPI����*/
  MPU6000_ON_CS();
				
  	 /*���ͼĴ�����*/
	SPI1_ReadWriteByte(reg| 0x80); 

	 /*��ȡ�Ĵ�����ֵ */
	reg_val = SPI1_ReadWriteByte(0xff);
	            
   	/*CSN���ߣ����*/
	 MPU6000_OFF_CS();		
   	
	return reg_val;
	
	}
	
		else if(mpu6000_spi_flag.spi5_flag==1)
	{
 	  uint8_t reg_val;


	  /*�õ�CSN��ʹ��SPI����*/
    MPU6000_ON_CS();
				
  	 /*���ͼĴ�����*/
	  SPI1_ReadWriteByte(reg| 0x80); 

	 /*��ȡ�Ĵ�����ֵ */
	  reg_val = SPI1_ReadWriteByte(0xff);
	            
   	/*CSN���ߣ����*/
	  MPU6000_OFF_CS();		
   	
	  return reg_val;
	
	}
	

}	









/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/

