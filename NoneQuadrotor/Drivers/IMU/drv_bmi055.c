/**********************************************************************************************************
*�ļ�˵����BMI055��������������
*ʵ�ֹ��ܣ�����BMI055
*�޸����ڣ�2018-11-11
*�޸����ߣ�crystal cup
*�޸ı�ע��
**********************************************************************************************************/
#include "drv_bmi055.h"
#include "copter.h"

/**********************************************************************************************************
*����ԭ��: void BMI055_Init(void)
*��������: ��ʼ��BMI055
*�������: none
*��������: none
*�޸�����: 2018-12-12
*��ע��Ϣ��
**********************************************************************************************************/
void BMI055_Init(void)
{
  //���ٶ� 
  SPI_BMI055_AccWriterReg(BMI055_REGA_BGW_SOFTRESET,0xB6); //��λ���ٶȴ�����
  delay_ms(100);
  SPI_BMI055_AccWriterReg(BMI055_REGA_PMU_RANGE,0x0C);     //���ü��ٶȵ����̷�Χ��+-16g  0x03:+-2g 0x05:+-4g 0x08:+-8g 0x0C:+-16g
  delay_ms(10);
  SPI_BMI055_AccWriterReg(BMI055_REGA_PMU_BW,0x0f);        //�˲�����1000HZ 0x0d:250HZ 0x0e:500HZ 0x0f:1000HZ
  delay_ms(10);
  SPI_BMI055_AccWriterReg(BMI055_REGA_PMU_LPW,0x00);        //����ģʽ
  delay_ms(10);
  SPI_BMI055_AccWriterReg(BMI055_REGA_ACCD_HBW,0x80);        //��������������˲�
  delay_ms(10);
  SPI_BMI055_AccWriterReg(BMI055_REGA_FIFO_CONFIG_1,0x80);        //��ʼ��FIFO���������X,Y,Z
  delay_ms(10);
	
	
  //������
  SPI_BMI055_AccWriterReg(BMI055_REGG_BGW_SOFTRESET,0xB6); //��λ���ٶȴ�����
  delay_ms(100);
  SPI_BMI055_AccWriterReg(BMI055_REGG_RANGE,0x00);         //�������������̷�Χ��+-2000deg/s
  delay_ms(10);
  SPI_BMI055_AccWriterReg(BMI055_REGG_BW,0x81);            //�˲��������ã�230Hz�˲�
  delay_ms(10);
  SPI_BMI055_AccWriterReg(BMI055_REGG_LPM1,0x00);           //����ģʽ
  delay_ms(10);
  SPI_BMI055_AccWriterReg(BMI055_REGG_FIFO_CONFIG_1,0x80);  //��ʼ��FIFO���������X,Y,Z
  delay_ms(10);
  SPI_BMI055_AccWriterReg(BMI055_REGG_INT_EN0,0xa0);        //
  delay_ms(10);

}

/**********************************************************************************************************
*����ԭ��: void BMI055_Init(void)
*��������: ��ʼ��BMI055
*�������: none
*��������: none
*�޸�����: 2018-12-12
*��ע��Ϣ��
**********************************************************************************************************/
uint16_t acc_data[3]={0};
uint16_t gyro_data[3]={0};
void BMI055_Read_Data(uint8_t *acc_buf,uint8_t *gyro_buf)
{
    uint8_t i = 0;
	  uint8_t j = 0;
    uint8_t addr = 0x00; 
    addr |= 0x80;//read
    //��ȡ���ٶ�����
    BMI055_ACC_ON_CS();
    addr |= BMI055_REGA_ACCD_X_LSB;
    SPI1_ReadWriteByte(addr);
    for(i=0;i<6;i++)
	{
        *(acc_buf+i) = SPI1_ReadWriteByte(0);    
        delay_us(10);		

  }
	acc_data[0]=((acc_buf[1]<<8)|acc_buf[0]);

	acc_data[0]=acc_data[0]>>4;
	acc_data[1]=((acc_buf[3]<<8)|acc_buf[2]);
	acc_data[1]=acc_data[1]>>4;
	
	acc_data[2]=((acc_buf[5]<<8)|acc_buf[4]);
	acc_data[2]=acc_data[2]>>4;
	
  BMI055_ACC_OFF_CS();
    //��ȡ����������
	BMI055_GYRO_ON_CS();
	addr |= BMI055_REGA_RATE_X_LSB;
  SPI1_ReadWriteByte(addr);          
    for(j=0;j<6;j++)
	{
      *(gyro_buf+j) = SPI1_ReadWriteByte(0);     
       delay_us(10);	
  }   
   gyro_data[0]=((gyro_buf[1]<<8)|gyro_buf[0]);	
	 gyro_data[1]=((gyro_buf[3]<<8)|gyro_buf[2]);	
	 gyro_data[2]=((gyro_buf[5]<<8)|gyro_buf[4]);	
    BMI055_GYRO_OFF_CS();

}




/**********************************************************************************************************
*����ԭ��: uint8_t BMI055_CHIP_Identification(void)
*��������: ʶ��BMI055
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-11
*��ע��Ϣ��
**********************************************************************************************************/
uint8_t BMI055_CHIP_Identification(void)
{

    if((BMI055_Gyro_Chip_Identification()==1)&&(BMI055_Acc_Chip_Identification()==1)) 
	{
	  return 1;
	
	}
	else
	{
	  return 0;
	
	}

}
/**********************************************************************************************************
*����ԭ��: uint8_t BMI055_Acc_Chip_Identification(void)
*��������: ʶ��BMI055��ACC
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-11
*��ע��Ϣ��
**********************************************************************************************************/
uint8_t BMI055_Acc_Chip_Identification(void)
{
    uint8_t res = 0;
    uint8_t reg = BMI055_REGA_BGW_CHIPID;//REG_CHIPID 
    reg |= 0x80;//read    

    //Check ACC CHIP_ID
    BMI055_ACC_ON_CS();
    SPI1_ReadWriteByte(reg);          
    res = SPI1_ReadWriteByte(0);      
    BMI055_ACC_OFF_CS();
    if(res == BMI055_ACC_CHIPID_DATA)
	{
        res = 1;
    }
		else
	{
		    res = 0;
	}

   return res;
}

/**********************************************************************************************************
*����ԭ��: uint8_t BMI055_Acc_Chip_Identification(void)
*��������: ʶ��BMI055��gyro
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-11
*��ע��Ϣ��
**********************************************************************************************************/
uint8_t BMI055_Gyro_Chip_Identification(void)
{
    uint8_t data = 0;
    uint8_t regester = BMI055_REGG_CHIPID;//REG_CHIPID 
    regester |= 0x80;//read    

    //Check gyro CHIP_ID
    BMI055_GYRO_ON_CS();
    SPI1_ReadWriteByte(regester);          
    data = SPI1_ReadWriteByte(0);      
    BMI055_GYRO_OFF_CS();
    if(data == BMI055_GYRO_CHIPID_DATA)
	{
        data = 1;
    }
		else
	{
		    data = 0;
	}

   return data;
}

/**********************************************************************************************************
*����ԭ��: uint8_t SPI_BMI055_AccWriterReg(uint8_t reg,uint8_t data)
*��������: ʶ��BMI055��gyro
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-11
*��ע��Ϣ��
**********************************************************************************************************/

uint8_t SPI_BMI055_AccWriterReg(uint8_t reg,uint8_t data)
{
	   uint8_t status;
     BMI055_ACC_ON_CS();
     status=HAL_SPI_Transmit(&SPI1_Handler,&reg,1,100); 
     HAL_SPI_Transmit(&SPI1_Handler,&data,1,100); 
	   BMI055_ACC_OFF_CS();
     return (status);
}

/**********************************************************************************************************
*����ԭ��: uint8_t SPI_BMI055_AccReadReg(uint8_t reg, uint8_t length, uint8_t *data)
*��������: ʶ��BMI055��gyro
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-11
*��ע��Ϣ��
**********************************************************************************************************/
uint8_t SPI_BMI055_AccReadReg(uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t reg_val;
	reg=reg|0x80;
	BMI055_ACC_ON_CS();
	HAL_SPI_Transmit(&SPI1_Handler,&reg,1,100); 
	reg=0x0f;
	reg_val=HAL_SPI_TransmitReceive(&SPI1_Handler,&reg,data,length,100);
	BMI055_ACC_OFF_CS();
    return reg_val;
   
}

/**********************************************************************************************************
*����ԭ��: uint8_t SPI_BMI055_GyroWriterReg(uint8_t reg,uint8_t data)
*��������: ʶ��BMI055��gyro
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-11
*��ע��Ϣ��
**********************************************************************************************************/

uint8_t SPI_BMI055_GyroWriterReg(uint8_t reg,uint8_t data)
{
	 uint8_t status;
     BMI055_GYRO_ON_CS();
     status=HAL_SPI_Transmit(&SPI1_Handler,&reg,1,100); 
     HAL_SPI_Transmit(&SPI1_Handler,&data,1,100); 
	 BMI055_GYRO_OFF_CS();
     return (status);
}

/**********************************************************************************************************
*����ԭ��: uint8_t SPI_BMI055_GyroReadReg(uint8_t reg, uint8_t length, uint8_t *data)
*��������: ʶ��BMI055��gyro
*�������: pvParameters
*��������: none
*�޸�����: 2018-12-11
*��ע��Ϣ��
**********************************************************************************************************/
uint8_t SPI_BMI055_GyroReadReg(uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t reg_val;
	reg=reg|0x80;
	BMI055_GYRO_ON_CS();
	HAL_SPI_Transmit(&SPI1_Handler,&reg,1,100); 
	reg=0x0f;
	reg_val=HAL_SPI_TransmitReceive(&SPI1_Handler,&reg,data,length,100);
	BMI055_GYRO_OFF_CS();
    return reg_val;
   
}

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/