/**********************************************************************************************************
*�ļ�˵����fm25v01-g RAM�����ļ�
*ʵ�ֹ��ܣ�ʵ��fm25v01-g����
*�޸����ڣ�2018-12-2
*�޸����ߣ�crystal cup
*�޸ı�ע��PI1 SPI2_SCK SPI2
           PI2 SPI2_MISO SPI2
           PI3 SPI2_MOSI SPI2
		   PF5 SPI2_CS (��ɲ���ͨ��)
**********************************************************************************************************/
#include "drv_fm25v01g.h"
#include "copter.h"



const uint8_t FM25V01G_ID[9] = {0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0xC2,0x21,0x00};

/**********************************************************************************************************
*����ԭ��: bool FM25V01G_Detect(void)
*��������: FM25V01GоƬ���
*�������: none
*��������: none
*�޸�����: 2018-12-3
*��ע��Ϣ��
**********************************************************************************************************/



bool FM25V01G_Detect(void)
{
	uint8_t i;
	uint8_t DeviceID_DATA[9]={0};
	uint8_t GetData[1]={0};
	uint8_t GetData2[5]={0xAA,0XAB,0XFE,0XEF,0XDD};
    uint8_t GetData3[5]={0};
	FM25V01G_ON_CS();

	SPI2_ReadWriteByte(FM25V01G_RDID);
	for(i=0;i<9;i++)
	{
	  DeviceID_DATA[i]=SPI2_ReadWriteByte(0x00); //��ȡ�豸ID������
	}
	FM25V01G_OFF_CS();
    if((FM25V01G_ID[0]==DeviceID_DATA[0])&&(FM25V01G_ID[1]==DeviceID_DATA[1])&&(FM25V01G_ID[2]==DeviceID_DATA[2])&&(FM25V01G_ID[3]==DeviceID_DATA[3])
	&&(FM25V01G_ID[4]==DeviceID_DATA[4])&&(FM25V01G_ID[5]==DeviceID_DATA[5])&&(FM25V01G_ID[6]==DeviceID_DATA[6])&&(FM25V01G_ID[7]==DeviceID_DATA[7])&&(FM25V01G_ID[8]==DeviceID_DATA[8]))
	{
		FM25V01G_WriteByte(0x2000,0xaf);
		FM25V01G_WriteMemory(0x3000,5,GetData2);
		delay_ms(10);
		GetData[0]=FM25V01G_ReadByte(0x2000);
		FM25V01G_ReadMemory(0x3000,5,GetData3);
        if((GetData[0]==0xaf)&&(GetData3[0]==GetData2[0])&&(GetData3[1]==GetData2[1])&&(GetData3[4]==GetData2[4])) //��д����
		{
		  return 1;
		}
	   
	   else
	   {
	      return 0;
	   }
	}
	else
	{
	  return 0;
	}
}




/**********************************************************************************************************
*����ԭ��:void FM25V01G_WriteByte(uint32_t address,uint8_t data)
*��������: FM25V01Gд�ֽ�
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/
void FM25V01G_WriteByte(uint32_t address,uint8_t data)
{
    uint8_t temH,temM,temL;
	temH=(uint8_t)((address&0xff0000)>>16);
    temM=(uint8_t)((address&0x00ff00)>>8);
    temL=(uint8_t)(address&0x0000ff);
	FM25V01G_ON_CS();
	SPI2_ReadWriteByte(FM25V01G_WREN);
	FM25V01G_OFF_CS();
	delay_us(2);
	
	FM25V01G_ON_CS();
	SPI2_ReadWriteByte(FM25V01G_WRITE);
	SPI2_ReadWriteByte(temH);
	SPI2_ReadWriteByte(temM);
	SPI2_ReadWriteByte(temL);
	SPI2_ReadWriteByte(data);
	FM25V01G_OFF_CS();
	delay_us(2);
	
	FM25V01G_ON_CS();
	SPI2_ReadWriteByte(FM25V01G_WRDI);
	FM25V01G_OFF_CS();

}

/**********************************************************************************************************
*����ԭ��:uint8_t FM25V01G_ReadByte(uint32_t address)
*��������:��FM25V01G�ض���ַ��ȡ����
*�������: address:��ַ
*��������: temp����ȡ����
*�޸�����: 2018-12-3
*��ע��Ϣ��
**********************************************************************************************************/
uint8_t FM25V01G_ReadByte(uint32_t address)
{
	uint8_t temp;
    uint8_t temH,temM,temL;

	temH=(uint8_t)((address&0Xff0000)>>16);
    temM=(uint8_t)((address&0X00ff00)>>8);
    temL=(uint8_t)(address&0X0000ff);
	FM25V01G_ON_CS();
	SPI2_ReadWriteByte(FM25V01G_READ);
	SPI2_ReadWriteByte(temH);
	SPI2_ReadWriteByte(temM);
	SPI2_ReadWriteByte(temL);
	temp=SPI2_ReadWriteByte(0x00);
	FM25V01G_OFF_CS();
    return temp;
}

/**********************************************************************************************************
*����ԭ��:void FM25V01G_WriteMemory(uint32_t address,uint32_t number,uint8_t *p) 
*��������:��FM25V01G�ض���ַд����
*�������: address:��ַ
*��������: temp����ȡ����
*�޸�����: 2018-12-3
*��ע��Ϣ��
**********************************************************************************************************/
void FM25V01G_WriteMemory(uint32_t address,uint32_t number,uint8_t *p) 
{ 
    uint8_t temH,temM,temL;
	uint32_t i;
	temH=(uint8_t)((address&0Xff0000)>>16);
    temM=(uint8_t)((address&0X00ff00)>>8);
    temL=(uint8_t)(address&0X0000ff);
	
	FM25V01G_ON_CS();
	SPI2_ReadWriteByte(FM25V01G_WREN);  //дʹ��
	FM25V01G_OFF_CS();
	delay_us(2);
	FM25V01G_ON_CS();
	SPI2_ReadWriteByte(FM25V01G_WRITE); //д����
	SPI2_ReadWriteByte(temH);
	SPI2_ReadWriteByte(temM);
	SPI2_ReadWriteByte(temL);
    for(i=0;i<number;i++)
	{
	 SPI2_ReadWriteByte(*p++);
	}
	FM25V01G_OFF_CS();
	delay_us(2);
	
	FM25V01G_ON_CS();
	SPI2_ReadWriteByte(FM25V01G_WRDI);
	FM25V01G_OFF_CS();
}
/**********************************************************************************************************
*����ԭ��:void FM25V01G_ReadMemory(uint32_t address,uint32_t number,uint8_t *p) 
*��������:��FM25V01G�ض���ַ��ȡ����
*�������: address:��ַ
*��������: temp����ȡ����
*�޸�����: 2018-12-3
*��ע��Ϣ��
**********************************************************************************************************/
void FM25V01G_ReadMemory(uint32_t address,uint32_t number,uint8_t *p) 
{ 
  uint8_t temH,temM,temL;
	uint32_t i;
	temH=(uint8_t)((address&0Xff0000)>>16);
  temM=(uint8_t)((address&0X00ff00)>>8);
  temL=(uint8_t)(address&0X0000ff);
	
	FM25V01G_ON_CS();
	SPI2_ReadWriteByte(FM25V01G_READ);

	SPI2_ReadWriteByte(temH);
	SPI2_ReadWriteByte(temM);
	SPI2_ReadWriteByte(temL);
  for(i=0;i<number;i++)
	{
	 *p++=SPI2_ReadWriteByte(0xf0);
	}
	FM25V01G_OFF_CS();

}



/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/