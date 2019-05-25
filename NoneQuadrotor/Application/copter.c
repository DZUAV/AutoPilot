/**********************************************************************************************************
*�ļ�˵������������ļ�
*ʵ�ֹ��ܣ�����ʼ��
*�޸����ڣ�2018-11-17
*�޸����ߣ�crystal cup
*�޸ı�ע��
**********************************************************************************************************/
#include "copter.h"

mavlink_system_t mavlink_system; 



/**********************************************************************************************************
*����ԭ��:void Board_Init(void)
*��������: ����ʼ������
*�������: pvParameters
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/
void Board_Init(void)
{
	Stm32_Clock_Init(432,16,2,9);//����ʱ��,216Mhz
	delay_init(216);
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ� 
	LTRrgb_LED_Init();
  SPI1_Init();	
	delay_ms(100);
	SPI2_Init();
	delay_ms(100);
	SPI4_Init();
	delay_ms(100);
	MS5611_Init();
  IIC3_Init();
	delay_ms(100);
	if(ICM_20602_Init())
	{
		  	LED1(0);
	
	}
	else
	{
	  	LED1(1);
	}
	if(ICM_20689_Init())
	{
		  LED0(0);
	
	}
	else
	{
	  	LED0(1);
	}
	if(BMI055_CHIP_Identification())
	{
		BMI055_Init();
		LED2(0);
	
	}
	else
	{
	  	LED2(1);
	}
	
	delay_ms(100);
	IST8310_Init();
	IST8310_Detect();
	FM25V01G_Detect();
	SD_Init();
	Sbus_Init();
	Radio_init_in();
  TIM1_PWM_Init(22499,19,9000); 

  fifo_init(&uart_tx_fifo, uart_tx_buf, UART_TX_BUFFER_SIZE);	
	fifo_init(&uart_rx_fifo, uart_rx_buf, UART_RX_BUFFER_SIZE);
	
	mavlink_system.sysid = MAV_TYPE_GENERIC;
	mavlink_system.compid = MAV_AUTOPILOT_GENERIC;
}


/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/