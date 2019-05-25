/**********************************************************************************************************
*文件说明：所有的中断文件配置文件
*实现功能：中断函数实现
*修改日期：2018-11-17
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/

#include "stm32f765_it.h"	  
#include "copter.h"	  

volatile uint32_t SysTickUptime = 0;
volatile uint32_t SysTickUptime1 = 0;

/**********************************************************************************************************
*函数原型: void SysTick_Handler(void)
*函数功能: 滴答定时器中断服务函数
*输入参数: none
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/
void SysTick_Handler(void)
{	
	 HAL_IncTick();
	 HAL_SYSTICK_IRQHandler(); //系统中断回调函数
	 SysTickUptime++; //计数累加
	 Task_Trigger_Start();

}

/**********************************************************************************************************
*函数原型: void USART2_IRQHandler(void)
*函数功能: USART2中断文件配置函数
*输入参数: none
*返回数据: none
*修改日期: 2018-11-17
*备注信息：实现移植mavlink
**********************************************************************************************************/
#define NoneQuadRotor_USE_MAVLINK 0
void USART2_IRQHandler(void)
{
//  if(NoneQuadRotor_USE_MAVLINK)
//  {
//		uint8_t c;	
//	   if((((USART2->CR1)&(0x01<<5))!=RESET)&&(((USART2->ISR)&(0x01<<5))!=RESET)) //数据接收中断标志处理
//		{
//		
//		  c=(uint16_t)((USART2->RDR)&(uint16_t)0x1FF); //读取数据
//		  fifo_write_ch(&uart_rx_fifo, c);	
//			USART2->CR1&=(~(1<<5));  	//串口中断发送使能 
//				
//		}
//		
//		 if((((USART2->CR1)&(0x01<<7))!=RESET)&&(((USART2->ISR)&(0x01<<7))!=RESET)) //数据发送中断标志
//		{
//				 LED1(0);	
//		  if(fifo_read_ch(&uart_tx_fifo, &c)) 
//			{
//			  while((USART2->ISR&0X40)==0);//等待发送结束  
//				USART2->TDR=(c & (uint16_t)0x01FF);//把数据发送完
//			}
//				
//			else 
//			{
//				USART2->TDR=(0x55 & (uint16_t)0x01FF);//发送0x55
//				while((USART2->ISR&0X40)==0);//等待发送结束 
//			}
//				
//			if (fifo_used(&uart_tx_fifo) == 0) // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
//			{
//				USART2->CR1&=(~(1<<7));  	//串口中断发送使能 
//			
//			}
//		}
//  
//  }
//  else
//  {
     uint8_t  com_data;
    if((USART2->ISR)&(0x08))//ORE中断
	{
		com_data = USART2->RDR;
	}
	if(((USART2->CR1)&(0x01<<5))&&((USART2->ISR)&(0x01<<5))) //数据接收中断标志处理
	{
		
		USART2->ISR=(uint16_t)(~( (uint16_t)0x01<<(uint16_t)(0x05)));
		com_data = USART2->RDR;
	    Communicate_DT_Data_Receive_Prepare(com_data); //接收外部中断
	}
    if(((USART2->CR1)&(0x01<<7))&&((USART2->ISR)&(0x01<<7))) //数据发送中断标志
	{
	   USART2->TDR = UART2_TxBuffer[UART2_TxCounter++]; //写DR清除中断标志    
	  if(UART2_TxCounter == UART2_count)
	  {
	   USART2->CR1 &= ~((uint16_t)0x0080);		//关闭TXE(发送中断)中断
	  }
	}
  
  
//  }
//	  

} 


/**********************************************************************************************************
*函数原型: void USART3_IRQHandler(void)
*函数功能: USART3中断文件配置函数
*输入参数: none
*返回数据: none
*修改日期: 2018-12-15
*备注信息：实现移植mavlink
**********************************************************************************************************/
void USART3_IRQHandler(void)
{
  Usart3_isr(); //接收中断
} 









/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/

