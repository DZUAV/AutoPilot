#ifndef __DRV_UART_H_
#define __DRV_UART_H_ 
#include "stm32f765xx.h"
#include "sys.h"
#include "stdio.h"	  


#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART2_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

#define USE_UART1_ENABLE   ENABLE
#define USE_UART2_ENABLE   ENABLE


extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	


extern uint8_t UART2_TxBuffer[256];
extern uint8_t UART2_TxCounter;
extern uint8_t UART2_count; 
extern uint8_t UART2_Rx_Buf[256];	//���ڽ��ջ���


extern int USART3_count ;
extern int USART3_rx_flag;
extern uint8_t USART3_rx_buf[256];
extern uint8_t USART3_rx_len;

void USART1_init(u32 pclk2,u32 bound);
void UART2_init(u32 pclk2,u32 bound); 
void Usart1_Send(unsigned char *DataToSend ,u8 data_num);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void USART3_init(u32 pclk2,u32 bound);
void Usart3_Send(unsigned char *DataToSend ,u8 data_num);
void Usart3_isr(void);
void Usart3_recv_cb(uint8_t ch);
#endif   
















