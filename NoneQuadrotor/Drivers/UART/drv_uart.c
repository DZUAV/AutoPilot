/**********************************************************************************************************
*�ļ�˵����������������
*ʵ�ֹ��ܣ����ô���
*�޸����ڣ�2018-11-17
*�޸����ߣ�crystal cup
*�޸ı�ע��
**********************************************************************************************************/
#include "drv_uart.h"	  
#include "copter.h"	
//////////////////////////////////////////////////////////////////

UART_HandleTypeDef huart3;


uint8_t UART1_TxBuffer[256];
uint8_t UART1_TxCounter=0;
uint8_t UART1_count=0; 

uint8_t UART2_TxBuffer[256];
uint8_t UART2_TxCounter;
uint8_t UART2_count; 
uint8_t UART2_Rx_Buf[256];	//���ڽ��ջ���
 
uint8_t UART3_TxBuffer[256];
uint8_t UART3_TxCounter;



int USART3_count = 0;
int USART3_rx_flag=0;
uint8_t USART3_rx_buf[256]={0};
uint8_t USART3_rx_len=0;
 
/**********************************************************************************************************
*����ԭ��: void UART1_init(u32 pclk2,u32 bound)
*��������: ��ʼ������USART1
*�������: pclk2:PCLK2ʱ��Ƶ��(Mhz) ;bound:������
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��GPS�źŴ���
*          PB6---UART1_TX
*          PB7---UART1_RX
**********************************************************************************************************/

void USART1_init(u32 pclk2,u32 bound)
{ 
  	
	  uint32_t	temp;	   
		temp=(pclk2*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������
		RCC->AHB1ENR|=1<<1;   	//ʹ��PORTD��ʱ��  
		RCC->APB2ENR|=1<<4;  	  //ʹ�ܴ���1ʱ�� 
		GPIO_Set(GPIOB,PIN6|PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB6,PB7,���ù���,�������
		GPIO_AF_Set(GPIOB,6,4);	//PB6,AF4--GPIO_AF4_USART1
		GPIO_AF_Set(GPIOB,7,4); //PB7,AF4---GPIO_AF4_USART1 	   
		//����������
		USART1->BRR=temp; 		//����������@OVER8=0 	
		USART1->CR1=0;		 	  //����CR1�Ĵ���
		USART1->CR1|=0<<28;	 	//����M1=0
		USART1->CR1|=0<<12;	 	//����M0=0&M1=0,ѡ��8λ�ֳ� 
		USART1->CR1|=0<<15; 	//����OVER8=0,16�������� 
		USART1->CR1|=1<<3;  	//���ڷ���ʹ�� 

		//ʹ�ܽ����ж� 
		USART1->CR1|=1<<2;  	  //���ڽ���ʹ��
		USART1->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
		MY_NVIC_Init(2,1,USART2_IRQn,2);//��2��������ȼ� 

		USART1->CR1|=1<<0;  	//����ʹ��
	
} 
 
/**********************************************************************************************************
*����ԭ��: void Usart1_Send(unsigned char *DataToSend ,u8 data_num)
*��������: ����1���ͺ���
*�������: 
*��������: 
*�޸�����: 
*��ע��Ϣ��
**********************************************************************************************************/
void Usart1_Send(unsigned char *DataToSend ,u8 data_num)
{
    uint8_t i;
	for(i=0;i<data_num;i++)
	{
		UART1_TxBuffer[UART1_count++] = *(DataToSend+i);
	}

	if(!(USART1->CR1 & USART_CR1_TXEIE))
	{
		USART1->CR1|=1<<7;  	//�����жϷ���ʹ�� 
		
	}

} 
 

/**********************************************************************************************************
*����ԭ��: void UART2_init(u32 pclk2,u32 bound)
*��������: ��ʼ��IO ����2
*�������: pclk2:PCLK2ʱ��Ƶ��(Mhz) ;bound:������
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��
**********************************************************************************************************/

void UART2_init(u32 pclk2,u32 bound)
{ 
  	
	  uint32_t	temp;	   
		temp=(pclk2*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������
		RCC->AHB1ENR|=1<<3;   	//ʹ��PORTD��ʱ��  
		RCC->APB1ENR|=1<<17;  	//ʹ�ܴ���2ʱ�� 
		GPIO_Set(GPIOD,PIN5|PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PD5,PD6,���ù���,�������
		GPIO_AF_Set(GPIOD,5,7);	//PD5,AF7
		GPIO_AF_Set(GPIOD,6,7);//PD6,AF7  	   
		//����������
		USART2->BRR=temp; 		//����������@OVER8=0 	
		USART2->CR1=0;		 	//����CR1�Ĵ���
		USART2->CR1|=0<<28;	 	//����M1=0
		USART2->CR1|=0<<12;	 	//����M0=0&M1=0,ѡ��8λ�ֳ� 
		USART2->CR1|=0<<15; 	//����OVER8=0,16�������� 
		USART2->CR1|=1<<3;  	//���ڷ���ʹ�� 

		//ʹ�ܽ����ж� 
		USART2->CR1|=1<<2;  	//���ڽ���ʹ��
		USART2->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
		MY_NVIC_Init(2,2,USART2_IRQn,2);//��2��������ȼ� 

		USART2->CR1|=1<<0;  	//����ʹ��
	
}

/**********************************************************************************************************
*����ԭ��: void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
*��������: ����2���ͺ���
*�������: 
*��������: 
*�޸�����: 
*��ע��Ϣ��
**********************************************************************************************************/
void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
{
    uint8_t i;
	for(i=0;i<data_num;i++)
	{
		UART2_TxBuffer[UART2_count++] = *(DataToSend+i);
	}

	if(!(USART2->CR1 & USART_CR1_TXEIE))
	{
		USART2->CR1|=1<<7;  	//�����жϷ���ʹ�� 
		
	}

}


/**********************************************************************************************************
*����ԭ��: void USART3_init(u32 pclk2,u32 bound)
*��������: ��ʼ��IO ����3
*�������: pclk2:PCLK2ʱ��Ƶ��(Mhz) ;bound:������
*��������: none
*�޸�����: 2018-11-17
*��ע��Ϣ��PD8--USART3_TX  PD9--USART3_RX
**********************************************************************************************************/

void USART3_init(u32 pclk2,u32 bound)
{ 
  	
	  uint32_t	temp;	   
		temp=(pclk2*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������
		RCC->AHB1ENR|=1<<3;   	//ʹ��PORTD��ʱ��  
		RCC->APB1ENR|=1<<18;  	//ʹ�ܴ���3ʱ�� 
		GPIO_Set(GPIOD,PIN8|PIN9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PD8,PD9,���ù���,�������
		GPIO_AF_Set(GPIOD,8,GPIO_AF7_USART3);	//PD8,AF7
		GPIO_AF_Set(GPIOD,9,GPIO_AF7_USART3);//PD9,AF7  	   
		//����������
		USART3->BRR=temp; 		//����������@OVER8=0 	
		USART3->CR1=0;		 	//����CR1�Ĵ���
		USART3->CR1|=0<<28;	 	//����M1=0
		USART3->CR1|=0<<12;	 	//����M0=0&M1=0,ѡ��8λ�ֳ� 
		USART3->CR1|=0<<15; 	//����OVER8=0,16�������� 
		USART3->CR1|=1<<3;  	//���ڷ���ʹ�� 

		//ʹ�ܽ����ж� 
		USART3->CR1|=1<<2;  	//���ڽ���ʹ��
		USART3->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
		MY_NVIC_Init(2,3,USART3_IRQn,2);//��2��������ȼ� 

		USART3->CR1|=1<<0;  	//����ʹ��
	
}

/**********************************************************************************************************
*����ԭ��: void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
*��������: ����2���ͺ���
*�������: 
*��������: 
*�޸�����: 
*��ע��Ϣ��
**********************************************************************************************************/
void Usart3_Send(unsigned char *DataToSend ,u8 data_num)
{
    uint8_t i;
	for(i=0;i<data_num;i++)
	{
		UART3_TxBuffer[USART3_count++] = *(DataToSend+i);
	}

	if(!(USART3->CR1 & USART_CR1_TXEIE))
	{
		USART3->CR1|=1<<7;  	//�����жϷ���ʹ�� 
		
	}

}



/**********************************************************************************************************
*����ԭ��: void Usart3_isr(void)
*��������: ����3�жϺ���
*�������: 
*��������: 
*�޸�����: 
*��ע��Ϣ��
**********************************************************************************************************/
void Usart3_isr(void)
{

   uint8_t ch;
	
	if(((USART3->ISR)&(UART_FLAG_RXNE))!=RESET)
	{
	  ch=USART3->RDR;
		Usart3_recv_cb(ch);
	  USART3->RDR=~(UART_FLAG_RXNE);
	}


}


/**********************************************************************************************************
*����ԭ��: void Usart3_recv_cb(uint8_t ch)
*��������: ����3���ջص�
*�������: 
*��������: 
*�޸�����: 
*��ע��Ϣ��
**********************************************************************************************************/

void Usart3_recv_cb(uint8_t ch)
{
	USART3_count = 0;
	USART3_rx_buf[USART3_rx_len] = ch;
	USART3_rx_len ++;
}




















//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)  
//���HAL��ʹ��ʱ,ĳЩ������ܱ����bug
int _ttywrch(int ch)    
{
    ch=ch;
	return ch;
}
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART2->ISR&0X40)==0);//ѭ������,ֱ���������   
	USART2->TDR = (u8) ch;      
	return ch;
}
#endif 
//end

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/