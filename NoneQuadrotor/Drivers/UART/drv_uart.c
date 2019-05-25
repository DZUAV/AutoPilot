/**********************************************************************************************************
*文件说明：串口驱动配置
*实现功能：配置串口
*修改日期：2018-11-17
*修改作者：crystal cup
*修改备注：
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
uint8_t UART2_Rx_Buf[256];	//串口接收缓存
 
uint8_t UART3_TxBuffer[256];
uint8_t UART3_TxCounter;



int USART3_count = 0;
int USART3_rx_flag=0;
uint8_t USART3_rx_buf[256]={0};
uint8_t USART3_rx_len=0;
 
/**********************************************************************************************************
*函数原型: void UART1_init(u32 pclk2,u32 bound)
*函数功能: 初始化串口USART1
*输入参数: pclk2:PCLK2时钟频率(Mhz) ;bound:波特率
*返回数据: none
*修改日期: 2018-11-17
*备注信息：GPS信号处理
*          PB6---UART1_TX
*          PB7---UART1_RX
**********************************************************************************************************/

void USART1_init(u32 pclk2,u32 bound)
{ 
  	
	  uint32_t	temp;	   
		temp=(pclk2*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算
		RCC->AHB1ENR|=1<<1;   	//使能PORTD口时钟  
		RCC->APB2ENR|=1<<4;  	  //使能串口1时钟 
		GPIO_Set(GPIOB,PIN6|PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB6,PB7,复用功能,上拉输出
		GPIO_AF_Set(GPIOB,6,4);	//PB6,AF4--GPIO_AF4_USART1
		GPIO_AF_Set(GPIOB,7,4); //PB7,AF4---GPIO_AF4_USART1 	   
		//波特率设置
		USART1->BRR=temp; 		//波特率设置@OVER8=0 	
		USART1->CR1=0;		 	  //清零CR1寄存器
		USART1->CR1|=0<<28;	 	//设置M1=0
		USART1->CR1|=0<<12;	 	//设置M0=0&M1=0,选择8位字长 
		USART1->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
		USART1->CR1|=1<<3;  	//串口发送使能 

		//使能接收中断 
		USART1->CR1|=1<<2;  	  //串口接收使能
		USART1->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
		MY_NVIC_Init(2,1,USART2_IRQn,2);//组2，最低优先级 

		USART1->CR1|=1<<0;  	//串口使能
	
} 
 
/**********************************************************************************************************
*函数原型: void Usart1_Send(unsigned char *DataToSend ,u8 data_num)
*函数功能: 串口1发送函数
*输入参数: 
*返回数据: 
*修改日期: 
*备注信息：
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
		USART1->CR1|=1<<7;  	//串口中断发送使能 
		
	}

} 
 

/**********************************************************************************************************
*函数原型: void UART2_init(u32 pclk2,u32 bound)
*函数功能: 初始化IO 串口2
*输入参数: pclk2:PCLK2时钟频率(Mhz) ;bound:波特率
*返回数据: none
*修改日期: 2018-11-17
*备注信息：
**********************************************************************************************************/

void UART2_init(u32 pclk2,u32 bound)
{ 
  	
	  uint32_t	temp;	   
		temp=(pclk2*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算
		RCC->AHB1ENR|=1<<3;   	//使能PORTD口时钟  
		RCC->APB1ENR|=1<<17;  	//使能串口2时钟 
		GPIO_Set(GPIOD,PIN5|PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PD5,PD6,复用功能,上拉输出
		GPIO_AF_Set(GPIOD,5,7);	//PD5,AF7
		GPIO_AF_Set(GPIOD,6,7);//PD6,AF7  	   
		//波特率设置
		USART2->BRR=temp; 		//波特率设置@OVER8=0 	
		USART2->CR1=0;		 	//清零CR1寄存器
		USART2->CR1|=0<<28;	 	//设置M1=0
		USART2->CR1|=0<<12;	 	//设置M0=0&M1=0,选择8位字长 
		USART2->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
		USART2->CR1|=1<<3;  	//串口发送使能 

		//使能接收中断 
		USART2->CR1|=1<<2;  	//串口接收使能
		USART2->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
		MY_NVIC_Init(2,2,USART2_IRQn,2);//组2，最低优先级 

		USART2->CR1|=1<<0;  	//串口使能
	
}

/**********************************************************************************************************
*函数原型: void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
*函数功能: 串口2发送函数
*输入参数: 
*返回数据: 
*修改日期: 
*备注信息：
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
		USART2->CR1|=1<<7;  	//串口中断发送使能 
		
	}

}


/**********************************************************************************************************
*函数原型: void USART3_init(u32 pclk2,u32 bound)
*函数功能: 初始化IO 串口3
*输入参数: pclk2:PCLK2时钟频率(Mhz) ;bound:波特率
*返回数据: none
*修改日期: 2018-11-17
*备注信息：PD8--USART3_TX  PD9--USART3_RX
**********************************************************************************************************/

void USART3_init(u32 pclk2,u32 bound)
{ 
  	
	  uint32_t	temp;	   
		temp=(pclk2*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算
		RCC->AHB1ENR|=1<<3;   	//使能PORTD口时钟  
		RCC->APB1ENR|=1<<18;  	//使能串口3时钟 
		GPIO_Set(GPIOD,PIN8|PIN9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PD8,PD9,复用功能,上拉输出
		GPIO_AF_Set(GPIOD,8,GPIO_AF7_USART3);	//PD8,AF7
		GPIO_AF_Set(GPIOD,9,GPIO_AF7_USART3);//PD9,AF7  	   
		//波特率设置
		USART3->BRR=temp; 		//波特率设置@OVER8=0 	
		USART3->CR1=0;		 	//清零CR1寄存器
		USART3->CR1|=0<<28;	 	//设置M1=0
		USART3->CR1|=0<<12;	 	//设置M0=0&M1=0,选择8位字长 
		USART3->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
		USART3->CR1|=1<<3;  	//串口发送使能 

		//使能接收中断 
		USART3->CR1|=1<<2;  	//串口接收使能
		USART3->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
		MY_NVIC_Init(2,3,USART3_IRQn,2);//组2，最低优先级 

		USART3->CR1|=1<<0;  	//串口使能
	
}

/**********************************************************************************************************
*函数原型: void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
*函数功能: 串口2发送函数
*输入参数: 
*返回数据: 
*修改日期: 
*备注信息：
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
		USART3->CR1|=1<<7;  	//串口中断发送使能 
		
	}

}



/**********************************************************************************************************
*函数原型: void Usart3_isr(void)
*函数功能: 串口3中断函数
*输入参数: 
*返回数据: 
*修改日期: 
*备注信息：
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
*函数原型: void Usart3_recv_cb(uint8_t ch)
*函数功能: 串口3接收回调
*输入参数: 
*返回数据: 
*修改日期: 
*备注信息：
**********************************************************************************************************/

void Usart3_recv_cb(uint8_t ch)
{
	USART3_count = 0;
	USART3_rx_buf[USART3_rx_len] = ch;
	USART3_rx_len ++;
}




















//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)  
//解决HAL库使用时,某些情况可能报错的bug
int _ttywrch(int ch)    
{
    ch=ch;
	return ch;
}
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART2->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART2->TDR = (u8) ch;      
	return ch;
}
#endif 
//end

/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/