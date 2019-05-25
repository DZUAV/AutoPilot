#ifndef __DRV_GPS__H__
#define __DRV_GPS__H__

#include "sys.h"
#define Ublox_Send_Date(dbuf,len) Usart1_Send(dbuf,len)
#define GPSUSART_MAX_RECV_LEN		800					//�����ջ����ֽ���
#define GPSUSART_RX_BUF_old_LEN 800
#define GPSUSART_MAX_SEND_LEN		300					//����ͻ����ֽ���
#define GPSUSART_RX_EN 					1						//0,������;1,����.

/////////////////////GPS NMEA-0183Э����Ҫ�����ṹ�嶨��///////////////////////////////////////////////////////////////////////
//������Ϣ
__packed typedef struct  
{										    
 	uint8_t num;		//���Ǳ��
	uint8_t eledeg;	//��������
	uint16_t azideg;	//���Ƿ�λ��
	uint8_t sn;		//�����		   
}nmea_slmsg;  
//UTCʱ����Ϣ
__packed typedef struct  
{										    
 	uint16_t year;	//���
	uint8_t month;	//�·�
	uint8_t date;	//����
	uint8_t hour; 	//Сʱ
	uint8_t min; 	//����
	uint8_t sec; 	//����
}nmea_utc_time;   

//NMEA 0183 Э����������ݴ�Žṹ��
__packed typedef struct  
{										    
 	uint8_t svnum;					//�ɼ�������
	nmea_slmsg slmsg[12];		//���12������
	nmea_utc_time utc;			//UTCʱ��
	uint32_t latitude;				//γ�� ������100000��,ʵ��Ҫ����100000
	uint8_t nshemi;					//��γ/��γ,N:��γ;S:��γ				  
	uint32_t longitude;			    //���� ������100000��,ʵ��Ҫ����100000
	uint8_t ewhemi;					//����/����,E:����;W:����
	uint8_t gpssta;					//GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.				  
 	uint8_t posslnum;				//���ڶ�λ��������,0~12.
 	uint8_t possl[12];				//���ڶ�λ�����Ǳ��
	uint8_t fixmode;					//��λ����:1,û�ж�λ;2,2D��λ;3,3D��λ
	uint16_t pdop;					//λ�þ������� 0~500,��Ӧʵ��ֵ0~50.0
	uint16_t hdop;					//ˮƽ�������� 0~500,��Ӧʵ��ֵ0~50.0
	uint16_t vdop;					//��ֱ�������� 0~500,��Ӧʵ��ֵ0~50.0 

	int altitude;			 	//���θ߶�,�Ŵ���10��,ʵ�ʳ���10.��λ:0.1m	 
	uint16_t speed;					//��������,�Ŵ���1000��,ʵ�ʳ���10.��λ:0.001����/Сʱ	 
}nmea_msg; 

extern nmea_msg gpsx; 											//GPS��Ϣ

//////////////////////////////////////////////////////////////////////////////////////////////////// 	
//UBLOX NEO-6M ����(���,����,���ص�)�ṹ��
__packed typedef struct  
{										    
 	uint16_t header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	uint16_t id;						//CFG CFG ID:0X0906 (С��ģʽ)
	uint16_t dlength;				//���ݳ��� 12/13
	uint32_t clearmask;				//�������������(1��Ч)
	uint32_t savemask;				//�����򱣴�����
	uint32_t loadmask;				//�������������
	uint8_t  devicemask; 		  	//Ŀ������ѡ������	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
	uint8_t  cka;		 			//У��CK_A 							 	 
	uint8_t  ckb;			 		//У��CK_B							 	 
}_ublox_cfg_cfg; 

//UBLOX NEO-6M ��Ϣ���ýṹ��
__packed typedef struct  
{										    
 	uint16_t header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	uint16_t id;						//CFG MSG ID:0X0106 (С��ģʽ)
	uint16_t dlength;				//���ݳ��� 8
	uint8_t  msgclass;				//��Ϣ����(F0 ����NMEA��Ϣ��ʽ)
	uint8_t  msgid;					//��Ϣ ID 
								//00,GPGGA;01,GPGLL;02,GPGSA;
								//03,GPGSV;04,GPRMC;05,GPVTG;
								//06,GPGRS;07,GPGST;08,GPZDA;
								//09,GPGBS;0A,GPDTM;0D,GPGNS;
	uint8_t  iicset;					//IIC���������    0,�ر�;1,ʹ��.
	uint8_t  uart1set;				//UART1�������	   0,�ر�;1,ʹ��.
	uint8_t  uart2set;				//UART2�������	   0,�ر�;1,ʹ��.
	uint8_t  usbset;					//USB�������	   0,�ر�;1,ʹ��.
	uint8_t  spiset;					//SPI�������	   0,�ر�;1,ʹ��.
	uint8_t  ncset;					//δ֪�������	   Ĭ��Ϊ1����.
 	uint8_t  cka;			 		//У��CK_A 							 	 
	uint8_t  ckb;			    	//У��CK_B							 	 
}_ublox_cfg_msg; 

//UBLOX NEO-6M UART�˿����ýṹ��
__packed typedef struct  
{										    
 	uint16_t header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	uint16_t id;						//CFG PRT ID:0X0006 (С��ģʽ)
	uint16_t dlength;				//���ݳ��� 20
	uint8_t  portid;					//�˿ں�,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
	uint8_t  reserved;				//����,����Ϊ0
	uint16_t txready;				//TX Ready��������,Ĭ��Ϊ0
	uint32_t mode;					//���ڹ���ģʽ����,��żУ��,ֹͣλ,�ֽڳ��ȵȵ�����.
 	uint32_t baudrate;				//����������
 	uint16_t inprotomask;		 	//����Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	uint16_t outprotomask;		 	//���Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	uint16_t reserved4; 				//����,����Ϊ0
 	uint16_t reserved5; 				//����,����Ϊ0 
 	uint8_t  cka;			 		//У��CK_A 							 	 
	uint8_t  ckb;			    	//У��CK_B							 	 
}_ublox_cfg_prt; 

//UBLOX NEO-6M ʱ���������ýṹ��
__packed typedef struct  
{										    
 	uint16_t header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	uint16_t id;						//CFG TP ID:0X0706 (С��ģʽ)
	uint16_t dlength;				//���ݳ���
	uint32_t interval;				//ʱ��������,��λΪus
	uint32_t length;				 	//������,��λΪus
	signed char status;			//ʱ����������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.			  
	uint8_t timeref;			   		//�ο�ʱ��:0,UTCʱ��;1,GPSʱ��;2,����ʱ��.
	uint8_t flags;					//ʱ���������ñ�־
	uint8_t reserved;				//����			  
 	signed short antdelay;	 	//������ʱ
 	signed short rfdelay;		//RF��ʱ
	signed int userdelay; 	 	//�û���ʱ	
	uint8_t cka;						//У��CK_A 							 	 
	uint8_t ckb;						//У��CK_B							 	 
}_ublox_cfg_tp; 

//UBLOX NEO-6M ˢ���������ýṹ��
__packed typedef struct  
{										    
 	uint16_t header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	uint16_t id;						//CFG RATE ID:0X0806 (С��ģʽ)
	uint16_t dlength;				//���ݳ���
	uint16_t measrate;				//����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
	uint16_t navrate;				//�������ʣ����ڣ����̶�Ϊ1
	uint16_t timeref;				//�ο�ʱ�䣺0=UTC Time��1=GPS Time��
 	uint8_t  cka;					//У��CK_A 							 	 
	uint8_t  ckb;					//У��CK_B							 	 
}_ublox_cfg_rate; 





void GPS_Init(void);
void Get_GPS_Position_Data(void);
void GPS_Get(uint8_t data);
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx);
uint32_t NMEA_Pow(uint8_t m,uint8_t n);
int NMEA_Str2num(uint8_t *buf,uint8_t*dx);
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf);
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf);
void Ublox_CheckSum(uint8_t *buf,u16 len,uint8_t* cka,uint8_t*ckb);
uint8_t Ublox_Cfg_Ack_Check(void);
uint8_t Ublox_Cfg_Cfg_Save(void);
uint8_t Ublox_Cfg_Msg(uint8_t msgid,uint8_t uart1set);
uint8_t Ublox_Cfg_Prt(uint32_t baudrate);
uint8_t Ublox_Cfg_Tp(uint32_t interval,uint32_t length,signed char status);
uint8_t Ublox_Cfg_Rate(uint16_t measrate,uint8_t reftime);



#endif
