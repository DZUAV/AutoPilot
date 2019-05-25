/**********************************************************************************************************
*�ļ�˵����GPS�����ļ����ú���
*ʵ�ֹ��ܣ�����GPS
*�޸����ڣ�2018-12-9
*�޸����ߣ�crystal cup
*�޸ı�ע��
**********************************************************************************************************/
#include "drv_gps.h"
#include "copter.h"

nmea_msg gpsx; 											//GPS��Ϣ
uint8_t GPS_Duty_time;

uint8_t GPSUSART_RX_ready =0;
uint8_t GPSUSART_RX_BUF[GPSUSART_MAX_RECV_LEN];
uint8_t GPSUSART_RX_BUF_old[GPSUSART_RX_BUF_old_LEN];
uint8_t GPSUSART_TX_BUF[GPSUSART_MAX_SEND_LEN]; 		//���ջ����������GPSUSART_MAX_RECV_LEN�ֽ�
uint16_t GPSUSART_RX_STA=0;


/**********************************************************************************************************
*����ԭ��: void GPS_Init(void)
*��������: GPS���ڳ�ʼ��
*�������: 
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void GPS_Init(void)
{
	 USART1_init(108,115200);//GPS���ڲ����ʳ�ʼ��
}


/**********************************************************************************************************
*����ԭ��: void Get_GPS_Position_Data(void)
*��������: ��ȡGPSλ����Ϣ
*�������: 
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void Get_GPS_Position_Data(void)
{
	u16 i,rxlen;
    if(GPSUSART_RX_STA>400)//��ֹ��������
  {
	rxlen=GPSUSART_RX_STA;	//�õ����ݳ���
	for(i=0;i<rxlen;i++)GPSUSART_RX_BUF_old[i]=GPSUSART_RX_BUF[i];
	GPSUSART_RX_STA=0;		   	//������һ�ν���
	GPSUSART_RX_BUF_old[i]=0;	//�Զ���ӽ�����
	GPS_Analysis(&gpsx,(uint8_t*)GPSUSART_RX_BUF_old);//�����ַ���
  }
	GPS_Duty_time++;
	GPS_Duty_time = GPS_Duty_time%2;
}


/**********************************************************************************************************
*����ԭ��:void GPS_Get(uint8_t data) 
*��������:��ȡGPS����
*�������: 
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void GPS_Get(uint8_t data)
{
		if(GPSUSART_RX_STA<GPSUSART_MAX_RECV_LEN)		//�����Խ�������
		{
			GPSUSART_RX_BUF[GPSUSART_RX_STA++]=data;		//��¼���յ���ֵ
		}
		else
		{
			GPSUSART_RX_STA=0;
			GPSUSART_RX_BUF[GPSUSART_RX_STA++]=data;		//��¼���յ���ֵ
		}
}

/**********************************************************************************************************
*����ԭ��: uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
*��������: ��buf����õ���cx���������ڵ�λ��
*�������: 0~0XFE,����������λ�õ�ƫ��.0XFF,�������ڵ�cx������			
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/
			  
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

/**********************************************************************************************************
*����ԭ��: uint32_t NMEA_Pow(uint8_t m,uint8_t n)
*��������: m^n�η�.		
*�������: 
*��������: m^n�η�.	
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}

/**********************************************************************************************************
*����ԭ��: int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
*��������: strת��Ϊ����,��','����'*'���� 
*�������: buf:���ִ洢�� dx:С����λ��,���ظ����ú���
*��������: ת�������ֵ	
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}



/**********************************************************************************************************
*����ԭ��: void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
*��������: ����GPGSV��Ϣ
*�������: gpsx:nmea��Ϣ�ṹ�� buf:���յ���GPS���ݻ������׵�ַ
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//�õ�GPGSV������
	posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//�õ����Ǳ��
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ��������� 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//�õ����������
			else break;
			slx++;	   
		}   
 		p=p1+1;//�л�����һ��GPGSV��Ϣ
	}   
}

/**********************************************************************************************************
*����ԭ��: void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
*��������: ����GPGGA��Ϣ
*�������: gpsx:nmea��Ϣ�ṹ�� buf:���յ���GPS���ݻ������׵�ַ
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GPGGA");
	posx=NMEA_Comma_Pos(p1,6);								//�õ�GPS״̬
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//�õ����θ߶�
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}



/**********************************************************************************************************
*����ԭ��: void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
*��������: ����GPGSA��Ϣ
*�������: gpsx:nmea��Ϣ�ṹ�� buf:���յ���GPS���ݻ������׵�ַ
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
	uint8_t i;   
	p1=(uint8_t*)strstr((const char *)buf,"$GPGSA");
	posx=NMEA_Comma_Pos(p1,2);								//�õ���λ����
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//�õ���λ���Ǳ��
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//�õ�PDOPλ�þ�������
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//�õ�HDOPλ�þ�������
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//�õ�VDOPλ�þ�������
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}


/**********************************************************************************************************
*����ԭ��: void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
*��������: ����GPRMC��Ϣ
*�������: gpsx:nmea��Ϣ�ṹ��
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��buf:���յ���GPS���ݻ������׵�ַ
**********************************************************************************************************/

void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	float rs;  
	p1=(uint8_t*)strstr((const char *)buf,"GPRMC");//"$GPRMC",������&��GPRMC�ֿ������,��ֻ�ж�GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	posx=NMEA_Comma_Pos(p1,6);								//������������
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
}


/**********************************************************************************************************
*����ԭ��: void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
*��������: ����GPVTG��Ϣ
*�������: gpsx:nmea��Ϣ�ṹ�� buf:���յ���GPS���ݻ������׵�ַ
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GPVTG");							 
	posx=NMEA_Comma_Pos(p1,7);								//�õ���������
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//ȷ������1000��
	}
}  

/**********************************************************************************************************
*����ԭ��: void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
*��������: ����GPS��Ϣ
*�������: 
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV����
	NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA���� 	
	NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA����
	NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC����
	NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG����
}
/**********************************************************************************************************
*����ԭ��: void Ublox_CheckSum(uint8_t *buf,u16 len,uint8_t* cka,uint8_t*ckb)
*��������: GPSУ��ͼ���
*�������: buf:���ݻ������׵�ַ len:���ݳ��� cka,ckb:����У����.
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ��
**********************************************************************************************************/

void Ublox_CheckSum(uint8_t *buf,u16 len,uint8_t* cka,uint8_t*ckb)
{
	u16 i;
	*cka=0;*ckb=0;
	for(i=0;i<len;i++)
	{
		*cka=*cka+buf[i];
		*ckb=*ckb+*cka;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////UBLOX ���ô���///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**********************************************************************************************************
*����ԭ��: uint8_t Ublox_Cfg_Ack_Check(void)
*��������: ���CFG����ִ�����
*�������: 
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ������ֵ:0,ACK�ɹ�
                  1,���ճ�ʱ����
                  2,û���ҵ�ͬ���ַ�
                  3,���յ�NACKӦ��
**********************************************************************************************************/

uint8_t Ublox_Cfg_Ack_Check(void)
{			 
	u16 len=0,i;
	uint8_t rval=0;
	while((GPSUSART_RX_STA&0X8000)==0 && len<100)//�ȴ����յ�Ӧ��   
	{
		len++;
		delay_ms(5);
	}		 
	if(len<250)   	//��ʱ����.
	{
		len=GPSUSART_RX_STA&0X7FFF;	//�˴ν��յ������ݳ��� 
		for(i=0;i<len;i++)if(GPSUSART_RX_BUF[i]==0XB5)break;//����ͬ���ַ� 0XB5
		if(i==len)rval=2;						//û���ҵ�ͬ���ַ�
		else if(GPSUSART_RX_BUF[i+3]==0X00)rval=3;//���յ�NACKӦ��
		else rval=0;	   						//���յ�ACKӦ��
	}else rval=1;								//���ճ�ʱ����
    GPSUSART_RX_STA=0;							//�������
	return rval;  
}

/**********************************************************************************************************
*����ԭ��: uint8_t Ublox_Cfg_Cfg_Save(void)
*��������: ���ñ���
*�������: none
*��������: ����ֵ:0,ִ�гɹ�;1,ִ��ʧ��.
*�޸�����: 2018-12-9
*��ע��Ϣ������ǰ���ñ������ⲿEEPROM����
**********************************************************************************************************/

uint8_t Ublox_Cfg_Cfg_Save(void)
{
	uint8_t i;
	_ublox_cfg_cfg *cfg_cfg=(_ublox_cfg_cfg *)GPSUSART_TX_BUF;
	cfg_cfg->header=0X62B5;		//cfg header
	cfg_cfg->id=0X0906;			//cfg cfg id
	cfg_cfg->dlength=13;		//����������Ϊ13���ֽ�.		 
	cfg_cfg->clearmask=0;		//�������Ϊ0
	cfg_cfg->savemask=0XFFFF; 	//��������Ϊ0XFFFF
	cfg_cfg->loadmask=0; 		//��������Ϊ0 
	cfg_cfg->devicemask=4; 		//������EEPROM����		 
	Ublox_CheckSum((uint8_t*)(&cfg_cfg->id),sizeof(_ublox_cfg_cfg)-4,&cfg_cfg->cka,&cfg_cfg->ckb);
	Ublox_Send_Date((uint8_t*)cfg_cfg,sizeof(_ublox_cfg_cfg));//�������ݸ�NEO-6M     
	for(i=0;i<6;i++)if(Ublox_Cfg_Ack_Check()==0)break;		//EEPROMд����Ҫ�ȽϾ�ʱ��,���������ж϶��
	return i==6?1:0;
}

/**********************************************************************************************************
*����ԭ��: uint8_t Ublox_Cfg_Msg(uint8_t msgid,uint8_t uart1set)
*��������: ����NMEA�����Ϣ��ʽ
*�������: msgid:Ҫ������NMEA��Ϣ��Ŀ,���������Ĳ�����
                  00,GPGGA;01,GPGLL;02,GPGSA;
              		03,GPGSV;04,GPRMC;05,GPVTG;
               		06,GPGRS;07,GPGST;08,GPZDA;
              		09,GPGBS;0A,GPDTM;0D,GPGNS;
*          uart1set:0,����ر�;1,�������.	  
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ������ֵ:0,ִ�гɹ�;����,ִ��ʧ��.
**********************************************************************************************************/

uint8_t Ublox_Cfg_Msg(uint8_t msgid,uint8_t uart1set)
{
	_ublox_cfg_msg *cfg_msg=(_ublox_cfg_msg *)GPSUSART_TX_BUF;
	cfg_msg->header=0X62B5;		//cfg header
	cfg_msg->id=0X0106;			//cfg msg id
	cfg_msg->dlength=8;			//����������Ϊ8���ֽ�.	
	cfg_msg->msgclass=0XF0;  	//NMEA��Ϣ
	cfg_msg->msgid=msgid; 		//Ҫ������NMEA��Ϣ��Ŀ
	cfg_msg->iicset=1; 			//Ĭ�Ͽ���
	cfg_msg->uart1set=uart1set; //��������
	cfg_msg->uart2set=1; 	 	//Ĭ�Ͽ���
	cfg_msg->usbset=1; 			//Ĭ�Ͽ���
	cfg_msg->spiset=1; 			//Ĭ�Ͽ���
	cfg_msg->ncset=1; 			//Ĭ�Ͽ���	  
	Ublox_CheckSum((uint8_t*)(&cfg_msg->id),sizeof(_ublox_cfg_msg)-4,&cfg_msg->cka,&cfg_msg->ckb);
	Ublox_Send_Date((uint8_t*)cfg_msg,sizeof(_ublox_cfg_msg));//�������ݸ�NEO-6M    
	return Ublox_Cfg_Ack_Check();
}

/**********************************************************************************************************
*����ԭ��: uint8_t Ublox_Cfg_Prt(uint32_t baudrate)
*��������: ����NMEA�����Ϣ��ʽ
*�������: baudrate:������,4800/9600/19200/38400/57600/115200/230400	  
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ������ֵ:0,ִ�гɹ�;����,ִ��ʧ��(���ﲻ�᷵��0��)
**********************************************************************************************************/


uint8_t Ublox_Cfg_Prt(uint32_t baudrate)
{
	_ublox_cfg_prt *cfg_prt=(_ublox_cfg_prt *)GPSUSART_TX_BUF;
	cfg_prt->header=0X62B5;		//cfg header
	cfg_prt->id=0X0006;			//cfg prt id
	cfg_prt->dlength=20;		//����������Ϊ20���ֽ�.	
	cfg_prt->portid=1;			//��������1
	cfg_prt->reserved=0;	 	//�����ֽ�,����Ϊ0
	cfg_prt->txready=0;	 		//TX Ready����Ϊ0
	cfg_prt->mode=0X08D0; 		//8λ,1��ֹͣλ,��У��λ
	cfg_prt->baudrate=baudrate; //����������
	cfg_prt->inprotomask=0X0007;//0+1+2
	cfg_prt->outprotomask=0X0007;//0+1+2
 	cfg_prt->reserved4=0; 		//�����ֽ�,����Ϊ0
 	cfg_prt->reserved5=0; 		//�����ֽ�,����Ϊ0 
	Ublox_CheckSum((uint8_t*)(&cfg_prt->id),sizeof(_ublox_cfg_prt)-4,&cfg_prt->cka,&cfg_prt->ckb);
	Ublox_Send_Date((uint8_t*)cfg_prt,sizeof(_ublox_cfg_prt));//�������ݸ�NEO-6M   
	delay_ms(200);				 //�ȴ�������� 
	USART1_init(108,baudrate);//GPS���ڲ����ʳ�ʼ��
	return Ublox_Cfg_Ack_Check();//���ﲻ�ᷴ��0,��ΪUBLOX��������Ӧ���ڴ������³�ʼ����ʱ���Ѿ���������.
} 

/**********************************************************************************************************
*����ԭ��: uint8_t Ublox_Cfg_Tp(uint32_t interval,uint32_t length,signed char status)
*��������: ����UBLOX NEO-6��ʱ���������
*�������: interval:������(us) length:������(us) status:��������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
*��������: ����ֵ:0,���ͳɹ�;����,����ʧ��.
*�޸�����: 2018-12-9
*��ע��Ϣ������ֵ:0,���ͳɹ�;����,����ʧ��.
**********************************************************************************************************/

uint8_t Ublox_Cfg_Tp(uint32_t interval,uint32_t length,signed char status)
{
	_ublox_cfg_tp *cfg_tp=(_ublox_cfg_tp *)GPSUSART_TX_BUF;
	cfg_tp->header=0X62B5;		//cfg header
	cfg_tp->id=0X0706;			//cfg tp id
	cfg_tp->dlength=20;			//����������Ϊ20���ֽ�.
	cfg_tp->interval=interval;	//������,us
	cfg_tp->length=length;		//������,us
	cfg_tp->status=status;	   	//ʱ����������
	cfg_tp->timeref=0;			//�ο�UTC ʱ��
	cfg_tp->flags=0;			//flagsΪ0
	cfg_tp->reserved=0;		 	//����λΪ0
	cfg_tp->antdelay=820;    	//������ʱΪ820ns
	cfg_tp->rfdelay=0;    		//RF��ʱΪ0ns
	cfg_tp->userdelay=0;    	//�û���ʱΪ0ns
	Ublox_CheckSum((uint8_t*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
	Ublox_Send_Date((uint8_t*)cfg_tp,sizeof(_ublox_cfg_tp));//�������ݸ�NEO-6M  
	return Ublox_Cfg_Ack_Check();
}

/*******************************************************************************************************************************
*����ԭ��: uint8_t Ublox_Cfg_Rate(u16 measrate,uint8_t reftime)
*��������: ����UBLOX NEO-6�ĸ�������	    
*�������: measrate:����ʱ��������λΪms�����ٲ���С��200ms��5Hz��reftime:�ο�ʱ�䣬0=UTC Time��1=GPS Time��һ������Ϊ1��
*��������: none
*�޸�����: 2018-12-9
*��ע��Ϣ������ֵ:0,���ͳɹ�;����,����ʧ��.
*******************************************************************************************************************************/

uint8_t Ublox_Cfg_Rate(u16 measrate,uint8_t reftime)
{
	_ublox_cfg_rate *cfg_rate=(_ublox_cfg_rate *)GPSUSART_TX_BUF;
 	if(measrate<200)return 1;	//С��200ms��ֱ���˳�
 	cfg_rate->header=0X62B5;	//cfg header
	cfg_rate->id=0X0806;	 	//cfg rate id
	cfg_rate->dlength=6;	 	//����������Ϊ6���ֽ�.
	cfg_rate->measrate=measrate;//������,us
	cfg_rate->navrate=1;		//�������ʣ����ڣ����̶�Ϊ1
	cfg_rate->timeref=reftime; 	//�ο�ʱ��ΪGPSʱ��
	Ublox_CheckSum((uint8_t*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);
	Ublox_Send_Date((uint8_t*)cfg_rate,sizeof(_ublox_cfg_rate));//�������ݸ�NEO-6M 
	return Ublox_Cfg_Ack_Check();
}





/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/