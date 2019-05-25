/**********************************************************************************************************
*文件说明：GPS驱动文件配置函数
*实现功能：配置GPS
*修改日期：2018-12-9
*修改作者：crystal cup
*修改备注：
**********************************************************************************************************/
#include "drv_gps.h"
#include "copter.h"

nmea_msg gpsx; 											//GPS信息
uint8_t GPS_Duty_time;

uint8_t GPSUSART_RX_ready =0;
uint8_t GPSUSART_RX_BUF[GPSUSART_MAX_RECV_LEN];
uint8_t GPSUSART_RX_BUF_old[GPSUSART_RX_BUF_old_LEN];
uint8_t GPSUSART_TX_BUF[GPSUSART_MAX_SEND_LEN]; 		//接收缓冲区，最大GPSUSART_MAX_RECV_LEN字节
uint16_t GPSUSART_RX_STA=0;


/**********************************************************************************************************
*函数原型: void GPS_Init(void)
*函数功能: GPS串口初始化
*输入参数: 
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void GPS_Init(void)
{
	 USART1_init(108,115200);//GPS串口波特率初始化
}


/**********************************************************************************************************
*函数原型: void Get_GPS_Position_Data(void)
*函数功能: 获取GPS位置信息
*输入参数: 
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void Get_GPS_Position_Data(void)
{
	u16 i,rxlen;
    if(GPSUSART_RX_STA>400)//禁止连续进入
  {
	rxlen=GPSUSART_RX_STA;	//得到数据长度
	for(i=0;i<rxlen;i++)GPSUSART_RX_BUF_old[i]=GPSUSART_RX_BUF[i];
	GPSUSART_RX_STA=0;		   	//启动下一次接收
	GPSUSART_RX_BUF_old[i]=0;	//自动添加结束符
	GPS_Analysis(&gpsx,(uint8_t*)GPSUSART_RX_BUF_old);//分析字符串
  }
	GPS_Duty_time++;
	GPS_Duty_time = GPS_Duty_time%2;
}


/**********************************************************************************************************
*函数原型:void GPS_Get(uint8_t data) 
*函数功能:获取GPS数据
*输入参数: 
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void GPS_Get(uint8_t data)
{
		if(GPSUSART_RX_STA<GPSUSART_MAX_RECV_LEN)		//还可以接收数据
		{
			GPSUSART_RX_BUF[GPSUSART_RX_STA++]=data;		//记录接收到的值
		}
		else
		{
			GPSUSART_RX_STA=0;
			GPSUSART_RX_BUF[GPSUSART_RX_STA++]=data;		//记录接收到的值
		}
}

/**********************************************************************************************************
*函数原型: uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
*函数功能: 从buf里面得到第cx个逗号所在的位置
*输入参数: 0~0XFE,代表逗号所在位置的偏移.0XFF,代表不存在第cx个逗号			
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/
			  
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

/**********************************************************************************************************
*函数原型: uint32_t NMEA_Pow(uint8_t m,uint8_t n)
*函数功能: m^n次方.		
*输入参数: 
*返回数据: m^n次方.	
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}

/**********************************************************************************************************
*函数原型: int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
*函数功能: str转换为数字,以','或者'*'结束 
*输入参数: buf:数字存储区 dx:小数点位数,返回给调用函数
*返回数据: 转换后的数值	
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}



/**********************************************************************************************************
*函数原型: void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
*函数功能: 分析GPGSV信息
*输入参数: gpsx:nmea信息结构体 buf:接收到的GPS数据缓冲区首地址
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//得到GPGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个GPGSV信息
	}   
}

/**********************************************************************************************************
*函数原型: void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
*函数功能: 分析GPGGA信息
*输入参数: gpsx:nmea信息结构体 buf:接收到的GPS数据缓冲区首地址
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GPGGA");
	posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}



/**********************************************************************************************************
*函数原型: void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
*函数功能: 分析GPGSA信息
*输入参数: gpsx:nmea信息结构体 buf:接收到的GPS数据缓冲区首地址
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
	uint8_t i;   
	p1=(uint8_t*)strstr((const char *)buf,"$GPGSA");
	posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//得到定位卫星编号
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}


/**********************************************************************************************************
*函数原型: void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
*函数功能: 分析GPRMC信息
*输入参数: gpsx:nmea信息结构体
*返回数据: none
*修改日期: 2018-12-9
*备注信息：buf:接收到的GPS数据缓冲区首地址
**********************************************************************************************************/

void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	uint32_t temp;	   
	float rs;  
	p1=(uint8_t*)strstr((const char *)buf,"GPRMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
}


/**********************************************************************************************************
*函数原型: void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
*函数功能: 分析GPVTG信息
*输入参数: gpsx:nmea信息结构体 buf:接收到的GPS数据缓冲区首地址
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GPVTG");							 
	posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
	}
}  

/**********************************************************************************************************
*函数原型: void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
*函数功能: 分析GPS信息
*输入参数: 
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
**********************************************************************************************************/

void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析
	NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA解析 	
	NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA解析
	NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC解析
	NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG解析
}
/**********************************************************************************************************
*函数原型: void Ublox_CheckSum(uint8_t *buf,u16 len,uint8_t* cka,uint8_t*ckb)
*函数功能: GPS校验和计算
*输入参数: buf:数据缓存区首地址 len:数据长度 cka,ckb:两个校验结果.
*返回数据: none
*修改日期: 2018-12-9
*备注信息：
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
/////////////////////////////////////////UBLOX 配置代码///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**********************************************************************************************************
*函数原型: uint8_t Ublox_Cfg_Ack_Check(void)
*函数功能: 检查CFG配置执行情况
*输入参数: 
*返回数据: none
*修改日期: 2018-12-9
*备注信息：返回值:0,ACK成功
                  1,接收超时错误
                  2,没有找到同步字符
                  3,接收到NACK应答
**********************************************************************************************************/

uint8_t Ublox_Cfg_Ack_Check(void)
{			 
	u16 len=0,i;
	uint8_t rval=0;
	while((GPSUSART_RX_STA&0X8000)==0 && len<100)//等待接收到应答   
	{
		len++;
		delay_ms(5);
	}		 
	if(len<250)   	//超时错误.
	{
		len=GPSUSART_RX_STA&0X7FFF;	//此次接收到的数据长度 
		for(i=0;i<len;i++)if(GPSUSART_RX_BUF[i]==0XB5)break;//查找同步字符 0XB5
		if(i==len)rval=2;						//没有找到同步字符
		else if(GPSUSART_RX_BUF[i+3]==0X00)rval=3;//接收到NACK应答
		else rval=0;	   						//接收到ACK应答
	}else rval=1;								//接收超时错误
    GPSUSART_RX_STA=0;							//清除接收
	return rval;  
}

/**********************************************************************************************************
*函数原型: uint8_t Ublox_Cfg_Cfg_Save(void)
*函数功能: 配置保存
*输入参数: none
*返回数据: 返回值:0,执行成功;1,执行失败.
*修改日期: 2018-12-9
*备注信息：将当前配置保存在外部EEPROM里面
**********************************************************************************************************/

uint8_t Ublox_Cfg_Cfg_Save(void)
{
	uint8_t i;
	_ublox_cfg_cfg *cfg_cfg=(_ublox_cfg_cfg *)GPSUSART_TX_BUF;
	cfg_cfg->header=0X62B5;		//cfg header
	cfg_cfg->id=0X0906;			//cfg cfg id
	cfg_cfg->dlength=13;		//数据区长度为13个字节.		 
	cfg_cfg->clearmask=0;		//清除掩码为0
	cfg_cfg->savemask=0XFFFF; 	//保存掩码为0XFFFF
	cfg_cfg->loadmask=0; 		//加载掩码为0 
	cfg_cfg->devicemask=4; 		//保存在EEPROM里面		 
	Ublox_CheckSum((uint8_t*)(&cfg_cfg->id),sizeof(_ublox_cfg_cfg)-4,&cfg_cfg->cka,&cfg_cfg->ckb);
	Ublox_Send_Date((uint8_t*)cfg_cfg,sizeof(_ublox_cfg_cfg));//发送数据给NEO-6M     
	for(i=0;i<6;i++)if(Ublox_Cfg_Ack_Check()==0)break;		//EEPROM写入需要比较久时间,所以连续判断多次
	return i==6?1:0;
}

/**********************************************************************************************************
*函数原型: uint8_t Ublox_Cfg_Msg(uint8_t msgid,uint8_t uart1set)
*函数功能: 配置NMEA输出信息格式
*输入参数: msgid:要操作的NMEA消息条目,具体见下面的参数表
                  00,GPGGA;01,GPGLL;02,GPGSA;
              		03,GPGSV;04,GPRMC;05,GPVTG;
               		06,GPGRS;07,GPGST;08,GPZDA;
              		09,GPGBS;0A,GPDTM;0D,GPGNS;
*          uart1set:0,输出关闭;1,输出开启.	  
*返回数据: none
*修改日期: 2018-12-9
*备注信息：返回值:0,执行成功;其他,执行失败.
**********************************************************************************************************/

uint8_t Ublox_Cfg_Msg(uint8_t msgid,uint8_t uart1set)
{
	_ublox_cfg_msg *cfg_msg=(_ublox_cfg_msg *)GPSUSART_TX_BUF;
	cfg_msg->header=0X62B5;		//cfg header
	cfg_msg->id=0X0106;			//cfg msg id
	cfg_msg->dlength=8;			//数据区长度为8个字节.	
	cfg_msg->msgclass=0XF0;  	//NMEA消息
	cfg_msg->msgid=msgid; 		//要操作的NMEA消息条目
	cfg_msg->iicset=1; 			//默认开启
	cfg_msg->uart1set=uart1set; //开关设置
	cfg_msg->uart2set=1; 	 	//默认开启
	cfg_msg->usbset=1; 			//默认开启
	cfg_msg->spiset=1; 			//默认开启
	cfg_msg->ncset=1; 			//默认开启	  
	Ublox_CheckSum((uint8_t*)(&cfg_msg->id),sizeof(_ublox_cfg_msg)-4,&cfg_msg->cka,&cfg_msg->ckb);
	Ublox_Send_Date((uint8_t*)cfg_msg,sizeof(_ublox_cfg_msg));//发送数据给NEO-6M    
	return Ublox_Cfg_Ack_Check();
}

/**********************************************************************************************************
*函数原型: uint8_t Ublox_Cfg_Prt(uint32_t baudrate)
*函数功能: 配置NMEA输出信息格式
*输入参数: baudrate:波特率,4800/9600/19200/38400/57600/115200/230400	  
*返回数据: none
*修改日期: 2018-12-9
*备注信息：返回值:0,执行成功;其他,执行失败(这里不会返回0了)
**********************************************************************************************************/


uint8_t Ublox_Cfg_Prt(uint32_t baudrate)
{
	_ublox_cfg_prt *cfg_prt=(_ublox_cfg_prt *)GPSUSART_TX_BUF;
	cfg_prt->header=0X62B5;		//cfg header
	cfg_prt->id=0X0006;			//cfg prt id
	cfg_prt->dlength=20;		//数据区长度为20个字节.	
	cfg_prt->portid=1;			//操作串口1
	cfg_prt->reserved=0;	 	//保留字节,设置为0
	cfg_prt->txready=0;	 		//TX Ready设置为0
	cfg_prt->mode=0X08D0; 		//8位,1个停止位,无校验位
	cfg_prt->baudrate=baudrate; //波特率设置
	cfg_prt->inprotomask=0X0007;//0+1+2
	cfg_prt->outprotomask=0X0007;//0+1+2
 	cfg_prt->reserved4=0; 		//保留字节,设置为0
 	cfg_prt->reserved5=0; 		//保留字节,设置为0 
	Ublox_CheckSum((uint8_t*)(&cfg_prt->id),sizeof(_ublox_cfg_prt)-4,&cfg_prt->cka,&cfg_prt->ckb);
	Ublox_Send_Date((uint8_t*)cfg_prt,sizeof(_ublox_cfg_prt));//发送数据给NEO-6M   
	delay_ms(200);				 //等待发送完成 
	USART1_init(108,baudrate);//GPS串口波特率初始化
	return Ublox_Cfg_Ack_Check();//这里不会反回0,因为UBLOX发回来的应答在串口重新初始化的时候已经被丢弃了.
} 

/**********************************************************************************************************
*函数原型: uint8_t Ublox_Cfg_Tp(uint32_t interval,uint32_t length,signed char status)
*函数功能: 配置UBLOX NEO-6的时钟脉冲输出
*输入参数: interval:脉冲间隔(us) length:脉冲宽度(us) status:脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.
*返回数据: 返回值:0,发送成功;其他,发送失败.
*修改日期: 2018-12-9
*备注信息：返回值:0,发送成功;其他,发送失败.
**********************************************************************************************************/

uint8_t Ublox_Cfg_Tp(uint32_t interval,uint32_t length,signed char status)
{
	_ublox_cfg_tp *cfg_tp=(_ublox_cfg_tp *)GPSUSART_TX_BUF;
	cfg_tp->header=0X62B5;		//cfg header
	cfg_tp->id=0X0706;			//cfg tp id
	cfg_tp->dlength=20;			//数据区长度为20个字节.
	cfg_tp->interval=interval;	//脉冲间隔,us
	cfg_tp->length=length;		//脉冲宽度,us
	cfg_tp->status=status;	   	//时钟脉冲配置
	cfg_tp->timeref=0;			//参考UTC 时间
	cfg_tp->flags=0;			//flags为0
	cfg_tp->reserved=0;		 	//保留位为0
	cfg_tp->antdelay=820;    	//天线延时为820ns
	cfg_tp->rfdelay=0;    		//RF延时为0ns
	cfg_tp->userdelay=0;    	//用户延时为0ns
	Ublox_CheckSum((uint8_t*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
	Ublox_Send_Date((uint8_t*)cfg_tp,sizeof(_ublox_cfg_tp));//发送数据给NEO-6M  
	return Ublox_Cfg_Ack_Check();
}

/*******************************************************************************************************************************
*函数原型: uint8_t Ublox_Cfg_Rate(u16 measrate,uint8_t reftime)
*函数功能: 配置UBLOX NEO-6的更新速率	    
*输入参数: measrate:测量时间间隔，单位为ms，最少不能小于200ms（5Hz）reftime:参考时间，0=UTC Time；1=GPS Time（一般设置为1）
*返回数据: none
*修改日期: 2018-12-9
*备注信息：返回值:0,发送成功;其他,发送失败.
*******************************************************************************************************************************/

uint8_t Ublox_Cfg_Rate(u16 measrate,uint8_t reftime)
{
	_ublox_cfg_rate *cfg_rate=(_ublox_cfg_rate *)GPSUSART_TX_BUF;
 	if(measrate<200)return 1;	//小于200ms，直接退出
 	cfg_rate->header=0X62B5;	//cfg header
	cfg_rate->id=0X0806;	 	//cfg rate id
	cfg_rate->dlength=6;	 	//数据区长度为6个字节.
	cfg_rate->measrate=measrate;//脉冲间隔,us
	cfg_rate->navrate=1;		//导航速率（周期），固定为1
	cfg_rate->timeref=reftime; 	//参考时间为GPS时间
	Ublox_CheckSum((uint8_t*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);
	Ublox_Send_Date((uint8_t*)cfg_rate,sizeof(_ublox_cfg_rate));//发送数据给NEO-6M 
	return Ublox_Cfg_Ack_Check();
}





/***********************************************************************************************************
*                               NoneQuadrotor UAV file_end
***********************************************************************************************************/