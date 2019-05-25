/******************************************************************************************************************
**头文件说明 ： 任务调度函数
**时      间 ： 2018-4-30
**作      者 ： 历小伟
**联系 方式  ： 15982962929
**备      注 ：
******************************************************************************************************************/



#include "ano_protocol.h"
#include "copter.h"



extern float MS5611_BUFFER[3];

uint8_t data_to_send[50];	 //发送数据缓存
uint8_t checkdata_to_send,checksum_to_send;

uint8_t msg_id;
uint8_t msg_data;
uint8_t send_check;
uint8_t send_version;
uint8_t send_status;
uint8_t send_senser;
uint8_t send_senser2;
uint8_t send_pid1;
uint8_t send_pid2;
uint8_t send_pid3;
uint8_t send_pid4;
uint8_t send_pid5;
uint8_t send_pid6;
uint8_t send_rcdata;
uint8_t send_offset;
uint8_t send_motopwm;
uint8_t send_power;
uint8_t send_user;
uint8_t send_speed;
uint8_t send_location;

/*==================================================================================================================*/
/*==================================================================================================================*
**函数原型 : void CONST_LED::init(void)
**功    能 : led 初始化函数
**输    入 : None
**输    出 : None
**备    注 : PE12---LED 
**           PA9----USB
**================================================================================================================*/
/*================================================================================================================*/

void  Communicate_DT_Send_Data(u8 *dataToSend , u8 length)
{
//#ifdef Communicate_DT_USE_USB_HID
//	Usb_Hid_Adddata(data_to_send,length);
//#endif
#ifdef ANO_Protocol_DT_USE_USARTx
	Usart2_Send(data_to_send, length);
#endif
}


/*==================================================================================================================*/
/*==================================================================================================================*
**函数原型 : void CONST_LED::init(void)
**功    能 : led 初始化函数
**输    入 : None
**输    出 : None
**备    注 : PE12---LED 
**           PA9----USB
**================================================================================================================*/
/*================================================================================================================*/
 void Communicate_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 sum = 0;
	u8 i=0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	

	for( i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	Communicate_DT_Send_Data(data_to_send, 7);
}

/*==================================================================================================================*/
/*==================================================================================================================*
**函数原型 : void CONST_LED::init(void)
**功    能 : led 初始化函数
**输    入 : None
**输    出 : None
**备    注 : PE12---LED 
**           PA9----USB
**================================================================================================================*/
/*================================================================================================================*/
 void Communicate_DT_Send_Msg(u8 id, u8 data)
{
		u8 sum = 0;
	u8 i=0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;
	data_to_send[4]=id;
	data_to_send[5]=data;
	
	

	for( i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	Communicate_DT_Send_Data(data_to_send, 7);
}





/*==================================================================================================================*/
/*==================================================================================================================*
**函数原型 : void CONST_LED::init(void)
**功    能 : led 初始化函数
**输    入 : None
**输    出 : None
**备    注 : PE12---LED 
**           PA9----USB
**================================================================================================================*/
/*================================================================================================================*/

void Communicate_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 10;
	static u8 senser2_cnt = 50;
	static u8 user_cnt 	  = 10;
	static u8 status_cnt 	= 15;
	static u8 rcdata_cnt 	= 20;
	static u8 motopwm_cnt	= 20;
	static u8 power_cnt		=	50;
	static u8 speed_cnt   = 50;
	static u8 location_cnt   = 200;
	
	if((cnt % senser_cnt) == (senser_cnt-1))  //?????+MPU6050+hmc5883l
		send_senser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))        //???+???
		send_senser2 = 1;	

	if((cnt % user_cnt) == (user_cnt-2))            //???
		send_user = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))        //????+????
		send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))        //?????
		send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))      //??PWM
		send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-2))         //??????
		send_power = 1;		
	
	if((cnt % speed_cnt) == (speed_cnt-3))        //????
		send_speed = 1;		
	
	if((cnt % location_cnt) == (location_cnt-3)) //GPS??
	{
		send_location += 1;		
	}
	
	if(++cnt>200) cnt = 0;
 
//////////////////////////??///////////////////////////////////////////////////////////
   if(msg_id)
	{
	    Communicate_DT_Send_Msg(msg_id,msg_data);
		msg_id = 0;
	}
	/////////////////////////??////////////////////////////////////////////////////////////
	if(send_check)
	{
		send_check = 0;
		Communicate_DT_Send_Check(checkdata_to_send,checksum_to_send);
	}
	else if(send_version)
	{
		send_version = 0;
		Communicate_DT_Send_Version(4,300,100,400,0);
	}
//////////////////////////??/////////////////////////////////////////////////////////
	else if(send_status)
	{
		send_status = 0;
		
		Communicate_DT_Send_Status(0,0,0,0,0,0);	
//		Communicate_DT_Send_Status(Roll,Pitch,Yaw,(EstAlt*100),3,fly_ready_islock);		
//		Communicate_DT_Send_Status(Roll,Pitch,Yaw,(0.1f *baro_height),0,fly_ready_islock);	
	}	
//////////////////////////??//////////////////////////////////////////////////////////
	else if(send_speed)
	{
		send_speed = 0;
	    Communicate_DT_Send_Speed(sensor1[3]*1000,sensor1[4]*1000,sensor1[5]*1000);
//		Communicate_DT_Send_Speed(accAlt_test*1000,30,wz_speed);
	}
///////////////////////////////////////////////////////////////////////////////////
	else if(send_user)
	{
		send_user = 0;
		Communicate_DT_Send_User();
	}
//////////////////////////?????///////////////////////////////////////////////////////////
	else if(send_senser)
	{	
		send_senser = 0;
		Communicate_DT_Send_Senser(sensor[0],sensor[1],sensor[2],sensor[3],sensor[4],sensor[5],magRaw.x,magRaw.y,magRaw.z);
//			Communicate_DT_Send_Senser(mpu6050.Acc.x,mpu6050.Acc.y,mpu6050.Acc.z,
//												mpu6050.Gyro.x,mpu6050.Gyro.y,mpu6050.Gyro.z,HMC5883L_MAG.HMC5883L_Val.x,HMC5883L_MAG.HMC5883L_Val.y,HMC5883L_MAG.HMC5883L_Val.z); //????????

	}	
	/////////////////////////////////////////////////////////////////////////////////////
	else if(send_senser2) 
	{
		send_senser2 = 0;
	    Communicate_DT_Send_Senser2(ms5611_pressure*100,MS5611_BUFFER[1]*100);
//		Communicate_DT_Send_Senser2(ms5611_pressure,baroAlt_test*100);
//		Communicate_DT_Send_Senser2(1000,20);//100????????

	}
	
	
	

/////////////////////////////?????////////////////////////////////////////////////////////////
	else if(send_rcdata)
	{
		send_rcdata = 0;
		//Communicate_DT_Send_RCData(RC_CH[2]+1500,RC_CH[3]+1500,RC_CH[0]+1500,RC_CH[1]+1500,RC_CH[4]+1500,RC_CH[5]+1500,RC_CH[6]+1500,RC_CH[7]+1500,0 +1500,0 +1500);
	}	
/////////////////////////////????////////////////////////////////////////////////////////////	
	else if(send_motopwm)
	{
		send_motopwm = 0;
//		Communicate_DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////////
	else if(send_power)
	{
		send_power = 0;
		//Communicate_DT_Send_Power(123,456);//GPS_angle
	//	Communicate_DT_Send_Power(GPS_angle[0]*100,GPS_angle[1]*100);//GPS_angle
	}
///////////////////////////////???//////////////////////////////////////////////////////////
	else if(send_pid1)
	{
		send_pid1 = 0;
//		Communicate_DT_Send_PID(1,inner_control.PID[PIDROLL].kp,inner_control.PID[PIDROLL].ki,inner_control.PID[PIDROLL].kd,
//											inner_control.PID[PIDPITCH].kp,inner_control.PID[PIDPITCH].ki,inner_control.PID[PIDPITCH].kd,
//											inner_control.PID[PIDYAW].kp,inner_control.PID[PIDYAW].ki,inner_control.PID[PIDYAW].kd);
	}	
//////////////////////////////??///////////////////////////////////////////////////////////
	else if(send_pid2)
	{
		send_pid2 = 0;
//		Communicate_DT_Send_PID(2,outer_control.PID[PIDROLL].kp,outer_control.PID[PIDROLL].ki,outer_control.PID[PIDROLL].kd,
//											outer_control.PID[PIDPITCH].kp,outer_control.PID[PIDPITCH].ki,outer_control.PID[PIDPITCH].kd,
//											outer_control.PID[PIDYAW].kp,outer_control.PID[PIDYAW].ki,outer_control.PID[PIDYAW].kd);
	}
	///////////////////////////////??PID///????????1000?/////////////////////////////////////////////////////
	else if(send_pid3)
	{
		send_pid3 = 0;
		
//		Communicate_DT_Send_PID(3,BF_PID.Baseflight_PID_Vel_P,BF_PID.Baseflight_PID_Vel_I,BF_PID.Baseflight_PID_Vel_D,  //????PID
//											 BF_PID.Baseflight_PID_Altitude_P, BF_PID.Baseflight_PID_Altitude_I, BF_PID.Baseflight_PID_Altitude_D,//??PID
//											pid_setup.groups.ctrl3.kp,pid_setup.groups.ctrl3.ki,pid_setup.groups.ctrl3.kd);//????PID
	
//		Communicate_DT_Send_PID(3,pid_setup.groups.hc_sp.kp,pid_setup.groups.hc_sp.ki,pid_setup.groups.hc_sp.kd,  //????PID
//											pid_setup.groups.hc_height.kp,pid_setup.groups.hc_height.ki,pid_setup.groups.hc_height.kd,//??PID
//											pid_setup.groups.ctrl3.kp,pid_setup.groups.ctrl3.ki,pid_setup.groups.ctrl3.kd);//????PID
	}
	else if(send_pid4)
	{
		send_pid4 = 0;
//		Communicate_DT_Send_PID(4,pid_setup.groups.ctrl4.kp,pid_setup.groups.ctrl4.ki,pid_setup.groups.ctrl4.kd,
//											0						,0						,0						,
//											0						,0						,0						);
	}
	else if(send_pid5)
	{
		send_pid5 = 0;
	//	Communicate_DT_Send_PID(5,0,0,0,0,0,0,0,0,0);
	}
	else if(send_pid6)
	{
		send_pid6 = 0;
	//	Communicate_DT_Send_PID(6,0,0,0,0,0,0,0,0,0);
	}
	
	else if(send_location == 2)
	{
		
		send_location = 0;
		//Communicate_DT_Send_Location(gpsx.gpssta,gpsx.svnum,gpsx.longitude *100,gpsx.latitude *100,gpsx.altitude);//--- ??????---????????????????
		//Communicate_DT_Send_Location(11,12,13 *10000000,14 *10000000,4);//--- STA---???????????????
		
	}
/////////////////////////////////////////////////////////////////////////////////////10000000
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//	Usb_Hid_Send();					
/////////////////////////////////////////////////////////////////////////////////////
}

/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		Communicate_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}


/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////
//
//u16 flash_save_en_cnt = 0;

//u16 RX_CH[RC_CH_NUM];

void Communicate_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	u8 i=0;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//????
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
		//	mpu6050.Acc_CALIBRATE = 1;
//			mpu6050.Cali_3d = 1;
		}
		else if(*(data_buf+4)==0X02)
			;
//			mpu6050.Gyro_CALIBRATE = 1;
		else if(*(data_buf+4)==0X03)
		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
		}
		else if(*(data_buf+4)==0X04)
		{
		//	hmc5883l_f.calibratingM=1;//??????

		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
		{
//			acc_3d_calibrate_f = 1;
		}
		else if(*(data_buf+4)==0X20)
		{
//			acc_3d_step = 0; //??,6?????0
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
//			send_pid1 = 1;
//			send_pid2 = 1;
//			send_pid3 = 1;
//			send_pid4 = 1;
//			send_pid5 = 1;
//			send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//??????
		{
//			send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//??????
		{
//			Para_ResetToFactorySetup();
		}
	}

//	if(*(data_buf+2)==0X03)
//	{
//		if( NS != 1 )
//		{
////			Feed_Rc_Dog(2);
//		}

//		RX_CH[THR] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
//		RX_CH[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
//		RX_CH[ROL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
//		RX_CH[PIT] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
//		RX_CH[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
//		RX_CH[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
//		RX_CH[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
//		RX_CH[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
//	}
//	if(*(data_buf+2)==0X10)								//PID1---???
//    {
//        inner_control.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        inner_control.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        inner_control.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//        inner_control.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        inner_control.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        inner_control.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        inner_control.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        inner_control.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        inner_control.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
//				if(f.send_check == 0)
//				{
//					f.send_check = 1;
//					checkdata_to_send = *(data_buf+2);
//					checksum_to_send = sum;
//				}
//			  PID_Para_Init();
//				flash_save_en_cnt = 1;
//    }
//    if(*(data_buf+2)==0X11)								//PID2--??
//    {
//        outer_control.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        outer_control.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        outer_control.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//        outer_control.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        outer_control.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        outer_control.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        outer_control.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        outer_control.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        outer_control.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
//        if(f.send_check == 0)
//				{
//					f.send_check = 1;
//					checkdata_to_send = *(data_buf+2);
//					checksum_to_send = sum;
//				}
//				PID_Para_Init();
//				flash_save_en_cnt = 1;
//    }
//    if(*(data_buf+2)==0X12)								//PID3
//    {	
//			
////			 //??PID--??
////        BF_PID.Baseflight_PID_Vel_P = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
////        BF_PID.Baseflight_PID_Vel_I = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
////        BF_PID.Baseflight_PID_Vel_D = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
////			   //??PID
////        BF_PID.Baseflight_PID_Altitude_P = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
////        BF_PID.Baseflight_PID_Altitude_I = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
////        BF_PID.Baseflight_PID_Altitude_D = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//			
//			   //??PID--??
//        pid_setup.groups.hc_sp.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        pid_setup.groups.hc_sp.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        pid_setup.groups.hc_sp.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//			   //??PID
//        pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//			
//			
//			
//        pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
//        if(f.send_check == 0)
//				{
//					f.send_check = 1;
//					checkdata_to_send = *(data_buf+2);
//					checksum_to_send = sum;
//				}
//				PID_Para_Init();
//				flash_save_en_cnt = 1;
//    }
//	if(*(data_buf+2)==0X13)								//PID4
//	{
//		    pid_setup.groups.ctrl4.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        pid_setup.groups.ctrl4.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        pid_setup.groups.ctrl4.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//			
////         pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
////         pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
////         pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//// 			
////         pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
////         pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
////         pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
//		if(f.send_check == 0)
//		{
//			f.send_check = 1;
//			checkdata_to_send = *(data_buf+2);
//			checksum_to_send = sum;
//		}
//		PID_Para_Init();
//		flash_save_en_cnt = 1;
//	}
//	if(*(data_buf+2)==0X14)								//PID5
//	{
//		if(f.send_check == 0)
//		{
//			f.send_check = 1;
//			checkdata_to_send = *(data_buf+2);
//			checksum_to_send = sum;
//		}
//	}
//	if(*(data_buf+2)==0X15)								//PID6
//	{
//		if(f.send_check == 0)
//		{
//			f.send_check = 1;
//			checkdata_to_send = *(data_buf+2);
//			checksum_to_send = sum;
//		}
//	}


}


/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Communicate_DT_Send_Data(data_to_send, _cnt);
}


/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Communicate_DT_Send_Data(data_to_send, _cnt);

}


/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	u8 sum = 0;
	u8 i=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x04;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=state;
	data_to_send[_cnt++]=sat_num;
	
	_temp2 = lon;//??
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	_temp2 = lat;//??
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	
	_temp = (s16)(100 *back_home_angle);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Communicate_DT_Send_Data(data_to_send, _cnt);

}

/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Communicate_DT_Send_Data(data_to_send, _cnt);
}


/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Communicate_DT_Send_Data(data_to_send, _cnt);
}

/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Communicate_DT_Send_Data(data_to_send, _cnt);
}

/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Communicate_DT_Send_Data(data_to_send, _cnt);
}
/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Communicate_DT_Send_Data(data_to_send, _cnt);
}


/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Communicate_DT_Send_Data(data_to_send, _cnt);
}
/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/

void Communicate_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Communicate_DT_Send_Data(data_to_send, _cnt);
}
extern u16 ultra_distance;
extern float ultra_speed;
extern float wz_speed_0,wz_speed,baro_speed,m_norm;
extern float ultra_ctrl_out;
extern float baro_height,baro_measure;
extern float yaw_mag,airframe_x_sp,airframe_y_sp,wx_sp,wy_sp;
extern float werr_x_gps,werr_y_gps,aerr_x_gps,aerr_y_gps;

/**********************************************************************************************************
*函数原型: void led2_task(void *pvParameters)
*函数功能: led2任务
*输入参数: pvParameters
*返回数据: none
*修改日期: 2018-10-31
*修改作者: cihang_uav
*备注信息：
**********************************************************************************************************/
void Communicate_DT_Send_User(void)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf1; //????
	data_to_send[_cnt++]=0;
	
//	
//	_temp = (s16)baro_measure;            //1
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);

//	_temp = (s16)wz_speed;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	_temp = (s16)wz_speed;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);	
//	
//	_temp = (s16)baro_speed;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//  _temp = (s16)baro_height;              //5
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
	
	

	data_to_send[3] = _cnt-4;
	

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Communicate_DT_Send_Data(data_to_send, _cnt);
}






/***********************************************************************************************************
*                               cihang_uav file_end
***********************************************************************************************************/
