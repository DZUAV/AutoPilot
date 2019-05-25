#ifndef __COPTER__H__
#define __COPTER__H__
#include "sys.h"
#include "stm32f7xx.h" 
#include "notice.h"
#include "stm32f765_it.h"	 
#include "systick.h"	 
#include "open_tel_mavlink.h"
#include "mavlink_usart_fifo.h"
#include "ano_protocol.h"
#include "radio.h"
#include "pid.h"
#include "sensor.h"
#include "attitude.h"
#include "mathtype.h"
#include "adrc.h"
#include "motor.h"

//Çý¶¯³õÊ¼»¯
#include "drv_led.h"  
#include "drv_uart.h"	 
#include "drv_spi.h"
#include "drv_iic.h"
#include "drv_ist8310.h"
#include "drv_icm20602.h"
#include "drv_bmi055.h"
#include "drv_ltr.h" 
#include "drv_sensor.h"
#include "drv_ms5611.h"
#include "scheduler.h"
#include "drv_icm20689.h"
#include "drv_mpu6000.h"
#include "drv_pwm.h"
#include "drv_ppm.h"
#include "drv_flash.h"
#include "drv_gps.h"
#include "drv_sdcard.h"
#include "drv_fm25v01g.h"
#include "drv_malloc.h"
#include "drv_buzzer.h"
#include "drv_can.h"
#include "drv_sbus.h" 

#include "ahrs_dcm.h"


#define ANO_Protocol_DT_USE_USARTx  1
//#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000002U) 
extern mavlink_system_t mavlink_system;



void Board_Init(void);


#endif


