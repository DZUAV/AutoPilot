#ifndef __SYSTICK__H__
#define __SYSTICK__H__

#include "sys.h"

extern int time_1h,time_1m,time_1s,time_1ms;//实现1h,1minute,1s,1ms,计时开始


#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFUL /*<< SysTick_LOAD_RELOAD_Pos*/)    /*!< SysTick LOAD: RELOAD Mask */
#define delay_ostickspersec 1000
#define SYSTICK_CLKSOURCE_HCLK         ((uint32_t)0x00000004U)

#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)
#define GET_TIME_NUM 	(8)		//设置获取时间数组的数量
enum
{
	NOW = 0, //0
	OLD,    //1
	NEW,   //2
};



//系统运行时间定义
typedef struct
{
	u8 check_flag;
	u8 err_flag;
	int16_t cnt_1ms;
	int16_t cnt_2ms;
	int16_t cnt_4ms;
	int16_t cnt_5ms;
	int16_t cnt_10ms;
	int16_t cnt_20ms;
	int16_t cnt_25ms;
	int16_t cnt_50ms;
	int16_t cnt_80ms;
	int16_t cnt_90ms;
	int16_t cnt_100ms;
	uint16_t time;
}system_loop_time;

extern system_loop_time LoopTime ;  //系统循环运行时间结构题定义




void Systick_Init(uint32_t Systick_Time);
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void SysTick_Handler(void);
uint32_t GetSysTime_us(void) ;
float Get_Cycle_Time(uint8_t item);
void Task_Trigger_Start(void);
void Loop_check(void);
#endif
