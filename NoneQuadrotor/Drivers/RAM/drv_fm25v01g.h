#ifndef __DRV_FM25V01G__H__
#define __DRV_FM25V01G__H__
#include "sys.h"
#include "stdbool.h"
#define  FM25V01G_WREN        0X06  //写使能
#define  FM25V01G_WRDI        0X04  //写不使能
#define  FM25V01G_RDSR        0X05  //读状态寄存器
#define  FM25V01G_WRSR        0X01  //写状态寄存器
#define  FM25V01G_READ        0X03  //读内存数据
#define  FM25V01G_FSTRD       0X0B  //快速读内存数据
#define  FM25V01G_WRITE       0X02  //写内存数据
#define  FM25V01G_SLEEP       0XB9  //进入睡眠模式
#define  FM25V01G_RDID        0X9F  //读设备ID
#define  FM25V01G_SNR         0XC3  //读S/N
#define  FM25V01G_DEVICE_ID_DATA   0X7F //读S/N


bool FM25V01G_Detect(void);
void FM25V01G_WriteByte(uint32_t address,uint8_t data);
uint8_t FM25V01G_ReadByte(uint32_t address);
void FM25V01G_WriteMemory(uint32_t address,uint32_t number,uint8_t *p);
void FM25V01G_ReadMemory(uint32_t address,uint32_t number,uint8_t *p) ;
#endif

