#ifndef __DRV_FM25V01G__H__
#define __DRV_FM25V01G__H__
#include "sys.h"
#include "stdbool.h"
#define  FM25V01G_WREN        0X06  //дʹ��
#define  FM25V01G_WRDI        0X04  //д��ʹ��
#define  FM25V01G_RDSR        0X05  //��״̬�Ĵ���
#define  FM25V01G_WRSR        0X01  //д״̬�Ĵ���
#define  FM25V01G_READ        0X03  //���ڴ�����
#define  FM25V01G_FSTRD       0X0B  //���ٶ��ڴ�����
#define  FM25V01G_WRITE       0X02  //д�ڴ�����
#define  FM25V01G_SLEEP       0XB9  //����˯��ģʽ
#define  FM25V01G_RDID        0X9F  //���豸ID
#define  FM25V01G_SNR         0XC3  //��S/N
#define  FM25V01G_DEVICE_ID_DATA   0X7F //��S/N


bool FM25V01G_Detect(void);
void FM25V01G_WriteByte(uint32_t address,uint8_t data);
uint8_t FM25V01G_ReadByte(uint32_t address);
void FM25V01G_WriteMemory(uint32_t address,uint32_t number,uint8_t *p);
void FM25V01G_ReadMemory(uint32_t address,uint32_t number,uint8_t *p) ;
#endif

