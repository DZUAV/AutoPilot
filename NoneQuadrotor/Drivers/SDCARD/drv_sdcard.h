#ifndef __DRV_SDCARD__H__
#define __DRV_SDCARD__H__
#include "sys.h"


#define SD_DMA_MODE     0	                        //1��DMAģʽ��0����ѯģʽ   

extern HAL_SD_CardInfoTypedef  SDCardInfo;                 //SD����Ϣ�ṹ��



uint8_t SD_Init(void);
void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
uint8_t SD_GetCardInfo(HAL_SD_CardInfoTypedef *cardinfo);
uint8_t SD_ReadDisk(uint8_t* buf,uint32_t sector,uint8_t cnt);
uint8_t SD_WriteDisk(uint8_t *buf,uint32_t sector,uint8_t cnt);

#endif
