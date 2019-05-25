#ifndef  __DRV_BMI055__H__
#define  __DRV_BMI055__H__

#include "sys.h"

#define BMI055_REGA_BGW_CHIPID    0x00
#define BMI055_REGA_ACCD_X_LSB    0x02
#define BMI055_REGA_ACCD_TEMP     0x08
#define BMI055_REGA_INT_STATUS_0  0x09
#define BMI055_REGA_INT_STATUS_1  0x0A
#define BMI055_REGA_INT_STATUS_2  0x0B
#define BMI055_REGA_INT_STATUS_3  0x0C
#define BMI055_REGA_FIFO_STATUS   0x0E
#define BMI055_REGA_PMU_RANGE     0x0F
#define BMI055_REGA_PMU_BW        0x10
#define BMI055_REGA_PMU_LPW       0x11
#define BMI055_REGA_ACCD_HBW      0x13
#define BMI055_REGA_BGW_SOFTRESET 0x14
#define BMI055_REGA_OUT_CTRL      0x20
#define BMI055_REGA_EST_LATCH     0x21
#define BMI055_REGA_FIFO_CONFIG_0 0x30
#define BMI055_REGA_PMU_SELF_TEST 0x32
#define BMI055_REGA_FIFO_CONFIG_1 0x3E
#define BMI055_REGA_FIFO_DATA     0x3F

#define BMI055_REGG_CHIPID        0x00
#define BMI055_REGA_RATE_X_LSB    0x02
#define BMI055_REGG_INT_STATUS_0  0x09
#define BMI055_REGG_INT_STATUS_1  0x0A
#define BMI055_REGG_INT_STATUS_2  0x0B
#define BMI055_REGG_INT_STATUS_3  0x0C
#define BMI055_REGG_FIFO_STATUS   0x0E
#define BMI055_REGG_RANGE         0x0F
#define BMI055_REGG_BW            0x10
#define BMI055_REGG_LPM1          0x11
#define BMI055_REGG_RATE_HBW      0x13
#define BMI055_REGG_BGW_SOFTRESET 0x14
#define BMI055_REGG_INT_EN0       0x15
#define BMI055_REGG_FIFO_CONFIG_1 0x3E
#define BMI055_REGG_FIFO_DATA     0x3F


#define BMI055_ACC_CHIPID_DATA    0xFA
#define BMI055_GYRO_CHIPID_DATA   0x0F



void BMI055_Init(void);
void BMI055_Read_Data(uint8_t *acc_buf,uint8_t *gyro_buf);
uint8_t BMI055_CHIP_Identification(void);
uint8_t BMI055_Acc_Chip_Identification(void);
uint8_t BMI055_Gyro_Chip_Identification(void);
uint8_t SPI_BMI055_AccWriterReg(uint8_t reg,uint8_t data);
uint8_t SPI_BMI055_AccReadReg(uint8_t reg, uint8_t length, uint8_t *data);
uint8_t SPI_BMI055_GyroWriterReg(uint8_t reg,uint8_t data);
uint8_t SPI_BMI055_GyroReadReg(uint8_t reg, uint8_t length, uint8_t *data);

#endif
