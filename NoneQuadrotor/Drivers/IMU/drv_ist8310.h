#ifndef __DRV_IST8310_H__
#define __DRV_IST8310_H__

#include "sys.h"
#include "stdbool.h"
#include "mathtool.h"



extern Vector3i_t magRaw;



/* IST8310的IIC地址选择
  CAD1  |  CAD0  |  地址 | 模拟IIC地址
------------------------------
  GND   |   GND  |  0CH  | 18H
  GND   |   VDD  |  0DH  | 1AH
  VDD   |   GND  |  0EH  | 1CH
  VDD   |   VDD  |  0FH  | 1EH
  如果CAD1和CAD0都是悬空的,地址为0EH
 */
#define IST8310_ADDRESS                 0x0E

#define IST8310_REG_HX_L                0x03
#define IST8310_REG_HX_H                0x04
#define IST8310_REG_HY_L                0x05
#define IST8310_REG_HY_H                0x06
#define IST8310_REG_HZ_L                0x07
#define IST8310_REG_HZ_H                0x08
#define IST8310_REG_WHOAMI              0x00
#define IST8310_REG_CNTRL1              0x0A
#define IST8310_REG_CNTRL2              0x0B
#define IST8310_REG_AVERAGE             0x41
#define IST8310_REG_PDCNTL              0x42
#define IST8310_ODR_SINGLE              0x01
#define IST8310_ODR_10_HZ               0x03
#define IST8310_ODR_20_HZ               0x05
#define IST8310_ODR_50_HZ               0x07
#define IST8310_ODR_100_HZ              0x06
#define IST8310_ODR_200_HZ              0x0B
#define IST8310_CHIP_ID                 0x10
#define IST8310_AVG_16                  0x24
#define IST8310_PULSE_DURATION_NORMAL   0xC0
#define IST8310_CNTRL2_RESET            0x01
#define IST8310_CNTRL2_DRPOL            0x04
#define IST8310_CNTRL2_DRENA            0x08

#define IST8310_MAG_TO_GAUSS            0.0015f





bool IST8310_Detect(void);
void IST8310_Init(void);
void IST8310_Update(void);
void IST8310_Read(Vector3f_t* mag);
static void IST8310_WriteReg(u8 REG_Address,u8 REG_data);
static uint8_t IST8310_ReadReg(u8 REG_Address);










#endif
