#ifndef __DRV_MPU6000_H__
#define __DRV_MPU6000_H__
#include "sys.h"

#define BYTE16(Type,ByteH,ByteL)  ((Type) (((uint16_t)(ByteH)<<8) | ((uint16_t)(ByteL))))



//mpu600¼Ä´æÆ÷¶¨Òå
#define MPU6000_SELF_TEST_XA        ((uint8_t)0x0D)
#define MPU6000_SELF_TEST_YA        ((uint8_t)0x0E)
#define MPU6000_SELF_TEST_ZA        ((uint8_t)0x0F)

#define MPU6000_SELF_TESTA_REG      ((uint8_t)0x10)

#define MPU6000_XG_OFFSET_H         ((uint8_t)0x13)
#define MPU6000_XG_OFFSET_L         ((uint8_t)0x14)
#define MPU6000_YG_OFFSET_H         ((uint8_t)0x15)
#define MPU6000_YG_OFFSET_L         ((uint8_t)0x16)
#define MPU6000_ZG_OFFSET_H         ((uint8_t)0x17)
#define MPU6000_ZG_OFFSET_L         ((uint8_t)0x18)

#define MPU6000_SMPLRT_DIV          ((uint8_t)0x19)
#define MPU6000_CONFIG              ((uint8_t)0x1A)
#define MPU6000_GYRO_CONFIG         ((uint8_t)0x1B)
#define MPU6000_ACCEL_CONFIG        ((uint8_t)0x1C)
#define MPU6000_ACCEL_CONFIG_2      ((uint8_t)0x1D)
#define MPU6000_LP_ACCEL_ODR        ((uint8_t)0x1E)
#define MPU6000_MOT_THR             ((uint8_t)0x1F)
#define MPU6000_FIFO_EN             ((uint8_t)0x23)
#define MPU6000_I2C_MST_CTRL        ((uint8_t)0x24)
#define MPU6000_I2C_SLV0_ADDR       ((uint8_t)0x25)
#define MPU6000_I2C_SLV0_REG        ((uint8_t)0x26)
#define MPU6000_I2C_SLV0_CTRL       ((uint8_t)0x27)
#define MPU6000_I2C_SLV1_ADDR       ((uint8_t)0x28)
#define MPU6000_I2C_SLV1_REG        ((uint8_t)0x29)
#define MPU6000_I2C_SLV1_CTRL       ((uint8_t)0x2A)
#define MPU6000_I2C_SLV2_ADDR       ((uint8_t)0x2B)
#define MPU6000_I2C_SLV2_REG        ((uint8_t)0x2C)
#define MPU6000_I2C_SLV2_CTRL       ((uint8_t)0x2D)
#define MPU6000_I2C_SLV3_ADDR       ((uint8_t)0x2E)
#define MPU6000_I2C_SLV3_REG        ((uint8_t)0x2F)
#define MPU6000_I2C_SLV3_CTRL       ((uint8_t)0x30)
#define MPU6000_I2C_SLV4_ADDR       ((uint8_t)0x31)
#define MPU6000_I2C_SLV4_REG        ((uint8_t)0x32)
#define MPU6000_I2C_SLV4_DO         ((uint8_t)0x33)
#define MPU6000_I2C_SLV4_CTRL       ((uint8_t)0x34)
#define MPU6000_I2C_SLV4_DI         ((uint8_t)0x35)
#define MPU6000_I2C_MST_STATUS      ((uint8_t)0x36)
#define MPU6000_INT_PIN_CFG         ((uint8_t)0x37)
#define MPU6000_INT_ENABLE          ((uint8_t)0x38)
#define MPU6000_INT_STATUS          ((uint8_t)0x3A)
#define MPU6000_ACCEL_XOUT_H        ((uint8_t)0x3B)
#define MPU6000_ACCEL_XOUT_L        ((uint8_t)0x3C)
#define MPU6000_ACCEL_YOUT_H        ((uint8_t)0x3D)
#define MPU6000_ACCEL_YOUT_L        ((uint8_t)0x3E)
#define MPU6000_ACCEL_ZOUT_H        ((uint8_t)0x3F)
#define MPU6000_ACCEL_ZOUT_L        ((uint8_t)0x40)
#define MPU6000_TEMP_OUT_H          ((uint8_t)0x41)
#define MPU6000_TEMP_OUT_L          ((uint8_t)0x42)
#define MPU6000_GYRO_XOUT_H         ((uint8_t)0x43)
#define MPU6000_GYRO_XOUT_L         ((uint8_t)0x44)
#define MPU6000_GYRO_YOUT_H         ((uint8_t)0x45)
#define MPU6000_GYRO_YOUT_L         ((uint8_t)0x46)
#define MPU6000_GYRO_ZOUT_H         ((uint8_t)0x47)
#define MPU6000_GYRO_ZOUT_L         ((uint8_t)0x48)
#define MPU6000_EXT_SENS_DATA_00    ((uint8_t)0x49)
#define MPU6000_EXT_SENS_DATA_01    ((uint8_t)0x4A)
#define MPU6000_EXT_SENS_DATA_02    ((uint8_t)0x4B)
#define MPU6000_EXT_SENS_DATA_03    ((uint8_t)0x4C)
#define MPU6000_EXT_SENS_DATA_04    ((uint8_t)0x4D)
#define MPU6000_EXT_SENS_DATA_05    ((uint8_t)0x4E)
#define MPU6000_EXT_SENS_DATA_06    ((uint8_t)0x4F)
#define MPU6000_EXT_SENS_DATA_07    ((uint8_t)0x50)
#define MPU6000_EXT_SENS_DATA_08    ((uint8_t)0x51)
#define MPU6000_EXT_SENS_DATA_09    ((uint8_t)0x52)
#define MPU6000_EXT_SENS_DATA_10    ((uint8_t)0x53)
#define MPU6000_EXT_SENS_DATA_11    ((uint8_t)0x54)
#define MPU6000_EXT_SENS_DATA_12    ((uint8_t)0x55)
#define MPU6000_EXT_SENS_DATA_13    ((uint8_t)0x56)
#define MPU6000_EXT_SENS_DATA_14    ((uint8_t)0x57)
#define MPU6000_EXT_SENS_DATA_15    ((uint8_t)0x58)
#define MPU6000_EXT_SENS_DATA_16    ((uint8_t)0x59)
#define MPU6000_EXT_SENS_DATA_17    ((uint8_t)0x5A)
#define MPU6000_EXT_SENS_DATA_18    ((uint8_t)0x5B)
#define MPU6000_EXT_SENS_DATA_19    ((uint8_t)0x5C)
#define MPU6000_EXT_SENS_DATA_20    ((uint8_t)0x5D)
#define MPU6000_EXT_SENS_DATA_21    ((uint8_t)0x5E)
#define MPU6000_EXT_SENS_DATA_22    ((uint8_t)0x5F)
#define MPU6000_EXT_SENS_DATA_23    ((uint8_t)0x60)
#define MPU6000_I2C_SLV0_DO         ((uint8_t)0x63)
#define MPU6000_I2C_SLV1_DO         ((uint8_t)0x64)
#define MPU6000_I2C_SLV2_DO         ((uint8_t)0x65)
#define MPU6000_I2C_SLV3_DO         ((uint8_t)0x66)
#define MPU6000_I2C_MST_DELAY_CTRL  ((uint8_t)0x67)
#define MPU6000_SIGNAL_PATH_RESET   ((uint8_t)0x68)
#define MPU6000_MOT_DETECT_CTRL     ((uint8_t)0x69)
#define MPU6000_USER_CTRL_REG       ((uint8_t)0x6A)

#define MPU6000_PWR_MGMT1_REG       ((uint8_t)0x6B)
#define MPU6000_PWR_MGMT2_REG       ((uint8_t)0x6C)
#define MPU6000_FIFO_CNTH_REG       ((uint8_t)0x72)
#define MPU6000_FIFO_CNTL_REG       ((uint8_t)0x73)
#define MPU6000_FIFO_RW_REG         ((uint8_t)0x74)
#define MPU6000_DEVICE_ID_REG       ((uint8_t)0x75)





void MPU6000_Self_Select(void);
void MPU6000_Init(void);
void Mpu6000_Get_Initial_Data(void);
uint8_t SPI_MPU6000_WriteReg(uint8_t reg,uint8_t dat);
uint8_t SPI_MPU6000_ReadReg(uint8_t reg);
uint8_t MPU6000_WriteReg(uint8_t reg,uint8_t dat);
uint8_t MPU6000_ReadReg(uint8_t reg);




#endif
