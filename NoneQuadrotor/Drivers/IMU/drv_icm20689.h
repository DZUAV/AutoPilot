#ifndef __ICM20689_H__
#define	__ICM20689_H__

#include "mathtool.h"
#include "drv_spi.h"
extern float sensor1[9];

#define ICM20689_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define ICM20689_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define ICM20689_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define ICM20689_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define ICM20689_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define ICM20689_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define ICM20689_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define ICM20689_RA_XA_OFFS_L_TC     0x07
#define ICM20689_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define ICM20689_RA_YA_OFFS_L_TC     0x09
#define ICM20689_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define ICM20689_RA_ZA_OFFS_L_TC     0x0B
#define ICM20689_RA_PRODUCT_ID       0x0C    // Product ID Register
#define ICM20689_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define ICM20689_RA_XG_OFFS_USRL     0x14
#define ICM20689_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define ICM20689_RA_YG_OFFS_USRL     0x16
#define ICM20689_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define ICM20689_RA_ZG_OFFS_USRL     0x18
#define ICM20689_RA_SMPLRT_DIV       0x19
#define ICM20689_RA_CONFIG           0x1A
#define ICM20689_RA_GYRO_CONFIG      0x1B
#define ICM20689_RA_ACCEL_CONFIG     0x1C
#define ICM20689_RA_ACCEL_CONFIG2    0x1D
#define ICM20689_RA_FF_DUR           0x1E
#define ICM20689_RA_MOT_THR          0x1F
#define ICM20689_RA_MOT_DUR          0x20
#define ICM20689_RA_ZRMOT_THR        0x21
#define ICM20689_RA_ZRMOT_DUR        0x22
#define ICM20689_RA_FIFO_EN          0x23
#define ICM20689_RA_I2C_MST_CTRL     0x24
#define ICM20689_RA_I2C_SLV0_ADDR    0x25
#define ICM20689_RA_I2C_SLV0_REG     0x26
#define ICM20689_RA_I2C_SLV0_CTRL    0x27
#define ICM20689_RA_I2C_SLV1_ADDR    0x28
#define ICM20689_RA_I2C_SLV1_REG     0x29
#define ICM20689_RA_I2C_SLV1_CTRL    0x2A
#define ICM20689_RA_I2C_SLV2_ADDR    0x2B
#define ICM20689_RA_I2C_SLV2_REG     0x2C
#define ICM20689_RA_I2C_SLV2_CTRL    0x2D
#define ICM20689_RA_I2C_SLV3_ADDR    0x2E
#define ICM20689_RA_I2C_SLV3_REG     0x2F
#define ICM20689_RA_I2C_SLV3_CTRL    0x30
#define ICM20689_RA_I2C_SLV4_ADDR    0x31
#define ICM20689_RA_I2C_SLV4_REG     0x32
#define ICM20689_RA_I2C_SLV4_DO      0x33
#define ICM20689_RA_I2C_SLV4_CTRL    0x34
#define ICM20689_RA_I2C_SLV4_DI      0x35
#define ICM20689_RA_I2C_MST_STATUS   0x36
#define ICM20689_RA_INT_PIN_CFG      0x37
#define ICM20689_RA_INT_ENABLE       0x38
#define ICM20689_RA_DMP_INT_STATUS   0x39
#define ICM20689_RA_INT_STATUS       0x3A
#define ICM20689_RA_ACCEL_XOUT_H     0x3B
#define ICM20689_RA_ACCEL_XOUT_L     0x3C
#define ICM20689_RA_ACCEL_YOUT_H     0x3D
#define ICM20689_RA_ACCEL_YOUT_L     0x3E
#define ICM20689_RA_ACCEL_ZOUT_H     0x3F
#define ICM20689_RA_ACCEL_ZOUT_L     0x40
#define ICM20689_RA_TEMP_OUT_H       0x41
#define ICM20689_RA_TEMP_OUT_L       0x42
#define ICM20689_RA_GYRO_XOUT_H      0x43
#define ICM20689_RA_GYRO_XOUT_L      0x44
#define ICM20689_RA_GYRO_YOUT_H      0x45
#define ICM20689_RA_GYRO_YOUT_L      0x46
#define ICM20689_RA_GYRO_ZOUT_H      0x47
#define ICM20689_RA_GYRO_ZOUT_L      0x48
#define ICM20689_RA_EXT_SENS_DATA_00 0x49
#define ICM20689_RA_MOT_DETECT_STATUS    0x61
#define ICM20689_RA_I2C_SLV0_DO      0x63
#define ICM20689_RA_I2C_SLV1_DO      0x64
#define ICM20689_RA_I2C_SLV2_DO      0x65
#define ICM20689_RA_I2C_SLV3_DO      0x66
#define ICM20689_RA_I2C_MST_DELAY_CTRL   0x67
#define ICM20689_RA_SIGNAL_PATH_RESET    0x68
#define ICM20689_RA_MOT_DETECT_CTRL      0x69
#define ICM20689_RA_USER_CTRL        0x6A
#define ICM20689_RA_PWR_MGMT_1       0x6B
#define ICM20689_RA_PWR_MGMT_2       0x6C
#define ICM20689_RA_BANK_SEL         0x6D
#define ICM20689_RA_MEM_START_ADDR   0x6E
#define ICM20689_RA_MEM_R_W          0x6F
#define ICM20689_RA_DMP_CFG_1        0x70
#define ICM20689_RA_DMP_CFG_2        0x71
#define ICM20689_RA_FIFO_COUNTH      0x72
#define ICM20689_RA_FIFO_COUNTL      0x73
#define ICM20689_RA_FIFO_R_W         0x74
#define ICM20689_RA_WHO_AM_I         0x75

#define ICM20689_SMPLRT_DIV      0       // 8000Hz

#define ICM20689_BIT_GYRO                    3
#define ICM20689_BIT_ACC                     2
#define ICM20689_BIT_TEMP                    1

#define ICM20689_LPF_256HZ       0
#define ICM20689_LPF_188HZ       1
#define ICM20689_LPF_98HZ        2
#define ICM20689_LPF_42HZ        3
#define ICM20689_LPF_20HZ        4
#define ICM20689_LPF_10HZ        5
#define ICM20689_LPF_5HZ         6

#define ICM20689_A_2mg                ((float)0.00006103f)  //g/LSB
#define ICM20689_A_4mg                ((float)0.00012207f)  //g/LSB
#define ICM20689_A_8mg                ((float)0.00024414f)  //g/LSB

#define ICM20689_G_s250dps            ((float)0.0076296f)  //dps/LSB
#define ICM20689_G_s500dps            ((float)0.0152592f)  //dps/LSB
#define ICM20689_G_s1000dps           ((float)0.0305185f)  //dps/LSB
#define ICM20689_G_s2000dps           ((float)0.0610370f)  //dps/LSB
#define ICM20689_WHO_AM_I_CONST		  (0X98)//WHO_AM_I±àÂë


uint8_t ICM_20689_Init(void);
void ICM_20689_Read_Data(void);
uint8_t ICM_20689_SpiDetect(void);
uint8_t ICM_20689_ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
uint8_t ICM_20689_WriteRegister(uint8_t reg,uint8_t data);


#endif








