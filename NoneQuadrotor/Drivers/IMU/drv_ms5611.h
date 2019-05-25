#ifndef __DRV_MS5611__H__
#define __DRV_MS5611__H__
#include "sys.h"

#include "stdbool.h"



extern int32_t ms5611_pressure;

#define BARO_CAL_CNT 200          //TEMP=2000+dt*c6/2^23
#define MS5611_ADDR             0x77   //0xee //

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR							0x08	//CMD_ADC_4096




// registers of the device
#define MS5611_D1 0x40
#define MS5611_D2 0x50
#define MS5611_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS5611_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 0x00
#define MS5611_OSR_512 0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08
#define MS5611_OSR_DEFAULT MS5611_OSR_4096

#define MS5611_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS5611_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS5611_PROM_REG_SIZE 2 // size in bytes of a prom registry.

// Self test parameters. Only checks that values are sane
#define MS5611_ST_PRESS_MAX   (1100.0f) //mbar
#define MS5611_ST_PRESS_MIN   (450.0f)  //mbar
#define MS5611_ST_TEMP_MAX    (60.0f)   //degree celcius
#define MS5611_ST_TEMP_MIN    (-20.0f)  //degree celcius

// Constants used to determine altitude from pressure
#define CONST_SEA_PRESSURE 102610.f //1026.1f //http://www.meteo.physik.uni-muenchen.de/dokuwiki/doku.php?id=wetter:stadt:messung
#define CONST_PF 0.1902630958f //(1/5.25588f) Pressure factor
#define CONST_PF2 44330.0f


void MS5611_Init(void);
int MS5611_Update(void);
u8 MS5611_Read_Prom(void);
void MS5611_Reset(void);

void MS5611_Read_Adc_Temperature(void);
void MS5611_Read_Adc_Pressure(void);
void MS5611_Start_Temperature(void);
void MS5611_Start_Press(void);

void MS5611_BaroAltCalculate(void);
int32_t MS5611_Get_BaroAlt(void);
uint8_t MS5611_WriteRegister(uint8_t reg,uint8_t data);
uint8_t MS5611_ReadRegister(uint8_t reg, uint8_t *data,uint8_t length);

#endif

