#ifndef __BMP180_H
#define __BMP180_H
#include "stm32f4xx_hal.h"
#include "main.h"
#define BMP180_ADDR (0x77 << 1)
#define OVER_SAMPLING_REGISTER 0xF4
#define AC1_MSB_ADRESS 0xAA



void BMP_I2CScanner(void);
void BMP_Calculate(void);
void BMP_Config(uint8_t);
void BMP_ReadTemp(void);
void BMP_ReadPressure(void);
float Read_Temperature(void);
float BMP_GetAltitude(void);
void ReadAll(void);
float LowPassFilter(float, float, float);
typedef enum{
	temp_oss = 0x2E,//Temperature oversampling
	oss0 = 0x34,//Low Power oversampling
	oss1 = 0x74,//Standart oversampling
	oss2 = 0xB4,//High Power oversampling
	oss3 = 0xF4 //Ultra High Power oversampling
}BMP_os_value;
typedef enum{
	oss0_delay = 5,
	oss1_delay = 8,
	oss2_delay = 14,
	oss3_delay = 26
}oss_delay;
#endif
