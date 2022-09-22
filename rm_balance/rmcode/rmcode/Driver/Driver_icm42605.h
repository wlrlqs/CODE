#ifndef __DRIVER_ICM42605_H
#define __DRIVER_ICM42605_H

#include "BSP.h"
#include "regmap_icm42605.h"
#include "std_lib.h"
#include "util.h"

#define ICM42605_SPI_MIN_PRESCALER SPI_BaudRatePrescaler_16
#define ICM42605_SPI_DATA_SIZE SPI_DataSize_8b
#define	ICM42605_CS PAout(15)

#define ICM42605_SPI_DEFAULT \
{ \
	.SPIx = SPI1, \
	.SPI_NSS = BSP_GPIOA15, \
	.SPI_SCK = BSP_GPIOB3, \
	.SPI_MISO = BSP_GPIOA6, \
	.SPI_MOSI = BSP_GPIOA7, \
}

typedef struct{
	formatTrans16Struct_t errorFlag;
	formatTrans16Struct_t gyroscope[3];
	formatTrans16Struct_t acceleration[3];
	formatTrans16Struct_t temperature;
	formatTrans16Struct_t dataCNTR;
    float gyroScale;
    float accelScale;
    uint8_t *gyroFSR;
    uint8_t *accelFSR;    
    uint8_t lastGyroFSR;
    uint8_t lastAccelFSR;
    uint8_t whoIAm;
    
	float time[2];
	float executionTime;
} icm42605Struct_t;

float *getIcm42605GyroScale(void);
float *getIcm42605AccelScale(void);
int16_t getIcm42605Gyroscope(uint8_t index);
int16_t getIcm42605Acceleration(uint8_t index);
int16_t getIcm42605Temperature(void);
uint16_t brustReadIcm42605(void);
bool driver_Icm42605_Init(uint8_t *gyroFSR, uint8_t *accelFSR);
void imu_data_read(void);
#endif
