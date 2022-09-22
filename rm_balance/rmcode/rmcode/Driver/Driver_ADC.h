#ifndef __DRIVER_ADC_H
#define __DRIVER_ADC_H
#include "BSP.h"
#include "driver.h"
#define ADC_1             0
#define ADC_2             1
#define ADC_3             2
#define ADC_4             3

#define SPEED_1             0
#define SPEED_2             1
#define SPEED_3             2
#define SPEED_4             3
extern __IO uint16_t adc_raw_value[10][4]; 
#endif
