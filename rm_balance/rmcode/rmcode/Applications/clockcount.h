#ifndef __CLOCKCOUNT_H
#define __CLOCKCOUNT_H

#include "Driver_ClockCount.h"
#include "FreeRTOS.h"

//#define TIME_US  

typedef struct{
	uint32_t lastClockTick;
	uint32_t saveTimer;
	double clockTick;
	uint8_t  index;
} clockCountStruct_t;

extern clockCountStruct_t  clockCountData;

double getClockCount(void);

#endif
