#ifndef __DRIVER_USERPRINTF_H
#define __DRIVER_USERPRINTF_H

#include "Driver_Slave_Sensor.h"

typedef struct {
	uint8_t header;
	uint8_t	tail;
	uint8_t buffer[255];
}dataTrsfLoopStruct_t;

typedef struct {
	uint8_t length;
	dataTrsfLoopStruct_t bufferLoop;
}dataTrsfBroadcastStruct_t; 

extern dataTrsfBroadcastStruct_t dataTrsfBroadcast;

void dataTrsfUsartIspFunction(uint8_t *array, uint16_t len);
void dataTrsfReceive(void);
void userPrintf(const char* format, ...);

#endif
