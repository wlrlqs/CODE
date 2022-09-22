#ifndef __DRIVER_UNITREEMOTOR_H
#define __DRIVER_UNITREEMOTOR_H

#include "bsp_usart.h"
#include "stm32f4xx.h"
#include "util.h"
#include "stdbool.h"
#include "joint.h"

typedef struct {
	uint8_t header;
	uint8_t	tail;
	uint8_t buffer[255];
}bufferLoopStruct_t;

typedef struct {
	uint8_t usartX;
	bufferLoopStruct_t bufferLoop;
}jointBroadcastStruct_t; 

extern jointSerialStruct_t jointSerial;
extern jointBroadcastStruct_t jointBroadcastLF;
extern jointBroadcastStruct_t jointBroadcastLB;
extern jointBroadcastStruct_t jointBroadcastRF;
extern jointBroadcastStruct_t jointBroadcastRB;

void jointUsart1IspFunction(uint8_t *array, uint16_t len);
void jointUsart6IspFunction(uint8_t *array, uint16_t len);

#endif


