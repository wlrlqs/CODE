#ifndef __BUFFERLOOP_MOD_H
#define	__BUFFERLOOP_MOD_H

#include "stm32f4xx.h"
#include "util.h"
#include "Driver_imuSC.h"

typedef void (*decodeFunction) (uint8_t *buffDecode);
void bufferLoopMod(uint8_t bufferBegin,uint8_t headerLength,uint8_t lengthPosition,
					buffefLoopStruct_t *bufferLoop,decodeFunction decodeTask);
void bufferLoopContactISP(uint8_t *array,uint16_t arrayLength,buffefLoopStruct_t *bufferLoop);

#endif
