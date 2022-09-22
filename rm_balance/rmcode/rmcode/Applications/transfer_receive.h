#ifndef __TRANSFER_RECEIVE_H
#define __TRANSFER_RECEIVE_H

#include "application.h"
#include "driver.h"
#include "Util.h"

#define TRANSFER_RECEIVE_PRIORITY 	7
#define TRANSFER_RECEIVE_PERIOD   	2
#define TRANSFER_RECEIVE_STACK_SIZE 512



typedef struct {
	TaskHandle_t 	xHandleTask;
	uint32_t 		loops;
} transferRecStruct_t;
typedef void transferTask_ptr(void);
void transferInit(void);
transferRecStruct_t *getTransferRecData(void);
#endif
