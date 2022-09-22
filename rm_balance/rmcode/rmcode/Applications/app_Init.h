#ifndef __APP_INIT_H
#define __APP_INIT_H

#include "FreeRTOS_board.h"

#define INIT_PRIORITIES 2
#define INIT_SIZE 512

#define USB_USART_PreemptionPriority        0X02
#define USB_USART_SubPriority               0X00
typedef struct {
	TaskHandle_t xHandleTask;	
	EventGroupHandle_t eventGroups;
}taskInit_t;

void appInit(void *Parameters);

extern taskInit_t taskInitData;
extern taskInit_t chassisRecodData;
extern taskInit_t chassisStartData;

void chassisRecod(void *Parameters);
void chassisStart(void *Parameters);
#endif 





