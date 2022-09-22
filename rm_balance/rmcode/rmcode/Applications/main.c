#include "board.h"

volatile uint32_t ulHighFrequencyTimerTicks = 0UL;




int main(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	xTaskCreate(appInit,"Init",INIT_SIZE,NULL,INIT_PRIORITIES, &taskInitData.xHandleTask);
	vTaskStartScheduler();
	usbVCP_Printf("Task run failed.");
	return 0;
}
