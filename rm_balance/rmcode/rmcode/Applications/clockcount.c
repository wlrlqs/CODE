#include "clockcount.h"

clockCountStruct_t  clockCountData;

double getClockCount(void){
	clockCountData.clockTick  = (double)(clockCountData.saveTimer*10000 + TIM5->CNT)*1e-4f; 
	return clockCountData.clockTick;	
}
