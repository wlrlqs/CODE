#include "Driver_unitreeMotor.h"
#include "joint.h"
#include "BSP.h"
#include <string.h>
#include <math.h>
//#include "slave_sensor.h"
//#include "supervisor.h"

jointBroadcastStruct_t jointBroadcastLF;
jointBroadcastStruct_t jointBroadcastLB;
jointBroadcastStruct_t jointBroadcastRF;
jointBroadcastStruct_t jointBroadcastRB;

//放在串口中断中的接收函数
void jointUsart1IspFunction(uint8_t *array, uint16_t len)  {
	if(len == 78){
		if(array[2] == 0x01){
			for(uint8_t i = 0 ; i < len; i++){
				jointBroadcastLF.bufferLoop.buffer[i] = array[i];
			}
		}else if(array[2] == 0x02){
			for(uint8_t i = 0 ; i < len; i++){
				jointBroadcastLB.bufferLoop.buffer[i] = array[i];
			}
		}
	}
}

void jointUsart6IspFunction(uint8_t *array, uint16_t len)  {
	if(len == 78){
		if(array[2] == 0x01){
			for(uint8_t i = 0 ; i < len; i++){
				jointBroadcastRF.bufferLoop.buffer[i] = array[i];
			}
		}else if(array[2] == 0x02){
			for(uint8_t i = 0 ; i < len; i++){
				jointBroadcastRB.bufferLoop.buffer[i] = array[i];
			}
		}
	}
}



