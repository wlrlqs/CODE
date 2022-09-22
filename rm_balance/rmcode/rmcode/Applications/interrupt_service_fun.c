#include "application.h"
#include "interrupt_service_fun.h"
#include "Driver_UserPrintf.h"


interruptUsartService_t usartInterruptFunc = {
	interruptServiceFunc_usart1,
	interruptServiceFunc_usart2,
	interruptServiceFunc_usart3,
	interruptServiceFunc_uart4,
	interruptServiceFunc_uart5,
	interruptServiceFunc_usart6,
	interruptServiceFunc_uart7,
	interruptServiceFunc_uart8,
};

interruptCanService_t canInterruptFunc = {
	interruptServiceFunc_can1,
	interruptServiceFunc_can2,
};
interruptUsartService_t * getUsartInterruptFunc(){
	return &usartInterruptFunc;
}
interruptCanService_t* getCanInterruptFunc(){
	return &canInterruptFunc ;
}

void interruptServiceFunc_usart1(uint8_t * array,uint16_t arrayLen){
	if(arrayLen){
		if(BOARD_TYPE == BOARD_CONTROL)
			imuUsartIspFunction(array,arrayLen);//接收数据部分
		else if(BOARD_TYPE == BOARD_CHASSIS){
			jointUsart1IspFunction(array,arrayLen);//接收数据部分
		}
	}
}

void interruptServiceFunc_usart2(uint8_t * array,uint16_t arrayLen){
	#if UAV_SBUS		
	if(arrayLen==25){
		digitalHi(&remoteControlData.rcIspReady);
		digitalIncreasing(&remoteControlData.rcError.errorCount);
	}
	#else
	if(arrayLen==18){
		digitalHi(&remoteControlData.rcIspReady);
 		digitalHi(&controlTransData.otherRcReadly);
		digitalIncreasing(&remoteControlData.rcError.errorCount.u32_temp);
	}
	#endif
}

void interruptServiceFunc_usart3(uint8_t * array,uint16_t arrayLen){
	if(arrayLen){
			//接收数据部分
	}
}

void interruptServiceFunc_uart4(uint8_t * array,uint16_t arrayLen){
	//裁判系统上电时会发送两次超过512个字节的数据
	if(arrayLen<256){
		uint8_t i;
		//把DMA数据存到环形buffer
		for(i=0;i<arrayLen;i++){
			bufferLoop.buffer[(u8)(bufferLoop.tail+i)] = array[i];
		}
		bufferLoop.tail += arrayLen;
		digitalIncreasing(&judgeData.judgeErrorCount);
	}
}

void interruptServiceFunc_uart5(uint8_t * array,uint16_t arrayLen){
	if(arrayLen){
			//接收数据部分
	}
}

void interruptServiceFunc_usart6(uint8_t * array,uint16_t arrayLen){
	if(arrayLen){
		if(BOARD_TYPE == BOARD_CONTROL){
			bufferLoopContactISP(array,arrayLen,&visionSerial.bufferLoop);
		}
		else if(BOARD_TYPE == BOARD_CHASSIS){
			jointUsart6IspFunction(array,arrayLen);//接收数据部分
		}
	}
}

void interruptServiceFunc_uart7(uint8_t * array,uint16_t arrayLen){
	if(arrayLen){
		if(BOARD_TYPE == BOARD_CONTROL){
			dataTrsfUsartIspFunction(array,arrayLen);
		}
		if(BOARD_TYPE == BOARD_CHASSIS){
			imuUsartIspFunction(array,arrayLen);//接收数据部分
		}
	}
}

void interruptServiceFunc_uart8(uint8_t * array,uint16_t arrayLen){
	if(arrayLen){
			//接收数据部分
	}
}


void interruptServiceFunc_can1(CanRxMsg *can_rx_msg){
	switch(BOARD_TYPE){
		//云台主控CAN1
		case BOARD_CONTROL:
			switch(robotConfigData.typeOfRobot){
				case INFANTRY_ID:
					infantry_can1_gimbal(can_rx_msg);
				break;
				case P_TANK_ID:
					p_tank_can1_gimbal(can_rx_msg);
					break;
				case AUXILIARY_ID:
                    auxiliary_can1_gimbal(can_rx_msg);
					break;
				case SENTRY_ID:
					sentry_can1_gimbal(can_rx_msg);
					break;
				case UAV_ID:
                    uav_can1_gimbal(can_rx_msg);
					break;
				case SMALLGIMBAL_ID:
					smallGimbal_can1_gimbal(can_rx_msg);
					break;
				case F_TANK_ID:
					f_tank_can1_gimbal(can_rx_msg);
					break;
			}
			break;
		//底盘从控CAN1
		case BOARD_CHASSIS:
			switch(robotConfigData.typeOfRobot){
				case INFANTRY_ID:
			   		infantry_can1_cahssis(can_rx_msg);
				break;
				case P_TANK_ID:
					p_tank_can1_cahssis(can_rx_msg);
					break;
				case AUXILIARY_ID:
                    auxiliary_can1_chassis(can_rx_msg);
					break;
				case SENTRY_ID:
					sentry_can1_cahssis(can_rx_msg);
					break;
				case UAV_ID:
					break;
				case SMALLGIMBAL_ID:
					break;
				case F_TANK_ID:
					f_tank_can1_chassis(can_rx_msg);
					break;
			}
			break;
	}
}

void interruptServiceFunc_can2(CanRxMsg *can_rx_msg){
	switch(BOARD_TYPE){
		//云台主控CAN2
		case BOARD_CONTROL:
			switch(robotConfigData.typeOfRobot){
				case INFANTRY_ID:
					infantry_can2_gimbal(can_rx_msg);
				break;
				case P_TANK_ID:
					p_tank_can2_gimbal(can_rx_msg);
					break;
				case AUXILIARY_ID:
					auxiliary_can2_gimbal(can_rx_msg);
					break;
				case SENTRY_ID:
					break;
				case UAV_ID:
					break;
				case SMALLGIMBAL_ID:
					break;
				case F_TANK_ID:
					f_tank_can2_gimbal(can_rx_msg);
					break;
			}
			break;
		//底盘从控CAN2
		case BOARD_CHASSIS:
			switch(robotConfigData.typeOfRobot){
				case INFANTRY_ID:
					infantry_can2_cahssis(can_rx_msg);
				break;
				case P_TANK_ID:
					p_tank_can2_cahssis(can_rx_msg);
					break;
				case AUXILIARY_ID:
                    auxiliary_can2_chassis(can_rx_msg);
					break;
				case SENTRY_ID:
					break;
				case UAV_ID:
					break;
				case SMALLGIMBAL_ID:
					break;
				case F_TANK_ID:
					f_tank_can2_chassis(can_rx_msg);
					break;
			}
			break;
	}	
}
