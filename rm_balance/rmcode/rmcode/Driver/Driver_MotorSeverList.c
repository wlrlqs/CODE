#include "Driver_MotorSeverList.h"
void infantryMainReceiveList(CanRxMsg *can_rx_message){
	switch(can_rx_message->StdId){ 
		case INFANTRY_CHASSIS_RF: 
		case INFANTRY_CHASSIS_LF: 
		case INFANTRY_CHASSIS_LB: 
		case INFANTRY_CHASSIS_RB:{ 
				static uint8_t i; 
				i = can_rx_message->StdId - INFANTRY_CHASSIS_RF; 
				motorSeverClass.Receive(&commandoMotorConfig[i+CHASSISMOTOR_RF],can_rx_message);
		}break; 
		case INFANTRY_YAW:{
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_message);
		}break;
		case INFANTRY_PITCH:{
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_message);
		}break;
		case INFANTRY_POKE:{
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_message);
		}break;
		case INFANTRY_SUPPLY:{
			motorSeverClass.Receive(&commandoMotorConfig[SUPPLYMOTOR],can_rx_message);
		}break;
		case INFANTRY_FRIC_L:{
			motorSeverClass.Receive(&commandoMotorConfig[FRICLMOTOR],can_rx_message);
		}break;
		case INFANTRY_FRIC_R:{
			motorSeverClass.Receive(&commandoMotorConfig[FRICRMOTOR],can_rx_message);
		}break;
	}
}

void infantrySlaveReceiveList(CanRxMsg *can_rx_message){
	switch(can_rx_message->StdId){ 
		case INFANTRY_CHASSIS_RF: 
		case INFANTRY_CHASSIS_LF: 
		case INFANTRY_CHASSIS_LB: 
		case INFANTRY_CHASSIS_RB:{ 
				static uint8_t i; 
				i = can_rx_message->StdId - INFANTRY_CHASSIS_RF; 
				motorSeverClass.Receive(&commandoMotorConfig[i+CHASSISMOTOR_RF],can_rx_message);
		}break; 
		case INFANTRY_YAW:{
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_message);
		}break;
		case INFANTRY_PITCH:{
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_message);
		}break;
		case INFANTRY_POKE:{
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_message);
		}break;
		case INFANTRY_SUPPLY:{
			motorSeverClass.Receive(&commandoMotorConfig[SUPPLYMOTOR],can_rx_message);
		}break;
		case INFANTRY_SYNCHRONOUS:{
			motorSeverClass.Receive(&commandoMotorConfig[SYNCHRONOUS_WHEEL],can_rx_message);
		}break;
		case INFANTRY_ARM:{
			motorSeverClass.Receive(&commandoMotorConfig[ARM_WHEEL],can_rx_message);
		}break;
	}
}

void infantryCanReceiveSpector(CanRxMsg *can_rx_message,robotConfigStruct_t *robotData){
	switch(robotData->boardType){
		case BOARD_CHASSIS:
			infantrySlaveReceiveList(can_rx_message);
		break;
		case BOARD_GIMBAL:
			infantryMainReceiveList(can_rx_message);
		break;
	}
}







