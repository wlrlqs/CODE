#include "type_can_isp.h"

void sentry_can1_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		case SENTRY_YAW:
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_msg);
			break;
		case SENTRY_PITCH:
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_msg);
			break;
		case SENTRY_POKE:
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_msg);
			break;
		case SENTRY_FRIC_L:
			motorSeverClass.Receive(&commandoMotorConfig[FRICLMOTOR],can_rx_msg);
			break;
		case SENTRY_FRIC_R:
			motorSeverClass.Receive(&commandoMotorConfig[FRICRMOTOR],can_rx_msg);
			break;
		case SENTRY_DIRECT:
			motorSeverClass.Receive(&commandoMotorConfig[STRY_MOVE_DIRECT],can_rx_msg);
			break;
		default:
			break;
	}
}

void sentry_can2_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		case SENTRY_YAW:
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_msg);
			break;
		case SENTRY_PITCH:
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_msg);
			break;
		case SENTRY_POKE:
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_msg);
			break;
		case SENTRY_FRIC_L:
			motorSeverClass.Receive(&commandoMotorConfig[FRICLMOTOR],can_rx_msg);
			break;
		case SENTRY_FRIC_R:
			motorSeverClass.Receive(&commandoMotorConfig[FRICRMOTOR],can_rx_msg);
			break;
		default:
			break;
	}
}

void sentry_can1_cahssis(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){ 
		case CHASSIS_RF: 
		case CHASSIS_LF: 
		case CHASSIS_LB: 
		case CHASSIS_RB:{ 
				static uint8_t i; 
				i = can_rx_msg->StdId - CHASSIS_RF; 
				motorSeverClass.Receive(&commandoMotorConfig[i+CHASSISMOTOR_RF],can_rx_msg);
			}break;
		default:
			break;
	}
}

void sentry_can2_cahssis(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){ 
		case CHASSIS_RF: 
		case CHASSIS_LF: 
		case CHASSIS_LB: 
		case CHASSIS_RB:{ 
				static uint8_t i; 
				i = can_rx_msg->StdId - CHASSIS_RF; 
				motorSeverClass.Receive(&commandoMotorConfig[i+CHASSISMOTOR_RF],can_rx_msg);
			}break;
		default:
			break;
	}
}
