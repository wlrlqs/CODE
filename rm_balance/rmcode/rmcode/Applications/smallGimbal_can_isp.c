#include "type_can_isp.h"

void smallGimbal_can1_gimbal(CanRxMsg *can_rx_msg){
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

void smallGimbal_can2_gimbal(CanRxMsg *can_rx_msg){
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
