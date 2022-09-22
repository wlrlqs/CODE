#include "type_can_isp.h"

void auxiliary_can1_gimbal(CanRxMsg *can_rx_msg){
    switch(can_rx_msg->StdId){
		case AUXILIARY_YAW:{
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_msg);
			break;
		}
		case AUXILIARY_PITCH:{
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_msg);
			break;
		}
		case ID_PNEUMATIC:{ 
			pneumatic_readData(can_rx_msg);
            break;
		}
		default:
			break;
	}
}

void auxiliary_can2_gimbal(CanRxMsg *can_rx_msg){
    switch(can_rx_msg->StdId){
		case AUXILIARY_LIFT_ELEVATOR:{
			motorSeverClass.Receive(&commandoMotorConfig[OUTFIT2],can_rx_msg);
			break;
		}
	    case AUXILIARY_RIGHT_ELEVATOR:{
			motorSeverClass.Receive(&commandoMotorConfig[OUTFIT3],can_rx_msg);
			break;
		}
	    case AUXILIARY_X_SLIDEWAY:{
			motorSeverClass.Receive(&commandoMotorConfig[OUTFIT4],can_rx_msg);
			break;
		}
	    case AUXILIARY_Y_SLIDEWAY:{
			motorSeverClass.Receive(&commandoMotorConfig[OUTFIT5],can_rx_msg);
			break;
		}
	    case AUXILIARY_FRONT_CLAW:{
			motorSeverClass.Receive(&commandoMotorConfig[OUTFIT1],can_rx_msg);
			break;
		}
		case AUXILIARY_LIFT_REVERSING:{
			motorSeverClass.Receive(&commandoMotorConfig[OUTFIT4],can_rx_msg);
			break;
		}
	    case AUXILIARY_RIGHT_REVERSING:{
			motorSeverClass.Receive(&commandoMotorConfig[OUTFIT5],can_rx_msg);
			break;
		}		
		case ID_PNEUMATIC:{ 
			pneumatic_readData(can_rx_msg);
            break;
		}		
		default:
			break;
	}
}

void auxiliary_can1_chassis(CanRxMsg *can_rx_msg){
    switch(can_rx_msg->StdId){ 
		case CHASSIS_RF: 
		case CHASSIS_LF: 
		case CHASSIS_LB: 
		case CHASSIS_RB:{ 
			static uint8_t i; 
			i = can_rx_msg->StdId - CHASSIS_RF; 
			motorSeverClass.Receive(&commandoMotorConfig[i+CHASSISMOTOR_RF],can_rx_msg);
			break;
		}
		default:
			break;
    }
}

void auxiliary_can2_chassis(CanRxMsg *can_rx_msg){
    switch(can_rx_msg->StdId){
        case ID_PNEUMATIC:{ 
			pneumatic_readData(can_rx_msg);
            break;
		}	
		default:
			break;
	}
}


