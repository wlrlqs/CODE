#include "type_can_isp.h"
/*新英雄CAN中断接收*/
void p_tank_can1_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		case P_TANK_YAW:
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_msg);
			break;

		case P_TANK_BIG_POKE:
			motorSeverClass.Receive(&commandoMotorConfig[SLAVE_POKEMOTOR],can_rx_msg);
			break;
		case P_TANK_BIG_SUPPLY:
			motorSeverClass.Receive(&commandoMotorConfig[SLAVE_SUPPLY],can_rx_msg);
		break;
		case ID_PNEUMATIC:{ 
			pneumatic_readData(can_rx_msg);
            break;
		}
        case ID_CAPACITOR:
			capacitor_readData(can_rx_msg);    //超级电容接受信息
			break;
		default:
			break;
	}
}

void p_tank_can2_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		case P_TANK_PITCH:
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_msg);
			break;
		case P_TANK_FRIC_L:
			motorSeverClass.Receive(&commandoMotorConfig[HERO_FRICLMOTOR],can_rx_msg);
			break;
		case P_TANK_FRIC_R:
			motorSeverClass.Receive(&commandoMotorConfig[HERO_FRICRMOTOR],can_rx_msg);
			break;
		case P_TANK_SMALL_POKE:
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_msg);
			break;
		case P_TANK_SMALL_SUPPLY:
			motorSeverClass.Receive(&commandoMotorConfig[SUPPLYMOTOR],can_rx_msg);
			break;
        
		default:
			break;
	}
}

void p_tank_can1_cahssis(CanRxMsg *can_rx_msg){
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

void p_tank_can2_cahssis(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		default:
			break;
	}
}
/*老英雄CAN中断接收*/
void f_tank_can1_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		case F_TANK_YAW:
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_msg);
			break;
		case F_TANK_PITCH:
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_msg);
			break;
		case F_TANK_POKE:
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_msg);
			break;
		case F_TANK_SUPPLY:
			motorSeverClass.Receive(&commandoMotorConfig[SUPPLYMOTOR],can_rx_msg);
		break;
		case F_TANK_FRIC_L:
			motorSeverClass.Receive(&commandoMotorConfig[FRICLMOTOR],can_rx_msg);
			break;
		case F_TANK_FRIC_R:
			motorSeverClass.Receive(&commandoMotorConfig[FRICRMOTOR],can_rx_msg);
			break;
        case ID_CAPACITOR:
			capacitor_readData(can_rx_msg);    //超级电容接受信息
			break;
		default:
			break;
	}
}

void f_tank_can2_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		case F_TANK_YAW:
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_msg);
			break;
		case F_TANK_PITCH:
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_msg);
			break;
		case F_TANK_POKE:
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_msg);
			break;
		case F_TANK_SUPPLY:
			motorSeverClass.Receive(&commandoMotorConfig[SUPPLYMOTOR],can_rx_msg);
		break;
		case F_TANK_FRIC_L:
			motorSeverClass.Receive(&commandoMotorConfig[FRICLMOTOR],can_rx_msg);
			break;
		case F_TANK_FRIC_R:
			motorSeverClass.Receive(&commandoMotorConfig[FRICRMOTOR],can_rx_msg);
			break;
		default:
			break;
	}
}

void f_tank_can1_chassis(CanRxMsg *can_rx_msg){
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

void f_tank_can2_chassis(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){ 
		case CHASSIS_RF: 
		case CHASSIS_LF: 
		case CHASSIS_LB: 
		case CHASSIS_RB:{ 
				static uint8_t i; 
				i = can_rx_msg->StdId - CHASSIS_RF; 
				motorSeverClass.Receive(&commandoMotorConfig[i+CHASSISMOTOR_RF],can_rx_msg);
			}break;
		case F_TANK_ARM:
			motorSeverClass.Receive(&commandoMotorConfig[ARM_WHEEL],can_rx_msg);
			break;
		case F_TANK_SYNCHRONOUS:
			motorSeverClass.Receive(&commandoMotorConfig[SYNCHRONOUS_WHEEL],can_rx_msg);
			break;
    case ID_CAPACITOR:
			capacitor_readData(can_rx_msg);
			break;
		default:
			break;
	}
}
