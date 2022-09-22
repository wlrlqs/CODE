#include "type_can_isp.h"
#include "motorHandle.h"
#include "shoot.h"
#include "chassis.h"

shootStruct_t shootData;
fankui fankuiData[3];

void infantry_can1_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		case INFANTRY_YAW:
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_msg);
			break;
		case INFANTRY_PITCH:
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_msg);
			break;
		case INFANTRY_POKE:
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_msg);
			break;
		case INFANTRY_SUPPLY:
			motorSeverClass.Receive(&commandoMotorConfig[SUPPLYMOTOR],can_rx_msg);
		break;
		case INFANTRY_FRIC_L:
			motorSeverClass.Receive(&commandoMotorConfig[FRICLMOTOR],can_rx_msg);
//		 usbVCP_Printf("FRICL speed = %d \t\n", motorHandleClass.Speed(&commandoMotorConfig[FRICLMOTOR]));
//		usbVCP_Printf("FRICR speed = %d \t\n", motorHandleClass.Speed(&commandoMotorConfig[FRICRMOTOR]));
//		usbVCP_Printf("shootSpeedLast = %f \t\n",shootData.shootSpeedLast);
//		usbVCP_Printf("FRICL pid Ref = %d \t\n",shootData.fricWheelSpeedRef[0]);
//		usbVCP_Printf("FRICL pid out = %d \t\n",shootData.fricWheelSpeedOut[0]);
		
//		usbVCP_Printf("FRICR pid Ref = %d \t\n",shootData.fricWheelSpeedRef[1]);
//		usbVCP_Printf("FRICR pid out = %d \t\n",shootData.fricWheelSpeedOut[1]);
//		usbVCP_Printf("bullet_speed = %f \t\n",judgeData.extShootData.bullet_speed);
			break;
		case INFANTRY_FRIC_R:
			motorSeverClass.Receive(&commandoMotorConfig[FRICRMOTOR],can_rx_msg);
		 
			break;
	    case ID_CAPACITOR:
			capacitor_readData(can_rx_msg);    //超级电容接受信息
			break;
		default:
			break;
	}
}

void infantry_can2_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		case INFANTRY_YAW:
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_msg);
			break;
		case INFANTRY_PITCH:
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_msg);
			break;
		case INFANTRY_POKE:
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_msg);
			break;
		case INFANTRY_SUPPLY:
			motorSeverClass.Receive(&commandoMotorConfig[SUPPLYMOTOR],can_rx_msg);
		break;
		case INFANTRY_FRIC_L:
			motorSeverClass.Receive(&commandoMotorConfig[FRICLMOTOR],can_rx_msg);
			break;
		case INFANTRY_FRIC_R:
			motorSeverClass.Receive(&commandoMotorConfig[FRICRMOTOR],can_rx_msg);
			break;	 
		case ID_CAPACITOR:
			capacitor_readData(can_rx_msg);    //超级电容接受信息
			break;
		default:
			break;
	}
}

void infantry_can1_cahssis(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){ 
		case CHASSIS_RF: 
		case CHASSIS_LF: 
		case CHASSIS_LB: 
		case CHASSIS_RB:{ 
				static uint8_t i; 
				i = can_rx_msg->StdId - CHASSIS_RF; 
				motorSeverClass.Receive(&commandoMotorConfig[i+CHASSISMOTOR_RF],can_rx_msg);
			}break;
		case INFANTRY_SYNCHRONOUS:
			motorSeverClass.Receive(&commandoMotorConfig[11],can_rx_msg);
			break;
		case INFANTRY_ARM:
			motorSeverClass.Receive(&commandoMotorConfig[12],can_rx_msg);
			break;
		default:
			break;
	}
}

void infantry_can2_cahssis(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){ 
		case CHASSIS_RF: 
		case CHASSIS_LF: 
		case CHASSIS_LB: 
		case CHASSIS_RB:{ 
				static uint8_t i; 
				i = can_rx_msg->StdId - CHASSIS_RF; 
				motorSeverClass.Receive(&commandoMotorConfig[i+CHASSISMOTOR_RF],can_rx_msg);
			}break;
		case INFANTRY_ARM:
			motorSeverClass.Receive(&commandoMotorConfig[ARM_WHEEL],can_rx_msg);
			break;
		case INFANTRY_SYNCHRONOUS:
			motorSeverClass.Receive(&commandoMotorConfig[SYNCHRONOUS_WHEEL],can_rx_msg);
			break;
		default:
			break;
	}
	
}

