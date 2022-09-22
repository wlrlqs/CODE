#include "type_can_isp.h"

void uav_can1_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
		case UAV_YAW: {
			motorSeverClass.Receive(&commandoMotorConfig[YAWMOTOR],can_rx_msg);
        }break;
        
		case UAV_PITCH: {
			motorSeverClass.Receive(&commandoMotorConfig[PITCHMOTOR],can_rx_msg);
        }break;
        
        case UAV_ROLL:  {
            motorSeverClass.Receive( &commandoMotorConfig[ROLLMOTOR],can_rx_msg );
        }break;
            
		case UAV_POKE: {
			motorSeverClass.Receive(&commandoMotorConfig[POKEMOTOR],can_rx_msg);
        }break;
        
		case UAV_FRIC_L:   {
			motorSeverClass.Receive(&commandoMotorConfig[FRICLMOTOR],can_rx_msg);
//			  usbVCP_Printf("%d",&commandoMotorConfig[FRICLMOTOR]);
			     
        }break;
        
		case UAV_FRIC_R:   {
			motorSeverClass.Receive(&commandoMotorConfig[FRICRMOTOR],can_rx_msg);
        }break;
  
		default:    break;
	}
}

void uav_can2_gimbal(CanRxMsg *can_rx_msg){
	switch(can_rx_msg->StdId){
        
        default:    break;
	}
}

