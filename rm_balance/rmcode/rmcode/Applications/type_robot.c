#include "Driver_USBVCP.h"
#include "type_robot.h"
#include "config.h"
#include "supervisor.h"
#include "Util.h"
#include "rc.h"
#include "Driver_Beep.h"
#include "control.h"
#include "tf_card_parameter.h"
#include "local_id.h"
#include "power.h"
#include "canfd.h"
/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

robotConfigStruct_t robotConfigData; 
/****************����˵��******************
    "YAW_INSTALL"       ----> yaw��װ����
	"PITCH_INSTALL"     ----> pitch��װ����
	"ROLL_INSTALL"      ----> roll��װ����
    "YAW_TYPE"          ----> yaw��������
	"PITCH_TYPE"        ----> pitch��������
	"ROLL_TYPE"         ----> roll��������
    "YAW_FIX"           ----> yaw�ᰲװ����
	"YAW_TURN"          ----> yaw����ת����
	"PITCH_FIX"         ----> pitch�ᰲװ����
	"PITCH_TURN"        ----> pitch����ת����
	"ROLL_FIX"          ----> roll�ᰲװ����
	"ROLL_TURN"         ----> roll����ת����
    "CHASSIS_CURRENT"   ----> ��������������
    "PITCH_MIN_RANGE"   ----> pitch����󸩽�
    "PITCH_MAX_RANGE"   ----> pitch���������
*******************************************/
void currentRobotParameterConfig(void){
	switch(robotConfigData.typeOfRobot){
		//����ID
		case INFANTRY_ID:{							
			getConfigData()->weaponType = SMALL_LAUNCHER;											
			getConfigData()->chassisCurrent= INFANTR_CHASSIS_CURRENT;					
			getConfigData()->pitchMinRange = INFANTRY_PITCH_MIN_RANGE;					
			getConfigData()->pitchMaxRange = INFANTRY_PITCH_MAX_RANGE;					
			PITCH_INSTALL_CONFIG = 1;		
			YAW_INSTALL_CONFIG = 1;  
//			PITCH_INSTALL_CONFIG = PITCH_FIX_CONFIG + PITCH_TURN_CONFIG;		
//  		YAW_INSTALL_CONFIG = YAW_FIX_CONFIG + YAW_TURN_CONFIG; 
			pitchMotorData.motorID = PITCH_TYPE_CONFIG;    
			yawMotorData.motorID = YAW_TYPE_CONFIG;        
			//�����������Ʋ�������
			getpowerData() ->powerLimit = INFANTR_POWER_LIMIT;
			getpowerData() ->warningPower = INFANTR_WARNING_POWER;
			getpowerData() ->WarningPowerBuff = INFANTR_WARNING_POWER_BUFF;
			getpowerData() ->noJudgeTotalCurrentLimit = INFANTR_NO_JUDGE_TOTAL_CURRENT_LIMIT;
			getpowerData() ->judgeTotalCurrentLimit = INFANTR_JUDGE_TOTAL_CURRENT_LIMIT;
			getpowerData() ->addPowerCurrent = INFANTR_ADD_POWER_CURRENT;
			robotConfigData.robotDeviceList = INFANTRY_DEVICE_LIST;
			robotConfigData.outfitNum = 3;
		}break;
        //��Ӣ�۳�ID
		case P_TANK_ID:{								
			getConfigData()->weaponType = DOUBLE_LAUNCHER;
			getConfigData()->chassisCurrent = TANK_CHASSIS_CURRENT;	
			getConfigData()->pitchMinRange = TANK_PITCH_MIN_RANGE;
			getConfigData()->pitchMaxRange = TANK_PITCH_MAX_RANGE;	
			PITCH_INSTALL_CONFIG = 2;		
			YAW_INSTALL_CONFIG = 2;  
//			PITCH_INSTALL_CONFIG = PITCH_FIX_CONFIG + PITCH_TURN_CONFIG;		
//			YAW_INSTALL_CONFIG = YAW_FIX_CONFIG + YAW_TURN_CONFIG; 
			pitchMotorData.motorID = PITCH_TYPE_CONFIG;    
			yawMotorData.motorID = YAW_TYPE_CONFIG;      
			//Ӣ�۹������Ʋ�������
			getpowerData() ->powerLimit = TANK_POWER_LIMIT;
			getpowerData() ->warningPower = TANK_WARNING_POWER;
			getpowerData() ->WarningPowerBuff = TANK_WARNING_POWER_BUFF;
			getpowerData() ->noJudgeTotalCurrentLimit = TANK_NO_JUDGE_TOTAL_CURRENT_LIMIT;
			getpowerData() ->judgeTotalCurrentLimit = TANK_JUDGE_TOTAL_CURRENT_LIMIT;
			getpowerData() ->addPowerCurrent = TANK_ADD_POWER_CURRENT;
			robotConfigData.robotDeviceList = P_TANK_DEVICE_LIST;
			robotConfigData.outfitNum = 3;
		}break;
		 //��Ӣ�۳�ID
		case F_TANK_ID:{								
			getConfigData()->weaponType = DOUBLE_LAUNCHER;
			getConfigData()->chassisCurrent = TANK_CHASSIS_CURRENT;         	
			getConfigData()->pitchMinRange = F_TANK_PITCH_MIN_RANGE;
			getConfigData()->pitchMaxRange = F_TANK_PITCH_MAX_RANGE;	
			PITCH_INSTALL_CONFIG = 2;		
			YAW_INSTALL_CONFIG = 2;  
//			PITCH_INSTALL_CONFIG = PITCH_FIX_CONFIG + PITCH_TURN_CONFIG;		
//			YAW_INSTALL_CONFIG = YAW_FIX_CONFIG + YAW_TURN_CONFIG; 
			pitchMotorData.motorID = PITCH_TYPE_CONFIG;    
			yawMotorData.motorID = YAW_TYPE_CONFIG;      
			//Ӣ�۹������Ʋ�������
			getpowerData() ->powerLimit = TANK_POWER_LIMIT;
			getpowerData() ->warningPower = TANK_WARNING_POWER;
			getpowerData() ->WarningPowerBuff = TANK_WARNING_POWER_BUFF;
			getpowerData() ->noJudgeTotalCurrentLimit = TANK_NO_JUDGE_TOTAL_CURRENT_LIMIT;
			getpowerData() ->judgeTotalCurrentLimit = TANK_JUDGE_TOTAL_CURRENT_LIMIT;
			getpowerData() ->addPowerCurrent = TANK_ADD_POWER_CURRENT;
			robotConfigData.robotDeviceList = F_TANK_DEVICE_LIST;
			robotConfigData.outfitNum = 3;
		}break;
		//���̳�ID
		case AUXILIARY_ID:{							
			getConfigData()->weaponType = SMALL_LAUNCHER;
			getConfigData()->chassisCurrent = CHASSIS_NO_LIMIT;	
			getConfigData()->pitchMinRange = AUXILIARY_PITCH_MIN_RANGE;
			getConfigData()->pitchMaxRange = AUXILIARY_PITCH_MAX_RANGE;		
			PITCH_INSTALL_CONFIG = PITCH_FIX_CONFIG + PITCH_TURN_CONFIG;		
			YAW_INSTALL_CONFIG = YAW_FIX_CONFIG + YAW_TURN_CONFIG; 
			pitchMotorData.motorID = PITCH_TYPE_CONFIG;    
			yawMotorData.motorID = YAW_TYPE_CONFIG;           
			robotConfigData.robotDeviceList = AUXILIARY_DEVICE_LIST;
			robotConfigData.outfitNum = 3;
		}break;
		//�ڱ�ID
		case SENTRY_ID:{								
			getConfigData()->weaponType = SMALL_LAUNCHER;
			getConfigData()->chassisCurrent = SENTRY_CHASSIS_CURRENT;	
			getConfigData()->pitchMinRange = SENTRY_PITCH_MIN_RANGE;
			getConfigData()->pitchMaxRange = SENTRY_PITCH_MAX_RANGE;	
//			PITCH_INSTALL_CONFIG = PITCH_FIX_CONFIG + PITCH_TURN_CONFIG;		
//			YAW_INSTALL_CONFIG = YAW_FIX_CONFIG + YAW_TURN_CONFIG; 
			PITCH_INSTALL_CONFIG = 2;		
			YAW_INSTALL_CONFIG = 1;  
			pitchMotorData.motorID = PITCH_TYPE_CONFIG;    
			yawMotorData.motorID = YAW_TYPE_CONFIG;     
			//�ڱ��������Ʋ�������    
			getpowerData() ->powerLimit = SENTRY_POWER_LIMIT;
			getpowerData() ->warningPower = SENTRY_WARNING_POWER;
			getpowerData() ->WarningPowerBuff = SENTRY_WARNING_POWER_BUFF;
			getpowerData() ->noJudgeTotalCurrentLimit = SENTRY_NO_JUDGE_TOTAL_CURRENT_LIMIT;
			getpowerData() ->judgeTotalCurrentLimit = SENTRY_JUDGE_TOTAL_CURRENT_LIMIT;
			getpowerData() ->addPowerCurrent = SENTRY_ADD_POWER_CURRENT;
			robotConfigData.robotDeviceList = SENTRY_DEVICE_LIST;
			robotConfigData.outfitNum = 1;
		}break;
		//���˻�ID
		case UAV_ID:{										
			getConfigData()->weaponType = SMALL_LAUNCHER;
			getConfigData()->chassisCurrent = CHASSIS_NO_LIMIT;	
			getConfigData()->pitchMinRange = UAV_PITCH_MIN_RANGE;
			getConfigData()->pitchMaxRange = UAV_PITCH_MAX_RANGE;	
			PITCH_INSTALL_CONFIG = PITCH_FIX_CONFIG + PITCH_TURN_CONFIG;		
			YAW_INSTALL_CONFIG = YAW_FIX_CONFIG + YAW_TURN_CONFIG; 
			pitchMotorData.motorID = PITCH_TYPE_CONFIG;    
			yawMotorData.motorID = YAW_TYPE_CONFIG;      
			robotConfigData.robotDeviceList = UAV_DEVICE_LIST;
			robotConfigData.outfitNum = 0;
		}break;
		//С��̨ID
		case SMALLGIMBAL_ID:{										
			getConfigData()->weaponType = DOUBLE_LAUNCHER;
			getConfigData()->chassisCurrent = CHASSIS_NO_LIMIT;
			getConfigData()->pitchMinRange = SMALLGIMBAL_PITCH_MIN_RANGE;
			getConfigData()->pitchMaxRange  = SMALLGIMBAL_PITCH_MAX_RANGE;	
//			PITCH_INSTALL_CONFIG = PITCH_FIX_CONFIG + PITCH_TURN_CONFIG;		
//			YAW_INSTALL_CONFIG = YAW_FIX_CONFIG + YAW_TURN_CONFIG; 
			PITCH_INSTALL_CONFIG = 3;		
			YAW_INSTALL_CONFIG = 2; 
			pitchMotorData.motorID = PITCH_TYPE_CONFIG;    
			yawMotorData.motorID = YAW_TYPE_CONFIG;   
			robotConfigData.robotDeviceList = SMALLGIMBAL_LIST;
			robotConfigData.outfitNum = 0;
		}break;
		
 		//LCM:  ���ڷ����ID
        case MISSILE_ID : {
 			    if(getConfigData()->configVersion < DEFAULT_CONFIG_VERSION){
					getConfigData()->weaponType = NO_WEAPONS;											
					getConfigData()->chassisCurrent= 0;					
					getConfigData()->pitchMinRange = 0;					
					getConfigData()->pitchMaxRange = 0;					
					PITCH_INSTALL_CONFIG = 0; 
					YAW_INSTALL_CONFIG = 0;
            }
			robotConfigData.robotDeviceList = MISSILE_DEVICE_LIST;       
        }break;       
        
		default: break;
	}
    //��TF���ж�ȡ��Ӧ����
	parameterReadDataFromTFCard(robotConfigData.typeOfRobot);							
}

//static void identifyTypeRobotBeepUpdate(uint8_t typeRobot){
//	switch(typeRobot){
//		case INFANTRY_ID: supervisorData.beepState = MUSIC_TYPE_INFANTRY; break;
//		case P_TANK_ID:	supervisorData.beepState = MUSIC_TYPE_TANK; break;
//		case AUXILIARY_ID:	supervisorData.beepState = MUSIC_TYPE_AUXILIARY; break;
//		case SENTRY_ID:	supervisorData.beepState = MUSIC_TYPE_SENTRY; break;
//		case UAV_ID: 	supervisorData.beepState = MUSIC_TYPE_UAV; break;
//		case SMALLGIMBAL_ID: 	supervisorData.beepState = MUSIC_TYPE_SMALLGIMBAL; break;
//		
//	}
//}
void robotBoardConfig(){
    //LCM: 
    if(getRobotType() > NO_ID && getRobotType() < MISSILE_ID){
        getConfigData()->robotType = getRobotType();
        getConfigData()->boardType = BOARD_CONTROL;
    }
    else if( getRobotType() != NO_ID && getRobotType() < COUNT_ID ) {
        //LCM: ���ڷ����
        getConfigData()->robotType = getRobotType();
        getConfigData()->boardType = BOARD_OTHER;        
    }
    else {
        getConfigData()->boardType = BOARD_CHASSIS;
    }
        robotConfigData.typeOfRobot	= getConfigData()->robotType;
        BOARD_TYPE = getConfigData()->boardType;
}
void robotDistinguish(void){
	digitalLo(&getcontrolData()->dataInitFlag);
    //��flash����ȡ������Ϣ
    robotBoardConfig();
    //û��IDʲô�����ɣ�ֱ�ӽ�ʶ������
    digitalHi(&robotConfigData.robotTypeFlag);
	if(getConfigData()->boardType == BOARD_CHASSIS){				
		can_fd_dataSend = arbitration_prepared(&can_fd_chassisData,TX_RESPONSE_ID_CHASSIS,CAN_DLC_64);
	}
	else{								
        //�������ID��ֱ������						
		currentRobotParameterConfig();
        //��ʾʶ�����	        
		can_fd_dataSend = arbitration_prepared(&can_fd_controlData,TX_RESPONSE_ID_GIMBAL,CAN_DLC_64);
	}
	robotConfigData.distinguishState = ROBOT_COMPLETE_IDENTIFIED;
}

void robotKeyInit(void){
    BSP_GPIO_Init(GPIO_SWITCH_1,GPIO_Mode_IPU);   //1
    BSP_GPIO_Init(GPIO_SWITCH_2,GPIO_Mode_IPU);   //2
    BSP_GPIO_Init(GPIO_SWITCH_3,GPIO_Mode_IPU);   //3
    BSP_GPIO_Init(GPIO_SWITCH_4,GPIO_Mode_IPU);   //4
}
formatTrans8Struct_t numID;  
uint8_t getRobotType(void){

	numID.byte.a_temp = SWITCH_4;
	numID.byte.b_temp = SWITCH_3;
	numID.byte.c_temp = SWITCH_2;
	numID.byte.d_temp = SWITCH_1;
    return numID.u8_temp;
}


//void init_data_prepared(uint16_t boardType){
//	arbitration_prepared(&idData,ID_CHECK,CAN_DLC_64);
//	switch(boardType){
//		case BOARD_CONTROL:{
//			do{
//				getCanFdTrans()->Send(&idData);
//				getCanFdTrans()->Receive(&can_fd_dataReceive);
//			}while(!robotConfigData.id_set);
//		}break;
//		case BOARD_CHASSIS:{
//			do{
//				getCanFdTrans()->Receive(&can_fd_dataReceive);
//			}while(robotConfigData.typeOfRobot == NO_ID);
//			digitalHi(&robotConfigData.id_set);
//			getCanFdTrans()->Send(&idData);
//		}break;
//	}
//}

