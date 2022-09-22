#include "type_robot.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "deforming.h"
#include "auto_task.h"
#include "auto_infantry.h"
#include "auto_tank.h"
#include "auto_auxiliary.h"
#include "auto_sentry.h"
#include "auto_uav.h"
#include "rc.h"
#include "judge.h"
#include "vision.h"
#include "config.h"

AutoTaskStruct_t *autoTaskData;
//��̨�����
void gimbalSwitch(uint8_t active){																				
	if(active){
		getGimbalData()->autoMode = ENABLE;
	}
	else{
		getGimbalData()->autoMode = DISABLE;
	}	
}
//���̼����
void chassisSwitch(uint8_t active){																				
	if(active){
		getchassisData()->autoMode = ENABLE;
	}
	else{
		getchassisData()->autoMode = DISABLE;
		getchassisData()->autoSpeedTarget.x = 0.0f;
		getchassisData()->autoSpeedTarget.y = 0.0f;
		getchassisData()->autoSpeedTarget.z = 0.0f;
	}	
}
//������������
void shootSwitch(uint8_t active){																					
	if(active){
		getshootData()->autoMode = ENABLE;
	}
	else{
		getshootData()->autoMode = DISABLE;
	}	
}
//���μ������
void deformingSwitch(uint8_t active){																			
	if(active){
		
	}
	else{
		getchassisData()->speedLimit = 1.0f;
	}
}

bool keyboradScanForTask(void){ 
static uint8_t lastKeyState_CTRL = 0;
static uint8_t lastKeyState_G = 0;	
    //�����ڼ���ģʽ���ڱ��²�������״̬��ɨ��
	if( !(robotConfigData.typeOfRobot == SENTRY_ID || robotConfigData.typeOfRobot == SMALLGIMBAL_ID) ){
		if(RC_MODE != RCSW_TOP){																							
			return false;																									
		}
	}
	/* ---- ��ǰ���������������ҷ��ڱ�ģʽ ---- */
	if( (autoTaskData->currentTask != 0 || autoTaskData->avoidTask) && \
	   !(robotConfigData.typeOfRobot == SENTRY_ID || robotConfigData.typeOfRobot == SMALLGIMBAL_ID) ){
        //�������F��������ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������																																			
		if(TASK_PRESS_F){																											
			digitalHi(&autoTaskData->breakSign);
			getchassisData()->chaseRef = 0.0f;
			autoTaskData->avoidTask = DISABLE;
			autoTaskData->aviodFlag = false;
			digitalLo(&getinfantryAutoData()->avoidSchedule);
		}
        //���CTRLû�г�������,����ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������
//		if(autoTaskData->currentTask == CTRL_TASK){															
//			if(!TASK_PRESS_CTRL)																							
//				autoDataInit(autoTaskData);
//		}
        //���Gû�г�������,����ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������
		if(autoTaskData->currentTask == G_TASK){
			if(!TASK_PRESS_G){																									
				static uint16_t time;
				time ++ ;//��ʼ��ʱ
				if(time > 25){//50ms�ɿ�����
					time = 0;//��һ���ɿ���ʱ
					autoDataInit(autoTaskData);
				}
			}
		}
        //���Zû�г�������,����ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������
		if(autoTaskData->currentTask == Z_TASK){
			if(!TASK_PRESS_Z){																									
				static uint16_t time;
				time ++ ;//��ʼ��ʱ
				if(time > 25){//50ms�ɿ�����
					time = 0;//��һ���ɿ���ʱ
					autoDataInit(autoTaskData);
				}
			}
		}
        //���Vû�г�������,����ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������
		if(autoTaskData->currentTask == V_TASK){
			if(!TASK_PRESS_V){																									
				static uint16_t time;
				time ++ ;//��ʼ��ʱ
				if(time > 25){//50ms�ɿ�����
					time = 0;//��һ���ɿ���ʱ
					autoDataInit(autoTaskData);
				}
			}
		}
		if(autoTaskData->currentTask == X_TASK && ROBOT != INFANTRY_ID){
			if(!TASK_PRESS_X){																									
				static uint16_t time;
				time ++ ;//��ʼ��ʱ
				if(time > 25){//50ms�ɿ�����
					time = 0;//��һ���ɿ���ʱ
					autoDataInit(autoTaskData);
				}
			}
		}
	}
	if(autoTaskData->currentTask != CTRL_TASK && autoTaskData->currentTask == NULL ){  
		/* --- ��ǰ��CTRL������ --- */
        //ֻ����X�������
		if(TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){							  
			autoTaskData->currentTask = X_TASK;
		}
        //ֻ����R�������
		else if(!TASK_PRESS_X && TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = R_TASK;
		}	
        //ֻ����CTRL�������
		else if(!TASK_PRESS_X && !TASK_PRESS_R && TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = CTRL_TASK;
		}
        //ֻ����Z�������
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = Z_TASK;
		}
        //ֻ����V�������
		else if((!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && TASK_PRESS_V  && !TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C)){						
			autoTaskData->currentTask = V_TASK;
		}
        //ֻ����Q�������
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = Q_TASK;
		}
        //ֻ����G�������
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = G_TASK;
		}
		//ֻ����C�������
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z && TASK_PRESS_C){
			if(ROBOT != INFANTRY_ID)
			autoTaskData->currentTask = C_TASK;
		}			
		
	}
    //����EŤ��
	if(TASK_PRESS_E){																																												
		autoTaskData->avoidTask = ENABLE;
		autoTaskData->aviodFlag = true;
		autoTaskData->aviodDuration = 0.0f;
	}

	//����ֱ������
	if(TASK_PRESS_CTRL && TASK_PRESS_G){    
		if(!lastKeyState_CTRL || !lastKeyState_G){
			if(!autoTaskData->fastSeed)
				autoTaskData->fastSeed = 1;
			else
				autoTaskData->fastSeed = 0;
		}
	}
	lastKeyState_CTRL = TASK_PRESS_CTRL;	
	lastKeyState_G    = TASK_PRESS_G;
	
	return true;
}

//�Զ��������
void autoTaskUpdate(void){																								
    //��������ڱ�ģʽ�£�SW2�ֲ��ڼ���λ�����޷�ִ���κ��Զ�����
	if(keyboradScanForTask()){																							
		switch(robotConfigData.typeOfRobot){
            //��������
			case INFANTRY_ID:	
				infantryAutoTaskUpdate(); 
				break;				
            //Ӣ������													
			case P_TANK_ID : 
					p_tankAutoTaskUpdate(); 
				break;	
			//��Ӣ������													
			case F_TANK_ID : 
					f_tankAutoTaskUpdate(); 
				break;	
            //���̳�����
			case AUXILIARY_ID: 
				auxiliaryAutoTaskUpdate(); 
				break;																	
            //�ڱ�����
			case SMALLGIMBAL_ID:
			case SENTRY_ID: 
				sentryAutoTaskUpdate(); 
				break;																	
            //���˻�����
			case UAV_ID: 
				uavAutoTaskUpdate(); 
				break;																	
		}
	}
	else{
        //��ֹ��ǰ���񲢶Ե�ǰ�������ݽ�������
		autoDataInit(autoTaskData);																						
	}
}
//���Զ�����Ľṹ�����ݽ�������
void autoDataInit(AutoTaskStruct_t *autoContrlData){										
	digitalClan(&autoContrlData->currentTask);
	digitalClan(&autoContrlData->breakSign);
	digitalClan(&autoContrlData->currentTask);
	digitalClan(&autoContrlData->schedule);
	digitalClan(&autoContrlData->taskDuration);
	digitalClan(&autoContrlData->taskState);
	digitalClan(&autoContrlData->autoDeviceData.gimbalAutoSate);
	digitalClan(&autoContrlData->autoDeviceData.chassisAutoState);
	digitalClan(&autoContrlData->autoDeviceData.shootAutoState);
	digitalClan(&autoContrlData->autoDeviceData.deformingAutoState);
	gimbalSwitch(DISABLE);
	chassisSwitch(DISABLE);
	shootSwitch(DISABLE);
	deformingSwitch(DISABLE);
	if(ROBOT == INFANTRY_ID){
		digitalLo(&get_infDeforming()->change_finish);
		digitalLo(&get_infDeforming()->up_step_switch);
		digitalClan(&get_infDeforming()->change_chassis_step);
		digitalClan(&getchassisData()->roDir);
		get_infDeforming()->arm_direction = ARM_STOP;
	}
	visionSendDataUpdate(TX2_STOP,getvisionData()->bullet);
}

void autoTaskInit(void){
    //ͨ��tf��д����������
	switch(robotConfigData.typeOfRobot){			
        //��������															
		case INFANTRY_ID:	
			autoTaskData = getinfantryAutoData();
			break;								
        //Ӣ������																						
		case P_TANK_ID : 
			autoTaskData = getTankAutoData();
			break;	
		 //��Ӣ������																						
		case F_TANK_ID : 
			autoTaskData = getTankAutoData();
			break;
        //���̳�����			
		case AUXILIARY_ID: 
			autoTaskData = getauxiliaryAutoData();
			break;																															
        //�ڱ�����
		case SMALLGIMBAL_ID:
		case SENTRY_ID:  
			autoTaskData = getsentryAutoData();
			break;																															
        //���˻�����
		case UAV_ID: 
			autoTaskData = getuavAutoData(); 
			break;																															
	}
    //��ʼ��������������
	autoDataInit(autoTaskData);		
    autoTaskData->capSchedule = 0;	
}
