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
//云台激活函数
void gimbalSwitch(uint8_t active){																				
	if(active){
		getGimbalData()->autoMode = ENABLE;
	}
	else{
		getGimbalData()->autoMode = DISABLE;
	}	
}
//底盘激活函数
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
//发射机构激活函数
void shootSwitch(uint8_t active){																					
	if(active){
		getshootData()->autoMode = ENABLE;
	}
	else{
		getshootData()->autoMode = DISABLE;
	}	
}
//变形激活参数
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
    //必须在键鼠模式或哨兵下才能启动状态机扫描
	if( !(robotConfigData.typeOfRobot == SENTRY_ID || robotConfigData.typeOfRobot == SMALLGIMBAL_ID) ){
		if(RC_MODE != RCSW_TOP){																							
			return false;																									
		}
	}
	/* ---- 当前存在任务的情况下且非哨兵模式 ---- */
	if( (autoTaskData->currentTask != 0 || autoTaskData->avoidTask) && \
	   !(robotConfigData.typeOfRobot == SENTRY_ID || robotConfigData.typeOfRobot == SMALLGIMBAL_ID) ){
        //如果摁下F键，则终止当前任务，并对当前任务内容进行清零																																			
		if(TASK_PRESS_F){																											
			digitalHi(&autoTaskData->breakSign);
			getchassisData()->chaseRef = 0.0f;
			autoTaskData->avoidTask = DISABLE;
			autoTaskData->aviodFlag = false;
			digitalLo(&getinfantryAutoData()->avoidSchedule);
		}
        //如果CTRL没有持续摁下,则终止当前任务，并对当前任务内容进行清零
//		if(autoTaskData->currentTask == CTRL_TASK){															
//			if(!TASK_PRESS_CTRL)																							
//				autoDataInit(autoTaskData);
//		}
        //如果G没有持续摁下,则终止当前任务，并对当前任务内容进行清零
		if(autoTaskData->currentTask == G_TASK){
			if(!TASK_PRESS_G){																									
				static uint16_t time;
				time ++ ;//开始计时
				if(time > 25){//50ms松开消抖
					time = 0;//下一次松开计时
					autoDataInit(autoTaskData);
				}
			}
		}
        //如果Z没有持续摁下,则终止当前任务，并对当前任务内容进行清零
		if(autoTaskData->currentTask == Z_TASK){
			if(!TASK_PRESS_Z){																									
				static uint16_t time;
				time ++ ;//开始计时
				if(time > 25){//50ms松开消抖
					time = 0;//下一次松开计时
					autoDataInit(autoTaskData);
				}
			}
		}
        //如果V没有持续摁下,则终止当前任务，并对当前任务内容进行清零
		if(autoTaskData->currentTask == V_TASK){
			if(!TASK_PRESS_V){																									
				static uint16_t time;
				time ++ ;//开始计时
				if(time > 25){//50ms松开消抖
					time = 0;//下一次松开计时
					autoDataInit(autoTaskData);
				}
			}
		}
		if(autoTaskData->currentTask == X_TASK && ROBOT != INFANTRY_ID){
			if(!TASK_PRESS_X){																									
				static uint16_t time;
				time ++ ;//开始计时
				if(time > 25){//50ms松开消抖
					time = 0;//下一次松开计时
					autoDataInit(autoTaskData);
				}
			}
		}
	}
	if(autoTaskData->currentTask != CTRL_TASK && autoTaskData->currentTask == NULL ){  
		/* --- 当前非CTRL任务下 --- */
        //只摁下X的情况下
		if(TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){							  
			autoTaskData->currentTask = X_TASK;
		}
        //只摁下R的情况下
		else if(!TASK_PRESS_X && TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = R_TASK;
		}	
        //只摁下CTRL的情况下
		else if(!TASK_PRESS_X && !TASK_PRESS_R && TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = CTRL_TASK;
		}
        //只摁下Z的情况下
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = Z_TASK;
		}
        //只摁下V的情况下
		else if((!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && TASK_PRESS_V  && !TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C)){						
			autoTaskData->currentTask = V_TASK;
		}
        //只摁下Q的情况下
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = Q_TASK;
		}
        //只摁下G的情况下
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z && !TASK_PRESS_C){						
			autoTaskData->currentTask = G_TASK;
		}
		//只摁下C的情况下
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z && TASK_PRESS_C){
			if(ROBOT != INFANTRY_ID)
			autoTaskData->currentTask = C_TASK;
		}			
		
	}
    //按下E扭腰
	if(TASK_PRESS_E){																																												
		autoTaskData->avoidTask = ENABLE;
		autoTaskData->aviodFlag = true;
		autoTaskData->aviodDuration = 0.0f;
	}

	//按下直接满速
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

//自动任务更新
void autoTaskUpdate(void){																								
    //如果不再哨兵模式下，SW2又不在键鼠档位，则无法执行任何自动任务
	if(keyboradScanForTask()){																							
		switch(robotConfigData.typeOfRobot){
            //步兵任务
			case INFANTRY_ID:	
				infantryAutoTaskUpdate(); 
				break;				
            //英雄任务													
			case P_TANK_ID : 
					p_tankAutoTaskUpdate(); 
				break;	
			//老英雄任务													
			case F_TANK_ID : 
					f_tankAutoTaskUpdate(); 
				break;	
            //工程车任务
			case AUXILIARY_ID: 
				auxiliaryAutoTaskUpdate(); 
				break;																	
            //哨兵任务
			case SMALLGIMBAL_ID:
			case SENTRY_ID: 
				sentryAutoTaskUpdate(); 
				break;																	
            //无人机任务
			case UAV_ID: 
				uavAutoTaskUpdate(); 
				break;																	
		}
	}
	else{
        //终止当前任务并对当前任务内容进行清零
		autoDataInit(autoTaskData);																						
	}
}
//对自动任务的结构体内容进行清零
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
    //通过tf卡写机器人种类
	switch(robotConfigData.typeOfRobot){			
        //步兵任务															
		case INFANTRY_ID:	
			autoTaskData = getinfantryAutoData();
			break;								
        //英雄任务																						
		case P_TANK_ID : 
			autoTaskData = getTankAutoData();
			break;	
		 //老英雄任务																						
		case F_TANK_ID : 
			autoTaskData = getTankAutoData();
			break;
        //工程车任务			
		case AUXILIARY_ID: 
			autoTaskData = getauxiliaryAutoData();
			break;																															
        //哨兵任务
		case SMALLGIMBAL_ID:
		case SENTRY_ID:  
			autoTaskData = getsentryAutoData();
			break;																															
        //无人机任务
		case UAV_ID: 
			autoTaskData = getuavAutoData(); 
			break;																															
	}
    //初始化所有任务数据
	autoDataInit(autoTaskData);		
    autoTaskData->capSchedule = 0;	
}
