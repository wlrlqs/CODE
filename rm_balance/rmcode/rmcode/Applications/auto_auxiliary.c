#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "vision.h"
#include "auto_auxiliary.h"
#include "DRIVER_VL53L0X.h"
#include "rc.h"
#include "keyboard.h"
#include "auto_tank.h"
#include "deforming.h"
#include "config.h"

#define MAX_WAITING_TIME 800

AutoTaskStruct_t auxiliaryAutoData;

AutoTaskStruct_t* getauxiliaryAutoData(){
    return &auxiliaryAutoData;
}


void auxiliaryTaskBegin(void){
	auxiliaryAutoData.taskState = EXECUTING;						//标记为进行中
	digitalIncreasing(&auxiliaryAutoData.schedule);					//给执行序列加一
}

static uint8_t getUpperStatus(){
	uint8_t num = 0;
	if(!UPPER_ELEC_SWITCH_UP1){	num++;	}
	if(!UPPER_ELEC_SWITCH_UP2){	num++;	}
	if(!UPPER_ELEC_SWITCH_UP3){	num++;	}
	if(!UPPER_ELEC_SWITCH_UP4){	num++;	}
	if(num > PROTECT_LIMIT){
		return 1;
	}else {
		return 0;
	}
}

static uint8_t getOtherStatus(uint8_t status){
    uint8_t num11 = 0;
    uint8_t data[4];
	if(!UPPER_LEFT_ELEC_SWITCH_ON)	{	data[0] = 0x08;	} else { data[0] = 0x00 ;}
	if(!UPPER_RIGHT_ELEC_SWITCH_ON)	{	data[1] = 0x02;	} else { data[1] = 0x00 ;}
	if(!UPPER_LEFT_ELEC_SWITCH_OFF)	{	data[2] = 0x04;	} else { data[2] = 0x00 ;}
	if(!UPPER_RIGHT_ELEC_SWITCH_OFF){	data[3] = 0x01;	} else { data[3] = 0x00 ;}
	num11 = data[0] | data[1] | data[2] | data[3] ;
	if(num11 == status){
		return 1;
	}else {
		return 0;
	}
}



/*
***************************************************
函 数 名：	auxiliaryAutoLandUpdate
功    能：   自动上岛任务更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备   注：
***************************************************
*/
/*
static void auxiliaryAutoLandUpate(void){
    if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;											//如果有打断则结束任务
	}
    switch(auxiliaryAutoData.schedule){
        case 1:{
            CHASSIS_UP;
			getchassisData()->speedLimit = 0.6f;
			getchassisData()->diffSpeedLimit = 3.0f;
            digitalIncreasing(&auxiliaryAutoData.schedule);
            break;
        }
        case 2:{
            if(!WHEEL_RIGHT_ELEC_SWITCH&&!WHEEL_LEFT_ELEC_SWITCH){
                CHASSIS_DOWN;
                digitalIncreasing(&auxiliaryAutoData.schedule);
            }
            break;
        }
        case 3:{
            break;
        }
        case 99:{
            CHASSIS_NOTHING;
			getchassisData()->diffSpeedLimit = 1.0f;
			getchassisData()->speedLimit = 1.0f;
            auxiliaryAutoData.taskState = END_OF_EXECUTION;
            break;
        
        }
        default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
    }
}
*/
/*
***************************************************
函 数 名：	auxiliarySupplyUpdate
功    能：   补给交互任务更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备    注：
***************************************************
*/
/*
static void auxiliarySupplyUpdate(){
    if(auxiliaryAutoData.breakSign){
            auxiliaryAutoData.schedule = 99;										
    }
    switch(auxiliaryAutoData.schedule){
        case 1:{
            UPPER_UP;
            digitalIncreasing(&auxiliaryAutoData.schedule);
            break;
        }
		case 2:{
			if(PRESS_G){	
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			if(!TASK_PRESS_Z){
				auxiliaryAutoData.schedule = 99;
			}
			break;
		}
		case 3:{
			BULLET_SUPPLY_ON;
			digitalIncreasing(&auxiliaryAutoData.schedule);
			break;
		}
		case 4:{
			if(!TASK_PRESS_Z){
				auxiliaryAutoData.schedule = 99;
			}
			break;
		}
        case 99:{
            UPPER_DOWN;
            BULLET_SUPPLY_OFF;
            auxiliaryAutoData.taskState = END_OF_EXECUTION;
            break;
        }
        default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
    }
    auxiliaryAutoData.taskDuration += AUTO_TASK_DURATION;
}
*/                                            //抓到矿石后抬起高度       //抓矿初始高度
static uint8_t auxiliaryGrabBulletOne (uint16_t HEIGHT_TO_GET_2,uint16_t HEIGHT_TO_GET_1){
	uint8_t grabFlag = 0;
	switch(auxiliaryAutoData.grabSchedule){
		case 0:{
			PAW_ON;
			PAW_OUT;
			digitalClan(&auxiliaryAutoData.grabCountTimes);
			digitalIncreasing(&auxiliaryAutoData.grabSchedule);
			grabFlag = 0;
			break;
		}
		case 1:{
			digitalIncreasing(&auxiliaryAutoData.grabCountTimes);
			if(auxiliaryAutoData.grabCountTimes > 250){
				PAW_OFF;
				digitalClan(&auxiliaryAutoData.grabCountTimes);
				digitalIncreasing(&auxiliaryAutoData.grabSchedule);
				grabFlag = 0;
			}
			break;
		}
		case 2:{
			digitalIncreasing(&auxiliaryAutoData.grabCountTimes);
			if(auxiliaryAutoData.grabCountTimes > 30){
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(HEIGHT_TO_GET_2);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					PAW_IN;
					digitalClan(&auxiliaryAutoData.grabCountTimes);
					digitalIncreasing(&auxiliaryAutoData.mineralnumber);
					digitalIncreasing(&auxiliaryAutoData.grabSchedule);
					grabFlag = 0;
				}
			}
			break;
		}
		case 3:{
			digitalIncreasing(&auxiliaryAutoData.grabCountTimes);
			if(auxiliaryAutoData.grabCountTimes > 150){
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(HEIGHT_TO_GET_1);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalClan(&auxiliaryAutoData.grabCountTimes);
					digitalClan(&auxiliaryAutoData.grabSchedule);
					grabFlag = 1;
				}
		    }
			break;
		}
		default:break;
	}
	return grabFlag;
}
static uint8_t auxiliaryExchangeOne(){
	uint8_t exchangeFlag = 0;
	switch(auxiliaryAutoData.exchangeSchedule){
		case 0:{
			PAW_OUT;
			digitalIncreasing(&auxiliaryAutoData.exchangeCountTimes);
			if(auxiliaryAutoData.exchangeCountTimes > 150){				
				digitalClan(&auxiliaryAutoData.exchangeCountTimes);
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);			
			}
			exchangeFlag = 0;
			break;
		}
		case 1:{
			if(auxiliaryAutoData.mineralnumber == 3){
				digitalLo(&auxiliaryAutoData.reversingLockFlag);
				REVERSING_BEHIND;
				if(PRESS_X){
					digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);
				}
			}
			else{
				digitalSet(&auxiliaryAutoData.exchangeSchedule,3);
			}
			exchangeFlag = 0;
			break;
		}
		case 2:{
			if(!PRESS_X){	
                REVERSING_FRONT;
				digitalHi(&auxiliaryAutoData.reversingLockFlag);
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);
			}
			exchangeFlag = 0;
			break;
		}
		case 3:{
			if(PRESS_X){
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);				
			}
			exchangeFlag = 0;
			break;
		}
		case 4:{
			if(!PRESS_X){
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);		
			}
			exchangeFlag = 0;
			break;
		}
		case 5:{//推矿操作
			auxiliaryDeformingData.reversingSpeed[0].dataRef = 1500.0;
			auxiliaryDeformingData.reversingSpeed[1].dataRef = 1500.0;			
			Y_FRONT;
			digitalIncreasing(&auxiliaryAutoData.exchangeCountTimes);
			if(auxiliaryAutoData.exchangeCountTimes > 300){
				digitalDecline(&auxiliaryAutoData.mineralnumber);
				digitalClan(&auxiliaryAutoData.exchangeCountTimes);
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);				
			}
			exchangeFlag = 0;
			break;
		}		
		case 6:{
			PAW_ON;
			digitalIncreasing(&auxiliaryAutoData.exchangeCountTimes);
			if(auxiliaryAutoData.exchangeCountTimes > 50){
				digitalClan(&auxiliaryAutoData.exchangeCountTimes);
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);		
			}
			exchangeFlag = 0;
			break;
		}
		case 7:{
			auxiliaryDeformingData.reversingSpeed[0].dataRef = -1500.0;
			auxiliaryDeformingData.reversingSpeed[1].dataRef = -1500.0;
			Y_BACK;
			digitalIncreasing(&auxiliaryAutoData.exchangeCountTimes);
			if(auxiliaryAutoData.exchangeCountTimes > 300){
				digitalClan(&auxiliaryAutoData.exchangeCountTimes);
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);
			}
			exchangeFlag = 0;
			break;
		}
		case 8:{
			PAW_OFF;
			digitalIncreasing(&auxiliaryAutoData.exchangeCountTimes);
			if(auxiliaryAutoData.exchangeCountTimes > 30){
				digitalClan(&auxiliaryAutoData.exchangeCountTimes);
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);
			}
			exchangeFlag = 0;
			break;
		}
		case 9:{
			auxiliaryDeformingData.reversingSpeed[0].dataRef = 1500.0;
			auxiliaryDeformingData.reversingSpeed[1].dataRef = 1500.0;	
			Y_FRONT;
			digitalIncreasing(&auxiliaryAutoData.exchangeCountTimes);
			if(auxiliaryAutoData.exchangeCountTimes > 200){
				digitalClan(&auxiliaryAutoData.exchangeCountTimes);
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);
			}
			break;
		}
		case 10:{
			auxiliaryDeformingData.reversingSpeed[0].dataRef = -1500.0;
			auxiliaryDeformingData.reversingSpeed[1].dataRef = -1500.0;	
			Y_BACK;
			digitalIncreasing(&auxiliaryAutoData.exchangeCountTimes);
			if(auxiliaryAutoData.exchangeCountTimes > 200){
				digitalClan(&auxiliaryAutoData.exchangeCountTimes);
				digitalIncreasing(&auxiliaryAutoData.exchangeSchedule);
			}
			break;
		}
		case 11:{
            PAW_IN;
			PAW_ON;
			auxiliaryDeformingData.reversingSpeed[0].dataRef = 0.0;
			auxiliaryDeformingData.reversingSpeed[1].dataRef = 0.0;
			digitalIncreasing(&auxiliaryAutoData.exchangeCountTimes);
			if(auxiliaryAutoData.exchangeCountTimes > 150){
				digitalClan(&auxiliaryAutoData.exchangeCountTimes);
				exchangeFlag = 1;
			}
			break;
		}
		default:break;
	}
	return exchangeFlag;	
	
}
//static uint8_t resetFrontClaw(){
//	uint8_t resetFlag = 0;
//	switch(auxiliaryAutoData.frontClawResetSchedule){
//		case 0:{
//			auxiliaryDeformingData.frontClawSpeed.dataRef = -1000.0;
//			if(FRONT_CLAW_RESET){
//				FRONT_CLAW_OFF;
//				auxiliaryDeformingData.frontClawSpeed.dataRef = 0.0;
//				digitalIncreasing(&auxiliaryAutoData.frontClawResetSchedule);
//				resetFlag = 0;
//			}
//			break;
//		}
//		case 1:{
//			FRONT_CLAW_DOWN;
//			digitalIncreasing(&auxiliaryAutoData.frontClawResetCountTimes);
//			if(auxiliaryAutoData.frontClawResetCountTimes > 150){
//				digitalClan(&auxiliaryAutoData.frontClawResetCountTimes);
//				digitalIncreasing(&auxiliaryAutoData.frontClawResetSchedule);
//				resetFlag = 0;
//			}
//			break;
//		}		
//		case 2:{
//			FRONT_CLAW_OFF;
//			FRONT_CLAW_IN;
//			digitalIncreasing(&auxiliaryAutoData.frontClawResetCountTimes);
//			if(auxiliaryAutoData.frontClawResetCountTimes > 150){
//				digitalClan(&auxiliaryAutoData.frontClawResetCountTimes);
//				digitalIncreasing(&auxiliaryAutoData.frontClawResetSchedule);
//				resetFlag = 0;
//			}
//			break;
//		}
//		case 3:{                
//			digitalClan(&auxiliaryAutoData.frontClawResetSchedule);
//			resetFlag = 1;
//			break;
//		}
//		default:break;
//	}
//	return resetFlag;	
//}
//注：这里为了测试时保证逻辑的正确性，将下面的||更改成 &&
static uint8_t conditionalJudgment(uint8_t status){
	return ((getOtherStatus(status) && (auxiliaryAutoData.countTimes > MAX_WAITING_TIME))  && getUpperStatus());
}
/*
***************************************************
函 数 名：	auxiliaryExchangeUpdata
功    能：   兑换站兑换任务更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备    注：（V+X）
***************************************************
*/
static void auxiliaryExchangeUpdata(){
	if(auxiliaryAutoData.breakSign ){
		digitalClan(&auxiliaryAutoData.exchangeSchedule);
		digitalClan(&auxiliaryAutoData.exchangeCountTimes);
		digitalClan(&auxiliaryAutoData.countTimes);
		digitalLo(&auxiliaryAutoData.breakSign);
		auxiliaryAutoData.schedule = 97;							//如果有打断则结束任务
	}
	switch(auxiliaryAutoData.schedule){
		case 1:{
			REVERSING_FRONT;
            getGimbalData()->yawAngleStopSet = 175.0f;
			getchassisData()->speedLimit = 0.2f;
			gimbalStopSwitch(ENABLE);
			digitalIncreasing(&auxiliaryAutoData.schedule);
			usbVCP_Printf("In the first status \r\n ");	
			break;
		}
		case 2:{
			if(!PRESS_V){
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(NORMAL_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);
					usbVCP_Printf("In the third status,and then go left \r\n");
				}
			}			
			break;
		}
		case 3:{	
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(THIRD_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);
					usbVCP_Printf("In the third status,and then go left \r\n");
				}
			break;
		}
	   {//兑换第一个矿石
		case 4:{
			if(PRESS_X){
			   digitalClan(&auxiliaryAutoData.exchangeSchedule);
			   digitalClan(&auxiliaryAutoData.exchangeCountTimes);
			   digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			break;	
		}
		case 5:{
			if(!PRESS_X){
				if(auxiliaryAutoData.mineralnumber == 3){
					digitalIncreasing(&auxiliaryAutoData.schedule);	
				}				
				else
					if(auxiliaryAutoData.mineralnumber == 2){
						digitalSet(&auxiliaryAutoData.schedule,8);
					}
				else
					if(auxiliaryAutoData.mineralnumber == 1){
						digitalSet(&auxiliaryAutoData.schedule,14);
					}
				else{
					digitalSet(&auxiliaryAutoData.schedule,97);
				}
			}
			break;
		}
		case 6:{
			if(auxiliaryExchangeOne()){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			break;
		}
	   }
       {//兑换第二个矿石
        case 7:{
			if(PRESS_X){
			   digitalClan(&auxiliaryAutoData.exchangeSchedule);
			   digitalClan(&auxiliaryAutoData.exchangeCountTimes);
			   digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			break;	
		}
		case 8:{
			if(!PRESS_X){
				REVERSING_BEHIND;
				PAW_ON;
				digitalIncreasing(&auxiliaryAutoData.countTimes);
				if(auxiliaryAutoData.countTimes > 100){	
					PAW_OUT;
					digitalLo(&auxiliaryAutoData.reversingLockFlag);
					digitalClan(&auxiliaryAutoData.countTimes);
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
			break;
		}
		case 9:{
			if(PRESS_X){
			   digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			break;
		}
	    case 10:{
			if(!PRESS_X){
				PAW_IN;
				REVERSING_FRONT;
				digitalHi(&auxiliaryAutoData.reversingLockFlag);
				digitalIncreasing(&auxiliaryAutoData.countTimes);
				if(auxiliaryAutoData.countTimes > 500){	
					PAW_OFF;
					digitalClan(&auxiliaryAutoData.countTimes);
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
			break;
		}
		case 11:{
			if(auxiliaryExchangeOne()){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		break;			
		}
	   }
	   //兑换第三个矿石
	   {
        case 12:{
			if(PRESS_X){
			   digitalClan(&auxiliaryAutoData.exchangeSchedule);
			   digitalClan(&auxiliaryAutoData.exchangeCountTimes);
			   digitalSet(&auxiliaryAutoData.schedule,14);
			}
			else
				if(PRESS_G){					
					digitalClan(&auxiliaryAutoData.exchangeSchedule);
					digitalClan(&auxiliaryAutoData.exchangeCountTimes);
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			break;	
		}
		case 13:{
			if(!PRESS_G){				
				digitalIncreasing(&auxiliaryAutoData.mineralnumber);
				digitalSet(&auxiliaryAutoData.schedule,7);
			}
			break;
		}
		case 14:{
			if(!PRESS_X){
				auxiliaryDeformingData.reversingSpeed[0].dataRef = 1500.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = 1500.0;
				digitalIncreasing(&auxiliaryAutoData.schedule);		
			}
			break;
		}
		case 15:{
				digitalIncreasing(&auxiliaryAutoData.schedule);	
			break;
		}
		case 16:{
			ORE1_IN;
			digitalIncreasing(&auxiliaryAutoData.countTimes);
			if(auxiliaryAutoData.countTimes > 1000){	
				digitalClan(&auxiliaryAutoData.countTimes);
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			break;			
		}
	    case 17:{
			digitalIncreasing(&auxiliaryAutoData.countTimes);
			if(auxiliaryAutoData.countTimes > 50){	
				auxiliaryDeformingData.reversingSpeed[0].dataRef = 0.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = 0.0;
				digitalClan(&auxiliaryAutoData.countTimes);
				digitalIncreasing(&auxiliaryAutoData.schedule);
				ORE1_OFF;
			}
			break;
		}
        case 18:{
			   digitalClan(&auxiliaryAutoData.exchangeSchedule);
			   digitalClan(&auxiliaryAutoData.exchangeCountTimes);
			   digitalIncreasing(&auxiliaryAutoData.schedule);
			break;	
		}
		case 19:{
			if(!PRESS_X){
				PAW_ON;
				REVERSING_BEHIND;
				digitalLo(&auxiliaryAutoData.reversingLockFlag);
				digitalIncreasing(&auxiliaryAutoData.schedule);	
			}
			break;
		}
		case 20:{
			if(PRESS_X){
			   digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			break;
		}
	    case 21:{
			if(!PRESS_X){
				PAW_OFF;
				REVERSING_FRONT;
				digitalHi(&auxiliaryAutoData.reversingLockFlag);
				digitalIncreasing(&auxiliaryAutoData.schedule);	
			}
			break;
		}
		case 22:{
			if(auxiliaryExchangeOne()){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		    break;			
		}
		case 23:{
			if(PRESS_X){
			   digitalClan(&auxiliaryAutoData.exchangeSchedule);
			   digitalClan(&auxiliaryAutoData.exchangeCountTimes);
			   digitalSet(&auxiliaryAutoData.schedule,97);
			}
			else
				if(PRESS_G){					
					digitalClan(&auxiliaryAutoData.exchangeSchedule);
					digitalClan(&auxiliaryAutoData.exchangeCountTimes);
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}			
			break;
	   }
		case 24:{
			if(!PRESS_G){				
				digitalIncreasing(&auxiliaryAutoData.mineralnumber);
				digitalSet(&auxiliaryAutoData.schedule,18);
			}
			break;			
		}
	}
		case 97 :{
			if(!PRESS_X){
				PAW_IN;
				Y_BACK;
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);		
				GO_TO_HEIGHT(FOURTH_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
            break;
		}
		case 98:{
			if(auxiliaryAutoData.mineralnumber != 0){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			else{
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(NORMAL_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}		
			}		
			break;
		}					
        //任务执行结束
		case 99:{
			digitalIncreasing(&auxiliaryAutoData.countTimes);
			if(auxiliaryAutoData.countTimes > 50){
				auxiliaryDeformingData.reversingSpeed[0].dataRef = 0.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = 0.0;				
				REVERSING_FRONT;
				getGimbalData()->yawAngleStopSet = 0.0f;
				getchassisData()->speedLimit = 1.0f;
				gimbalStopSwitch(DISABLE);
				digitalClan(&auxiliaryAutoData.grabCountTimes);
				digitalClan(&auxiliaryAutoData.grabSchedule);
				digitalClan(&auxiliaryAutoData.countTimes);
				auxiliaryAutoData.taskState = END_OF_EXECUTION;
				usbVCP_Printf("In the end of ALL,FINISH \r\n");
			}
            break;	
		}
        default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
  }
}
/*
***************************************************
函 数 名：	auxiliaryGrabBulletUpdata
功    能：   大资源岛抓弹任务更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备    注：（Z+X）
***************************************************
*/
static void auxiliaryGrabBigUpdata(){
    if(auxiliaryAutoData.breakSign ){
		digitalClan(&auxiliaryAutoData.grabCountTimes);
		digitalClan(&auxiliaryAutoData.countTimes);
		digitalLo(&auxiliaryAutoData.breakSign);
		auxiliaryAutoData.schedule = 97;							//如果有打断则结束任务
	}
    switch(auxiliaryAutoData.schedule){
        case 1:{
			REVERSING_FRONT;
            getGimbalData()->yawAngleStopSet = 175.0f;
			getchassisData()->speedLimit = 0.2f;
			gimbalStopSwitch(ENABLE);
			digitalIncreasing(&auxiliaryAutoData.schedule);
			usbVCP_Printf("In the first status \r\n ");
            break;
        }
		case 2:{			
			if(!PRESS_Z){
				//大资源岛单抓
				auxiliaryDeformingData.reversingSpeed[0].dataRef = 1500.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = 1500.0;
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(FOURTH_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);	 
			   }
			}
            break;
        }
		case 3:{			
				Y_FRONT;
				digitalIncreasing(&auxiliaryAutoData.countTimes);
				if(auxiliaryAutoData.countTimes > 300 || (auxiliaryAutoData.mineralnumber == 0)){	
					digitalClan(&auxiliaryAutoData.countTimes);
					digitalIncreasing(&auxiliaryAutoData.schedule);
					auxiliaryDeformingData.reversingSpeed[0].dataRef = 0.0;
					auxiliaryDeformingData.reversingSpeed[1].dataRef = 0.0;
				}
			break;
        }		
		case 4:{
			if(PRESS_X){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}	
			break;
		}
		case 5:{	
			if(!PRESS_X){
				if(auxiliaryAutoData.mineralnumber == 0){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
				else
					if(auxiliaryAutoData.mineralnumber == 1){
						digitalSet(&auxiliaryAutoData.schedule,8);
					}
				else
					if(auxiliaryAutoData.mineralnumber == 2){
						digitalSet(&auxiliaryAutoData.schedule,97);
					}
				else{
                    digitalSet(&auxiliaryAutoData.schedule,97);
				}	
				usbVCP_Printf("In the third status,and then go left \r\n");
		     }
			break;
		}
	   {//第一个矿石
		case 6:{	
			if(!PRESS_G){
				if(auxiliaryGrabBulletOne(FIFTH_HEIGHT_GET,FOURTH_HEIGHT_GET)){
					digitalIncreasing(&auxiliaryAutoData.schedule);
					usbVCP_Printf("In the third status,and then go left \r\n");
				}
			}
			break;
		}		

		case 7:{//抓矿回退和前进选择
			if(PRESS_X){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			else 
				if(PRESS_G){
					digitalDecline(&auxiliaryAutoData.mineralnumber);
					digitalSet(&auxiliaryAutoData.schedule,6);
				}
			break;
		}
	   } 
	   {//第二个矿石	
        case 8:{
			if(!PRESS_X){
				ORE1_ON;
				auxiliaryDeformingData.reversingSpeed[0].dataRef = -1500.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = -1500.0;
				digitalIncreasing(&auxiliaryAutoData.countTimes);
				if(auxiliaryAutoData.countTimes > 100){	
					digitalClan(&auxiliaryAutoData.countTimes);
					digitalIncreasing(&auxiliaryAutoData.schedule);
					PAW_ON;
				}
			}
			break;
		}
		case 9:{
			ORE1_OUT;
			digitalIncreasing(&auxiliaryAutoData.countTimes);
			if(auxiliaryAutoData.countTimes > 300){				
				auxiliaryDeformingData.reversingSpeed[0].dataRef = 0.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = 0.0;
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			break;
		}
		case 10:{
			if(!PRESS_G){//此处添加!PRESS_G是为了配合按G键
				if(auxiliaryGrabBulletOne(FIFTH_HEIGHT_GET,FOURTH_HEIGHT_GET)){
					digitalIncreasing(&auxiliaryAutoData.schedule);
					//usbVCP_Printf("In the third status,and then go left \r\n");
				}
			}	
			break;
		}

		case 11:{//抓矿回退和前进选择
			if(PRESS_X){
				digitalSet(&auxiliaryAutoData.schedule,97);
			}
			else 
				if(PRESS_G){
					digitalDecline(&auxiliaryAutoData.mineralnumber);
					digitalSet(&auxiliaryAutoData.schedule,10);
				}
			break;
		}		
	  }
	  {	//第三个矿石	
        case 12:{
			if(!PRESS_X){
				PAW_ON;
				digitalIncreasing(&auxiliaryAutoData.countTimes);
				if(auxiliaryAutoData.countTimes > 100){	
					digitalClan(&auxiliaryAutoData.countTimes);
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
			digitalSet(&auxiliaryAutoData.schedule,97);
			break;
		}
		case 13:{
				//ORE2_ON;
				digitalSet(&auxiliaryAutoData.schedule,97);
			break;
		}
		case 14:{
			
			if(!PRESS_G){
				if(auxiliaryGrabBulletOne(FIFTH_HEIGHT_GET,FOURTH_HEIGHT_GET)){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
			digitalSet(&auxiliaryAutoData.schedule,97);
			break;
		}

		case 15:{//抓矿回退和前进选择
			if(PRESS_X){
				digitalSet(&auxiliaryAutoData.schedule,97);
			}
			else 
				if(PRESS_G){
					digitalDecline(&auxiliaryAutoData.mineralnumber);
					digitalSet(&auxiliaryAutoData.schedule,97);
				}
			break;
		}
	   }		
		case 16:{
			digitalSet(&auxiliaryAutoData.schedule,97);
//			digitalHi(&auxiliaryDeformingData.Y_statebreak_Flag);
//			BACK_TO_Y(Y_BEHIND_GET);
//			if(!auxiliaryDeformingData.Y_statebreak_Flag){	
//				digitalSet(&auxiliaryAutoData.schedule,97);
//			}
			break;
		}
		case 97 :{
			auxiliaryDeformingData.reversingSpeed[0].dataRef = -1500.0;
			auxiliaryDeformingData.reversingSpeed[1].dataRef = -1500.0;
			if(!PRESS_X){
				PAW_IN;
				Y_BACK;
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);		
				GO_TO_HEIGHT(FOURTH_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
            break;	
		}
		case 98:{
			if(auxiliaryAutoData.mineralnumber != 0){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			else{
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(NORMAL_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}		
			}		
			break;
		}				
        //任务执行结束
		case 99:{
			digitalIncreasing(&auxiliaryAutoData.countTimes);
			if(auxiliaryAutoData.countTimes > 50){
				auxiliaryDeformingData.reversingSpeed[0].dataRef = 0.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = 0.0;
				REVERSING_FRONT;
				getGimbalData()->yawAngleStopSet = 0.0f;
				getchassisData()->speedLimit = 1.0f;
				gimbalStopSwitch(DISABLE);
				digitalClan(&auxiliaryAutoData.grabCountTimes);
				digitalClan(&auxiliaryAutoData.grabSchedule);
				digitalClan(&auxiliaryAutoData.countTimes);
				auxiliaryAutoData.taskState = END_OF_EXECUTION;
				usbVCP_Printf("In the end of ALL,FINISH \r\n");
			}
            break;	
		}
        default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
    }
}

/*
***************************************************
函 数 名：	auxiliaryGrabSmallUpdata
功    能：  小资源岛抓弹任务更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备    注：（C+X）(G重复当前操作)
***************************************************
*/
static void auxiliaryGrabSmallUpdata(){
    if(auxiliaryAutoData.breakSign ){
		digitalClan(&auxiliaryAutoData.grabCountTimes);
		digitalClan(&auxiliaryAutoData.countTimes);
		digitalLo(&auxiliaryAutoData.breakSign);
		auxiliaryAutoData.schedule = 97;							//如果有打断则结束任务
	}
    switch(auxiliaryAutoData.schedule){
        case 1:{
			REVERSING_FRONT;
            getGimbalData()->yawAngleStopSet = 175.0f;
			getchassisData()->speedLimit = 0.2f;
			gimbalStopSwitch(ENABLE);
			digitalIncreasing(&auxiliaryAutoData.schedule);
			usbVCP_Printf("In the first status \r\n ");
            break;
        }
		case 2:{			
			if(!PRESS_C){
				//小资源岛单抓
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(FIRST_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);	 
			   }
			}
            break;
        }
		case 3:{		
				digitalIncreasing(&auxiliaryAutoData.schedule);
			break;
        }		
		case 4:{
			if(PRESS_X){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}	
			break;
		}
		case 5:{	
			if(!PRESS_X){
				if(auxiliaryAutoData.mineralnumber == 0){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
				else
					if(auxiliaryAutoData.mineralnumber == 1){
						digitalSet(&auxiliaryAutoData.schedule,8);
					}
				else
					if(auxiliaryAutoData.mineralnumber == 2){
						digitalSet(&auxiliaryAutoData.schedule,12);
					}
				else{
                    digitalSet(&auxiliaryAutoData.schedule,97);
				}	
				usbVCP_Printf("In the third status,and then go left \r\n");
		     }
			break;
		}
	   {//第一个矿石
		case 6:{	
			if(!PRESS_G){
				auxiliaryDeformingData.reversingSpeed[0].dataRef = -1500.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = -1500.0;
				if(auxiliaryGrabBulletOne(SECOND_HEIGHT_GET,FIRST_HEIGHT_GET)){
					digitalIncreasing(&auxiliaryAutoData.schedule);
					usbVCP_Printf("In the third status,and then go left \r\n");
				}
			}
			break;
		}		

		case 7:{//抓矿回退和前进选择
			if(PRESS_X){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			else 
				if(PRESS_G){
					digitalDecline(&auxiliaryAutoData.mineralnumber);
					digitalSet(&auxiliaryAutoData.schedule,6);
				}
			break;
		}
	   } 
	   {//第二个矿石	
        case 8:{
			if(!PRESS_X){
				ORE1_ON;
				digitalIncreasing(&auxiliaryAutoData.countTimes);
				if(auxiliaryAutoData.countTimes > 100){	
					digitalClan(&auxiliaryAutoData.countTimes);
					digitalIncreasing(&auxiliaryAutoData.schedule);
					PAW_ON;
				}
			}
			break;
		}
		case 9:{
			ORE1_OUT;
			digitalIncreasing(&auxiliaryAutoData.countTimes);
			if(auxiliaryAutoData.countTimes > 1000){
				auxiliaryDeformingData.reversingSpeed[0].dataRef = 0.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = 0.0;

				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			break;
		}
		case 10:{
			if(!PRESS_G){//此处添加!PRESS_G是为了配合按G键
				if(auxiliaryGrabBulletOne(SECOND_HEIGHT_GET,FIRST_HEIGHT_GET)){
					digitalIncreasing(&auxiliaryAutoData.schedule);
					//usbVCP_Printf("In the third status,and then go left \r\n");
				}
			}	
			break;
		}

		case 11:{//抓矿回退和前进选择
			if(PRESS_X){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			else 
				if(PRESS_G){
					digitalDecline(&auxiliaryAutoData.mineralnumber);
					digitalSet(&auxiliaryAutoData.schedule,10);
				}
			break;
		}		
	  }
	  {	//第三个矿石	
        case 12:{
			if(!PRESS_X){
				PAW_ON;
				digitalIncreasing(&auxiliaryAutoData.countTimes);
				if(auxiliaryAutoData.countTimes > 100){	
					digitalClan(&auxiliaryAutoData.countTimes);
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
			break;
		}
		case 13:{
				digitalIncreasing(&auxiliaryAutoData.schedule);
			break;
		}
		case 14:{
			if(!PRESS_G){
				if(auxiliaryGrabBulletOne(SECOND_HEIGHT_GET,FIRST_HEIGHT_GET)){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
			break;
		}
		case 15:{//抓矿回退和前进选择
			if(PRESS_X){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			else 
				if(PRESS_G){
					digitalDecline(&auxiliaryAutoData.mineralnumber);
					digitalSet(&auxiliaryAutoData.schedule,14);
				}
			break;
		}
	   }		
		case 16:{
			if(!PRESS_X){
				PAW_IN;
				Y_BACK;
				digitalSet(&auxiliaryAutoData.schedule,97);
			}
			break;
		}
		case 97 :{
			if(!PRESS_X){
				PAW_IN;
				Y_BACK;
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);		
				GO_TO_HEIGHT(FOURTH_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
            break;
		}
		case 98:{
			if(auxiliaryAutoData.mineralnumber != 0){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			else{
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(NORMAL_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}		
			}		
			break;
		}				
        //任务执行结束
		case 99:{
			digitalIncreasing(&auxiliaryAutoData.countTimes);
			if(auxiliaryAutoData.countTimes > 50){
				auxiliaryDeformingData.reversingSpeed[0].dataRef = 0.0;
				auxiliaryDeformingData.reversingSpeed[1].dataRef = 0.0;
				REVERSING_FRONT;
				getGimbalData()->yawAngleStopSet = 0.0f;
				getchassisData()->speedLimit = 1.0f;
				gimbalStopSwitch(DISABLE);
				digitalClan(&auxiliaryAutoData.grabCountTimes);
				digitalClan(&auxiliaryAutoData.grabSchedule);
				digitalClan(&auxiliaryAutoData.countTimes);
				auxiliaryAutoData.taskState = END_OF_EXECUTION;
				usbVCP_Printf("In the end of ALL,FINISH \r\n");
			}
            break;	
		}
        default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
    }
}
/*
***************************************************
函 数 名：	auxiliaryfrontClawUpdate
功    能：   底盘小爪子任务更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备    注： （SHIFT）
***************************************************
*/
//static void  auxiliaryfrontClawUpdate(){
//	if(auxiliaryAutoData.breakSign){
//		digitalHi(&auxiliaryAutoData.frontClawLockFlag);  //上锁底盘爪子电机	
//		digitalClan(&auxiliaryAutoData.countTimes);
//		digitalLo(&auxiliaryAutoData.breakSign);
//		auxiliaryAutoData.schedule = 95;							//如果有打断则结束任务
//	}
//    switch(auxiliaryAutoData.schedule){
//        case 1:{
//			if(!PRESS_E){
//				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);			
//				GO_TO_HEIGHT(FIFTH_HEIGHT_GET);  
//				if(!auxiliaryDeformingData.heightstatebreak_Flag){
//					digitalIncreasing(&auxiliaryAutoData.schedule);					
//				} 	
//			}			
//			break;
//        }
//		case 2:{
//            FRONT_CLAW_OUT;		
//			digitalIncreasing(&auxiliaryAutoData.countTimes);
//			if(auxiliaryAutoData.countTimes > 150){	
//				digitalLo(&auxiliaryAutoData.frontClawLockFlag);	  //解锁底盘爪子电机			
//				digitalClan(&auxiliaryAutoData.countTimes);
//				digitalIncreasing(&auxiliaryAutoData.schedule);
//				FRONT_CLAW_ON;
//			}	
//		break;	
//		}
//		case 3:{
//			if(PRESS_R){
//				digitalIncreasing(&auxiliaryAutoData.schedule);
//			}
//			if(PRESS_X){
//				digitalSet(&auxiliaryAutoData.schedule,9);
//			}				
//			break;
//		}
//		case 4:{
//			if(!PRESS_R){
//				FRONT_CLAW_OFF;	
//				digitalIncreasing(&auxiliaryAutoData.countTimes);
//				if(auxiliaryAutoData.countTimes > 50){			
//					digitalClan(&auxiliaryAutoData.countTimes);				
//					digitalIncreasing(&auxiliaryAutoData.schedule);	
//				}	
//			}
//			break;
//		}
//		case 5:{
//			if(PRESS_G){			
//				digitalIncreasing(&auxiliaryAutoData.schedule);
//			}
//			break;
//		}
//		case 6:{
//            if(!PRESS_G){	
//				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);			
//				GO_TO_HEIGHT(NORMAL_HEIGHT_GET);  
//				if(!auxiliaryDeformingData.heightstatebreak_Flag){
//					digitalIncreasing(&auxiliaryAutoData.schedule);
//					FRONT_CLAW_ON;
//				} 	
//			}			
//			break;
//		}
//		case 7:{
//			digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);			
//			GO_TO_HEIGHT(FIFTH_HEIGHT_GET);  
//			if(!auxiliaryDeformingData.heightstatebreak_Flag){
//				digitalIncreasing(&auxiliaryAutoData.schedule);
//			} 	
//			break;
//		}
//		case 8:{
//			if(PRESS_X){
//				digitalIncreasing(&auxiliaryAutoData.schedule);
//			}
//			else
//				if(PRESS_R){
//					digitalSet(&auxiliaryAutoData.schedule,4);
//				}
//			break;
//		}
//		case 9:{
//			if(!PRESS_X){
//				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);			
//				GO_TO_HEIGHT(NORMAL_HEIGHT_GET);
//				if(!auxiliaryDeformingData.heightstatebreak_Flag){
//					digitalIncreasing(&auxiliaryAutoData.schedule);
//				} 					
//			}
//			break;
//		}
//		case 10:{
//			if(PRESS_X){
//				digitalIncreasing(&auxiliaryAutoData.schedule);
//			}
//			break;
//		}
//		case 11:{
//			if(!PRESS_X){
//				FRONT_CLAW_OFF;
//				digitalIncreasing(&auxiliaryAutoData.countTimes);
//				if(auxiliaryAutoData.countTimes > 50){			
//					digitalClan(&auxiliaryAutoData.countTimes);				
//					digitalIncreasing(&auxiliaryAutoData.schedule);	
//				}	
//			}
//			break;
//		}
//		case 12:{
//			if(PRESS_X){
//				digitalIncreasing(&auxiliaryAutoData.schedule);
//			}
//			break;
//		}
//		case 13:{
//			if(!PRESS_X){
//			    getGimbalData()->yawAngleStopSet = 175.0f;
//			    gimbalStopSwitch(ENABLE);
//				//此处需添加切换摄像头操作
//				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);			
//				GO_TO_HEIGHT(THIRD_HEIGHT_GET);
//				if(!auxiliaryDeformingData.heightstatebreak_Flag){
//					digitalIncreasing(&auxiliaryAutoData.schedule);
//				} 	
//			}
//			break;
//		}
//		case 14:{
//				FRONT_CLAW_UP;
//				digitalIncreasing(&auxiliaryAutoData.countTimes);
//				if(auxiliaryAutoData.countTimes > 150){			
//					digitalClan(&auxiliaryAutoData.countTimes);				
//					digitalIncreasing(&auxiliaryAutoData.schedule);	
//				}				
//			break;
//		}
//		case 15:{
//			if(PRESS_X){
//                digitalIncreasing(&auxiliaryAutoData.schedule);	
//			}
//			break;
//		}
//		case 16:{
//			if(!PRESS_X){
//				FRONT_CLAW_ON;
//				digitalIncreasing(&auxiliaryAutoData.countTimes);
//				if(auxiliaryAutoData.countTimes > 50){	
//                    digitalHi(&auxiliaryAutoData.frontClawLockFlag);  //上锁底盘爪子电机					
//					digitalClan(&auxiliaryAutoData.countTimes);				
//					digitalIncreasing(&auxiliaryAutoData.schedule);	
//				}
//            }
//			break;
//		}
//		case 17:{
//			if(PRESS_X){
//				digitalIncreasing(&auxiliaryAutoData.schedule);	
//			}
//			break;
//		}
//		case 18:{
//			if(!PRESS_X){
//				if(resetFrontClaw()){
//				getGimbalData()->yawAngleStopSet = 0.0f;
//				gimbalStopSwitch(DISABLE);
//				digitalIncreasing(&auxiliaryAutoData.schedule);	
//				}
//			}
//			break;
//		}			
//		case 19:{
//			if(PRESS_X){
//				digitalIncreasing(&auxiliaryAutoData.schedule);	
//			}			
//			break;
//		}
//		case 20:{
//			if(!PRESS_X){
//				PAW_ON;
//				//切换摄像头
//			    getGimbalData()->yawAngleStopSet = 175.0f;
//				getchassisData()->speedLimit = 0.2f;
//			    gimbalStopSwitch(ENABLE);
//				digitalIncreasing(&auxiliaryAutoData.schedule);	
//			}
//			break;		
//		}
//		case 21:{
//			if(PRESS_X){
//				digitalIncreasing(&auxiliaryAutoData.schedule);	
//			}
//			break;
//		}			
//		case 22:{
//			if(!PRESS_X){
//				PAW_OUT;
//				digitalIncreasing(&auxiliaryAutoData.countTimes);
//				if(auxiliaryAutoData.countTimes > 300){			
//					digitalClan(&auxiliaryAutoData.countTimes);
//					PAW_OFF;					
//					digitalIncreasing(&auxiliaryAutoData.schedule);	
//				}		
//			}
//			break;
//		}
//		case 23:{
//			if(PRESS_X){
//				digitalIncreasing(&auxiliaryAutoData.schedule);	
//			}
//			break;
//		}
//		case 24:{
//			if(!PRESS_X){
//				digitalHi(&auxiliaryDeformingData.Y_statebreak_Flag);
//				ADVANCE_TO_Y(Y_FRONT_GET);
//				if(!auxiliaryDeformingData.Y_statebreak_Flag){	
//					PAW_ON;	
//					digitalIncreasing(&auxiliaryAutoData.schedule);
//			    }
//			}
//			break;
//		}
//		case 25:{
//				digitalHi(&auxiliaryDeformingData.Y_statebreak_Flag);
//				BACK_TO_Y(Y_BEHIND_GET);
//				if(!auxiliaryDeformingData.Y_statebreak_Flag){	
//					digitalSet(&auxiliaryAutoData.schedule,97);
//			    }			
//			break;
//		}
//		case 95 :{
//				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);			
//				GO_TO_HEIGHT(FIFTH_HEIGHT_GET);
//				if(!auxiliaryDeformingData.heightstatebreak_Flag){
//					digitalIncreasing(&auxiliaryAutoData.schedule);
//				} 				
//			break;
//		}
//		case 96 :{
//			if(resetFrontClaw()){
//				digitalIncreasing(&auxiliaryAutoData.schedule);
//			}
//			break;
//		}
//        //任务执行结束
//		case 97 :{
//			digitalLo(&auxiliaryDeformingData.Y_PlaceInitFinishFlag);
//			digitalLo(&auxiliaryDeformingData.X_PlaceInitFinishFlag);
//			digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
//			GO_TO_HEIGHT(FOURTH_HEIGHT_GET);
//			if(!auxiliaryDeformingData.heightstatebreak_Flag){
//				digitalIncreasing(&auxiliaryAutoData.schedule);
//			}
//            break;
//		}
//		case 98:{
//			if((auxiliaryDeformingData.X_PlaceInitFinishFlag) && (auxiliaryDeformingData.Y_PlaceInitFinishFlag)){
//				if(auxiliaryAutoData.mineralnumber != 0){
//					digitalIncreasing(&auxiliaryAutoData.schedule);
//				}
//				else{
//					digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
//					GO_TO_HEIGHT(NORMAL_HEIGHT_GET);
//					if(!auxiliaryDeformingData.heightstatebreak_Flag){
//						digitalIncreasing(&auxiliaryAutoData.schedule);
//					}		
//				}
//			}
//			break;
//		}			
//        //任务执行结束
//		case 99:{
//			digitalIncreasing(&auxiliaryAutoData.countTimes);
//			if(auxiliaryAutoData.countTimes > 50){
//				getGimbalData()->yawAngleStopSet = 0.0f;
//				getchassisData()->speedLimit = 1.0f;
//				gimbalStopSwitch(DISABLE);
//				digitalClan(&auxiliaryAutoData.grabCountTimes);
//				digitalClan(&auxiliaryAutoData.grabSchedule);
//				digitalClan(&auxiliaryAutoData.countTimes);
//				auxiliaryAutoData.taskState = END_OF_EXECUTION;
//				usbVCP_Printf("In the end of ALL,FINISH \r\n");
//			}
//            break;	
//		}
//        default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;		
//	}
//}
/*
***************************************************
函 数 名：	auxiliaryRescueUpdate
功    能：   拖车救援任务更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备    注： （SHIFT）
***************************************************
*/
static void auxiliaryRescueUpdate(){
	
    	//如果有打断（中断）
	if(auxiliaryAutoData.breakSign){
		digitalLo(&auxiliaryAutoData.breakSign);
		auxiliaryAutoData.schedule = 99;														
	}
	//对位限速
	if(PRESS_CTRL){
		getchassisData()->speedLimit = 0.2f;
	}
	else
		if(!PRESS_CTRL){
			static uint16_t time;
			time ++ ;//开始计时
			if(time > 25){//50ms松开消抖
				time = 0;//下一次松开计时
				getchassisData()->speedLimit = 1.0f;
			}	
		}
    switch(auxiliaryAutoData.schedule){
        case 1:{	
            AUXILIARY_RESCUE_OUT;	
			break;
        }		
        case 99:{
				AUXILIARY_RESCUE_IN;
				auxiliaryAutoData.taskState = END_OF_EXECUTION;	
            break;
        }
        default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;	
	}
	auxiliaryAutoData.taskDuration += AUTO_TASK_DURATION;
}
/*
***************************************************
函 数 名：	auxiliarySwipeCardUpdate
功    能：  刷卡复活救援任务更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备    注： （Q）
***************************************************
*/
static void auxiliarySwipeCardUpdate(){
	  //如果有打断（中断）
	if(auxiliaryAutoData.breakSign){
		digitalLo(&auxiliaryAutoData.breakSign);
		auxiliaryAutoData.schedule = 99;														
	}
		//对位限速
	if(PRESS_CTRL){
		getchassisData()->speedLimit = 0.2f;
	}
	else
		if(!PRESS_CTRL){
			static uint16_t time;
			time ++ ;//开始计时
			if(time > 25){//50ms松开消抖
				time = 0;//下一次松开计时
				getchassisData()->speedLimit = 1.0f;
			}	
		}
    switch(auxiliaryAutoData.schedule){
        case 1:{	
            AUXILIARY_SWIPE_OUT;	
			break;
        }		
        case 99:{
			AUXILIARY_SWIPE_IN;
			auxiliaryAutoData.taskState = END_OF_EXECUTION;	
            break;
        }
        default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;	
	}
	auxiliaryAutoData.taskDuration += AUTO_TASK_DURATION;
	
}

/*
***************************************************
函 数 名：	auxiliaryProtectOutPostUpdate
功    能：   工程机器人挡前哨站模式更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备    注：
***************************************************
*/
static void auxiliaryProtectOutPostUpdate(){
	if(auxiliaryAutoData.breakSign ){
		digitalLo(&auxiliaryAutoData.breakSign);
		auxiliaryAutoData.schedule = 97;							//如果有打断则结束任务
	}
	//限速
	if(PRESS_CTRL){
		getchassisData()->speedLimit = 0.5f;
	}
	else
		if(!PRESS_CTRL){
			static uint16_t time;
			time ++ ;//开始计时
			if(time > 25){//50ms松开消抖
				time = 0;//下一次松开计时
				getchassisData()->speedLimit = 1.0f;
			}	
		}
    switch(auxiliaryAutoData.schedule){
		case 1:{			
			if(!PRESS_X){
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(SECOND_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);	 
			   }
			}
            break;
        }
		case 2:{
			//此处留出一个空闲序列
			break;
		}
        //任务执行结束
		case 97 :{
			PAW_IN;
			Y_BACK;
			digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);		
			GO_TO_HEIGHT(FOURTH_HEIGHT_GET);
			if(!auxiliaryDeformingData.heightstatebreak_Flag){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
            break;
		}
		case 98:{
			if(auxiliaryAutoData.mineralnumber != 0){
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
			else{
				digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
				GO_TO_HEIGHT(NORMAL_HEIGHT_GET);
				if(!auxiliaryDeformingData.heightstatebreak_Flag){
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}		
			}		
			break;
		}				
        //任务执行结束
		case 99:{
			digitalIncreasing(&auxiliaryAutoData.countTimes);
			if(auxiliaryAutoData.countTimes > 50){
				auxiliaryAutoData.taskState = END_OF_EXECUTION;
				usbVCP_Printf("In the end of ALL,FINISH \r\n");
			}
            break;	
		}
        default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
    }
	
	
}
/*
***************************************************
函 数 名：	auxiliaryAutoTaskUpdate
功    能：   工程机器人自动任务更新
入口参数：	无
返 回 值：	无
应用范围：	外部调用
备    注：
***************************************************
*/
void auxiliaryAutoTaskUpdate(void){
	//躲避任务  （INFANTRY_MANUAL）手动模式
//	auxiliaryAviodUpdate(); 
	//必须有可执行任务
	if(auxiliaryAutoData.currentTask != AUXILIARY_MANUAL) {				
		//如果任务刚刚开始执行
		if(auxiliaryAutoData.taskState == UNEXECUTED) {				
			//执行开始序列
			auxiliaryTaskBegin();																			
		}
		//如果在执行中
		else if(auxiliaryAutoData.taskState == EXECUTING) {					
			switch(auxiliaryAutoData.currentTask) {
                //V+X兑换站兑换矿石
                case AUXILIARY_EXCHANGE: {    
					auxiliaryExchangeUpdata();															
					break;
				}
				//X:挡前哨战模式
				case AUXILIARY_PROTECT_OUTPOST:{
					auxiliaryProtectOutPostUpdate();
					break;
				}
                //SHIFT :拖车
                case AUXILIARY_RESCUE:{                                                        
                    auxiliaryRescueUpdate();
                    break;
                }
				//Q：刷卡复活
				case AUXILIARY_SWIPE_CARD:{
					auxiliarySwipeCardUpdate();
					break;
				}
				//C+X:小资源岛抓矿
				case AUXILIARY_SMALL_RESOURSE:{
					auxiliaryGrabSmallUpdata();
					break;
				}
				//Z+X:大资源岛抓矿
				case AUXILIARY_BIG_RESOURSE:{
					auxiliaryGrabBigUpdata();
					break;	
				}
//				//E:(+X)底盘小爪子（R抓G放）
//				case AUXILIARY_FRONT_CLAW:{
//					auxiliaryfrontClawUpdate();
//					break;
//				}
                //如果是其他命令，则直接重新初始化结构体
                default: {																							
					autoDataInit(&auxiliaryAutoData); 
				break;
                }
			}
		}
		else{
			//如果执行完毕或执行错误，则重新
			autoDataInit(&auxiliaryAutoData);						
		}
	}
}
