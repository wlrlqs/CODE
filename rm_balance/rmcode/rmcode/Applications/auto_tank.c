#include "config.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "deforming.h"
#include "vision.h"
#include "auto_tank.h"
#include "rc.h"
#include "DRIVER_VL53L0X.h"
#include "keyboard.h"
#include "judge.h"
AutoTaskStruct_t tankAutoData;
ftankfricFlagStruct_t ftankFricState;
AutoTaskStruct_t* getTankAutoData(){
        return &tankAutoData;
}
ftankfricFlagStruct_t* getftankFricState(void){
	  return &ftankFricState;
}

void tankTaskBegin(void){
	//标记为进行中
	tankAutoData.taskState = EXECUTING;						
	//给执行序列加一		
	digitalIncreasing(&tankAutoData.schedule);					
}
//在比赛开始和复活时自动开启摩擦轮，并可通过单机右键开关摩擦轮
void ftankChangeFricState(void){
	//如果处于比赛状态，并且血量不为0
	if((judgeData.extGameRobotState.remain_HP != 0) ){
		//如果摩擦轮处于关闭状态，则打开摩擦轮，否则关闭摩擦轮
		if(ftankFricState.ftankFricStateFlag == 0)  
			ftankFricState.ftankFricStateFlag = 1;
		else ftankFricState.ftankFricStateFlag = 0;
	}
}

/* p_tank Hand over to auxiliary */
void p_tankBulletAroundUpdate(void){
    switch(tankAutoData.schedule){                                              //用schedule来增加、减少和判断任务进度
        case 0:{                     
            if(tankAutoData.keyboard_X_Flag){
                digitalIncreasing(&tankAutoData.schedule);
                digitalLo(&tankAutoData.keyboard_X_Flag);
            }
            else if(TASK_PRESS_X){
                digitalHi(&tankAutoData.keyboard_X_Flag);
            }
        }break;
        case 1:{
					getshootData()->bigsupplySpeedOut = -3500;
					digitalIncreasing(&tankAutoData.loadtime);
                if(tankAutoData.loadtime >  0X490){
									digitalClan(&tankAutoData.loadtime);
									digitalIncreasing(&tankAutoData.schedule);
								}
        }break;
        case 2:{
            if(TASK_PRESS_X){
						getshootData()->bigsupplySpeedOut = 3500;
                    PAout(2)=0;
                    PAout(3)=0;
                if(tankAutoData.loadtime >  0x090){
								digitalSet(&tankAutoData.schedule,99);
									digitalClan(&tankAutoData.loadtime);
                    digitalLo(&tankAutoData.keyboard_X_Flag);
                }
                digitalIncreasing(&tankAutoData.loadtime);
            }
//            else if(TASK_PRESS_X){
//                digitalHi(&tankAutoData.keyboard_X_Flag);
//            }
        }break;

                
        case 99:{
            digitalClan(&tankAutoData.schedule);
            digitalLo(&tankAutoData.keyboard_X_Flag);
            digitalClan(&tankAutoData.loadtime);
            digitalLo(&P_HERO_42_LID);       //关闭大弹仓
						getshootData()->bigsupplySpeedOut = -3500;
        break;
        }

    }
}

/* p_tank mortar */
void p_tankMortarUpdate(void){
	//如果有打断（中断）
	if(tankAutoData.breakSign){													
		//直接跳到最后一步
		tankAutoData.schedule = 4;												
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_CTRL && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 3;
	}
	//用schedule来增加、减少和判断任务进度
	switch(tankAutoData.schedule){											
		case 1: {
            gimbalStopSwitch(DISABLE);
			gimbalSwitch(DISABLE);
			digitalIncreasing(&tankAutoData.schedule);
		} break;
		case 2:{ 		 
			if(getGimbalData()->initFinishFlag){
			digitalHi(&getGimbalData()->followLock);
			gimbalSwitch(ENABLE);
			getchassisData()->ctrlMode = CHASSIS_STOP;
			digitalIncreasing(&tankAutoData.schedule);
			}
		} break;
		case 3:{ 	
			//迫击炮模式维持20s
			if(tankAutoData.taskDuration < 30.0f){			
			visionMortar();
				//右键带预判打击	
				if(KB_PJEJUDGMENT){																
					getvisionData()->manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
					getvisionData()->manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE;
				}
			}
			else{
				digitalIncreasing(&tankAutoData.schedule);
			}
		} break;
		case 4: {
			gimbalStopSwitch(DISABLE);
			gimbalSwitch(DISABLE);
			getvisionData()->prejudgFlag = false;	
			shootDataReset();	
			digitalLo(&getGimbalData()->followLock);
			digitalClan(&getvisionData()->manualPitchBias);
			digitalClan(&getvisionData()->manualYawBias);
			tankAutoData.schedule = 99;
		}  break;			 
		//只有调到99时才能退出
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						
		//如果到达列表中没有进度，则任务出错
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}



/* p_tank Hand over to depot */
void p_tankBulletSupplyUpdate(void){
	//如果有打断（中断）
	if(tankAutoData.breakSign){													
		//直接跳到最后一步
		tankAutoData.schedule = 3;												
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_R && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 1;
	}
	//用schedule来增加、减少和判断任务进度
	switch(tankAutoData.schedule){											
		case 1: 
			if(tankAutoData.taskDuration < 1.0f){
				//开启小弹仓
				digitalHi(&P_HERO_17_LID);      
				//加速过程和最大速度都调整到原来的50%
				getchassisData()->speedLimit = 0.5f;								
			}
			else{
				digitalIncreasing(&tankAutoData.schedule);
			}
			break;
		case 2: 	
			//30秒内完成补弹
			if(tankAutoData.taskDuration > 30.0f){			
				digitalIncreasing(&tankAutoData.schedule);
			}
			break;
		case 3:
			digitalLo(&P_HERO_42_LID);      						//关闭大弹仓
			digitalLo(&P_HERO_17_LID);      						//关闭小弹仓
			getchassisData()->speedLimit = 1.0f;							//调整回原来的值
			tankAutoData.schedule = 99;
			break;
		//只有调到99时才能退出
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;	
		//如果到达列表中没有进度，则任务出错
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}



/*p_tank Switching in 17mm bullet and 42mm bullet */
void p_tankChangeHitModeUpdate(void){									
	if(TASK_PRESS_Z ){
		digitalTogg(&getshootData()->p_tankshootmode);
	}
}

/*p_tank automatic aiming */
void p_tankAutomaticAimUpdate(void){										
	//用schedule来增加、减少和判断任务进度
	switch(tankAutoData.schedule){											
		case 1: gimbalSwitch(DISABLE);												
				chassisSwitch(DISABLE);
				shootVisionInit();
				//配置TX2数据
				visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,BIG_BULLET);							
				digitalIncreasing(&(tankAutoData.schedule));	
				break;
		case 2: gimbalSwitch(ENABLE);	
						//自瞄模式决策//右键带预判打击
						if(KB_PJEJUDGMENT){												
							getvisionData()->prejudgFlag = true;						
							getvisionData()->manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE * 0.5f;
							getvisionData()->manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
						}
						//左键 不带预判打击
						else{																			
							getvisionData()->prejudgFlag = false;											
							digitalClan(&getvisionData()->manualPitchBias);
							digitalClan(&getvisionData()->manualYawBias);
						}
	
						if(tankAutoData.breakSign){
							//如果有打断则结束任务
							tankAutoData.schedule = 99;							
							getvisionData()->prejudgFlag = false;	
							shootDataReset();	
						}			
						break;
		//只有调到99时才能退出
		case 99: tankAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  
		//如果到达列表中没有进度，则任务出错
		default: tankAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/*p_tank automatic Aviod */
void p_tankAviodUpdate(void){
	static uint8_t avoidTurn = 0; 
	if(tankAutoData.avoidTask){
		if(avoidTurn){
			if(getchassisData()->chaseRef < AVOID_RANGE)
				getchassisData()->chaseRef += AVOID_RATE;
			else if(getchassisData()->chaseRef > AVOID_RANGE){
				getchassisData()->chaseRef -= AVOID_RATE;
				avoidTurn = 0;
			}
		}
		else{
			if(getchassisData()->chaseRef > -AVOID_RANGE)
				getchassisData()->chaseRef -= AVOID_RATE;
			else if(getchassisData()->chaseRef < -AVOID_RANGE){
				getchassisData()->chaseRef += AVOID_RATE;
				avoidTurn = 1;
			}
		}
		//如果计时大于6.0s的时间，则将当前任务结束
		if(tankAutoData.aviodDuration > R_TIME){					
			getchassisData()->chaseRef = 0.0f;
			tankAutoData.avoidTask = DISABLE;
		}	
		tankAutoData.aviodDuration += AUTO_TASK_DURATION;	
	}
	else{	
	//大陀螺旋转躲避
	        static uint8_t lastKeySate,kmRotateFlag,rcRotateFlag,rotateDirection;
		       if(RC_ROTATE > 100)
				  rcRotateFlag = 1;
			   else
				rcRotateFlag = 0;
			   //一键按下旋转
			   if(!lastKeySate && !kmRotateFlag){																	
				    if(PRESS_Q){
					    kmRotateFlag = 1;
					    //下一次自转改变转向
					    rotateDirection = !rotateDirection;
				    }
			   }
			   else{
				//再次按下解除旋转
				    if(!lastKeySate && kmRotateFlag){
					     if(PRESS_Q || TASK_PRESS_F)
						kmRotateFlag = 0;
				     }
				      //死亡自动解除小陀螺
				    if(tankAutoData.closeRotate){
					    kmRotateFlag = 0;
					    tankAutoData.closeRotate = false;
				     }
			  }
			  lastKeySate = PRESS_Q;
			
			   if(kmRotateFlag || rcRotateFlag){
				   if(rotateDirection)
				       getchassisData()->chaseRef -= (AVOID_RATE * 4);
				   else
					   getchassisData()->chaseRef += (AVOID_RATE * 4);
				       tankAutoData.rotateFlag = true;
				       //陀螺旋转时间
				       tankAutoData.rotateTime += RTIMEFEQ;
				        //陀螺旋转速度比
				       getchassisData()->rotate = (f32_t)0.6*sinf(tankAutoData.rotateTime)+1.2f;
				       //速度比限幅
				       getchassisData()->rotate = SINF_RUDD(getchassisData()->rotate);
			   }
	
			else{
				getchassisData()->chaseRef = 0.0f;
				tankAutoData.rotateFlag = false;
				getchassisData()->rotate = 1.0f;
				digitalClan(&tankAutoData.rotateTime);
			}
	}
}

/*p_tank Suicide fire*/
void p_tankSuicideFireUpdate(void){
	if(PRESS_C){
		digitalHi(&getshootData()->suicideFireFlag);
	}
	else{
		digitalLo(&getshootData()->suicideFireFlag);
	}
}
/* p_tank Autotask list*/
AutotankStruct_t tankAutoSerial = {
	p_tankBulletAroundUpdate,
	p_tankMortarUpdate,
	p_tankBulletSupplyUpdate,
	p_tankChangeHitModeUpdate,
	p_tankAutomaticAimUpdate,
	p_tankAviodUpdate,
	p_tankSuicideFireUpdate,
	p_tankSuicideFireUpdate,
};
void p_tankAutoTaskUpdate(void){
     super_capacitorTask();
	//躲避任务（和步兵躲避任务相同）
	tankAutoSerial.p_tank_q_task();																	
	//自杀开火模式更新
	tankAutoSerial.p_tank_c_task();													                                   
	//必须有可执行任务  TANK_MANUAL（手动)
	if(tankAutoData.currentTask != TANK_MANUAL){										
		//如果任务刚刚开始执行
		if(tankAutoData.taskState == UNEXECUTED){										
			//执行开始序列
			tankTaskBegin();																
		}
		//如果在执行中
		else if(tankAutoData.taskState == EXECUTING){		
			switch(tankAutoData.currentTask){

				//42mm弹药交接
				case TANK_BULLET_TRANSFER: {	
					tankAutoSerial.p_tank_x_task();
					break;
				}
				//迫击炮模式
				case TANK_MORTAR: {														
					tankAutoSerial.p_tank_ctrl_task();
					break;
				}
				//17mm弹药补给站交互
				case TANK_BULLET_SUPPLY: {									   
					tankAutoSerial.p_tank_r_task();
					break;
				}
				//切换打击模式
				case TANK_CHANGE_HIT_MODE: {									 
					tankAutoSerial.p_tank_z_task();
					break;
				}	
				//辅助瞄准
				case TANK_AUTOMATIC_AIM: {										
					tankAutoSerial.p_tank_v_task();
					break;
				}
				//如果是其他命令，则直接重新初始化结构体
				default: {																	
					//将数据清零			
					autoDataInit(&tankAutoData); 										
					break;
				}
			}
		}
		//如果执行完毕或执行错误，则重新
		else{																									
			autoDataInit(&tankAutoData);
		}
	}
}
/******************************************************/
/*****This is an boundary between two type of tank*****/
/******************************************************/
void f_tankTaskBegin(void){
	//标记为进行中
	tankAutoData.taskState = EXECUTING;						
	//给执行序列加一		
	digitalIncreasing(&tankAutoData.schedule);					
}
/* 辅助打击&自瞄任务更新 */
void f_tankAutomaticAimUpdate(void){										
	//用schedule来增加、减少和判断任务进度
	switch(tankAutoData.schedule){											
		case 1: gimbalSwitch(DISABLE);												
				chassisSwitch(DISABLE);
				shootVisionInit();
				//配置TX2数据
				visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,BIG_BULLET);							
				digitalIncreasing(&(tankAutoData.schedule));	
				break;
		case 2: gimbalSwitch(ENABLE);	
						//自瞄模式决策//右键带预判打击
						if(KB_PJEJUDGMENT){												
							getvisionData()->prejudgFlag = true;						
							getvisionData()->manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE * 0.5f;
							getvisionData()->manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
						}
						//左键 不带预判打击
						else{																			
							getvisionData()->prejudgFlag = false;											
							digitalClan(&getvisionData()->manualPitchBias);
							digitalClan(&getvisionData()->manualYawBias);
						}
	
						if(tankAutoData.breakSign){
							//如果有打断则结束任务
							tankAutoData.schedule = 99;							
							getvisionData()->prejudgFlag = false;	
							shootDataReset();	
						}			
						break;
		//只有调到99时才能退出
		case 99: tankAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  
		//如果到达列表中没有进度，则任务出错
		default: tankAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* 自动掉头42mm弹药交接任务更新 */
void f_tankBulletAroundUpdate(void){
 if(tankAutoData.breakSign){
	 tankAutoData.schedule = 4;
	 digitalLo(&tankAutoData.breakSign);
 }
 
    switch(tankAutoData.schedule){                                              //用schedule来增加、减少和判断任务进度
        case 1: 
		   digitalHi(&tankAutoData.get_pill_Flag);
           getshootData()->supplyRef = -2000;
           getGimbalData()->chassisChange = getConfigData()->yawCenter + 8192;
		   getchassisData()->direction = -1;
		   digitalIncreasing(&tankAutoData.schedule);
		   break;
        case 2:	
		   if(tankAutoData.taskDuration > 1.0f){				  
			   getshootData()->supplyRef = -150;
			   digitalIncreasing(&tankAutoData.schedule);
		   }
           break;
        case 3:
            if(TASK_PRESS_R){
                digitalIncreasing(&tankAutoData.schedule);
			} 		
            break;
        case 4:    
            getshootData()->supplyRef = 2000; 
			getGimbalData()->chassisChange = getConfigData()->yawCenter;
		    getchassisData()->direction = 1;
		    digitalLo(&tankAutoData.get_pill_Flag);
            digitalClan(&tankAutoData.taskDuration);	
         	digitalIncreasing(&tankAutoData.schedule);	
            break;
        case 5:
		    if(tankAutoData.taskDuration > 1.0f){		
                getshootData()->supplyRef = 150;
 		        digitalSet(&tankAutoData.schedule,99);
			}
			break;
		case 99:			
			tankAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);	    
		    break;						 
		//如果到达列表中没有进度，则任务出错
		default: 
			tankAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);
		    break;	
    }
	tankAutoData.taskDuration+=AUTO_TASK_DURATION;	
}

/* 摇摆躲避子弹任务更新 */
void f_tankAviodUpdate(void){
	static uint8_t avoidTurn = 0;
	//扭腰加小陀螺
	if(tankAutoData.avoidTask){
		switch(tankAutoData.avoidSchedule){
			case 0:
				// 车体以45度角迎敌
				if(getchassisData()->chaseRef < AVIOD_INITIAL_ANGEL)				
					getchassisData()->chaseRef += AVOID_RATE;
				else
					digitalIncreasing(&tankAutoData.avoidSchedule);
				break;
			case 1:
				if(avoidTurn){
					//120
					if(getchassisData()->chaseRef < (AVIOD_INITIAL_ANGEL+AVOID_RANGE))			
						getchassisData()->chaseRef += AVOID_RATE;
					else if(getchassisData()->chaseRef >= (AVIOD_INITIAL_ANGEL+AVOID_RANGE)){
						getchassisData()->chaseRef -= AVOID_RATE;
						digitalLo(&avoidTurn);
					}
				}
				else{
					//-30
					if(getchassisData()->chaseRef > (AVIOD_INITIAL_ANGEL-AVOID_RANGE))			
						getchassisData()->chaseRef -= AVOID_RATE;
					else if(getchassisData()->chaseRef <= (AVIOD_INITIAL_ANGEL-AVOID_RANGE)){
						getchassisData()->chaseRef += AVOID_RATE;
						digitalHi(&avoidTurn);
					}
				}
			 break;
		}
		if(tankAutoData.breakSign){							 
			getchassisData()->chaseRef = 0.0f;
			tankAutoData.aviodFlag = false;
			//在扭腰结束后能保证下次扭腰先45度角迎敌
			digitalClan(&tankAutoData.avoidSchedule);							 
			tankAutoData.avoidTask = DISABLE;
			digitalLo(&tankAutoData.breakSign);
		}	
	}
	else{	
	//大陀螺旋转躲避
	        static uint8_t lastKeySate,kmRotateFlag,rcRotateFlag,rotateDirection;
		       if(RC_ROTATE > 100)
				  rcRotateFlag = 1;
			   else
				rcRotateFlag = 0;
			   //一键按下旋转
			   if(!lastKeySate && !kmRotateFlag){																	
				    if(PRESS_Q){
					    kmRotateFlag = 1;
					    //下一次自转改变转向
					    rotateDirection = !rotateDirection;
				    }
			   }
			   else{
				//再次按下解除旋转
				    if(!lastKeySate && kmRotateFlag){
					     if(PRESS_Q || TASK_PRESS_F)
						kmRotateFlag = 0;
				     }
				      //死亡自动解除小陀螺
				    if(tankAutoData.closeRotate){
					    kmRotateFlag = 0;
					    tankAutoData.closeRotate = false;
				     }
			  }
			  lastKeySate = PRESS_Q;
			
			   if(kmRotateFlag || rcRotateFlag){
				   if(rotateDirection)
				       getchassisData()->chaseRef -= (AVOID_RATE * 4);
				   else
					   getchassisData()->chaseRef += (AVOID_RATE * 4);
				       tankAutoData.rotateFlag = true;
				       //陀螺旋转时间
				       tankAutoData.rotateTime += RTIMEFEQ;
				        //陀螺旋转速度比
				       getchassisData()->rotate = (f32_t)0.6*sinf(tankAutoData.rotateTime)+1.2f;
				       //速度比限幅
				       getchassisData()->rotate = SINF_RUDD(getchassisData()->rotate);
			   }
	
			else{
				getchassisData()->chaseRef = 0.0f;
				tankAutoData.rotateFlag = false;
				getchassisData()->rotate = 1.0f;
				digitalClan(&tankAutoData.rotateTime);
			}
	}
}

///* 快速掉头任务更新 */
//void tankTurnAroundUpdate(void){
//	//如果有打断（中断）
//	if(tankAutoData.breakSign){													
//		//直接跳到最后一步
//		tankAutoData.schedule = 3;												
//		digitalLo(&tankAutoData.breakSign);
//	}
//	//用schedule来增加、减少和判断任务进度
//	switch(tankAutoData.schedule){											
//		case 1:{
//			//快速起步
//			autoTaskData->fastSeed = 1;									
//			//车体旋转180°
//			getGimbalData()->yawAngleRef -= 180.0f;						
//			digitalIncreasing(&tankAutoData.schedule);
//		}
//		break;
//		case 2:{
//			if(tankAutoData.taskDuration > 0.5f){
//				//3秒内完成掉头
//				if(tankAutoData.taskDuration < 3.0f){			
//					//掉头完成
//					if(getGimbalData()->yawAngleFbd > getGimbalData()->yawAngleRef - 5.0f || getGimbalData()->yawAngleFbd < getGimbalData()->yawAngleRef + 5.0f){    
//						autoTaskData->fastSeed = 0;
//						tankAutoData.schedule = 99;
//					}
//				}
//				else
//					digitalIncreasing(&tankAutoData.schedule);
//			}
//		}		
//		break;
//		case 3:
//			//立即停止旋转
//			getGimbalData()->yawAngleRef = getGimbalData()->yawAngleFbd;					
//			autoTaskData->fastSeed = 0;
//			tankAutoData.schedule = 99;
//			break;
//		//只有调到99时才能退出
//		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						
//		//如果到达列表中没有进度，则任务出错
//		default: tankAutoData.taskState = EXECUTION_ERROR; break;							
//	}
//	tankAutoData.taskDuration += AUTO_TASK_DURATION;
//}
/* 自杀开火模式更新 */
void tankSuicideFireUpdate(void){
	if(PRESS_C){
		digitalHi(&getshootData()->suicideFireFlag);
	}
	else{
		digitalLo(&getshootData()->suicideFireFlag);
	}
}
void f_tankSuicideFireUpdate(void){
	if(PRESS_C){
		digitalHi(&getshootData()->suicideFireFlag);
	}
	else{
		digitalLo(&getshootData()->suicideFireFlag);
	}
}

void f_tankAutoTaskUpdate(void){
    		//如果处于比赛状态，并且刚刚复活时
	if((judgeData.extGameRobotState.remain_HP!=0) && (ftankFricState.ftankFricNumFlag==1)){
		//复活时执行，下次复活之前不需再执行该if的内容
		ftankFricState.ftankFricNumFlag = 0;
		//打开摩擦轮
		ftankFricState.ftankFricStateFlag = 1;
	}
	//如果处于比赛状态，并且死亡时
	else if((judgeData.extGameRobotState.remain_HP==0) && (ftankFricState.ftankFricNumFlag==0)){
		//已死亡，等待复活时打开摩擦轮
		ftankFricState.ftankFricNumFlag = 1;
		//关闭摩擦轮
		ftankFricState.ftankFricStateFlag = 0;	
	}
      //开关摩擦轮
  if(keyBoardCtrlData.rkSta == KEY_PRESS_ONCE){
		ftankChangeFricState();
	}
     super_capacitorTask();
	//躲避任务（和步兵躲避任务相同）
	f_tankAviodUpdate();																	
	//自杀开火模式更新
	f_tankSuicideFireUpdate();													                                   
	//必须有可执行任务  TANK_MANUAL（手动)
	if(tankAutoData.currentTask != TANK_MANUAL){										
		//如果任务刚刚开始执行
		if(tankAutoData.taskState == UNEXECUTED){										
			//执行开始序列
			f_tankTaskBegin();																
		}
		//如果在执行中
		else if(tankAutoData.taskState == EXECUTING){		
			switch(tankAutoData.currentTask){
				//辅助瞄准
				case F_TANK_AUTOMATIC_AIM: 										
				   f_tankAutomaticAimUpdate();			      
				   break;
				//42mm弹药交接
				case F_TANK_BULLET_TRANSFER:					   
				   f_tankBulletAroundUpdate();					
				   break;
				//如果是其他命令，则直接重新初始化结构体
				default: 																	
					//将数据清零			
				   autoDataInit(&tankAutoData); 															
				   break;
		    }
		}
		//如果执行完毕或执行错误，则重新
		else{																									
			autoDataInit(&tankAutoData);
		}
	}
}

