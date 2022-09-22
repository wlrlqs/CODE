#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "vision.h"
#include "rc.h"
#include "auto_infantry.h"
#include "config.h"
#include "imu.h"
#include "motorHandle.h"

fricFlagStruct_t infantryFricState;
AutoTaskStruct_t infantryAutoData;
infantry_deformingStruct_t infantry_deforming;
changeChassisTaskStruct_t changeChassisTaskData;         //更改底盘数据
infantry_deformingStruct_t *get_infDeforming(void){
	return &infantry_deforming;
}

fricFlagStruct_t* getinfantryFricState(void){
	  return &infantryFricState;
}

AutoTaskStruct_t* getinfantryAutoData(){
    return &infantryAutoData;
}
void infantryTaskBegin(void){
	//标记为进行中
	infantryAutoData.taskState = EXECUTING;								
	//给执行序列加一		
	digitalIncreasing(&infantryAutoData.schedule);	
}
//辅助打击/自瞄任务更新

void infantryAutomaticAimUpdate(void){		
	//用schedule来增加、减少和判断任务进度
	switch(infantryAutoData.schedule){													
		case 1: gimbalSwitch(DISABLE);												
            chassisSwitch(DISABLE);
				//配置TX2数据
				visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);
        digitalIncreasing(&(infantryAutoData.schedule));	
            break;
		case 2: gimbalSwitch(ENABLE);	
            shootVisionInit();
            //自瞄模式决策//右键 带预判打击
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
									
            if(infantryAutoData.breakSign){
                //如果有打断则结束任务
                infantryAutoData.schedule = 99;									
                getvisionData()->prejudgFlag = false;
                shootDataReset();	
            }			
            break; 
		//只有调到99时才能退出
		case 99: infantryAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						 
		//如果到达列表中没有进度，则任务出错
		default: infantryAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;
}

//辅助打击/自瞄任务更新
void infantryAutomaticBuffUpdate(void){										
	static shootMode_e originalShootMode = MANUAL_SINGLE;
	//用schedule来增加、减少和判断任务进度
	switch(infantryAutoData.schedule){													
		case 1: gimbalSwitch(DISABLE);
            chassisSwitch(DISABLE);
            originalShootMode = getshootData()->shootMode_17mm;
			if(infantryAutoData.currentTask == INFANTRY_ACTIVE_BUFF)
				//配置TX2数据
				visionSendDataUpdate(TX2_DISTINGUISH_BUFF,SMALL_BULLET);
			else
				visionSendDataUpdate(TX2_DISTINGUISH_BIGBUFF,SMALL_BULLET);
            digitalIncreasing(&(infantryAutoData.schedule));	
            break;
		case 2: gimbalSwitch(ENABLE);	
            chassisSwitch(ENABLE);
			//底盘暂停
			digitalHi(&getchassisData()->suspend_chassis);					
            shootVisionInit();
            getshootData()->shootMode_17mm = MANUAL_SINGLE;
            //自瞄模式决策//左键
            if(KB_NO_PJEJUDGMENT){														
                getvisionData()->prejudgFlag = false;														
            }
            
            //按下W/S/A/D进入调整模式
			if(!remoteControlData.dt7Value.keyBoard.bit.W && \
				!remoteControlData.dt7Value.keyBoard.bit.S && \
				!remoteControlData.dt7Value.keyBoard.bit.A && \
				!remoteControlData.dt7Value.keyBoard.bit.D){
					getvisionData()->buff_mod = NO_STATE;
					infantryAutoData.waitTime = 0;
					infantryAutoData.keyRelease = false;
					
			}
			else{
				if(remoteControlData.dt7Value.keyBoard.bit.W && !infantryAutoData.keyRelease){
					getvisionData()->buff_mod = W_STATE;
					infantryAutoData.waitTime++;
					if(infantryAutoData.waitTime >= 10){
						getvisionData()->buff_mod = NO_STATE;
						infantryAutoData.keyRelease = true;
					}
				}
				else if(remoteControlData.dt7Value.keyBoard.bit.S && !infantryAutoData.keyRelease){
					getvisionData()->buff_mod = S_STATE;
					infantryAutoData.waitTime++;
					if(infantryAutoData.waitTime >= 10){
						getvisionData()->buff_mod = NO_STATE;
						infantryAutoData.keyRelease = true;
					}
				}
				else if(remoteControlData.dt7Value.keyBoard.bit.A && !infantryAutoData.keyRelease){
					getvisionData()->buff_mod = A_STATE;
					infantryAutoData.waitTime++;
					if(infantryAutoData.waitTime >= 10){
						getvisionData()->buff_mod = NO_STATE;
						infantryAutoData.keyRelease = true;
					}
				}
				else if(remoteControlData.dt7Value.keyBoard.bit.D && !infantryAutoData.keyRelease){
					getvisionData()->buff_mod = D_STATE;
					infantryAutoData.waitTime++;
					if(infantryAutoData.waitTime >= 10){
						getvisionData()->buff_mod = NO_STATE;
						infantryAutoData.keyRelease = true;
					}
				}
			}
            					
            if(infantryAutoData.breakSign){
                //如果有打断则结束任务
                infantryAutoData.schedule = 99;
				digitalLo(&getchassisData()->suspend_chassis);
                getvisionData()->prejudgFlag = false;
                getshootData()->shootMode_17mm = originalShootMode;
                shootDataReset();	
            }			
            break; 
		//只有调到99时才能退出
		case 99: infantryAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						 
		//如果到达列表中没有进度，则任务出错
		default: infantryAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;
}

//子弹交接任务更新（拨弹）
void infantryBulletTransferUpdate(void){									
	//如果有打断（中断）
	if(infantryAutoData.breakSign){		
		//翻转机构速度期望值				
		getshootData()->supplyRef = 2000;		
		infantryAutoData.taskDuration = 62.1f;
		//直接跳到最后一步
		infantryAutoData.schedule = 4;	
		getshootData()->supplyPlaceFlag = supply_close;
		digitalLo(&infantryAutoData.breakSign) ; 
	}
	//按B开弹仓
	if((getshootData()->supplyNumFlag != 1) && infantryAutoData.taskDuration <60.0f ){ 
		getshootData()->supplyNumFlag = 1;
		infantryAutoData.taskDuration = 0.0f;
		infantryAutoData.schedule = 2;
	}
	//用schedule来增加、减少和判断任务进度
	switch(infantryAutoData.schedule){
		case 1:
			getshootData()->supplyRef = 80;
			break;
		case 2: 
			if(infantryAutoData.taskDuration < 0.3f){
				getshootData()->supplyRef = -2000;
				getshootData()->supplyPlaceFlag = supply_move;
				//加速过程和最大速度都调整到原来的40%
				getchassisData()->speedLimit = 0.4f;								
			}
			else{
				getshootData()->supplyRef = -80;
				getshootData()->supplyPlaceFlag=supply_open;
				digitalIncreasing(&infantryAutoData.schedule);
			}
			break;
		case 3: 
			//这里改成了30秒
			if(infantryAutoData.taskDuration > 62.0f){		
				getshootData()->supplyRef = 2000;
				getshootData()->supplyPlaceFlag = supply_move;
				digitalIncreasing(&infantryAutoData.schedule);
			}
			 break;
		case 4:
			//调整回原来的值
			getchassisData()->speedLimit = 1.0f;				
			//2秒钟时间关闭	  
			if(infantryAutoData.taskDuration > 62.3f){			
				getshootData()->supplyRef = 80;
				getshootData()->supplyPlaceFlag = supply_close;
				infantryAutoData.schedule = 99;
			}
			break;
		//只有调到99时才能退出
		case 99: 
			infantryAutoData.taskState = END_OF_EXECUTION; 
		  getshootData()->supplyNumFlag = 0;
		break;						
		//如果到达列表中没有进度，则任务出错
		default: infantryAutoData.taskState = EXECUTION_ERROR; break;							
	}	
	//计时
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;	
}

void infantryAviodUpdate(void){													 
	static int8_t sign = 1;
	//扭腰加小陀螺
	if(infantryAutoData.avoidTask){                    //按下E的时候使能
		static uint8_t lastKeyE;
		if(!lastKeyE && !infantryAutoData.aviodFlag && getrobotMode() != MODE_RELAX){																	
				if(PRESS_E){
					infantryAutoData.aviodFlag = 1;
					//下一次自转改变转向
				}
			}
		else{ 
			//再次按下解除旋转
			if(lastKeyE && infantryAutoData.aviodFlag){
				if(PRESS_E || TASK_PRESS_F)
					infantryAutoData.aviodFlag = 0;
			}
			//死亡自动解除小陀螺
			if(infantryAutoData.closeRotate){
				infantryAutoData.aviodFlag = 0;				
				infantryAutoData.closeRotate = false;
			}
		}
		lastKeyE = PRESS_E;
	if(PRESS_Q){
		infantryAutoData.avoidTask = DISABLE;
		autoTaskData->aviodFlag = false;		 
	 }		
	}
	else{	
	//小陀螺旋转躲避
	static uint8_t lastKeySate,kmRotateFlag,rcRotateFlag,rotateDirection;
		if(RC_ROTATE > 100)
				rcRotateFlag = 1;
			else
				rcRotateFlag = 0;
		if(RC_ROTATE < -100)
				infantryAutoData.aviodFlag = 1;
			else
				infantryAutoData.aviodFlag = 0;		
			//一键按下旋转                                ≠下控
			if(!lastKeySate && !kmRotateFlag && getrobotMode() != MODE_RELAX){																	
				if(PRESS_Q){
					kmRotateFlag = 1;
					//下一次自转改变转向
					infantryAutoData.km_rotate = 1;
					rotateDirection = !rotateDirection;
				}
			}
			else{ 
				//再次按下解除旋转
				if(!lastKeySate && kmRotateFlag){
					if(PRESS_Q || TASK_PRESS_F)
						kmRotateFlag = 0;
						infantryAutoData.km_rotate = 0;
				}
				//死亡自动解除小陀螺
				if(infantryAutoData.closeRotate){
					kmRotateFlag = 0;
					infantryAutoData.km_rotate = 1;					
					infantryAutoData.closeRotate = false;
				}
			}
			
			lastKeySate = PRESS_Q;
			
			if(kmRotateFlag || rcRotateFlag){
				if(rotateDirection)
				 getchassisData()->chaseRef -= (AVOID_RATE * 4);
				else
					getchassisData()->chaseRef += (AVOID_RATE * 4);
							
				infantryAutoData.rotateFlag = true;
				
				//陀螺旋转时间
				infantryAutoData.rotateTime += RTIMEFEQ;
				
				//陀螺旋转速度比
				getchassisData()->rotate = (f32_t)0.6*sinf(infantryAutoData.rotateTime)+1.5f;
				
				//速度比限幅
				getchassisData()->rotate = SINF_RUDD(getchassisData()->rotate);
				
#if ROTATE_ORIENT
				//开启陀螺结束指向
				infantryAutoData.rotateEnb = true;		
#endif
			}
			else{
#if !ROTATE_ORIENT
				getchassisData()->chaseRef = 0.0f;
#endif
				infantryAutoData.rotateFlag = false;
				getchassisData()->rotate = 1.0f;
				digitalClan(&infantryAutoData.rotateTime);
			}
		}
}
//陀螺结束云台指向
void rotateOff(void){
	int16_t rotateCode;
	//消除零偏位
	rotateCode = motorHandleClass.Encoder(&commandoMotorConfig[YAWMOTOR]) - getConfigData()->yawCenter;
	//负数处理
	if(T_OR_F0(rotateCode)) rotateCode = 16384 + rotateCode;
	//指向开
	if(infantryAutoData.rotateEnb){
		//指向只有在陀螺任务结束后进行
		if(!infantryAutoData.rotateFlag){
			switch(infantryAutoData.enbStep){
				case 0:{
					//云台YAW当前位置位于前装甲板±90°范围内
					if(rotateCode <= 4096 || rotateCode > 12288){
						//枪口指向车头
						getGimbalData()->chassisChange = getConfigData()->yawCenter;
						getchassisData()->roDir = 0x00;
					}
					//云台YAW当前位置位于后装甲板±90°范围内
					else if(rotateCode > 4096 && rotateCode <= 12288){
						//枪口指向车尾
						getGimbalData()->chassisChange = getConfigData()->yawCenter + 8192;
						getchassisData()->roDir = 0x01;
					}
					digitalIncreasing(&infantryAutoData.enbStep);
				}break;
				case 1:{
					//如果枪口指向车头则正向运动
					if(getGimbalData()->chassisChange == getConfigData()->yawCenter){
						getchassisData()->direction = 1;
					}
					else{
					//枪口指向车尾则反向运动
						getchassisData()->direction = -1;
					}
					infantryAutoData.enbStep = 99;
				}break;
				case 99:{
					//指向关,只有在下次开启陀螺后结束再次开启
					infantryAutoData.rotateEnb = false;
					infantryAutoData.enbStep = 0;
					getchassisData()->chaseRef = 0.0f;
				}break;
			}
		}
	}
	else{
		getchassisData()->chaseRef = 0.0f;
		infantryAutoData.enbStep = 0;
	}
}



//在比赛开始和复活时自动开启摩擦轮，并可通过单机右键开关摩擦轮
void infantryChangeFricState(void){
	//如果处于比赛状态，并且血量不为0
	if((judgeData.extGameRobotState.remain_HP != 0) && (TASK_PRESS_V ==0)){
		//如果摩擦轮处于关闭状态，则打开摩擦轮，否则关闭摩擦轮
		if(infantryFricState.infantryFricStateFlag == 0)  
			infantryFricState.infantryFricStateFlag = 1;
		else infantryFricState.infantryFricStateFlag = 0;
	}
}

void infantryShootTaskUpdate(void){
int8_t sign = 1;
	if(infantryAutoData.breakSign) infantryAutoData.schedule = 99;
	switch(infantryAutoData.schedule){
		case 1:
			if(PRESS_Z){
				sign = 1;    //增加射速
				digitalIncreasing(&infantryAutoData.schedule);
			}
		  if(PRESS_C){ 
				sign = -1;    //减少射速
				digitalIncreasing(&infantryAutoData.schedule);
			}
			if(infantryAutoData.schedule == 2){
			switch(getshootData()->speed_limit){
			  case SPEED_17MM_LOW: getshootData()->infantry_add_fire_speed_low += sign*10; break;
			  case SPEED_17MM_MID: getshootData()->infantry_add_fire_speed_mid += sign*10; break;
	  		  case SPEED_17MM_HIGH: getshootData()->infantry_add_fire_speed_high += sign*10; break;
			  case SPEED_17MM_EXTREME: getshootData()->infantry_add_fire_speed_extreme += sign*10; break;
			}
		 }
		break;
		case 2:
			if(!PRESS_Z&&!PRESS_C)  digitalDecline(&infantryAutoData.schedule);
		break;
		//只有调到99时才能退出
		case 99: infantryAutoData.taskState = END_OF_EXECUTION; break;
				//如果到达列表中没有进度，则任务出错
		default: infantryAutoData.taskState = EXECUTION_ERROR; break;	
	}		
}

//步兵底盘云台分离
void separate_gimbal_chassiss(void){
     if(PRESS_C){
	    infantryAutoData.separate_flag = 1;
	 }
      else{
	    infantryAutoData.separate_flag = 0;
	  }	 
}

void infantryAutoTaskUpdate(void){
		//如果处于比赛状态，并且刚刚复活时
	if((judgeData.extGameRobotState.remain_HP!=0) && (infantryFricState.infantryFricNumFlag==1)){
		//复活时执行，下次复活之前不需再执行该if的内容
		infantryFricState.infantryFricNumFlag = 0;
		//打开摩擦轮
		infantryFricState.infantryFricStateFlag = 1;
	}
	//如果处于比赛状态，并且死亡时
	else if((judgeData.extGameRobotState.remain_HP==0) && (infantryFricState.infantryFricNumFlag==0)){
		//已死亡，等待复活时打开摩擦轮
		infantryFricState.infantryFricNumFlag = 1;
		//关闭摩擦轮
		infantryFricState.infantryFricStateFlag = 0;	
	}

	super_capacitorTask();
	
	if(infantryAutoData.currentTask != INFANTRY_CHANGE_SHOOT_RATE){
	        separate_gimbal_chassiss();
	}
	else{
	  infantryAutoData.separate_flag = 0;
	}
	
	if(RC_ROTATE < -100){
			infantryAutoData.currentTask = INFANTRY_AUTOMATIC_AIM;  //自瞄任务 手动模式
		}
	if(infantryAutoData.currentTask != INFANTRY_ACTIVE_BUFF || infantryAutoData.currentTask != INFANTRY_ACTIVE_B_BUFF){
		infantryAviodUpdate(); //躲避任务  （INFANTRY_MANUAL）手动模式
	
#if ROTATE_ORIENT
		rotateOff();
#endif
	}
	else{
		//打符时结束躲避
		getchassisData()->chaseRef = 0.0f;
		infantryAutoData.aviodFlag = false;
		digitalClan(&infantryAutoData.avoidSchedule);				
		infantryAutoData.avoidTask = DISABLE;		
	}					
  //开关摩擦轮
  if(keyBoardCtrlData.rkSta == KEY_PRESS_ONCE){
		infantryChangeFricState();
	}
	
	//必须有可执行任务
	if(infantryAutoData.currentTask 
		!= INFANTRY_MANUAL){		
		if(infantryAutoData.currentTask != INFANTRY_BULLET_TRANSFER \
			&& infantryAutoData.lastTask == INFANTRY_BULLET_TRANSFER){
			getshootData()->supplyRef = 80;
		}
		//如果任务刚刚开始执行   未执行（UNEXECUTED）
		if(infantryAutoData.taskState == UNEXECUTED){					
			//执行开始序列
			infantryTaskBegin();																
		}
		//如果在执行中
		else if(infantryAutoData.taskState == EXECUTING){		
			//当前任务
			switch(infantryAutoData.currentTask){								
				//辅助瞄准
				case INFANTRY_AUTOMATIC_AIM:  {										
					infantryAutomaticAimUpdate();
					break;
				}
				//神符打击
				case INFANTRY_ACTIVE_BUFF:
				case INFANTRY_ACTIVE_B_BUFF:{											
					infantryAutomaticBuffUpdate();
					break;
				}
				//子弹交接
				case INFANTRY_BULLET_TRANSFER: {									
					infantryBulletTransferUpdate();
					break;
				}
				//调射速
				case INFANTRY_CHANGE_SHOOT_RATE:
					infantryShootTaskUpdate();
				break;
				//如果是其他命令，则直接重新初始化结构体
				default: {																				
					autoDataInit(&infantryAutoData); 
					break;
				}
			}
		}
		//如果执行完毕或执行错误，则重新初始化结构体
		else{																									
			autoDataInit(&infantryAutoData);
		}
	}
	//更新上一刻的任务
	infantryAutoData.lastTask = infantryAutoData.currentTask;
}

