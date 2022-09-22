#include "application.h"
#include "motorHandle.h"
#include "Driver_imuSC.h"
#include "supercapacitor.h"
supervisorStruct_t supervisorData;

/*--------------------------------------------- 
-----------------------------------------------
//		警告：公共代码区域，故障责任重大		 //
//			除特定区域可以自由可以更改外，		 //
//	其余部分更改，请与公共区域负责人取得更改权限  // 
-----------------------------------------------
---------------------------------------------*/

//监控任务是否正常 每个任务都有loop+1
void supervisorTaskCheck(void){																				
	static uint32_t lastTaskLoops[LIST_OF_TASK];
	if((wirelessData.loops - lastTaskLoops[WIRELESS_TASK])>0)
		supervisorData.taskState[WIRELESS_TASK] = TASK_REGULAR;
	else
		supervisorData.taskState[WIRELESS_TASK] = TASK_FAULT;
	if((getcontrolData()->loops - lastTaskLoops[CONTROL_TASK])>0)
		supervisorData.taskState[CONTROL_TASK] = TASK_REGULAR;
	else
		supervisorData.taskState[CONTROL_TASK] = TASK_FAULT;
	lastTaskLoops[WIRELESS_TASK] = wirelessData.loops;				
	lastTaskLoops[CONTROL_TASK] = getcontrolData()->loops;
}

//用于快速切换解锁状态的
void supervisorArmedSwitch(uint8_t valve){														
	if(valve){
        //上锁  先清0
		supervisorData.state &= ~STATE_DISARMED;													
        //解锁
		supervisorData.state |= STATE_ARMED;															
	}
	else{
        //解锁		先清0
		supervisorData.state &= ~STATE_ARMED;															
        //切换成上锁状态
		supervisorData.state |= STATE_DISARMED;														
	}
}

//用于切换监控状态机状态的	
void supervisorStateSwitch(uint16_t state,uint8_t valve){																																		
	if (valve)
		supervisorData.state |= state;
	else
		supervisorData.state &= ~state;
}

//MAG校准指令   （TQH:磁力计校准）
void supervisorMagCali(void){																					
	supervisorStateSwitch(STATE_MAGCALI,ENABLE);
}

//IMU校准指令   (TQH:惯性传感器)
void supervisorImuCali(uint8_t accTare){															
	if(accTare)
		supervisorData.imuCali = CALI_GYO_ACC;
	else
		supervisorData.imuCali = CALI_ONLY_GYO;
}

//设置RGB闪烁状态
void supervisorLedSwitch(void){				
	switch(BOARD_TYPE){
		case BOARD_CHASSIS:{
			if(supervisorData.state & STATE_TRANS_ERROR){
				//通信丢失灯
				supervisorData.ledState = LED_DOUBLE_TRANS; //数组序号赋值	
			 }
			else{
				//正常工作灯
				supervisorData.ledState = LED_WORKINGORDER;                     	
			}
		}
		break;
		case BOARD_CONTROL:{
			if(!getcontrolData()->chassisBoardDataFinish){
				//云台主控初始化失败
				supervisorData.ledState = LED_CONTROLINIT_FAIL;											
			}
			else if(supervisorData.state & STATE_SENSOR_IMU_ERROR){
				//云台板错误工作灯
				supervisorData.ledState = LED_HARDWORK_FAIL;											
			}
			else if(supervisorData.state & STATE_IMUCALI){
				//IMU校准工作灯
				supervisorData.ledState = LED_IMU_CALI;                               	
			}
			else if(supervisorData.gimbalCaliReset){														
				//云台校准
				supervisorData.ledState = LED_GIMBAL_CALI;												
			}
			else if(supervisorData.state & STATE_RADIO_LOSS){
				//遥控器丢失工作灯
				supervisorData.ledState = LED_RADIO_LOSS;    		
			}
			else if(supervisorData.state & STATE_TRANS_ERROR){
				//通信丢失灯
				supervisorData.ledState = LED_DOUBLE_TRANS; 
			}
			else{
				//正常工作灯
				supervisorData.ledState = LED_WORKINGORDER;                     	
			}
		}
		break;
        
        //LCM: 其他板RGB灯效
        case BOARD_OTHER : {
            
			if(supervisorData.state & STATE_RADIO_LOSS){
				//遥控器丢失工作灯
				supervisorData.ledState = LED_RADIO_LOSS;    		
			}
			else if(supervisorData.state & STATE_TRANS_ERROR){
				//通信丢失灯
				supervisorData.ledState = LED_DOUBLE_TRANS; 
			}
			else{
				//正常工作灯
				supervisorData.ledState = LED_WORKINGORDER;                     	
			}           
        }break;        
        
	}
}

//Tx2
static void supervisorTX2TransferError(){
	visionSerial.serialError.intervalNum = visionSerial.serialError.errorCount.u32_temp - visionSerial.serialError.lastErrorCount;
	visionSerial.serialError.lastErrorCount = visionSerial.serialError.errorCount.u32_temp;
	if(!visionSerial.serialError.intervalNum){
		//处理
		
	}
	else{
	
	}
}
//主控校错
static void supervisorControlTransferError(){
	controlSerial.serialError.intervalNum = controlSerial.serialError.errorCount.u32_temp - controlSerial.serialError.lastErrorCount;
	controlSerial.serialError.lastErrorCount = controlSerial.serialError.errorCount.u32_temp;
    
    //LCM: UAV没有底盘板，不需要双控通信
    if(ROBOT == UAV_ID || ROBOT == SMALLGIMBAL_ID) {
        supervisorStateSwitch(STATE_TRANS_ERROR,DISABLE);
        digitalClan(&controlSerial.serialError.waitingConnect);
    }
    else {
        if(!controlSerial.serialError.intervalNum){
            supervisorStateSwitch(STATE_TRANS_ERROR,ENABLE);   
            digitalLo(&getcontrolData()->chassisBoardDataFinish);
            //处理
            //等待连接
            digitalIncreasing(&controlSerial.serialError.waitingConnect);
        }
        else{
            supervisorStateSwitch(STATE_TRANS_ERROR,DISABLE);
            digitalClan(&controlSerial.serialError.waitingConnect);
        }
    }
    
	//断联时间超出200ms
	if(controlSerial.serialError.waitingConnect > LOST_TIME){
		//下控
		setSuspendRobotMode(getrobotMode());
		setRobotMode(MODE_RELAX);
		//停止主控任务更新
		/*TQH*/
//		IWDG_Init();		//1s后  reset
		/**/
	}
	else{
		//回到上一刻状态
//		setRobotMode(getSuspendrobotMode());
		digitalClan(&chassisSerial.serialError.waitingConnect);
		
	}
}

//底盘接收通信校错
static void supervisorChassisTransferError(void){
	chassisSerial.serialError.intervalNum = chassisSerial.serialError.errorCount.u32_temp - chassisSerial.serialError.lastErrorCount;
	chassisSerial.serialError.lastErrorCount = chassisSerial.serialError.errorCount.u32_temp;
	if(!chassisSerial.serialError.intervalNum){
		supervisorStateSwitch(STATE_TRANS_ERROR,ENABLE);
		
	}
	else {
		supervisorStateSwitch(STATE_TRANS_ERROR,DISABLE);
	}
}



//检测丢控
void supervisorRadioLoss(void){																				
	remoteControlData.rcError.intervalNum = remoteControlData.rcError.errorCount.u32_temp - remoteControlData.rcError.lastErrorCount;
	remoteControlData.rcError.lastErrorCount = remoteControlData.rcError.errorCount.u32_temp;
	if(!remoteControlData.rcError.intervalNum){
        //检测到丢控
		supervisorStateSwitch(STATE_RADIO_LOSS,ENABLE);                  	
		supervisorData.beepState = MUSIC_RADIO_LOSS;
	}
	else{
        //标记无丢控
		supervisorStateSwitch(STATE_RADIO_LOSS,DISABLE);                  
		digitalLo(&controlTransData.otherRcReadly);
	}
}

//检测裁判系统
void supervisorJudgeError_chassis(void){																			
	judgeData.intervalNum = judgeData.judgeErrorCount - judgeData.judgeLastErrorCount;
	judgeData.judgeLastErrorCount = judgeData.judgeErrorCount;
	if(!judgeData.intervalNum){
		supervisorStateSwitch(STATE_JUDGE_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_JUDGE_ERROR,DISABLE); 
	}
}

void supervisorJudgeError_gimbal(void){
	if(!get_judgeData()->extGameRobotState.max_HP){
		supervisorStateSwitch(STATE_JUDGE_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_JUDGE_ERROR,DISABLE); 
	}
}

//检测视觉端
void supervisorVisionError(void){																			
	getvisionData()->visionErrorCount = getvisionData()->CNTR.u16_temp;
	getvisionData()->intervalNum = getvisionData()->visionErrorCount - getvisionData()->visionLastErrorCount;
	getvisionData()->visionLastErrorCount = getvisionData()->visionErrorCount;
	if(!getvisionData()->intervalNum){
		supervisorStateSwitch(STATE_VISION_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_VISION_ERROR,DISABLE); 
	}	
}

//检测陀螺仪是否有数据反馈  
void supervisorSlaveimuError(void){	
	getScimuData()->intervalNum = getScimuData()->errorCount - getScimuData()->lastErrorCount;
	getScimuData()->lastErrorCount =getScimuData()->errorCount;
	if(!getScimuData()->intervalNum){
		supervisorStateSwitch(STATE_SENSOR_IMU_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_SENSOR_IMU_ERROR,DISABLE); 
	}
}

//检测功率板
void supervisorCurrentError(void){																		
	getchassisData()->currentError.intervalNum = getchassisData()->currentError.errorCount.u32_temp - getchassisData()->currentError.lastErrorCount;
	getchassisData()->currentError.lastErrorCount = getchassisData()->currentError.errorCount.u32_temp;
	if(!getchassisData()->currentError.intervalNum){
		supervisorStateSwitch(STATE_CURRENT_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_CURRENT_ERROR,DISABLE); 
	}	
}

//检测超级电容是否有数据返回
void supervisorSupercapacitorError(void){
  getcapacitorData()->intervalNum = getcapacitorData()->CapacitorErrorCount - getcapacitorData()->lastErrorCount;
	getcapacitorData()->lastErrorCount = getcapacitorData()->CapacitorErrorCount;
	if(!getcapacitorData()->lastErrorCount)
		supervisorStateSwitch(STATE_CAPACITANCE_ERROR,ENABLE);
	else
		supervisorStateSwitch(STATE_CAPACITANCE_ERROR,DISABLE);
}

//检测基础动力是否正常
void supervisorMotorError(void){																			
	uint8_t scanErrorNum = 0;
	if(check_shooter_working() != 0)
		scanErrorNum++;
	if(check_gimbal_working() != 0)
		scanErrorNum++;
	if(getchassisData()->chassis_fault != 0)
		scanErrorNum++;
	if(scanErrorNum){
		supervisorStateSwitch(STATE_MOTOR_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_MOTOR_ERROR,DISABLE); 
	}
}

//Flash储存
void supervisorFlash(void){	   												
    //如果想要对flash进行操作，直接将supervisorData.flashSave拉高一次即可								
	if(supervisorData.flashSave){                                       
        //检测到需要存入Flash		参数保存提示音
		supervisorData.beepState = MUSIC_PARAMCALI;								
        //写入flash				
		configFlashWrite();  
    if(BOARD_TYPE == BOARD_CONTROL)  getchassisData()->chassisFlash = 1;		
        //写入TF卡PID参数
		parameterWriteDataFormFlash(robotConfigData.typeOfRobot);					
		digitalLo(&supervisorData.flashSave);                             	
	} 	
}

//云台码盘值校准
void supervisorGimbalCali(void){												
	getConfigData()->pitchCenter = motorHandleClass.Encoder(&commandoMotorConfig[PITCHMOTOR]);
	getConfigData()->yawCenter = motorHandleClass.Encoder(&commandoMotorConfig[YAWMOTOR]);
	getConfigData()->rollCenter = motorHandleClass.Encoder(&commandoMotorConfig[ROLLMOTOR]);
	getchassisData()->yawCenterSave = getConfigData()->yawCenter;
	digitalHi(&supervisorData.flashSave);
}

//机器人类型校准
void supervisorDefineRbot(void){	
	robotConfigData.typeOfRobot  = NO_ID;
	robotDistinguish();
}


void supervisorDefineID(void){
    getConfigData()->localID = 0x0101;
}
//
void dbusFunction(void){
	static uint16_t Loops[CONFIG_LIST] = {0,0,0,0,0,0};
	static uint8_t Digital[CONFIG_LIST] = {0,0,0,0,0,0};
#if UAV_SBUS			
			if(((RC_ROLL > CALI_RC_VALUE) && (RC_PITCH > CALI_RC_VALUE)) \
				|| (wirelessData.imuCalibrate)){
#else 
			if((((RC_TRANSVERSE > CALI_RC_VALUE) && (RC_LONGITUDINAL > CALI_RC_VALUE)) \
				|| (wirelessData.imuCalibrate))){
#endif			
				if(!supervisorData.busyState){
					//IMU校准
					if(Loops[IMU] >= WAITLOOPS && Digital[IMU]!= High){
                        //遥控器所进行的校准，是不带加速度计校准的	
						supervisorImuCali(DISABLE);																					
						supervisorData.beepState = MUSIC_IMUCALI;
						digitalHi(&Digital[IMU]);
						digitalClan(&Loops[IMU]);
					}
					else if(wirelessData.imuCalibrate){
                        //IMU校准
						supervisorImuCali(ENABLE);                                        
						supervisorData.beepState = MUSIC_IMUCALI;
						digitalLo(&wirelessData.imuCalibrate);
					}
					else
						digitalIncreasing(&Loops[IMU]);
				}
			}
			else{
				digitalClan(&Loops[IMU]);
			}
			
			
#if UAV_SBUS        //RM版																														
			if(((RC_ROLL < -CALI_RC_VALUE) && (RC_PITCH > CALI_RC_VALUE)) \
				|| (wirelessData.magCalibrate)){
#else
			if((((RC_TRANSVERSE < -CALI_RC_VALUE) && (RC_LONGITUDINAL > CALI_RC_VALUE)) \
				|| (wirelessData.magCalibrate))){
#endif
				if(!supervisorData.busyState){	
					//MAG校准
					if(Loops[MAG] >= WAITLOOPS && Digital[MAG]!= High){
                        //MAG校准指令
						supervisorMagCali();																				
						supervisorData.beepState = MUSIC_MAGCALI;
						digitalHi(&Digital[MAG]);
						digitalClan(&Loops[MAG]);
					}
					else if(wirelessData.magCalibrate){
                        //磁力计校准
						supervisorMagCali();                                        
						supervisorData.beepState = MUSIC_MAGCALI;
						digitalLo(&wirelessData.magCalibrate);
					}
					else
						digitalIncreasing(&Loops[MAG]);
				}
			}
			else{
				digitalClan(&Loops[MAG]);
			}
			
			
#if UAV_SBUS
			if((RC_ROLL < -CALI_RC_VALUE) && (RC_PITCH < -CALI_RC_VALUE)){
#else
			if((RC_TRANSVERSE < -CALI_RC_VALUE) && (RC_LONGITUDINAL < -CALI_RC_VALUE)){
#endif
				if(!supervisorData.busyState){	
					digitalIncreasing(&Loops[FLASH_OPERATION]);
					if(Loops[FLASH_OPERATION] >= WAITLOOPS && Digital[FLASH_OPERATION]!= High){
                        //储存flash
						digitalHi(&supervisorData.flashSave);												
						digitalHi(&Digital[FLASH_OPERATION]);
						digitalClan(&Loops[FLASH_OPERATION]);
					}
				}
			}
			else{																														
				digitalClan(&Loops[FLASH_OPERATION]);
			}
			
			
#if UAV_SBUS	
			if((RC_THROT > CALI_RC_VALUE) && (RC_RUDD > CALI_RC_VALUE)){
#else	
			if((RC_PITCH > CALI_RC_VALUE) && (RC_RUDD > CALI_RC_VALUE)){
#endif	
				if(!supervisorData.busyState){
					digitalIncreasing(&Loops[DEFINE_ROBOT]);
					if(Loops[DEFINE_ROBOT] >= WAITLOOPS && Digital[DEFINE_ROBOT]!= High){
                        //机器类型识别
						supervisorDefineRbot();																			
						digitalHi(&Digital[DEFINE_ROBOT]);
						digitalClan(&Loops[DEFINE_ROBOT]);
					}
				}
			}
			else{
				digitalClan(&Loops[DEFINE_ROBOT]);
			}
			

#if UAV_SBUS	
			if((RC_THROT > CALI_RC_VALUE) && (RC_RUDD < -CALI_RC_VALUE)){
#else	
			if((RC_PITCH > CALI_RC_VALUE) && (RC_RUDD < -CALI_RC_VALUE)){
#endif			
				if(!supervisorData.busyState){
					digitalIncreasing(&Loops[DEFINE_ID]);
					if(Loops[DEFINE_ID] >= WAITLOOPS && Digital[DEFINE_ID]!= High){
                        //主控板ID设置
						supervisorDefineID();																				
						digitalHi(&Digital[DEFINE_ID]);
						digitalClan(&Loops[DEFINE_ID]);						
					}
				}
			}
			else{
				digitalClan(&Loops[DEFINE_ID]);
			}
			
#if UAV_SBUS
			if((RC_RUDD < -CALI_RC_VALUE) && (RC_PITCH < -CALI_RC_VALUE)){
#else
			if((RC_TRANSVERSE > CALI_RC_VALUE) && (RC_LONGITUDINAL < -CALI_RC_VALUE)){
#endif	
				if(!supervisorData.busyState){
					digitalIncreasing(&Loops[GIMBAL_CALI]);
					if(Loops[GIMBAL_CALI] >= WAITLOOPS && Digital[GIMBAL_CALI]!= High){
                        //云台码盘值校准
						digitalHi(&supervisorData.gimbalCaliReset);									
						digitalHi(&Digital[GIMBAL_CALI]);
						digitalClan(&Loops[GIMBAL_CALI]);
					}
				}
			}
			else{
				if(supervisorData.gimbalCaliReset){
                    //只有在松开摇杆后才会执行校准云台码盘
					supervisorGimbalCali();																				
					digitalLo(&supervisorData.gimbalCaliReset);
				}
				digitalClan(&Loops[GIMBAL_CALI]);
			}
		
#if UAV_SBUS			
			if((Digital[IMU] != Low \
				|| Digital[MAG] != Low \
				|| Digital[FLASH_OPERATION] != Low \
				|| Digital[DEFINE_ROBOT] != Low \
				|| Digital[YAW_CALI] != Low \
				|| Digital[GIMBAL_CALI] !=Low\
				|| Digital[DEFINE_ID] !=Low)
				&&  (abs(RC_ROLL) < 10 && abs(RC_PITCH) < 10 \
				&& abs(RC_RUDD) < 10 && abs(RC_THROT) < 10))){			
                //校准从所有摇杆完全松开才开始				
#else
			if((Digital[IMU] != Low \
				|| Digital[MAG] != Low \
				|| Digital[FLASH_OPERATION] != Low \
				|| Digital[DEFINE_ROBOT] != Low \
				|| Digital[YAW_CALI] != Low \
				|| Digital[GIMBAL_CALI] !=Low\
				|| Digital[DEFINE_ID] !=Low)
				&& (abs(RC_LONGITUDINAL) < 10 && abs(RC_TRANSVERSE) < 10 \
				&& abs(RC_PITCH) < 10 && abs(RC_RUDD) < 10)){										
#endif          //校准从所有摇杆完全松开才开始
				digitalLo(&Digital[IMU]);
				digitalLo(&Digital[MAG]);
				digitalLo(&Digital[FLASH_OPERATION]);
				digitalLo(&Digital[DEFINE_ROBOT]);
				digitalLo(&Digital[YAW_CALI]);
				digitalLo(&Digital[GIMBAL_CALI]);
                digitalLo(&Digital[DEFINE_ID]);
			}

	}
				
//底盘模块检测
void static supervisorChassisError(){
	//通信链路监控
	supervisorChassisTransferError();
	supervisorSlaveimuError();
	if(!(supervisorData.loops % 10)){
		//检测裁判系统数据
		supervisorJudgeError_chassis();	
		//检测功率控制
		if(robotConfigData.robotDeviceList & DEVICE_CURRENT){
			supervisorCurrentError();	
		} 											
	}
}
//主控模块检测
void static supervisorControlError(){
	//遥控器功能
	dbusFunction();
	//视觉通信链路监控
	supervisorTX2TransferError();

    //主控通信链路监控
    supervisorControlTransferError();
	//超级电容检测
	controlDeviceConfirm(DEVICE_CAPACITANCE,supervisorSupercapacitorError);
	//云台板检测
	supervisorSlaveimuError();
	/**/
	
	/**/
	if(!(supervisorData.loops % 10)){
        //电机检测																		
		supervisorMotorError();		
		//裁判系统检测
		supervisorJudgeError_gimbal();
		//丢控状态检测，1Hz
		supervisorRadioLoss();	
		//视觉端检测
		controlDeviceConfirm(DEVICE_VISION,supervisorVisionError);											
	}
	//在解锁状态下
	/*-- 在上锁状态下的状态机监控 --*/
	if(supervisorData.state & STATE_ARMED){														
		//控制权检测		如果在底部就上锁
		if(RC_MODE == RCSW_BOTTOM){		  																
			//将解锁状态切换成上锁状态
			supervisorArmedSwitch(DISABLE);														
			//上锁的提示音		
			supervisorData.beepState = MUSIC_DISARMED;										
		}
	}/*-- 在解锁状态下的状态机监控 --*/
	else if(supervisorData.state & STATE_DISARMED){										
			if(!(supervisorData.state & STATE_MAGCALI) \
				&& !(supervisorData.state & STATE_IMUCALI) \
				&& (robotConfigData.distinguishState != ROBOT_BEING_IDENTIFIED) \
				&& (supervisorData.gimbalCaliReset != ENABLE)){
                //执行各种校准必须处于不忙碌且有ID的状态
				supervisorData.busyState = false;															
			}
			else{
				supervisorData.busyState = true;	
			}
            //解锁命令扫描
			if(RC_MODE == RCSW_MID || RC_MODE== RCSW_TOP){									
				supervisorArmedSwitch(ENABLE);
				supervisorData.beepState=MUSIC_ARMED;												  
				supervisorData.ArmSwitch=ENABLE;
				digitalClan(&supervisorData.armTime);
			}
		}
	if(robotConfigData.typeOfRobot == NO_ID){
        //没有ID会一直响
		supervisorData.beepState = MUSIC_NO_ID;													
	}
	
}

//LCM: 其他板验错
static void supervisorOtherError(void) {
	//遥控器功能
	dbusFunction();
	if(!(supervisorData.loops % 10)){
		//丢控状态检测，1Hz//裁判系统检测
		supervisorRadioLoss();											
	}
	//在解锁状态下 
	/*-- 在上锁状态下的状态机监控 --*/
	if(supervisorData.state & STATE_ARMED){														
		//控制权检测		如果在底部就上锁
		if(RC_MODE == RCSW_BOTTOM){		  																
			//将解锁状态切换成上锁状态
			supervisorArmedSwitch(DISABLE);														
			//上锁的提示音		
			supervisorData.beepState = MUSIC_DISARMED;										
		}
	}/*-- 在解锁状态下的状态机监控 --*/
	else if(supervisorData.state & STATE_DISARMED){										
			if(!(supervisorData.state & STATE_MAGCALI) \
				&& !(supervisorData.state & STATE_IMUCALI) \
				&& (robotConfigData.distinguishState != ROBOT_BEING_IDENTIFIED) \
				&& (supervisorData.gimbalCaliReset != ENABLE)){
                //执行各种校准必须处于不忙碌且有ID的状态
				supervisorData.busyState = false;															
			}
			else{
				supervisorData.busyState = true;	
			}
            //解锁命令扫描
			if(RC_MODE == RCSW_MID || RC_MODE== RCSW_TOP){									
				supervisorArmedSwitch(ENABLE);
				supervisorData.beepState=MUSIC_ARMED;												  
				supervisorData.ArmSwitch=ENABLE;
				digitalClan(&supervisorData.armTime);
			}
		}
	if(robotConfigData.typeOfRobot == NO_ID){
        //没有ID会一直响
		supervisorData.beepState = MUSIC_NO_ID;													
	}	
}

//全局检测
void static supervisorErrorRobot(){
	boardTypeConfirm(BOARD_CHASSIS,supervisorChassisError);
	boardTypeConfirm(BOARD_CONTROL,supervisorControlError);
    //LCM: 其他板检测
    boardTypeConfirm( BOARD_OTHER , supervisorOtherError );    
}
	
void supervisorUpdateTask(void *Parameters){
    //获取任务系统运行的时钟节拍数
	TickType_t xLastWakeTime = xTaskGetTickCount();											
	while(true){
        //绝对延时（周期性延时函数）
		vTaskDelayUntil(&xLastWakeTime,SUPER_STACK_PERIOD);	
//        //检测任务是否正常
//		supervisorTaskCheck();			
		//灯条、RGB与蜂鸣器更新
		warningUpdate();		
        //Flash写入检测										
		supervisorFlash();                                                
        //tf卡检测
		tFCardUpdate();																											
		//板载三色LED状态切换	设置灯的监控指示
		supervisorErrorRobot();
		//双控监控机
		supervisorLedSwitch();		
		digitalIncreasing(&supervisorData.loops);
	}
}

//蜂鸣器初始化，和LED一起属于监管状态机
void supervisorInit(void){
	supervisorArmedSwitch(DISABLE);
	sightClass.Init();
	//SK6812灯条初始化
	SK6812Config();
	supervisorData.taskEvent[SUPERVISOR_TASK] = xTaskCreate(supervisorUpdateTask, \
                                                            "SUPE", \
                                                            SPUER_STACK_SIZE, \
                                                            NULL,SPUER_PRIORITY, \
                                                            &supervisorData.xHandleTask);
    usbVCP_Printf("supervisorInit Successfully \r\n");
}



