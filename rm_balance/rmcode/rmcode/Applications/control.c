#include "application.h"
#include "driver.h"
#include "vision.h"
#include "motorHandle.h"
#include "broadcast_demo.h"

/*--------------------------------------------- 
-----------------------------------------------
//		警告：公共代码区域，故障责任重大		 //
//			除特定区域可以自由可以更改外，		 //
//	其余部分更改，请与公共区域负责人取得更改权限  // 
-----------------------------------------------
---------------------------------------------*/

controlStruct_t controlData;
robotModeStruct_t robotMode = MODE_RELAX;			//解除控置
robotModeStruct_t lastRobotMode = MODE_RELAX;
robotModeStruct_t suspendRobotMode;
uint8_t ControlMode(void){
	return RC_MODE;
}
controlStruct_t* getcontrolData(){
    return &controlData;
}
robotModeStruct_t getrobotMode(){
    return robotMode;
}
robotModeStruct_t getlastrobotMode(){
    return lastRobotMode;
}
robotModeStruct_t getSuspendrobotMode(){
	return suspendRobotMode;
}
void setRobotMode(robotModeStruct_t setMode){
	robotMode = setMode;
}

void setSuspendRobotMode(robotModeStruct_t setMode){
	suspendRobotMode = setMode;
}

//此函数用于获取全局控制模式信息
void getGlobalMode(void){													
#ifndef SHIELD_JUDGE

#endif
	if(supervisorData.state & STATE_RADIO_LOSS){
        //上电恒温好后丢控就STOP模式
		robotMode = MODE_STOP;               		  
	}
	else{
		switch(BOARD_TYPE){
			case BOARD_CONTROL:
				switch(robotMode){
					//如果当前为MODE_INIT
					case MODE_INIT :{ 									
						//如果SW2打到最低档时	  
						if(RC_MODE == RCSW_BOTTOM){					
							//设置MODE_RELAX  
							robotMode = MODE_RELAX;             
						}
						//如果云台回中完成 可以模式切换
						else if(getGimbalData()->initFinishFlag){	  
							digitalLo(&getGimbalData()->initFinishFlag);
							//读取遥控器sw2的状态	
							robotMode = (robotModeStruct_t)ControlMode(); 		
						}
					}break;
					//如果当前为键盘模式 
					case MODE_KM :        								  				
					//如果当前为摇杆模式																		
					case MODE_RC :{ 										
						//如果SW2打到最低档时	  			
						if(RC_MODE == RCSW_BOTTOM){					 
							//设置模式为RELAX模式	 
							robotMode = MODE_RELAX;             
						}
						else {
							//否则可以进行模式切换	读取SW2的状态
							robotMode = (robotModeStruct_t)ControlMode();
						}
					}break;
					//如果当前检测到模式为丢控模式，说明又检测到遥控器，不然程序运行不到这里，没有丢控
					case MODE_STOP :{ 								
						//检测到遥控器切换到初始化模式		  
						robotMode = MODE_INIT;        			  
					}break;
					case MODE_RELAX:{
						relax_chassis();
						if(RC_MODE != RCSW_BOTTOM && !controlData.temperatureFaultState){
							robotMode = MODE_INIT;										
						}
					}break;
					default : break;
				}
				break;
			case BOARD_CHASSIS:
				robotMode = (robotModeStruct_t)getcontrolData()->chassisBoardControlMode;
				break;
            
            //LCM: 其他板 模式处理
            case BOARD_OTHER : {
				switch(robotMode){
					//如果当前为MODE_INIT
					case MODE_INIT :{ 									
						//如果SW2打到最低档时	  
						if(RC_MODE == RCSW_BOTTOM){					
							//设置MODE_RELAX  
							robotMode = MODE_RELAX;             
						}
						else {	 
							//读取遥控器sw2的状态	
							robotMode = (robotModeStruct_t)ControlMode(); 		
						}
					}break;
					//如果当前为键盘模式 
					case MODE_KM :        								  				
					//如果当前为摇杆模式																		
					case MODE_RC :{ 										
						//如果SW2打到最低档时	  			
						if(RC_MODE == RCSW_BOTTOM){					 
							//设置模式为RELAX模式	 
							robotMode = MODE_RELAX;             
						}
						else {
							//否则可以进行模式切换	读取SW2的状态
							robotMode = (robotModeStruct_t)ControlMode();
						}
					}break;
					//如果当前检测到模式为丢控模式，说明又检测到遥控器，不然程序运行不到这里，没有丢控
					case MODE_STOP :{ 								
						//检测到遥控器切换到初始化模式		  
						robotMode = MODE_INIT;        			  
					}break;
					case MODE_RELAX:{
						if(RC_MODE != RCSW_BOTTOM && !controlData.temperatureFaultState){
							robotMode = MODE_INIT;										
						}
					}break;
					default : break;
				}                
            }break;
            
		}		
	}
}


static void motorTemperatureControl(void){
	if(motorHandleClass.Temperature(&commandoMotorConfig[PITCHMOTOR]) >= LIMITTEMPERATURE \
		|| motorHandleClass.Temperature(&commandoMotorConfig[YAWMOTOR]) >= LIMITTEMPERATURE){	
		supervisorData.beepState = MUSIC_HIGHTEMPERATURE;
		controlData.temperatureFaultState = true;
		fullReverse();
		robotMode = MODE_RELAX;
	}
	if((motorHandleClass.Temperature(&commandoMotorConfig[PITCHMOTOR]) <= SAFETEMPERATURE \
		&& motorHandleClass.Temperature(&commandoMotorConfig[YAWMOTOR]) <= SAFETEMPERATURE) \
		&& controlData.temperatureFaultState){
		controlData.temperatureFaultState = false;
		digitalClan(&supervisorData.beepState);
	}
}
//控制板类型更新函数
void boardTypeConfirm(uint16_t boardFlag,DeviceActivation_t *boardType){
	if(BOARD_TYPE & boardFlag){
		boardType();
	}
}
//初始化与更新函数
void controlDeviceConfirm(uint16_t deviceFlag,DeviceActivation_t *deviceFunction){
	if(robotConfigData.robotDeviceList & deviceFlag){
		deviceFunction();
	}
}
//底盘板控制初始化
static void congfigChassisInit(void){
	//底盘初始化
	chassisInit();				
    //机甲变形初始化 								
	controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingInit);
}
//云台控制初始化
static void congtrolGlobalInit(void){
	//云台初始化
	controlDeviceConfirm(DEVICE_GIMBAL,gimbalInit);																														
	//机甲变形初始化 								
	controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingInit);							
	//发射补给机构初始化
	controlDeviceConfirm(DEVICE_SHOOT,shootInit);                               
	//视觉发送信息初始化                             
	visionSendDataInit();                           					
	//气动初始化							
	pneumaticInit();
	//自动任务初始化
	autoTaskInit();
	//舵机初始化
//	servoInitClass.Init();
	controlData.temperatureFaultState = false;
	getinfantryAutoData()->rotateEnb = false;
	getinfantryAutoData()->enbStart = false;
	getinfantryAutoData()->enbStep = 0;
	getinfantryAutoData()->rcYaw = &getGimbalData()->yawMotorAngle;
	//主控信息加载完成
	//getcontrolData()->boardDataFinish = INIT_STATE;
}

//LCM: 其他板设备初始化
static void otherDeviceInit() {
    
}

//LCM: 其他板初始化
static void controlOtherInit(void) {
	//自动任务初始化
	autoTaskInit();
    otherDeviceInit();
    
	controlData.temperatureFaultState = false;
	getinfantryAutoData()->rotateEnb = false;
	getinfantryAutoData()->enbStart = false;
	getinfantryAutoData()->enbStep = 0;
	getinfantryAutoData()->rcYaw = &getGimbalData()->yawMotorAngle;
	//主控信息加载完成
	//getcontrolData()->boardDataFinish = INIT_STATE;    
}

static void controlRobotInit(){
	//主控板相关功能初始化
	boardTypeConfirm(BOARD_CONTROL,congtrolGlobalInit);
	//底盘板相关功能初始化
	boardTypeConfirm(BOARD_CHASSIS,congfigChassisInit);
    //LCM: 其他板 相关功能初始化
    boardTypeConfirm( BOARD_OTHER , controlOtherInit );    
}

static void controlChassisUpdata(void){
	if(!(controlData.loops % 2)){	
			if(judgeData.initFlag)
				//裁判系统数据更新
				judgeTask();																											
	}
	//底盘更新
	chassisUpdate();
	//CAN发送更新
    if(getConfigData()->robotType){
        canSendUpdate();	
    }
}

static void controlControlUpdata(void){
	if(!(controlData.loops % 2)){	//DCQ: 無人機單控裁判系統接收數據
			if(judgeData.initFlag)
				//裁判系统数据更新
				judgeTask();																											
	}
	if(remoteControlData.initFlag)
		//遥控器数据更新
		rcUpdateTask();																											
	if(controlData.dataInitFlag){
		//电机高温处理	
		motorTemperatureControl();		
		//获取机器人模式
		getGlobalMode();               
		//获取摩擦轮模式	     																
        getShootMode();									
	}						  	
	//自动控制执行
	autoTaskUpdate();	
	//主控发送至底盘
	/********待添加*******/
	//机甲变形更新	
	controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingUpdate);		
	//云台更新			
	controlDeviceConfirm(DEVICE_GIMBAL,gimbalUpdate);															
	//发射机构更新
	controlDeviceConfirm(DEVICE_SHOOT,shootUpdate);			
	//CAN发送更新
	controlDeviceConfirm(DEVICE_CANSEND,canSendUpdate);	
	//4ms一次的控制
	if(!(controlData.loops % 2)){																				
		//补给机构更新
		controlDeviceConfirm(DEVICE_SUPPLY,supplyUpdate);													
	}
	getGimbalData()->lastCtrlMode =	getGimbalData()->ctrlMode;	
	lastRobotMode = robotMode;
	
}

//LCM: 其他板更新
static void controlOtherUpdate(void) {
	if(remoteControlData.initFlag)
		//遥控器数据更新
		rcUpdateTask();																											
	if(controlData.dataInitFlag){
		//获取机器人模式
		getGlobalMode();               								
	}						  	
	//自动控制执行
	autoTaskUpdate();
    //LCM: 飞镖发射架更新
    controlDeviceConfirm( DEVICE_MISSILE , missileUpdate );
	//LCM: 飞镖发射架CAN发送更新
	controlDeviceConfirm( DEVICE_MISSILE , missileCanSend );
    
	getGimbalData()->lastCtrlMode =	getGimbalData()->ctrlMode;	
	lastRobotMode = robotMode;	
}

void controlSetup(void){
	//人机控制交互的初始化
	rcInit();
	//视觉任务
	visionInit();
    //电机配置表初始化
	motorSeverInitClass.Init();
    //CAN初始化
	canSendInit();
	if(ROBOT == UAV_ID){
		//裁判系统初始化
	jugdeInit();
	}
	//总控制初始化
	controlInit();
    //云台板初始化完成
    //digitalHi(&robotConfigData.id_set);
    digitalHi(&getcontrolData()->controlBoardDataFinish);
}
//底盘板上电初始化
void chassisSetup(void){
    //事件标志组
	xEventGroupWaitBits(taskInitData.eventGroups,CONTROL_RES_OK,pdTRUE,pdTRUE,portMAX_DELAY);
    //机器人ID，板型重识别
    robotBoardConfig();
	//裁判系统初始化
	jugdeInit();
    //电机配置表初始化
	motorSeverInitClass.Init();
    //CAN初始化
	canSendInit();
	//ADC采集初始化											
	adcClass.Init(); 
	//总控制初始化
	controlInit();
	//初始化完成
	digitalHi(&getcontrolData()->chassisBoardDataFinish);
}

//LCM:  其他板初始化
void otherSetup( void ) {
	//人机控制交互的初始化
	rcInit();
    //CAN初始化
	canSendInit();
	//总控制初始化
	controlInit();
    //云台板初始化完成
    //digitalHi(&robotConfigData.id_set);   
    digitalHi(&getcontrolData()->controlBoardDataFinish);    
}

static void controlRobotUpdata(){
    super_capacitorTask();
	boardTypeConfirm(BOARD_CHASSIS,controlChassisUpdata);
	boardTypeConfirm(BOARD_CONTROL,controlControlUpdata);
    //LCM:  其他板更新
    boardTypeConfirm( BOARD_OTHER , controlOtherUpdate );   
}

void controlUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(true){	
		vTaskDelayUntil(&xLastWakeTime,CONTROL_PERIOD);
		//imu传感器错误则不继续执行
//		if(supervisorData.state & STATE_SENSOR_ERROR)													
//			continue;
//		if(!supervisorData.taskEvent[SUPERVISOR_TASK])
//			continue;		
		//必须在成功读取到机器人信息后才能执行初始化
		if(robotConfigData.distinguishState == ROBOT_COMPLETE_IDENTIFIED){					
			//所有控制全部初始化，机器人有的结构进行初始化
			controlRobotInit();																								
			//拉低防止重复初始化，防止重复给ID
			robotConfigData.distinguishState = ROBOT_NO_NEED_TO_IDENTIFY;																															
			digitalHi(&controlData.dataInitFlag);
		}
		//控制板分类，并更新
		controlRobotUpdata();
        uploadCustomDataLoop();        
		digitalIncreasing(&controlData.loops);       
	}
}

void controlInit(void){
	supervisorData.taskEvent[CONTROL_TASK] = xTaskCreate(controlUpdateTask, \
														"CONTROL", \
														CONTROL_STACK_SIZE, \
														NULL, \
														CONTROL_PRIORITY, \
														&controlData.xHandleTask);
    initCustomContent();
    usbVCP_Printf("ControlInit Successfully \r\n");
}


	

