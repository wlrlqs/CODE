#include "config.h"
#include "control.h"
#include "rc.h"
#include "imu.h"
#include "keyboard.h"
#include "gimbal.h"
#include "vision.h"
#include "chassis.h"
#include "supervisor.h"
#include "Driver_RMMotor.h"
#include "Driver_USBVCP.h"
#include "motorHandle.h"

/*--------------------------------------------- 
-----------------------------------------------
//		警告：公共代码区域，故障责任重大		 //
//			除特定区域可以自由可以更改外，		 //
//	其余部分更改，请与公共区域负责人取得更改权限  // 
-----------------------------------------------
---------------------------------------------*/

remoteControlStruct_t remoteControlData;    

//把绝对值从660转化到500的函数
void rcSbusScale(sbusStruct_t *raw,sbusStruct_t *real){					
	uint8_t i;
	f32_t ch[CH_NUMBER];
	for(i = 0;i < CH_NUMBER;i++){  
		ch[i] = (f32_t)*(&(raw->CH0)+i) / 660 * 500;
		ch[i] = constrainFloat(ch[i],-500,500);
		*(&(real->CH0) + i) = (int16_t)ch[i];
	}
}

//把绝对值从660转化到500的函数
void rcDt7Scale(dt7Struct_t *raw,dt7Struct_t *real){						
	uint8_t i;
	f32_t ch[CH_NUMBER];
	for(i = 0;i < CH_NUMBER;i++){  
		ch[i] = (f32_t)*(&(raw->CH0)+i) / 660 * 500;		
		ch[i] = constrainFloat(ch[i],-500.0f,500.0f);
		*(&(real->CH0) + i) = (int16_t)ch[i];
	}
	real->S1 = raw->S1;
	real->S2 = raw->S2;
}

//底盘速度赋值,摇杆最大值对应底盘速度最大值	
static void chassisOperationFunc(int16_t forwardBack, int16_t leftRight, int16_t rotate){
	if(getrobotMode() == MODE_RC){
		remoteControlData.chassisSpeedTarget.x = leftRight / getConfigData()->rcResolution * \
												 getConfigData()->chassisRCSpeed * REAL_MOTOR_SPEED_SCALE;
		remoteControlData.chassisSpeedTarget.y = forwardBack / getConfigData()->rcResolution *\
												 getConfigData()->chassisRCSpeed * REAL_MOTOR_SPEED_SCALE;
		remoteControlData.chassisSpeedTarget.z = rotate / getConfigData()->rcResolution * \
												 getConfigData()->chassisRCSpeed * REAL_MOTOR_SPEED_SCALE;
	}
	else{
		remoteControlData.chassisSpeedTarget.x = 0;
		remoteControlData.chassisSpeedTarget.y = 0;
		remoteControlData.chassisSpeedTarget.z = 0;	
	}
}

void remoteCtrlChassisHook(void){
  chassisOperationFunc(RC_LONGITUDINAL, RC_TRANSVERSE, RC_RUDD);
}

//云台速度摇杆赋值
static void gimbalOperationFunc(int16_t pitCtrl, int16_t yawCtrl){
  remoteControlData.pitchGyroTarget =  pitCtrl * getConfigData()->gimbalGTRScale * 2;
  remoteControlData.yawGyroTarget   =  yawCtrl * getConfigData()->gimbalGTRScale * 2;
}

void remoteCtrlGimbalHook(void){
  gimbalOperationFunc(RC_PITCH, RC_RUDD);
}

//获取云台控制数据
void getGimbalCtrlDate(void){
	//云台码盘值连续处理	
	getGimbalData()->pitchMotorAngle = motorHandleClass.Position(&commandoMotorConfig[PITCHMOTOR],getConfigData()->pitchCenter);
	#ifdef DOUBLE_GUN_HERO
	getGimbalData()->yawMotorAngle 	 = commandoMotorConfig[YAWMOTOR]*ENCODER_ANGLE_RATIO14; 
	#else 
	getGimbalData()->yawMotorAngle 	 = motorHandleClass.Position(&commandoMotorConfig[YAWMOTOR],getGimbalData()->chassisChange);
	#endif
	if(ROBOT == UAV_ID){
		getGimbalData()->rollMotorAngle  = motorHandleClass.Position(&commandoMotorConfig[ROLLMOTOR],getConfigData()->rollCenter);
		getGimbalData()->rollGyroAngle = SC_ROLL;
	}
	//pitch轴角度
	getGimbalData()->pitchGyroAngle = SC_PITCH;
	//yaw轴角度		
	getGimbalData()->yawGyroAngle   = SC_YAW;
	
    //云台键盘操作数据处理
	keyboardGimbalHook();                 				
    //云台摇杆操作数据处理
	remoteCtrlGimbalHook();	              				
}

//获取底盘控制数据
void getChassisCtrlDate(void){						
    //底盘键盘操作数据处理			
    keyboardChassisHook();                        
    //底盘摇杆操作数据处理
    remoteCtrlChassisHook();                      
}

void rcUpdateTask(void){
#if UAV_SBUS 
	if(remoteControlData.rcIspReady){
		Driver_SBUS_Decode_RemoteData(&remoteControlData.sbusValue.rcRawData,Array_USART1_RX);
		digitalLo(&remoteControlData.rcIspReady);
	}
	rcSbusScale(&remoteControlData.sbusValue.rcRawData,&remoteControlData.sbusValue.rcRealData);
#else
	if(remoteControlData.rcIspReady){
		Driver_RMDT7_Decode_RemoteData(&remoteControlData.dt7Value,RECEIVER_BUFF);
		digitalLo(&remoteControlData.rcIspReady);
	}
	rcDt7Scale(&remoteControlData.dt7Value.rcRawData,&remoteControlData.dt7Value.rcRealData);
#endif
	keyBoardCtrlData.leftKeyState  = leftKeyDriver();
	keyBoardCtrlData.rightKeyState = rightKeyDriver();		
	getKeyboardMouseState();
	getChassisCtrlDate();		
	digitalIncreasing(&remoteControlData.loops);
}
void rcInit(void){
	DT7IintClass.Init();
	remoteControlData.initFlag = true;
    usbVCP_Printf("RCInit Successfully \r\n");
}
 



