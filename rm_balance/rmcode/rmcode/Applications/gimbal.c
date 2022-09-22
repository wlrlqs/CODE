#include "gimbal.h"
#include "slave_sensor.h"
#include "keyboard.h"
#include "rc.h"
#include "ramp.h"
#include "config.h"
#include "Driver_RMMotor.h"
#include "imu.h"
#include "chassis.h"
#include "cansend.h"
#include "judge.h"
#include "control.h"
#include "vision.h"
#include "SEGGER_RTT.h"
#include "math.h"
#include "slave_sensor.h"
#include "Driver_USBVCP.h"
#include "Driver_UserPrintf.h"
#include "motorHandle.h"

/*
***************************************************
函 数 名：	gimbalUpdate
功		能：云台任务更新
入口参数：	gimbalData.visionYawCmd				横向坐标系反馈
			gimbalData.visionPitchCmd			纵向坐标系反馈
			gimbalData.autoMode					切换自动任务标志
返 回 值：	无
应用范围：	外部调用
备		注：
***************************************************
*/

gimbalStruct_t gimbalData;
//云台斜坡初始化
ramp_t yawVisionRamp = RAMP_GEN_DAFAULT;											
ramp_t pitchVisionRamp = RAMP_GEN_DAFAULT;
//云台斜坡初始化
static ramp_t yawRamp = RAMP_GEN_DAFAULT;											
static ramp_t pitchRamp = RAMP_GEN_DAFAULT;
static ramp_t rollRamp = RAMP_GEN_DAFAULT;
gimbalStruct_t* getGimbalData(){
    return &gimbalData;
}

int8_t getInstallDirect(uint8_t installPara,bool type){         //YAW轴安装方向
	int8_t res;
	if(type)
		res = (installPara & 0x02)?-1:1;
	else
		res = (installPara & 0x01)?-1:1;
	return res;
}

float testpitch = 0;
float testyaw = 0;
void gimbalUpdate(void){
	
//	userPrintf("%d,%f\r\n",commandoMotorConfig[7].motor_staus->currunt,getchassisData()->pitchAngleFbd);
	getGimbalCtrlDate();
	getGimbalData()->yawEncoder = motorHandleClass.Encoder(&commandoMotorConfig[YAWMOTOR]) - getConfigData()->yawCenter;
	if(T_OR_F0(getGimbalData()->yawEncoder)) 
		getGimbalData()->yawEncoder = 16384 + getGimbalData()->yawEncoder;
	//平衡步兵
	if((gimbalData.yawEncoder >= 4096 || gimbalData.yawEncoder < 12288) && !getinfantryAutoData()->aviodFlag){
		getchassisData()->direction = 1;
	}
	else if(getinfantryAutoData()->aviodFlag){
		if(gimbalData.yawEncoder >= 0 || gimbalData.yawEncoder < 8192){
			getchassisData()->direction = 1;
		}
		else	
			getchassisData()->direction = 0;
	}
	else
		getchassisData()->direction = 0;
	
	switch (getrobotMode()){	//初始化模式
		case MODE_INIT:           
			gimbalInitHandle();   
			break;
		case MODE_RC: 			     
		case MODE_KM:{
			if(gimbalData.ctrlMode == GIMBAL_INIT){
				gimbalInitHandle();
			}
			else if(gimbalData.ctrlMode == GIMBAL_NORMAL){
				gimbalFollowHandle();
			}
			else if(gimbalData.ctrlMode == GIMBAL_STOP){
				gimbalStopHandle();
			}
			break;
		}
        //丢控模式
		case MODE_STOP:{	 
			gimbalStopHandle();	 
			break;
		}
        //解除控制权
		case MODE_RELAX:{        
			gimbalRelaxHandle();  
			break;
		}
		default:                                        
			break;
	}

	gimbalData.time[0] = getClockCount();
	gimbalData.intervalTime = (f32_t)(gimbalData.time[0] - gimbalData.time[1]);
	gimbalData.time[1] = gimbalData.time[0];
	//
	gimbalData.yawEncoder = motorHandleClass.Encoder(&commandoMotorConfig[YAWMOTOR]);//yaw轴电机编码器值 

	//自瞄已经瞄上
	if(gimbalData.autoMode && getvisionData()->captureFlag && getvisionData()->cailSuccess){
        //自动模式下同步当前值到角度期望
		gimbalData.pitchAngleRef = getvisionData()->pitchCmd;				 
		gimbalData.yawAngleRef = getvisionData()->yawCmd;	
		//如果是自瞄状态云台角度反馈取陀螺仪角度（不加偏置）
		gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;
		gimbalData.yawAngleFbd = gimbalData.yawGyroAngle - gimbalData.yawAngleSave;
	}
	//云台迫击炮模式
	if(gimbalData.autoMode && (ROBOT == P_TANK_ID)&&(autoTaskData->currentTask == TANK_MORTAR)){ 
        //自动模式下同步当前值到角度期望
		gimbalData.pitchAngleRef = getvisionData()->mortarPitCmd;				
		gimbalData.yawAngleRef = getvisionData()->mortarYawCmd;	
		//如果是自瞄状态云台角度反馈取陀螺仪角度（不加偏置）
		gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;
		gimbalData.yawAngleFbd = gimbalData.yawGyroAngle - gimbalData.yawAngleSave;
	}
	
	if(isnan(gimbalData.yawAngleOut)||isnan(gimbalData.pitchAngleOut)){
		pidZeroState(gimbalData.pitchAnglePID);
		pidZeroState(gimbalData.pitchSpeedPID);
		pidZeroState(gimbalData.yawAnglePID);
		pidZeroState(gimbalData.yawSpeedPID);
		pidZeroState(gimbalData.rollAnglePID);
		pidZeroState(gimbalData.rollSpeedPID);
		digitalClan(&gimbalData.yawAngleOut);
		digitalClan(&gimbalData.pitchAngleOut);
		digitalClan(&gimbalData.rollAngleOut);
		digitalClan(&gimbalData.yawSpeedOut);
		digitalClan(&gimbalData.pitchSpeedOut);
		digitalClan(&gimbalData.rollSpeedOut);
		gimbalData.pitchAngleRef = gimbalData.pitchAngleFbd;				
		gimbalData.yawAngleRef = gimbalData.yawAngleFbd;
		gimbalData.rollAngleRef = gimbalData.rollAngleFbd;
	}
#if YAW_SPEED_SINGLE	
	//只有鼠标给云台期望时才执行
	if(getrobotMode() == MODE_KM && !(gimbalData.autoMode && getvisionData()->captureFlag && getvisionData()->cailSuccess) \
		&& !getinfantryAutoData()->rotateFlag && ROBOT == INFANTRY_ID){      //为什么只闭速度环
		//鼠标移动时只闭速度环
		gimbalData.yawSpeedRef = keyBoardCtrlData.yawSpeedTarget;
		//该轴没有速度期望时闭角度环
		if(!keyBoardCtrlData.yawSpeedTarget){
			if(gimbalData.angleCycleStep == 2 || gimbalData.angleCycleStep == 0){
				gimbalData.yawAngleOut = pidUpdate(gimbalData.yawAnglePID,gimbalData.yawAngleRef,gimbalData.yawAngleFbd,gimbalData.intervalTime); 
				gimbalData.yawSpeedRef = -gimbalData.yawAngleOut;																		
			}
		}
	}
	else{
		gimbalData.yawAngleOut = pidUpdate(gimbalData.yawAnglePID,gimbalData.yawAngleRef,gimbalData.yawAngleFbd,gimbalData.intervalTime); 
		gimbalData.yawSpeedRef = -gimbalData.yawAngleOut;
	}
#else
	gimbalData.yawAngleOut = pidUpdate(gimbalData.yawAnglePID,gimbalData.yawAngleRef,gimbalData.yawAngleFbd,gimbalData.intervalTime); 
	gimbalData.yawSpeedRef = -gimbalData.yawAngleOut;
#endif

	gimbalData.pitchAngleOut = pidUpdate(gimbalData.pitchAnglePID, gimbalData.pitchAngleRef, gimbalData.pitchAngleFbd,gimbalData.intervalTime);
	gimbalData.pitchSpeedRef = -gimbalData.pitchAngleOut;
	// 速度环反馈用imu角速度速度
#if IMU_HERO	
	  gimbalData.pitchSpeedFbd = IMU_RATEX;  
#else	  
	  gimbalData.pitchSpeedFbd = IMU_RATEY; 
#endif
	gimbalData.yawSpeedFbd   = IMU_RATEZ;
	if(ROBOT == UAV_ID){            
		gimbalData.rollAngleOut  = pidUpdate(gimbalData.rollAnglePID, gimbalData.rollAngleRef, gimbalData.rollAngleFbd,gimbalData.intervalTime);            
		gimbalData.rollSpeedRef  = -gimbalData.rollAngleOut;
		gimbalData.rollSpeedFbd  = IMU_RATEX;                        
		gimbalData.rollSpeedOut  = getInstallDirect(ROLL_INSTALL_CONFIG,INSTALL_TURN)*pidUpdate(gimbalData.rollSpeedPID, gimbalData.rollSpeedRef,gimbalData.rollSpeedFbd,gimbalData.intervalTime);            
	}
    if(ROBOT!=P_TANK_ID){
	gimbalData.pitchSpeedOut = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_TURN)*pidUpdate(gimbalData.pitchSpeedPID,gimbalData.pitchSpeedRef,gimbalData.pitchSpeedFbd,gimbalData.intervalTime);
	gimbalData.yawSpeedOut   = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN)*pidUpdate(gimbalData.yawSpeedPID,gimbalData.yawSpeedRef,gimbalData.yawSpeedFbd,gimbalData.intervalTime);
    }
else{//气动英雄的电机安装顺序跟其他兵种不一样
    gimbalData.pitchSpeedOut = getInstallDirect(PITCH_INSTALL_CONFIG,0)*pidUpdate(gimbalData.pitchSpeedPID,gimbalData.pitchSpeedRef,gimbalData.pitchSpeedFbd,gimbalData.intervalTime);
	gimbalData.yawSpeedOut   = getInstallDirect(YAW_INSTALL_CONFIG,1)*pidUpdate(gimbalData.yawSpeedPID,gimbalData.yawSpeedRef,gimbalData.yawSpeedFbd,gimbalData.intervalTime);
}
	if(getrobotMode() == MODE_RELAX){
		digitalClan(&gimbalData.yawSpeedOut);
		digitalClan(&gimbalData.pitchSpeedOut);
		digitalClan(&gimbalData.rollSpeedOut);
	}
//	usbVCP_Printf("pitch_ang_ref = %f\r\n",gimbalData.pitchAngleRef);
//	usbVCP_Printf("pitch_speed_ref = %f\r\n",gimbalData.pitchSpeedRef);
//	usbVCP_Printf("pitch_ang_out = %f\r\n",gimbalData.pitchAngleFbd);
//	usbVCP_Printf("pitch_speed_out = %f\r\n",gimbalData.pitchSpeedFbd);
//	usbVCP_Printf("%f %f %f %f\r\n",gimbalData.pitchAngleRef,gimbalData.pitchAngleFbd,gimbalData.pitchSpeedRef,gimbalData.pitchSpeedFbd);
}

static void gimbalFollowHandle (void){
#if YAW_SPEED_SINGLE
	static f32_t lastYawSpeedRef;
#endif
	digitalClan(&gimbalData.motorFlag);
	gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle ;
	gimbalData.yawAngleFbd 	 = gimbalData.yawGyroAngle - gimbalData.yawAngleSave;
	gimbalData.yawAngleRef 	 += remoteControlData.yawGyroTarget + keyBoardCtrlData.yawGyroTarget;
	gimbalData.pitchAngleRef += remoteControlData.pitchGyroTarget + keyBoardCtrlData.pitchGyroTarget;
	testpitch += remoteControlData.pitchGyroTarget + keyBoardCtrlData.pitchGyroTarget;
#if YAW_SPEED_SINGLE	
	if(getrobotMode() == MODE_KM && !(gimbalData.autoMode && getvisionData()->captureFlag) \
		&& !getinfantryAutoData()->rotateFlag && ROBOT == INFANTRY_ID){
		//鼠标从运动到停止，不能以此时角度闭环
		if(!keyBoardCtrlData.yawSpeedTarget && lastYawSpeedRef){
			gimbalData.angleCycleStep = 1;
		}
		//等到云台速度接近为0时，开始闭角度环
		if((fabs(IMU_RATEZ) < 0.08f) && (gimbalData.angleCycleStep == 1)){
			gimbalData.yawAngleRef = gimbalData.yawAngleFbd;
			gimbalData.angleCycleStep = 2;
		}
		lastYawSpeedRef = keyBoardCtrlData.yawSpeedTarget;
	}
#endif
	//pitch轴角度限幅
	gimbalData.pitchAngleRef = constrainFloat(gimbalData.pitchAngleRef,getConfigData()->pitchMinRange,getConfigData()->pitchMaxRange); 
}

static void gimbalInitHandle(void){	
	static f32_t pitchRampAngle;
	static f32_t yawRampAngle;
	static float rollRampAngle;
	static TickType_t xLastWakeTime = 0;
    //每一次进入初始化模式都获取当前角度并初始化斜坡函数
	if(getlastrobotMode() != getrobotMode()||gimbalData.ctrlMode != gimbalData.lastCtrlMode){																			
		xLastWakeTime = xTaskGetTickCount();
		gimbalRampInit();
        //获取pitch轴imu当前角度
		pitchRampAngle = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;  				
        //获取yaw码盘当前角度
		yawRampAngle   = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;   					
		if(ROBOT == UAV_ID)
			//获取roll码盘当前角度     
			rollRampAngle   = getInstallDirect(ROLL_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.rollMotorAngle;
	}
    //设置pitch轴反馈//pitch轴回中
	gimbalData.pitchAngleFbd = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;	
	gimbalData.pitchAngleRef = pitchRampAngle * (1 - LinearRampCalc(&pitchRamp,2));																			
    //设置yaw轴反馈//yaw轴回中
	gimbalData.yawAngleFbd = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;   		
	gimbalData.yawAngleRef = yawRampAngle * (1 - LinearRampCalc(&yawRamp,2));       																		
  	if(ROBOT == UAV_ID){
		//设置roll轴反馈//roll轴回中     
		gimbalData.rollAngleFbd = getInstallDirect(ROLL_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.rollMotorAngle;   		
		gimbalData.rollAngleRef = rollRampAngle * (1 - LinearRampCalc(&rollRamp,2));
	}
	//等待pitch、yaw轴回中
	if(((gimbalData.yawMotorAngle < 0.5f && gimbalData.yawMotorAngle > -0.5f)\
	&&(gimbalData.pitchMotorAngle < 0.5f && gimbalData.pitchMotorAngle > -0.5f))\
	|| xTaskGetTickCount() - xLastWakeTime > getConfigData()->backCenterTime * 2){
        //设置云台模式为跟随模式
		gimbalData.ctrlMode  = GIMBAL_NORMAL;		
		if ((robotConfigData.typeOfRobot == SENTRY_ID)
			||(robotConfigData.typeOfRobot == UAV_ID)){
            //设置底盘模式为分离模式
			getchassisData()->ctrlMode = CHASSIS_SEPARATE_GIMBAL;		
        }																																														
		else{
            //设置底盘模式为跟随模式
			getchassisData()->ctrlMode = CHASSIS_FOLLOW_GIMBAL;	
        }				
		gimbalData.pitchAngleSave = gimbalData.pitchGyroAngle;		
		//保存从初始化模式到跟随模式的陀螺仪角度
		gimbalData.yawAngleSave = gimbalData.yawGyroAngle;
		//期望值清零
		digitalClan(&gimbalData.yawAngleRef);                    	
		digitalClan(&gimbalData.pitchAngleRef);
		if(ROBOT == UAV_ID){
			if(gimbalData.rollMotorAngle < 0.5f && gimbalData.rollMotorAngle > -0.5f){
				//保存从初始化模式到跟随模式的陀螺仪角度
				gimbalData.rollAngleSave = gimbalData.rollGyroAngle;
				//期望值清零
				digitalClan(&gimbalData.rollAngleRef);
				//云台回中完成标志置一
				digitalHi(&gimbalData.initFinishFlag);
			}
		}
		else
			//云台回中完成标志置一
			digitalHi(&gimbalData.initFinishFlag);                   	
	}
}

static void gimbalStopHandle(void){
	static f32_t pitchRampAngle;
	static f32_t yawRampAngle;
	static float rollRampAngle;
	static TickType_t xLastWakeTime = 0;
	//每一次进入初始化模式都获取当前角度并初始化斜坡函数
	if(getlastrobotMode() != getrobotMode() || gimbalData.ctrlMode != gimbalData.lastCtrlMode){																						
		xLastWakeTime = xTaskGetTickCount();
		gimbalRampInit();
		//获取pitch码盘当前角度
		pitchRampAngle = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;  			
		//获取yaw码盘当前角度			
		yawRampAngle   = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;    						
		if(ROBOT == UAV_ID)
			//获取roll码盘当前角度     
			rollRampAngle   = getInstallDirect(ROLL_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.rollMotorAngle;
	}
	//设置pitch轴反馈	
	gimbalData.pitchAngleFbd = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;				
	//设置yaw轴反馈
	gimbalData.yawAngleFbd = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;  						
	if(ROBOT == UAV_ID)
		//设置roll轴反馈      
		gimbalData.rollAngleFbd = getInstallDirect(ROLL_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.rollMotorAngle;
	if(gimbalData.motorFlag){
		//pitch轴回中
		gimbalData.pitchAngleRef = pitchRampAngle + (gimbalData.pitchAngleStop - pitchRampAngle) * LinearRampCalc(&pitchRamp,1);			
		//yaw轴回中	  
		gimbalData.yawAngleRef = yawRampAngle + (gimbalData.yawAngleStop - yawRampAngle) * LinearRampCalc(&yawRamp,1);					
		if(ROBOT == UAV_ID)
			//roll轴回中	    
			gimbalData.rollAngleRef = rollRampAngle + (gimbalData.rollAngleStop - rollRampAngle) * LinearRampCalc(&rollRamp,1);
	}
	else{
		digitalClan(&gimbalData.pitchAngleStop);
		digitalClan(&gimbalData.yawAngleStop);
		digitalClan(&gimbalData.rollAngleStop);
        //pitch轴回中
		gimbalData.pitchAngleRef = pitchRampAngle * (1 - LinearRampCalc(&pitchRamp,2));													
		//yaw轴回中
		gimbalData.yawAngleRef = yawRampAngle * (1 - LinearRampCalc(&yawRamp,2));
		if(ROBOT == UAV_ID)
			//roll轴回中
			gimbalData.rollAngleRef = rollRampAngle * (1 - LinearRampCalc(&rollRamp,2));
	}

  	//等待pitch、yaw轴回中
	if(((gimbalData.yawMotorAngle < gimbalData.yawAngleStop + 2.0f && \
        gimbalData.yawMotorAngle > gimbalData.yawAngleStop - 2.0f)\
		&& (gimbalData.pitchMotorAngle < gimbalData.pitchAngleStop + 2.0f && \
        gimbalData.pitchMotorAngle > gimbalData.pitchAngleStop - 2.0f))\
		|| xTaskGetTickCount() - xLastWakeTime > getConfigData()->backCenterTime * 2 ){
        //保存从初始化模式到跟随模式的陀螺仪角度
		gimbalData.pitchAngleSave = gimbalData.pitchGyroAngle;								
		gimbalData.yawAngleSave = gimbalData.yawGyroAngle;
		if(ROBOT == UAV_ID){
			if(gimbalData.rollMotorAngle < gimbalData.rollAngleStop + 2.0f && \
			   gimbalData.rollMotorAngle > gimbalData.rollAngleStop - 2.0f){
			   gimbalData.rollAngleSave = gimbalData.rollGyroAngle;
			   if(!gimbalData.motorFlag)
				   digitalClan(&gimbalData.rollAngleRef);
		   }
	   }
		if(!gimbalData.motorFlag){
			//期望值清零
			digitalClan(&gimbalData.yawAngleRef);                   						
			digitalClan(&gimbalData.pitchAngleRef);
		}		
	}
}

static void gimbalRelaxHandle(void){
	 digitalLo(&gimbalData.initFinishFlag);
	 digitalLo(&gimbalData.motorFlag);
	 pidZeroState(gimbalData.pitchAnglePID);
	 pidZeroState(gimbalData.pitchSpeedPID);
	 pidZeroState(gimbalData.yawAnglePID);
	 pidZeroState(gimbalData.yawSpeedPID);
	 pidZeroState(gimbalData.rollAnglePID);
	 pidZeroState(gimbalData.rollSpeedPID);
	 gimbalData.ctrlMode = GIMBAL_INIT;
	 gimbalData.chassisChange = getConfigData()->yawCenter;
}

//云台闭角度环
void gimbalStopSwitch(uint8_t active){														
	if(active){
		digitalHi(&gimbalData.motorFlag);																					
		gimbalData.yawAngleStop = gimbalData.yawAngleStopSet;
		gimbalData.pitchAngleStop = gimbalData.pitchAngleStopSet;
		gimbalData.ctrlMode = GIMBAL_STOP;
		getchassisData()->ctrlMode = CHASSIS_SEPARATE_GIMBAL;
	}
	else{
		gimbalData.ctrlMode = GIMBAL_INIT;
		digitalLo(&gimbalData.motorFlag);
	}	
}

//斜坡函数初始化
void gimbalRampInit(void){ 													  				
    RampInit(&pitchRamp, getConfigData()->backCenterTime/2);
    RampInit(&yawRamp, getConfigData()->backCenterTime/2);
		RampInit(&rollRamp, getConfigData()->backCenterTime/2);
    RampInit(&pitchVisionRamp, 100);
    RampInit(&yawVisionRamp, 100);
}

void fullReverse(void){
	pidZeroState(gimbalData.pitchAnglePID);
	pidZeroState(gimbalData.pitchSpeedPID);
	pidZeroState(gimbalData.yawAnglePID);
	pidZeroState(gimbalData.yawSpeedPID);
	pidZeroState(gimbalData.rollAnglePID);
	pidZeroState(gimbalData.rollSpeedPID);
	digitalClan(&gimbalData.yawAngleOut);
	digitalClan(&gimbalData.pitchAngleOut);
	digitalClan(&gimbalData.rollAngleOut);
	digitalClan(&gimbalData.yawSpeedOut);
	digitalClan(&gimbalData.pitchSpeedOut);
	digitalClan(&gimbalData.rollSpeedOut);
	gimbalData.pitchAngleRef = gimbalData.pitchAngleFbd;				
	gimbalData.yawAngleRef = gimbalData.yawAngleFbd;
	gimbalData.rollAngleRef = gimbalData.rollAngleFbd;
}

void gimbalInit(void){
	if(ROBOT == INFANTRY_ID)
	BSP_GPIO_Init(BSP_GPIOB8,GPIO_Mode_IPU);
	gimbalData.ctrlMode  = GIMBAL_RELAX;
	gimbalRampInit();
	gimbalData.pitchSpeedPID = pidInit( &getConfigData()->pitchRatePID);
	gimbalData.pitchAnglePID = pidInit( &getConfigData()->pitchAnglePID);
  gimbalData.yawSpeedPID = pidInit(&getConfigData()->yawRatePID);
	gimbalData.yawAnglePID = pidInit( &getConfigData()->yawAnglePID);
	gimbalData.rollSpeedPID = pidInit(&getConfigData()->rollRatePID);
	gimbalData.rollAnglePID = pidInit(&getConfigData()->rollAnglePID);
	gimbalData.chassisChange = getConfigData()->yawCenter;
	//期望值给0保持水平        
	digitalClan(&gimbalData.rollAngleRef);
}
