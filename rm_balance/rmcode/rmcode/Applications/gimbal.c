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
�� �� ����	gimbalUpdate
��		�ܣ���̨�������
��ڲ�����	gimbalData.visionYawCmd				��������ϵ����
			gimbalData.visionPitchCmd			��������ϵ����
			gimbalData.autoMode					�л��Զ������־
�� �� ֵ��	��
Ӧ�÷�Χ��	�ⲿ����
��		ע��
***************************************************
*/

gimbalStruct_t gimbalData;
//��̨б�³�ʼ��
ramp_t yawVisionRamp = RAMP_GEN_DAFAULT;											
ramp_t pitchVisionRamp = RAMP_GEN_DAFAULT;
//��̨б�³�ʼ��
static ramp_t yawRamp = RAMP_GEN_DAFAULT;											
static ramp_t pitchRamp = RAMP_GEN_DAFAULT;
static ramp_t rollRamp = RAMP_GEN_DAFAULT;
gimbalStruct_t* getGimbalData(){
    return &gimbalData;
}

int8_t getInstallDirect(uint8_t installPara,bool type){         //YAW�ᰲװ����
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
	//ƽ�ⲽ��
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
	
	switch (getrobotMode()){	//��ʼ��ģʽ
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
        //����ģʽ
		case MODE_STOP:{	 
			gimbalStopHandle();	 
			break;
		}
        //�������Ȩ
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
	gimbalData.yawEncoder = motorHandleClass.Encoder(&commandoMotorConfig[YAWMOTOR]);//yaw����������ֵ 

	//�����Ѿ�����
	if(gimbalData.autoMode && getvisionData()->captureFlag && getvisionData()->cailSuccess){
        //�Զ�ģʽ��ͬ����ǰֵ���Ƕ�����
		gimbalData.pitchAngleRef = getvisionData()->pitchCmd;				 
		gimbalData.yawAngleRef = getvisionData()->yawCmd;	
		//���������״̬��̨�Ƕȷ���ȡ�����ǽǶȣ�����ƫ�ã�
		gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;
		gimbalData.yawAngleFbd = gimbalData.yawGyroAngle - gimbalData.yawAngleSave;
	}
	//��̨�Ȼ���ģʽ
	if(gimbalData.autoMode && (ROBOT == P_TANK_ID)&&(autoTaskData->currentTask == TANK_MORTAR)){ 
        //�Զ�ģʽ��ͬ����ǰֵ���Ƕ�����
		gimbalData.pitchAngleRef = getvisionData()->mortarPitCmd;				
		gimbalData.yawAngleRef = getvisionData()->mortarYawCmd;	
		//���������״̬��̨�Ƕȷ���ȡ�����ǽǶȣ�����ƫ�ã�
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
	//ֻ��������̨����ʱ��ִ��
	if(getrobotMode() == MODE_KM && !(gimbalData.autoMode && getvisionData()->captureFlag && getvisionData()->cailSuccess) \
		&& !getinfantryAutoData()->rotateFlag && ROBOT == INFANTRY_ID){      //Ϊʲôֻ���ٶȻ�
		//����ƶ�ʱֻ���ٶȻ�
		gimbalData.yawSpeedRef = keyBoardCtrlData.yawSpeedTarget;
		//����û���ٶ�����ʱ�սǶȻ�
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
	// �ٶȻ�������imu���ٶ��ٶ�
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
else{//����Ӣ�۵ĵ����װ˳����������ֲ�һ��
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
		//�����˶���ֹͣ�������Դ�ʱ�Ƕȱջ�
		if(!keyBoardCtrlData.yawSpeedTarget && lastYawSpeedRef){
			gimbalData.angleCycleStep = 1;
		}
		//�ȵ���̨�ٶȽӽ�Ϊ0ʱ����ʼ�սǶȻ�
		if((fabs(IMU_RATEZ) < 0.08f) && (gimbalData.angleCycleStep == 1)){
			gimbalData.yawAngleRef = gimbalData.yawAngleFbd;
			gimbalData.angleCycleStep = 2;
		}
		lastYawSpeedRef = keyBoardCtrlData.yawSpeedTarget;
	}
#endif
	//pitch��Ƕ��޷�
	gimbalData.pitchAngleRef = constrainFloat(gimbalData.pitchAngleRef,getConfigData()->pitchMinRange,getConfigData()->pitchMaxRange); 
}

static void gimbalInitHandle(void){	
	static f32_t pitchRampAngle;
	static f32_t yawRampAngle;
	static float rollRampAngle;
	static TickType_t xLastWakeTime = 0;
    //ÿһ�ν����ʼ��ģʽ����ȡ��ǰ�ǶȲ���ʼ��б�º���
	if(getlastrobotMode() != getrobotMode()||gimbalData.ctrlMode != gimbalData.lastCtrlMode){																			
		xLastWakeTime = xTaskGetTickCount();
		gimbalRampInit();
        //��ȡpitch��imu��ǰ�Ƕ�
		pitchRampAngle = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;  				
        //��ȡyaw���̵�ǰ�Ƕ�
		yawRampAngle   = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;   					
		if(ROBOT == UAV_ID)
			//��ȡroll���̵�ǰ�Ƕ�     
			rollRampAngle   = getInstallDirect(ROLL_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.rollMotorAngle;
	}
    //����pitch�ᷴ��//pitch�����
	gimbalData.pitchAngleFbd = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;	
	gimbalData.pitchAngleRef = pitchRampAngle * (1 - LinearRampCalc(&pitchRamp,2));																			
    //����yaw�ᷴ��//yaw�����
	gimbalData.yawAngleFbd = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;   		
	gimbalData.yawAngleRef = yawRampAngle * (1 - LinearRampCalc(&yawRamp,2));       																		
  	if(ROBOT == UAV_ID){
		//����roll�ᷴ��//roll�����     
		gimbalData.rollAngleFbd = getInstallDirect(ROLL_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.rollMotorAngle;   		
		gimbalData.rollAngleRef = rollRampAngle * (1 - LinearRampCalc(&rollRamp,2));
	}
	//�ȴ�pitch��yaw�����
	if(((gimbalData.yawMotorAngle < 0.5f && gimbalData.yawMotorAngle > -0.5f)\
	&&(gimbalData.pitchMotorAngle < 0.5f && gimbalData.pitchMotorAngle > -0.5f))\
	|| xTaskGetTickCount() - xLastWakeTime > getConfigData()->backCenterTime * 2){
        //������̨ģʽΪ����ģʽ
		gimbalData.ctrlMode  = GIMBAL_NORMAL;		
		if ((robotConfigData.typeOfRobot == SENTRY_ID)
			||(robotConfigData.typeOfRobot == UAV_ID)){
            //���õ���ģʽΪ����ģʽ
			getchassisData()->ctrlMode = CHASSIS_SEPARATE_GIMBAL;		
        }																																														
		else{
            //���õ���ģʽΪ����ģʽ
			getchassisData()->ctrlMode = CHASSIS_FOLLOW_GIMBAL;	
        }				
		gimbalData.pitchAngleSave = gimbalData.pitchGyroAngle;		
		//����ӳ�ʼ��ģʽ������ģʽ�������ǽǶ�
		gimbalData.yawAngleSave = gimbalData.yawGyroAngle;
		//����ֵ����
		digitalClan(&gimbalData.yawAngleRef);                    	
		digitalClan(&gimbalData.pitchAngleRef);
		if(ROBOT == UAV_ID){
			if(gimbalData.rollMotorAngle < 0.5f && gimbalData.rollMotorAngle > -0.5f){
				//����ӳ�ʼ��ģʽ������ģʽ�������ǽǶ�
				gimbalData.rollAngleSave = gimbalData.rollGyroAngle;
				//����ֵ����
				digitalClan(&gimbalData.rollAngleRef);
				//��̨������ɱ�־��һ
				digitalHi(&gimbalData.initFinishFlag);
			}
		}
		else
			//��̨������ɱ�־��һ
			digitalHi(&gimbalData.initFinishFlag);                   	
	}
}

static void gimbalStopHandle(void){
	static f32_t pitchRampAngle;
	static f32_t yawRampAngle;
	static float rollRampAngle;
	static TickType_t xLastWakeTime = 0;
	//ÿһ�ν����ʼ��ģʽ����ȡ��ǰ�ǶȲ���ʼ��б�º���
	if(getlastrobotMode() != getrobotMode() || gimbalData.ctrlMode != gimbalData.lastCtrlMode){																						
		xLastWakeTime = xTaskGetTickCount();
		gimbalRampInit();
		//��ȡpitch���̵�ǰ�Ƕ�
		pitchRampAngle = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;  			
		//��ȡyaw���̵�ǰ�Ƕ�			
		yawRampAngle   = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;    						
		if(ROBOT == UAV_ID)
			//��ȡroll���̵�ǰ�Ƕ�     
			rollRampAngle   = getInstallDirect(ROLL_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.rollMotorAngle;
	}
	//����pitch�ᷴ��	
	gimbalData.pitchAngleFbd = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;				
	//����yaw�ᷴ��
	gimbalData.yawAngleFbd = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;  						
	if(ROBOT == UAV_ID)
		//����roll�ᷴ��      
		gimbalData.rollAngleFbd = getInstallDirect(ROLL_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.rollMotorAngle;
	if(gimbalData.motorFlag){
		//pitch�����
		gimbalData.pitchAngleRef = pitchRampAngle + (gimbalData.pitchAngleStop - pitchRampAngle) * LinearRampCalc(&pitchRamp,1);			
		//yaw�����	  
		gimbalData.yawAngleRef = yawRampAngle + (gimbalData.yawAngleStop - yawRampAngle) * LinearRampCalc(&yawRamp,1);					
		if(ROBOT == UAV_ID)
			//roll�����	    
			gimbalData.rollAngleRef = rollRampAngle + (gimbalData.rollAngleStop - rollRampAngle) * LinearRampCalc(&rollRamp,1);
	}
	else{
		digitalClan(&gimbalData.pitchAngleStop);
		digitalClan(&gimbalData.yawAngleStop);
		digitalClan(&gimbalData.rollAngleStop);
        //pitch�����
		gimbalData.pitchAngleRef = pitchRampAngle * (1 - LinearRampCalc(&pitchRamp,2));													
		//yaw�����
		gimbalData.yawAngleRef = yawRampAngle * (1 - LinearRampCalc(&yawRamp,2));
		if(ROBOT == UAV_ID)
			//roll�����
			gimbalData.rollAngleRef = rollRampAngle * (1 - LinearRampCalc(&rollRamp,2));
	}

  	//�ȴ�pitch��yaw�����
	if(((gimbalData.yawMotorAngle < gimbalData.yawAngleStop + 2.0f && \
        gimbalData.yawMotorAngle > gimbalData.yawAngleStop - 2.0f)\
		&& (gimbalData.pitchMotorAngle < gimbalData.pitchAngleStop + 2.0f && \
        gimbalData.pitchMotorAngle > gimbalData.pitchAngleStop - 2.0f))\
		|| xTaskGetTickCount() - xLastWakeTime > getConfigData()->backCenterTime * 2 ){
        //����ӳ�ʼ��ģʽ������ģʽ�������ǽǶ�
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
			//����ֵ����
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

//��̨�սǶȻ�
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

//б�º�����ʼ��
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
	//����ֵ��0����ˮƽ        
	digitalClan(&gimbalData.rollAngleRef);
}
