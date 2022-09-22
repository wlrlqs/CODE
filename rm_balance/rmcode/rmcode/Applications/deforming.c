#include "deforming.h"
#include "rc.h"
#include "keyboard.h"
#include "config.h"
#include "ramp.h"
#include "DRIVER_VL53L0X.h"
#include "Driver_ADC.h"
#include "Driver_ITV.h"
#include "judge.h"
#include "chassis.h"
#include "gimbal.h"
#include "cansend.h"
#include "control.h"
#include "auto_tank.h"
#include "auto_auxiliary.h"
#include "imu.h"
#include "motorHandle.h"
auxiliaryStruct_t auxiliaryDeformingData; 
sentryDeformingStruct_t sentryDeformingData;

auxiliaryStruct_t* getAuxiliaryDeformingData(){
    return &auxiliaryDeformingData;
}
static void chassisNothing(){
	auxiliaryDeformingData.elevatorAngle[0].dataOut = -130.0;
	auxiliaryDeformingData.elevatorAngle[1].dataOut = -130.0;
	digitalLo(&auxiliaryDeformingData.heightstatebreak_Flag);
}
//static void X_Left(){
//	auxiliaryDeformingData.xSlidewaySpeed.dataRef = -7000.0;
//	digitalHi(&auxiliaryDeformingData.X_statebreak_Flag);	
//}
//static void X_Right(){
//	auxiliaryDeformingData.xSlidewaySpeed.dataRef = 7000.0;
//	digitalHi(&auxiliaryDeformingData.X_statebreak_Flag);
//}
//static void X_Stop(){
//	auxiliaryDeformingData.xSlidewaySpeed.dataRef = 0.0;
//	digitalLo(&auxiliaryDeformingData.X_statebreak_Flag);	
//}
//static void Y_Forward(){
//	auxiliaryDeformingData.ySlidewaySpeed.dataRef = -2500.0;
//	digitalHi(&auxiliaryDeformingData.Y_statebreak_Flag);
//}
//static void Y_Back(){
//	auxiliaryDeformingData.ySlidewaySpeed.dataRef = 2500.0;
//	digitalHi(&auxiliaryDeformingData.Y_statebreak_Flag);
//}
//static void Y_Stop(){
//	auxiliaryDeformingData.ySlidewaySpeed.dataRef = 0.0;
//	digitalLo(&auxiliaryDeformingData.Y_statebreak_Flag);
//}
//����
void GO_TO_HEIGHT(uint16_t HEIGHT_TO_GET){

	if((auxiliaryDeformingData.heightstatebreak_Flag) &&(HEIGHT_TO_GET > 0) && (HEIGHT_TO_GET < HIGHESTHEIGHT_GET)){
		auxiliaryDeformingData.elevatorAngle[0].dataRef = HEIGHT_TO_GET;
		auxiliaryDeformingData.elevatorAngle[1].dataRef = HEIGHT_TO_GET;
		for(uint8_t index = 0;index < 2;index++){
			if((auxiliaryDeformingData.elevatorAngle[index].dataFbd < (HEIGHT_TO_GET+10)) && (auxiliaryDeformingData.elevatorAngle[index].dataFbd > (HEIGHT_TO_GET -10))){
				digitalLo(&auxiliaryDeformingData.heightstatebreak_Flag);
				digitalLo(&auxiliaryDeformingData.elevatorSpeedLimitFlag);
			}	
	}		
	}
	else{
		digitalLo(&auxiliaryDeformingData.heightstatebreak_Flag);
		digitalLo(&auxiliaryDeformingData.elevatorSpeedLimitFlag);		
	}
}

//void LEFT_TO_X(uint8_t X_TO_GET){
//	if(auxiliaryDeformingData.X_statebreak_Flag){
//		if(X_TO_GET || X_LEFT_GET){
//			X_Stop();
//			digitalLo(&auxiliaryDeformingData.X_PlaceJudgeFlag);
//		}
//		else{
//			 X_Left();
//		}	
//	}
//}
//void RIGHT_TO_X(uint8_t X_TO_GET){
//	if(auxiliaryDeformingData.X_statebreak_Flag){
//		if(X_TO_GET || X_RIGHT_GET){
//			X_Stop();
//			digitalLo(&auxiliaryDeformingData.X_PlaceJudgeFlag);
//		}
//		else{
//			 X_Right();
//		}	
//	}	
//}
//void ADVANCE_TO_Y(uint8_t Y_TO_GET){
//	if(auxiliaryDeformingData.Y_statebreak_Flag){
//		if(Y_TO_GET || Y_FRONT_GET){
//			Y_Stop();
//		}
//		else{
//			 Y_Forward();
//		}	
//	}	
//}
//void BACK_TO_Y(uint8_t Y_TO_GET){
//	if(auxiliaryDeformingData.Y_statebreak_Flag){
//		if(Y_TO_GET || Y_BEHIND_GET){
//			Y_Stop();
//		}
//		else{
//			 Y_Back();
//		}	
//	}	
//}
//uint8_t  returnToBack_Y(){
//	uint8_t returnstate = 0;
//	if(!Y_BEHIND_GET){
//		digitalHi(&auxiliaryDeformingData.Y_statebreak_Flag);
//		BACK_TO_Y(Y_BEHIND_GET);
//		returnstate = 0;
//	}
//	else{
//		Y_Stop();
//		returnstate = 1;
//	}
//	
//	return returnstate;
//}
//static void X_MalpositionToMiddle(){
//	if(X_MIDDLE_GET){
//		X_Stop();
//		digitalLo(&auxiliaryDeformingData.X_PlaceJudgeFlag);
//	}
//	else
//		if(X_RIGHT_GET){
//		digitalLo(&auxiliaryDeformingData.X_PlaceJudgeFlag);
//	}
//	else{
//		RIGHT_TO_X(X_RIGHT_GET);		
//	}
//}
//uint8_t retuernToMiddle_X(){
//	static uint8_t X_Place;
//	uint8_t returnstate = 0;
//	if(!auxiliaryDeformingData.X_PlaceJudgeFlag){
//		if(X_RIGHT_GET){
//			X_Place = X_RIGHT;
//			digitalHi(&auxiliaryDeformingData.X_statebreak_Flag);	
//		}
//		else
//			if(X_LEFT_GET){
//				X_Place = X_LEFT;
//				digitalHi(&auxiliaryDeformingData.X_statebreak_Flag);		
//			}		
//		else
//			if(X_MIDDLE_GET){
//				X_Place = X_MIDDLE;
//			}
//			else{
//				X_Place = X_MALPOSITION;
//				digitalHi(&auxiliaryDeformingData.X_statebreak_Flag);	
//			}
//	digitalHi(&auxiliaryDeformingData.X_PlaceJudgeFlag);
//	}	
//	
//    switch(X_Place){
//		case X_LEFT:{
//			RIGHT_TO_X(X_MIDDLE_GET);
//			returnstate = 0;
//			break;
//		}
//		case X_RIGHT:{
//			LEFT_TO_X(X_MIDDLE_GET);
//			returnstate = 0;
//			break;
//		}
//		case X_MIDDLE:{
//			returnstate = 1;
//			digitalLo(&auxiliaryDeformingData.X_PlaceJudgeFlag);
//			break;
//		}
//		case X_MALPOSITION:{
//			X_MalpositionToMiddle();
//			returnstate = 0;
//			break;
//		}		
//    }
//	test_X_Place = X_Place;
//	return returnstate;
//}
//void reset_X_Y_ALL(){//!!! �ø�λ�Դ��߶ȱ仯����ʼ�߶ȱ��벻��ͻ�������
//	if(!auxiliaryDeformingData.X_PlaceInitFinishFlag){
//		if(retuernToMiddle_X()){
//			digitalHi(&auxiliaryDeformingData.X_PlaceInitFinishFlag);
//		}     
//	}//Y�����
//	if(!auxiliaryDeformingData.Y_PlaceInitFinishFlag){
//		if(returnToBack_Y()){
//			digitalHi(&auxiliaryDeformingData.Y_PlaceInitFinishFlag);
//		}
//	}//X�����
//	if(!auxiliaryDeformingData.AllInitFinishFlag){
//		ORE1_OFF;
//		ORE2_OFF;
//		PAW_IN;
//		PAW_ON;	
//		if(returnToBack_Y() && retuernToMiddle_X() && (!auxiliaryDeformingData.X_PlaceJudgeFlag)&& (!auxiliaryDeformingData.Y_statebreak_Flag)){
//			digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
//			GO_TO_HEIGHT(FOURTH_HEIGHT_GET);
//            if(!auxiliaryDeformingData.heightstatebreak_Flag){
//				digitalHi(&auxiliaryDeformingData.AllInitFinishFlag);
//			}				
//		}
//	}//��������
//}
void protectDeforming(){
//	BULLET_SUPPLY_OFF;
//	POP_UP_OFF;
//	PAW_DOWN;
//	PAW_ON;
//	PAW_IN;
	//UPPER_LEFT_OFF;
	//UPPER_RIGHT_OFF;
	//UPPER_FRONT_OFF;
}

void setDeformingZero(){
//    chassisNothing();
//    BULLET_SUPPLY_OFF;
//    PAW_ON;
//    PAW_IN;
//    UPPER_LEFT_OFF;
//    UPPER_RIGHT_OFF;
//    UPPER_FRONT_OFF;
//    POP_UP_OFF;
}

/*
***************************************************
�� �� ����	motorRawAngle
��    �ܣ�  ���λ�����ݹ��㴦��
��ڲ�����	��
�� �� ֵ��	��
Ӧ�÷�Χ��	���ļ��ڵ���
��   ע��
***************************************************
*/	
//void motorRawAngle(rmmotorTarData_t* tarMotor,motorCanDataRecv_t* srcMotor){
//	tarMotor->last_fdbPosition = tarMotor->fdbPosition;
//	tarMotor->fdbPosition = srcMotor->rawangle;
//	if(tarMotor->fdbPosition - tarMotor->last_fdbPosition >= 8075)
//	{
//		tarMotor->round --;
//	}
//	else if(tarMotor -> fdbPosition - tarMotor->last_fdbPosition < 16)
//	{
//		tarMotor->round ++;
//	}
//	tarMotor->real_position = tarMotor->fdbPosition + tarMotor->round * 8191;
//}
//void chassisDeformingUpdata(){                                      //���̸�
//    static uint8_t nowSwitch = 0;
//    static uint8_t lastSwitch = 0;
//    nowSwitch = RC_GEAR;
//    
//    lastSwitch = nowSwitch;

//}
void chassisChaseSwitch(uint8_t isEnable){
	if(isEnable){
        //�������̸���
		getchassisData()->chaseRef = 0;													
	}
	else{
        //�رյ��̸���
		getchassisData()->chaseRef = getchassisData()->chaseFbd;			
	}
}
/****************���̱��ε��pid����********************/
void auxilaryMotorPIDUpdate(){
	static uint16_t changePneumaCount;
    static uint8_t upFlag = 0;
	 PBout(8) = 0;

    auxiliaryDeformingData.time[0] = getClockCount();
	auxiliaryDeformingData.intervalTime = (f32_t)(auxiliaryDeformingData.time[0] - auxiliaryDeformingData.time[1]);
	auxiliaryDeformingData.time[1] = auxiliaryDeformingData.time[0];
	if((!(getauxiliaryAutoData()->reversingLockFlag)) && (getrobotMode() == MODE_KM)){
	   if(remoteControlData.dt7Value.mouse.Press_L){	   
	       auxiliaryDeformingData.reversingSpeed[0].dataRef = 1500.0;
		   auxiliaryDeformingData.reversingSpeed[1].dataRef = 1500.0;
	   }
   else 
	   if(remoteControlData.dt7Value.mouse.Press_R){
	       auxiliaryDeformingData.reversingSpeed[0].dataRef = -1500.0;
		   auxiliaryDeformingData.reversingSpeed[1].dataRef = -1500.0;
	   }
	   else{
	       auxiliaryDeformingData.reversingSpeed[0].dataRef = 0.0;
		   auxiliaryDeformingData.reversingSpeed[1].dataRef =0.0;
	   }
   }
	//reset_X_Y_ALL();
   if(pneumaticData.pneumatic_1.millayData_mm[0] == 0){//���ߴ��������ѱ���
	   pneumaticData.pneumatic_1.millayData_mm[0] = pneumaticData.pneumatic_1.millayData_mm[1];
   }
   else
		if(pneumaticData.pneumatic_1.millayData_mm[1] == 0){
	   pneumaticData.pneumatic_1.millayData_mm[1] = pneumaticData.pneumatic_1.millayData_mm[0];
   }
	//��������ո߶Ȼ�
	for(uint8_t index = 0;index < 2;index++){
       auxiliaryDeformingData.elevatorAngle[index].dataFbd = (f32_t)pneumaticData.pneumatic_1.millayData_mm[index];
       auxiliaryDeformingData.elevatorAngle[index].dataOut = pidUpdate(auxiliaryDeformingData.elevatorAnglePID[index], \
       auxiliaryDeformingData.elevatorAngle[index].dataRef, auxiliaryDeformingData.elevatorAngle[index].dataFbd,auxiliaryDeformingData.intervalTime);
   }
   if((RC_GEAR  == RCSW_TOP) && (RC_MODE  == RCSW_BOTTOM) && (RC_TRANSVERSE >490) && (RC_LONGITUDINAL >490)){	   
	   changePneumaCount++;	   
   }
   else
	   if(!auxiliaryDeformingData.heightstatebreak_Flag){
		   changePneumaCount = 0;
	   }
   if(changePneumaCount > 1000){
	   digitalHi(&auxiliaryDeformingData.heightstatebreak_Flag);
	   if(!upFlag){
		   GO_TO_HEIGHT(FIFTH_HEIGHT_GET);
		   if(!auxiliaryDeformingData.heightstatebreak_Flag){
			  digitalHi(&upFlag);
			  changePneumaCount =0;
		   }
	   }
	   else
		   if(upFlag){
			   GO_TO_HEIGHT(NORMAL_HEIGHT_GET);
			   if(!auxiliaryDeformingData.heightstatebreak_Flag){
				  digitalLo(&upFlag);
				  changePneumaCount =0;
			   }  
		   } 
   }
	if(RC_GEAR  == RCSW_MID){
		chassisNothing();
	}
	
	//����������ٶȻ�
   for(uint8_t index = 0;index < 2;index++){
	   auxiliaryDeformingData.elevatorSpeed[index].dataRef = -auxiliaryDeformingData.elevatorAngle[index].dataOut;
       auxiliaryDeformingData.elevatorSpeed[index].dataFbd = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[OUTFIT2 + index]);
       auxiliaryDeformingData.elevatorSpeed[index].dataOut = pidUpdate(auxiliaryDeformingData.elevatorSpeedPID[index], \
       auxiliaryDeformingData.elevatorSpeed[index].dataRef, auxiliaryDeformingData.elevatorSpeed[index].dataFbd,auxiliaryDeformingData.intervalTime);
   }
   
   for(uint8_t index = 0;index < 2;index++){	  
       auxiliaryDeformingData.reversingSpeed[index].dataFbd = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[OUTFIT4 + index]);
       auxiliaryDeformingData.reversingSpeed[index].dataOut = pidUpdate(auxiliaryDeformingData.reversingSpeedPID[index], \
       auxiliaryDeformingData.reversingSpeed[index].dataRef, auxiliaryDeformingData.reversingSpeed[index].dataFbd,auxiliaryDeformingData.intervalTime);
   }

   //X��
//   auxiliaryDeformingData.xSlidewaySpeed.dataFbd = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[OUTFIT4]);
//   auxiliaryDeformingData.xSlidewaySpeed.dataOut = pidUpdate(auxiliaryDeformingData.xSlidewaySpeedPID, \
//   auxiliaryDeformingData.xSlidewaySpeed.dataRef, auxiliaryDeformingData.xSlidewaySpeed.dataFbd,auxiliaryDeformingData.intervalTime);
   //Y��
//   auxiliaryDeformingData.ySlidewaySpeed.dataFbd = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[OUTFIT5]);
//   auxiliaryDeformingData.ySlidewaySpeed.dataOut = pidUpdate(auxiliaryDeformingData.ySlidewaySpeedPID, \
//   auxiliaryDeformingData.ySlidewaySpeed.dataRef, auxiliaryDeformingData.ySlidewaySpeed.dataFbd,auxiliaryDeformingData.intervalTime); 
   //ǰצ
//   auxiliaryDeformingData.frontClawSpeed.dataFbd = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[OUTFIT1]);
//   auxiliaryDeformingData.frontClawSpeed.dataOut = pidUpdate(auxiliaryDeformingData.frontClawSpeedPID, \
//   auxiliaryDeformingData.frontClawSpeed.dataRef, auxiliaryDeformingData.frontClawSpeed.dataFbd,auxiliaryDeformingData.intervalTime);   
   
   if(isnan(auxiliaryDeformingData.elevatorSpeed[0].dataOut) || isnan(auxiliaryDeformingData.elevatorSpeed[1].dataOut) || isnan(auxiliaryDeformingData.reversingSpeed[0].dataOut) || isnan(auxiliaryDeformingData.reversingSpeed[1].dataOut)){
	   pidZeroState(auxiliaryDeformingData.elevatorSpeedPID[0]);
	   pidZeroState(auxiliaryDeformingData.elevatorAnglePID[0]);
	   pidZeroState(auxiliaryDeformingData.elevatorSpeedPID[1]);
	   pidZeroState(auxiliaryDeformingData.elevatorAnglePID[1]);
	   pidZeroState(auxiliaryDeformingData.xSlidewaySpeedPID);
	   pidZeroState(auxiliaryDeformingData.ySlidewaySpeedPID);
	   pidZeroState(auxiliaryDeformingData.frontClawSpeedPID);
   	   pidZeroState(auxiliaryDeformingData.reversingSpeedPID[0]);
	   pidZeroState(auxiliaryDeformingData.reversingSpeedPID[1]);
		   
	   digitalClan(&auxiliaryDeformingData.elevatorSpeed[0].dataOut);
	   digitalClan(&auxiliaryDeformingData.elevatorSpeed[1].dataOut);
   
   	   digitalClan(&auxiliaryDeformingData.reversingSpeed[0].dataOut);
	   digitalClan(&auxiliaryDeformingData.reversingSpeed[1].dataOut);
   
	   digitalClan(&auxiliaryDeformingData.xSlidewaySpeed.dataOut);
	   digitalClan(&auxiliaryDeformingData.ySlidewaySpeed.dataOut);
	   digitalClan(&auxiliaryDeformingData.frontClawSpeed.dataOut);
		   
       auxiliaryDeformingData.elevatorSpeed[0].dataRef =  auxiliaryDeformingData.elevatorSpeed[0].dataFbd; 
	   auxiliaryDeformingData.elevatorSpeed[1].dataRef =  auxiliaryDeformingData.elevatorSpeed[1].dataFbd;
	   
	   auxiliaryDeformingData.reversingSpeed[0].dataRef =  auxiliaryDeformingData.reversingSpeed[0].dataFbd; 
	   auxiliaryDeformingData.reversingSpeed[1].dataRef =  auxiliaryDeformingData.reversingSpeed[1].dataFbd;
	   
       auxiliaryDeformingData.xSlidewaySpeed.dataRef =  auxiliaryDeformingData.xSlidewaySpeed.dataFbd; 
	   auxiliaryDeformingData.ySlidewaySpeed.dataRef =  auxiliaryDeformingData.ySlidewaySpeed.dataFbd;
       auxiliaryDeformingData.frontClawSpeed.dataRef =  auxiliaryDeformingData.frontClawSpeed.dataFbd; 		   
   }
}





/*************************
Ӣ�۳��߼�����
**************************/
void tankPneumaticUpdate(){
	p_tank_can1_sentData(0x405);
}

/*************************
�����߼�����
**************************/
//��������
void infantry_deformingTask(void){
	//�ص�ԭ״̬
	if(getinfantryAutoData()->breakSign){									
		digitalLo(&get_infDeforming()->change_finish);
		digitalLo(&get_infDeforming()->up_step_switch);
		get_infDeforming()->change_chassis_step = 99;
		getGimbalData()->chassisChange = getConfigData()->yawCenter;
		digitalClan(&getchassisData()->roDir);
		get_infDeforming()->arm_direction = ARM_STOP;
		getinfantryAutoData()->taskState = END_OF_EXECUTION;
		digitalLo(&getinfantryAutoData()->breakSign) ;
	}
	//�Ƚ��е��̻���
	int16_t rotateCode;
	//������ƫλ
	rotateCode = motorHandleClass.Encoder(&commandoMotorConfig[YAWMOTOR]) - getConfigData()->yawCenter;
	//��������
	if(T_OR_F0(rotateCode)) rotateCode = commandoMotorConfig[YAWMOTOR].motor_lib.motor_encoder + rotateCode;
	if(!get_infDeforming()->change_finish){
		
		switch(get_infDeforming()->change_chassis_step){
			case 0:{
				//�жϳ�ͷ�Ƿ��ڵ�������
				if((rotateCode <= (commandoMotorConfig[YAWMOTOR].motor_lib.motor_encoder / 4)) || (rotateCode >= ((commandoMotorConfig[YAWMOTOR].motor_lib.motor_encoder / 4) * 3))){
					getGimbalData()->chassisChange = getConfigData()->yawCenter + (commandoMotorConfig[YAWMOTOR].motor_lib.motor_encoder / 2);
     			digitalIncreasing(&get_infDeforming()->change_chassis_step);
				}
				//����ڵ�����������Ҫת����
				else
					digitalHi(&get_infDeforming()->change_finish);
			}break;
			case 1:{
				if((rotateCode > (commandoMotorConfig[YAWMOTOR].motor_lib.motor_encoder / 4)) && (rotateCode < ((commandoMotorConfig[YAWMOTOR].motor_lib.motor_encoder / 4) * 3))){
					//�˶�������
					digitalHi(&getchassisData()->roDir);
					digitalHi(&get_infDeforming()->change_finish);
					get_infDeforming()->change_chassis_step = 99;
				}
			}break;
			case 99:
				digitalClan(&getchassisData()->chaseRef);
				digitalClan(&get_infDeforming()->change_chassis_step);
			break;
		}
	}
	else
		//����ת����ͬ�����ͻ�е�ۿ���
		digitalHi(&get_infDeforming()->up_step_switch);
	//����Z��е����ǰ
	if(PRESS_Z && get_infDeforming()->up_step_switch)
		get_infDeforming()->arm_direction = ARM_FORWARD;
	//����X��е�ۺ���
	else if(PRESS_X && get_infDeforming()->up_step_switch)
		get_infDeforming()->arm_direction = ARM_BACK;
	//��е��ֹͣ
	else
		get_infDeforming()->arm_direction = ARM_STOP;
	
}

//ͬ��������
void synchronous_wheel_control(uint8_t __switch,f32_t intervalTime){
	if(__switch)
		getchassisData()->syncWheelRef = -5000.0f;
	else
		getchassisData()->syncWheelRef = 0.0f;
	getchassisData()->syncWheelFbd = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[SYNCHRONOUS_WHEEL]);
	getchassisData()->syncWheelOut = pidUpdate(getchassisData()->syncWheelPID,getchassisData()->syncWheelRef,getchassisData()->syncWheelFbd,intervalTime);
}

//��е�ۿ���
void arm_wheel_control(uint8_t keyControl,f32_t intervalTime){
	switch(keyControl){
		case ARM_STOP:
			getchassisData()->armWheelRef = 0.0f;
		break;
		case ARM_FORWARD:
			getchassisData()->armWheelRef = 2000.0f;
		break;
		case ARM_BACK:
			getchassisData()->armWheelRef = -2000.0f;
		break;
	}
	getchassisData()->armWheelFbd = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[ARM_WHEEL]);
	getchassisData()->armWheelOut = pidUpdate(getchassisData()->armWheelPID,getchassisData()->armWheelRef,getchassisData()->armWheelFbd,intervalTime);
}

//��̨�׻�������
void up_step_control(uint8_t __switch,f32_t intervalTime){
	synchronous_wheel_control(__switch,intervalTime);
	arm_wheel_control(get_infDeforming()->arm_direction,intervalTime);
}

/*************************
�������θ���
**************************/
static void auxiliaryDeformingUpdate(){
	pneumatic_can2_sentData(0x405);	
	auxilaryMotorPIDUpdate();
}
//��������εĿ���
void mechaDeformingUpdate(void)								
{  
    if(robotConfigData.typeOfRobot == P_TANK_ID){
        tankPneumaticUpdate();
    }
    else if(robotConfigData.typeOfRobot == AUXILIARY_ID ){
       auxiliaryDeformingUpdate();			
		if(judgeData.extGameRobotState.remain_HP < judgeData.extGameRobotState.max_HP*0.1f){
			protectDeforming();
		}
    }
	else if(robotConfigData.typeOfRobot == SENTRY_ID){ //�ڱ���
		if((getrobotMode() == MODE_RC) || (getrobotMode() == MODE_KM)){
			if(getrobotMode() == MODE_RC){
				if(SENTRY_REMOTE_SPEED > 0.0f){
					sentryDeformingData.direction = 1;
				}
				else if(SENTRY_REMOTE_SPEED < 0.0f){
					sentryDeformingData.direction = -1;
				}
				else{
					sentryDeformingData.direction = 0;
				}
			}
			//KMģʽ
			else{
				if(SENTRY_AUTO_SPEED == 0.0f){
					sentryDeformingData.direction = 0;
				}
	
			}
			sentryDeformingData.deformRef = SENTRY_DEFORMING_MOVREF;		
			sentryDeformingData.dataOut = sentryDeformingData.deformRef*sentryDeformingData.direction;
		}
	}
}


/*************************
���̳���ʼ��
**************************/
void auxiliaryDeformingInit(){
	//������ͷ�̵���
	BSP_GPIO_Init(REVERSING_GPIO,GPIO_Mode_Out_PP);
	REVERSING_FRONT;//REVERSING_BEHIND  REVERSING_FRONT
	auxiliaryDeformingData.elevatorSpeedPID[0] = pidInit(&getConfigData()->elevatorSpeedPID);
	auxiliaryDeformingData.elevatorSpeedPID[1] = pidInit(&getConfigData()->elevatorSpeedPID);
	auxiliaryDeformingData.elevatorAnglePID[0] = pidInit(&getConfigData()->elevatorAnglePID);
	auxiliaryDeformingData.elevatorAnglePID[1] = pidInit(&getConfigData()->elevatorAnglePID);	
	auxiliaryDeformingData.xSlidewaySpeedPID = pidInit(&getConfigData()->xSlidewaySpeedPID);
	auxiliaryDeformingData.ySlidewaySpeedPID = pidInit(&getConfigData()->ySlidewaySpeedPID);
	auxiliaryDeformingData.frontClawSpeedPID = pidInit(&getConfigData()->frontClawSpeedPID);
	auxiliaryDeformingData.reversingSpeedPID[0] = pidInit(&getConfigData()->frontClawSpeedPID);
    auxiliaryDeformingData.reversingSpeedPID[1] = pidInit(&getConfigData()->frontClawSpeedPID);
	
	digitalClan(&auxiliaryDeformingData.elevatorAngle[0].dataRef);
    digitalClan(&auxiliaryDeformingData.elevatorAngle[1].dataRef);	
	digitalClan(&auxiliaryDeformingData.elevatorSpeed[0].dataRef);
    digitalClan(&auxiliaryDeformingData.elevatorSpeed[1].dataRef);
	digitalClan(&auxiliaryDeformingData.xSlidewaySpeed.dataRef);
    digitalClan(&auxiliaryDeformingData.ySlidewaySpeed.dataRef);	
    digitalClan(&auxiliaryDeformingData.frontClawSpeed.dataRef);
	digitalClan(&auxiliaryDeformingData.reversingSpeed[0].dataRef);
    digitalClan(&auxiliaryDeformingData.reversingSpeed[1].dataRef);
	
	digitalLo(&auxiliaryDeformingData.elevatorSpeedLimitFlag);
	digitalLo(&auxiliaryDeformingData.heightstatebreak_Flag);
	digitalLo(&auxiliaryDeformingData.X_statebreak_Flag);
	digitalLo(&auxiliaryDeformingData.Y_statebreak_Flag);
    digitalHi(&auxiliaryDeformingData.Y_PlaceInitFinishFlag);	
	digitalLo(&auxiliaryDeformingData.X_PlaceJudgeFlag);
	digitalHi(&auxiliaryDeformingData.X_PlaceInitFinishFlag);
	digitalLo(&auxiliaryDeformingData.heightstatetrimming_Flag);//���Ҹ߶�΢����־λ
	digitalHi(&auxiliaryDeformingData.AllInitFinishFlag);//�տ�ʼ��λ���Y��
	
	digitalClan(&getauxiliaryAutoData()->mineralnumber);//�����ǰ��ʯ����
	
	auxiliaryDeformingData.elevatorAngle[0].dataRef = NORMAL_HEIGHT_GET;
	auxiliaryDeformingData.elevatorAngle[1].dataRef = NORMAL_HEIGHT_GET;

}
/*************************
�ڱ���ʼ��
**************************/
void sentryHardwareInit(void){
	BSP_GPIO_Init(BSP_GPIOE7,GPIO_Mode_IPU);
	BSP_GPIO_Init(BSP_GPIOE8,GPIO_Mode_IPU);
}

/************************
������ʼ��
**************************/
void mechaDeformingInit(void)
{
	if(robotConfigData.typeOfRobot == P_TANK_ID){
        
	}
	else if(robotConfigData.typeOfRobot == AUXILIARY_ID){		
       auxiliaryDeformingInit();
	}
	else if(robotConfigData.typeOfRobot == SENTRY_ID){
		sentryHardwareInit();
	}
}
