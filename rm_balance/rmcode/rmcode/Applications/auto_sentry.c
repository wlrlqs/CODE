#include "auto_sentry.h"
#include "vision.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "config.h"
#include "rc.h"
#include "Util.h"
#include "supervisor.h"
#include "judge.h"
#include "slave_sensor.h"
//���ýṹ��
AutoTaskStruct_t sentryAutoData;
//�ڱ�ר�ýṹ��
stryAutoTaskStruct_t sentryAtClData;

AutoTaskStruct_t* getsentryAutoData(){
    return &sentryAutoData;
}

void sentryTaskBegin(void){
    //���Ϊ������
	sentryAutoData.taskState = EXECUTING;									
    //��ִ�����м�һ
	digitalIncreasing(&sentryAutoData.schedule);							
}

void sentrySineSweep(s16 yawPatrolMinAngle, s16 yawPatrolMaxAngle, s16 pitchPatrolMinAngle, s16 pitchPatrolMaxAngle, \
					 u8 yawStayTime, u8 pitchStayTime, s16 captureMaxAngle, bool patrolMode) {
	static bool yawUpFlag = 0;
    static bool pitchUpFlag = 0;
	//����װ�װ��������ֵ�������Ӧ�����̽Ƕ�ֵ
	//ɨ�迪ʼλ����װ�װ�����λ����ͬ
	if( ((sentryAtClData.angleSave > 4096) && (YAW_CENTER_ENCODER > 4096)) || \
			((sentryAtClData.angleSave < 4096) && (YAW_CENTER_ENCODER < 4096)) ){
		sentryAtClData.yawRTMotorAngle = getGimbalData()->yawMotorAngle;
	}
	//ɨ�迪ʼλ����װ�װ�����λ���෴
	else if( ((sentryAtClData.angleSave < 4096) && (YAW_CENTER_ENCODER > 4096)) || \
		((sentryAtClData.angleSave > 4096) && (YAW_CENTER_ENCODER < 4096)) ){
		if(getGimbalData()->yawMotorAngle < 0){
			sentryAtClData.yawRTMotorAngle = 180 + getGimbalData()->yawMotorAngle;
		}
		else{
			sentryAtClData.yawRTMotorAngle = getGimbalData()->yawMotorAngle - 180;
		}
	}
	//���ݵ����װ���������Ӧ�ĽǶ�
	sentryAtClData.yawRTMotorAngle *= getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN);
	//Ѳ��ģʽ�½��յ�����̨��ʶ���ź�
	if(patrolMode){
		getGimbalData()->yawAngleRef  =  -controlTransData.yawMotorAngle.float_temp;
		//pitch����Ƕȼ�С�����ƶ�
		if(!pitchUpFlag){        
			if(sentryAtClData.pitchCNT >= pitchStayTime){
				digitalDecline(&getGimbalData()->pitchAngleRef);
				digitalClan(&sentryAtClData.pitchCNT);
			}
			else if(getGimbalData()->pitchGyroAngle <= PITCH_PATROL_MINANGLE){
				pitchUpFlag = true;
				digitalClan(&sentryAtClData.pitchCNT);
			}
		}
		//pitch����Ƕ��������ƶ�		
		else{
			if(sentryAtClData.pitchCNT >= pitchStayTime){
				digitalIncreasing(&getGimbalData()->pitchAngleRef);
				digitalClan(&sentryAtClData.pitchCNT);
			}
			else if(getGimbalData()->pitchGyroAngle >= captureMaxAngle){
				pitchUpFlag = false;
				digitalClan(&sentryAtClData.pitchCNT);
			}       
		}
	}
	//����ɨ��ģʽ
	else{
		//yaw��������ֵ��С�����ƶ�
		if(!yawUpFlag){
			if(sentryAtClData.yawCNT >= yawStayTime){
				digitalIncreasing(&getGimbalData()->yawAngleRef);
				digitalClan(&sentryAtClData.yawCNT);
			}

			if(sentryAtClData.yawRTMotorAngle <= yawPatrolMinAngle){
				yawUpFlag = true;
				digitalClan(&sentryAtClData.yawCNT);
			}
			
		}
		//yaw��������ֵ�������ƶ�
		else{
			if(sentryAtClData.yawCNT >= yawStayTime){
				digitalDecline(&getGimbalData()->yawAngleRef);
				digitalClan(&sentryAtClData.yawCNT);
			}
						
			if(sentryAtClData.yawRTMotorAngle >= yawPatrolMaxAngle){
				yawUpFlag = false;
				digitalClan(&sentryAtClData.yawCNT);
			}		
		}
		
		//pitch����Ƕȼ�С�����ƶ�
		if(!pitchUpFlag){        
			if(sentryAtClData.pitchCNT >= pitchStayTime){
				digitalDecline(&getGimbalData()->pitchAngleRef);
				digitalClan(&sentryAtClData.pitchCNT);
			}
			else if(getGimbalData()->pitchGyroAngle <= pitchPatrolMinAngle){
				pitchUpFlag = true;
				digitalClan(&sentryAtClData.pitchCNT);
			}
		}
		//pitch����Ƕ��������ƶ�		
		else{
			if(sentryAtClData.pitchCNT >= pitchStayTime){
				digitalIncreasing(&getGimbalData()->pitchAngleRef);
				digitalClan(&sentryAtClData.pitchCNT);
			}
			else if(getGimbalData()->pitchGyroAngle >= pitchPatrolMaxAngle){
				pitchUpFlag = false;
				digitalClan(&sentryAtClData.pitchCNT);
			}       
		}
	}
	
	digitalIncreasing(&sentryAtClData.yawCNT);
	digitalIncreasing(&sentryAtClData.pitchCNT);
}

void sentryAngleUpdate(void) {
	static u8 patrolSchedule = 0;
	static bool lastotherSameTargetFlag = 0;
	
	switch(robotConfigData.typeOfRobot){
		case SENTRY_ID:{
			if( (slaveTransData.otherCaptureFlag != lastotherSameTargetFlag) && slaveTransData.otherCaptureFlag ){
				digitalHi(&patrolSchedule);
			}
			else{																									
				digitalLo(&patrolSchedule);
			}
			
			switch(patrolSchedule){
				//����Ѳ��ģʽ
				case PATROL_MODE:{					
					sentrySineSweep(YAW_PATROL_MINANGLE, YAW_PATROL_MAXANGLE , PITCH_PATROL_MINANGLE, PITCH_PATROL_MAXANGLE, \
									YAW_STAY_TIME, PITCH_STAY_TIME, 0 , slaveTransData.otherCaptureFlag);
					break;
				}
				//����̨��Ѳ��ģʽ���յ�����̨��ʶ����Ϣ��������̨�ﵽ��С����̨��������Χ��Ŀ��
				case FOLLOW_MODE:{
					getGimbalData()->pitchAngleRef = slaveTransData.pitchGyroAngle.float_temp;	//pitch�����ʶ����̨�������ǽǶ�
					getGimbalData()->yawAngleRef = slaveTransData.yawMotorAngle.float_temp;	//yaw�����ʶ����̨������ֵ�Ƕ�
					break;
				}			
			}
			
			break;
		}
		
		case SMALLGIMBAL_ID:{
			if( (controlTransData.otherCaptureFlag != lastotherSameTargetFlag) && controlTransData.otherCaptureFlag ){
				digitalHi(&patrolSchedule);
			}
			else{																									
				digitalLo(&patrolSchedule);
			}
			
			switch(patrolSchedule){
				//����Ѳ��ģʽ
				case PATROL_MODE:{
					sentrySineSweep(SMYAW_PATROL_MINANGLE, SMYAW_PATROL_MAXANGLE , SMPITCH_PATROL_MINANGLE, SMPITCH_PATROL_MAXANGLE, \
									SMYAW_STAY_TIME, SMPITCH_STAY_TIME, controlTransData.pitchGyroAngle.float_temp, controlTransData.otherCaptureFlag);
					break;
				}
				//����̨��Ѳ��ģʽ���յ�����̨��ʶ����Ϣ��������̨�ﵽ��С����̨��������Χ��Ŀ��
				case FOLLOW_MODE:{
					getGimbalData()->pitchAngleRef = controlTransData.pitchGyroAngle.float_temp;	//pitch�����ʶ����̨�������ǽǶ�
					getGimbalData()->yawAngleRef = -controlTransData.yawMotorAngle.float_temp;	//yaw�����ʶ����̨������ֵ�Ƕ�
					break;
				}			
			}
			
			break;
		}
	}
	lastotherSameTargetFlag = controlTransData.otherCaptureFlag;
}

void sentryArmorPositionJudge(bool armorNum){
	//������������ֵȷ��ǰ��װ�װ�λ��
	if(armorNum){
		if( YAW_CENTER_ENCODER > 4096 ){
			sentryAtClData.frontCenterAngle = YAW_CENTER_ENCODER - 4096;
		}
		else{
			sentryAtClData.frontCenterAngle = YAW_CENTER_ENCODER + 4096;
		}
	}
	else{
		sentryAtClData.frontCenterAngle = YAW_CENTER_ENCODER;
	}
	//ǰװ�װ�����ֵ���ں�װ�װ�����ֵ
	if(sentryAtClData.frontCenterAngle > 4096){
		sentryAtClData.backCenterAngle = sentryAtClData.frontCenterAngle - 4096;
		//����̨ 
		if( getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == 1 ){
			//����̨���ڱ������
			if( (YAW_MOTOR_ENCODER > sentryAtClData.backCenterAngle) &&  (YAW_MOTOR_ENCODER < sentryAtClData.frontCenterAngle) ){				
			    digitalHi(&sentryAtClData.flagAddEncoder);
				digitalLo(&sentryAtClData.flagReduceEncoder);
			}
			//����̨���ڱ����ұ�
			else{
				digitalHi(&sentryAtClData.flagReduceEncoder);
				digitalLo(&sentryAtClData.flagAddEncoder);
			}
		}
		//����̨
		else{
			//����̨���ڱ������
			if((YAW_MOTOR_ENCODER < sentryAtClData.backCenterAngle) ||  (YAW_MOTOR_ENCODER > sentryAtClData.frontCenterAngle)){				
				digitalHi(&sentryAtClData.flagReduceEncoder);
				digitalLo(&sentryAtClData.flagAddEncoder);
			}
			//����̨���ڱ����ұ�
			else{
				digitalHi(&sentryAtClData.flagAddEncoder);
				digitalLo(&sentryAtClData.flagReduceEncoder);
			}
		}
	}
	//ǰװ�װ�����ֵС���ں�װ�װ�����ֵ
	else{
		sentryAtClData.backCenterAngle = sentryAtClData.frontCenterAngle + 4096;
		//����̨
		if( getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == 1 ){
			//����̨���ڱ������
			if( (YAW_MOTOR_ENCODER < sentryAtClData.frontCenterAngle) || (YAW_MOTOR_ENCODER > sentryAtClData.backCenterAngle) ){				
				digitalHi(&sentryAtClData.flagAddEncoder);
				digitalLo(&sentryAtClData.flagReduceEncoder);
			}
			//����̨���ڱ����ұ�
			else{
				digitalHi(&sentryAtClData.flagReduceEncoder);
				digitalLo(&sentryAtClData.flagAddEncoder);				
			}
		}
		//����̨
		else{
			//����̨���ڱ������
			if( (YAW_MOTOR_ENCODER > sentryAtClData.frontCenterAngle) && (YAW_MOTOR_ENCODER < sentryAtClData.backCenterAngle) ){				
				digitalHi(&sentryAtClData.flagReduceEncoder);
				digitalLo(&sentryAtClData.flagAddEncoder);
			}
			//����̨���ڱ����ұ�
			else{
				digitalHi(&sentryAtClData.flagAddEncoder);
				digitalLo(&sentryAtClData.flagReduceEncoder);
			}
		}		
	}
}

void sentryTurnExecute(){
	static int8_t coefficient;
	if(sentryAtClData.flagAddEncoder){
			coefficient = -2;
			if( (YAW_MOTOR_ENCODER > sentryAtClData.frontCenterAngle - 100) && (YAW_MOTOR_ENCODER < sentryAtClData.frontCenterAngle + 100) ){
				digitalLo(&sentryAtClData.flagAddEncoder);
				coefficient = -1;
				sentryAutoData.schedule = 3;
			}
	}
	else if(sentryAtClData.flagReduceEncoder){
		coefficient = 2;
		if( (YAW_MOTOR_ENCODER > sentryAtClData.frontCenterAngle - 100) && (YAW_MOTOR_ENCODER < sentryAtClData.frontCenterAngle + 100) ){
			digitalLo(&sentryAtClData.flagReduceEncoder);
			coefficient = 1;
			sentryAutoData.schedule = 3;
		}
	}
		
	getGimbalData()->yawAngleRef += TURNRATE * YAWADD * coefficient * getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN);
	
	if(sentryAutoData.schedule == 3){
		sentryAtClData.angleSave = YAW_MOTOR_ENCODER;
	}
}

void sentryPatrolTaskUpdate(void){

	switch(sentryAutoData.schedule){
		case 1:{
			chassisSwitch(DISABLE);
			gimbalSwitch(DISABLE);
			sentryArmorPositionJudge(SENTRY_FRONT_ARMOR);
			digitalIncreasing(&sentryAutoData.schedule);
		}break;
		case 2:{																										
			sentryTurnExecute();
		}break;
		case 3:{
//			if(judgeData.extGameState.game_progress == 4)
				sentryAngleUpdate();  
		}break;
		//���ڱ��У�ֻ��ҡ�˲����г�����Ȩλ��ʱ������������
		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		
		//��������б���û�н��ȣ����������
		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			
	}
}

/************************************
�����������ϢΪװ�װ��С��Ϣ
armorTypeFbd			<-->	depthFbd
armorTypeLastFbd	<-->	depthLastFbd
************************************/
void sentryAttackTaskUpdate(void){
    //ʱ����
	static TickType_t sentryFireLastWakeTime = 0;
    
	switch(sentryAtClData.attackMode){
		//δ����Ŀ��Ѳ��ģʽ
		case 0:{
			sentryAutoData.currentTask = SENTRY_PATROL;
			break;
		}
		case 1:{
			if(sentryAtClData.lostTargetCNT > 300){
				sentryAtClData.attackMode = DISABLE;
			}
			else{
				sentryAtClData.attackMode = ENABLE;
			}
			switch(sentryAtClData.visionSchedule){
                case 0: gimbalSwitch(DISABLE);
                    chassisSwitch(DISABLE);
                    //����TX2����
                    visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);
                    digitalIncreasing(&sentryAtClData.visionSchedule);	
                    sentryFireLastWakeTime = xTaskGetTickCount();
                    break;
                case 1: gimbalSwitch(ENABLE);	
                    shootVisionInit ();
                    //����ģʽ����
                    getvisionData()->prejudgFlag = false;		
				
                    //�����������
                    if(getvisionData()->captureFlag) {
                        if(getshootData()->fricMotorFlag/*&& (judgeData.extGameState.game_progress == 4)*/){   //�ڱ���
                            sentryFireLastWakeTime = xTaskGetTickCount();
                            digitalHi(&getshootData()->fireFlag_17mm);     
                        }
                    }	
					
                    if(xTaskGetTickCount() - sentryFireLastWakeTime > 200){
						digitalLo(&getshootData()->fireFlag_17mm);		
                        digitalLo(&getshootData()->shootTrigger_17mm);							
                    }	    
                    break;
			}
			break;
		}
	}	
}

void sentryUnderAttackTaskUpdate(void) {
	switch(sentryAutoData.schedule){
		case 1:{
			sentryAtClData.yawCNT = 2*YAW_STAY_TIME;
			gimbalSwitch(DISABLE);
			chassisSwitch(DISABLE);
			// ����Ѫ����װ�װ壨0�ţ������� 
			if( judgeData.extRobotHurt.armor_id == 0x00 &&  judgeData.extRobotHurt.hurt_type == 0x00 ){	
				//��ֵ��װ�װ�0������ֵ��ͬ
				if( (YAW_CENTER_ENCODER > 4096 && getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == 1) || \
					(YAW_CENTER_ENCODER > 4096 && getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == -1) ){
					sentryArmorPositionJudge(SENTRY_FRONT_ARMOR);
				}
				//��ֵ��װ�װ�0������ֵ�෴
				else if( (YAW_CENTER_ENCODER <= 4096 && getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == 1) || \
					(YAW_CENTER_ENCODER <= 4096 && getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == -1)){
					sentryArmorPositionJudge(SENTRY_BACK_ARMOR);
				}
			}
			// 1��װ�װ屻���� 
			else if( judgeData.extRobotHurt.armor_id == 0x01 &&  judgeData.extRobotHurt.hurt_type == 0x00 ){	
					//��ֵ��װ�װ�1������ֵ��
					if( (YAW_CENTER_ENCODER > 4096 && getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == 1) || \
						(YAW_CENTER_ENCODER > 4096 && getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == -1) ){
						sentryArmorPositionJudge(SENTRY_BACK_ARMOR);
					}
					//��ֵ��װ�װ�1������ֵͬ
					else if( (YAW_CENTER_ENCODER <= 4096 && getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == 1) || \
					(YAW_CENTER_ENCODER <= 4096 && getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) == -1) ){
						sentryArmorPositionJudge(SENTRY_FRONT_ARMOR);
					}				
				}
				digitalIncreasing(&sentryAutoData.schedule);
		}break;
		case 2:{
			sentryTurnExecute();
		}break;																								
		case 3:{		
            //120������ɨ��																							
			sentryAngleUpdate();
		}break;
        //���ڱ��У�ֻ��ҡ�˲����г�����Ȩλ��ʱ������������
		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		
        //��������б���û�н��ȣ����������
		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			
	}
}

void sentryAntimissileTaskUpdate(void) {
	static TickType_t sentryFireLastWakeTime = 0;
	switch(sentryAutoData.schedule){
		case 1:{
			chassisSwitch(DISABLE);
			gimbalSwitch(DISABLE);
			sentryArmorPositionJudge(SENTRY_FRONT_ARMOR);
			digitalIncreasing(&sentryAutoData.schedule);
		}break;
		case 2:{																										
			sentryTurnExecute();
		}break;
		case 3:{
			getGimbalData()->yawAngleRef += ANTIMISSILE_YAW_ANGLE;
			sentryAutoData.schedule = 4;
		}break;
		case 4:{
			if((!sentryAtClData.sentryLeft) || (!sentryAtClData.sentryRight)){
				if(getshootData()->fricMotorFlag /*&& (judgeData.extGameState.game_progress == 4)*/){   //�ڱ���
					sentryFireLastWakeTime = xTaskGetTickCount();
					digitalHi(&getshootData()->fireFlag_17mm);     
				}
			}	
			if(xTaskGetTickCount() - sentryFireLastWakeTime > 200){
				digitalLo(&getshootData()->fireFlag_17mm);		
				digitalLo(&getshootData()->shootTrigger_17mm);		  
			}	
		}break;
	}	
	getGimbalData()->pitchAngleRef = ANTIMISSILE_PITCH_ANGLE;
}

void sentryModeUpdate() {
	static u8 lastMode;
	static u16 lastHP;
	static bool captureFlag = 0,lastCaptureFlag = 0,modeInit = false;
    
	captureFlag = getvisionData()->captureFlag;
    
	if(!modeInit) {
		lastHP = judgeData.extGameRobotState.remain_HP;
		modeInit = true;
	}
    
	if(!captureFlag) {	
        //û��ʶ��Ŀ������������״̬
		digitalIncreasing(&sentryAtClData.lostTargetCNT);
	}
	else if(captureFlag) {        
        sentryAtClData.attackMode = ENABLE;	        
		if((!lastCaptureFlag) && (sentryAutoData.currentTask == SENTRY_ATTACK)){
			sentryAtClData.attackMode = DISABLE;
			getvisionData()->distinguishMode = TX2_STOP;
			sentryAtClData.visionSchedule = 0;
			getvisionData()->prejudgFlag = false;
			getvisionData()->miniPTZEnableAim = false;			
			shootDataReset();
		}
        digitalClan(&sentryAtClData.lostTargetCNT);
	}
	lastCaptureFlag = captureFlag;

	if(sentryAtClData.attackMode) {	
		sentryAutoData.taskDuration = 0;
        //���Ӿ��������빥��ģʽ
		sentryAutoData.currentTask = SENTRY_ATTACK;
	}
	else if((judgeData.extGameRobotState.remain_HP != lastHP)&&( judgeData.extRobotHurt.hurt_type == 0x00)){
		sentryAutoData.taskDuration = 0;
		sentryAutoData.currentTask = SENTRY_UNTER_ATTACK;
	}
    //�������������������Ѳ��ģʽ
	else if( (sentryAutoData.taskDuration > 0.8f && sentryAutoData.currentTask == SENTRY_ATTACK) \
			    ||(sentryAutoData.taskDuration > 0.8f && sentryAutoData.currentTask == SENTRY_ANTIMISSILE) \
				|| (sentryAutoData.taskDuration > 6.0f && sentryAutoData.currentTask == SENTRY_UNTER_ATTACK) \
				|| (sentryAutoData.currentTask == SENTRY_MANUAL))	
		sentryAutoData.currentTask = SENTRY_PATROL;		
	else
		sentryAutoData.taskDuration += AUTO_TASK_DURATION;
	
	if(RC_ROTATE > 100 && sentryAtClData.gimbalType == GIMBAL_ABOVE) {
		sentryAutoData.taskDuration = 0;
		sentryAutoData.currentTask = SENTRY_ANTIMISSILE;
	}
	
	if( (lastMode != sentryAutoData.currentTask) && (sentryAutoData.schedule != 1) )
		sentryAutoData.schedule = 1;
    
	lastHP = judgeData.extGameRobotState.remain_HP;
	lastMode = sentryAutoData.currentTask;
}

static void sentryGimbalModeUpdate(void){

	switch(sentryAutoData.currentTask){
		case SENTRY_ANTIMISSILE:	sentryAtClData.gimbalTask = GIMBAL_ANTIMISSILE;	break;
		case SENTRY_ATTACK :		sentryAtClData.gimbalTask = GIMBAL_ATTACK;	    break;
		case SENTRY_UNTER_ATTACK:	sentryAtClData.gimbalTask = GIMBAL_UNTER_ATTACK;break;
		case SENTRY_PATROL :		sentryAtClData.gimbalTask = GIMBAL_PATROL;	    break;						
	}	
}

static void sentryChasissModeUpdate(void){
	switch(sentryAutoData.currentTask){
		case SENTRY_ATTACK :		sentryAtClData.chassisTask = CHASSIS_NORMAL;	 break;
		case SENTRY_UNTER_ATTACK:	sentryAtClData.chassisTask = CHASSIS_NORMAL;     break;
		case SENTRY_PATROL :		sentryAtClData.chassisTask = CHASSIS_NORMAL;	 break;						
	}
	
	if(RC_ROTATE > 100){
		sentryAtClData.chassisTask = CHASSIS_ANTIMISSILE;		
	}	
}

void sentryChassisDataUpdate(void){
	controlTransData.othersenorFlag0 = PEin(7);
	controlTransData.othersenorFlag1 = PEin(8);
}

static void readChassisSensorData(void){
	sentryAtClData.sentryLeft = controlTransData.othersenorFlag0;
	sentryAtClData.sentryRight = controlTransData.othersenorFlag1;
}

static void sentryChassisIinit(void) {
	getchassisData()->speedLimit = 1.0f;
	getchassisData()->landingSpeedy =  BASESPEED;
	getchassisData()->landingSpeedy = (float)getchassisData()->landingSpeedy * REAL_MOTOR_SPEED_SCALE / 1.0f;
}

static void sentryChassisAccelerate(void){
	
}

static void sentryRandomMove(void){
	//�ڱ�����ʵʱʱ��
	sentryAtClData.randomRTTime = xTaskGetTickCount();
	switch(sentryAtClData.randomStep){
		case 0:{
			sentryAtClData.randomStartTime = xTaskGetTickCount();
			sentryAtClData.randomValue = rand()%10000;
			if(sentryAtClData.randomValue <= 500){     //1000
					sentryAtClData.randomValue = 500;  //1000
				}
			else if(sentryAtClData.randomValue >= 1800){   //2800
				while(sentryAtClData.randomValue  > 1800){ //2800
					sentryAtClData.randomValue /= 2;
				};
			}
			sentryAtClData.randomStep++;
		}break;
		case 1:{
			if(sentryAtClData.randomRTTime - sentryAtClData.randomStartTime > 500){  //1000
				sentryAtClData.randomStep ++;
			}
			else if(sentryAtClData.randomRTTime - sentryAtClData.randomStartTime <= 0){
				sentryAtClData.randomStep = 0;			
			}
			
		}break;
		case 2:{
			if(sentryAtClData.randomRTTime - sentryAtClData.randomStartTime > sentryAtClData.randomValue ){
				sentryDeformingData.direction = -sentryDeformingData.direction;
				sentryAtClData.randomStep = 0;
			}
			else if(sentryAtClData.randomRTTime - sentryAtClData.randomStartTime <= 0){
				sentryAtClData.randomStep = 0;
			}
		}
		default:break;
	}
	
}

static void sentryCrashproof(void){
	//��⵽�������
	if((!sentryAtClData.sentryLeft) && (getchassisData()->landingSpeedy != 0)){
		sentryDeformingData.direction = -1;	
		sentryAtClData.randomStep = 0;
	}
	//��⵽�ұ�����
    else if((!sentryAtClData.sentryRight) && (getchassisData()->landingSpeedy != 0)){
		sentryDeformingData.direction = 1;
		sentryAtClData.randomStep = 0;
    }
}

static void sentryMoveSpeed(u16 moveSpeed){ 	
//	if(moveSpeed > 0){
//		getchassisData()->landingSpeedy = (float)moveSpeed * REAL_MOTOR_SPEED_SCALE / 1.0f;
//	}
//	else if(moveSpeed <= 0){
//		getchassisData()->landingSpeedy = -(float)moveSpeed * REAL_MOTOR_SPEED_SCALE / 1.0f;		
//	}	
	getchassisData()->landingSpeedy = moveSpeed;
}

static void sentryAntimissileMove(){
	//������Ŀ�귽���ƶ�
	if(sentryDeformingData.direction == TARGETSIDE){
		//�˶��������Ӵ�
		if((!sentryAtClData.sentryLeft) || (!sentryAtClData.sentryRight)){
			//ֹͣ�˶�
			digitalClan(&getchassisData()->landingSpeedy);
		}
	}
	//����Ŀ�귽�����ƶ�
	else if(sentryDeformingData.direction == -TARGETSIDE){
		//����
		sentryDeformingData.direction = -sentryDeformingData.direction;
	}
}
/*�ڱ���̨�˶�����*/
static void autoGimbalMove(void){
	sentryGimbalModeUpdate();
	switch(sentryAtClData.gimbalType){
		//����̨
		case GIMBAL_BELOW:{
			switch(sentryAtClData.gimbalTask){
				case GIMBAL_ATTACK :		sentryAttackTaskUpdate();		break;
				case GIMBAL_UNTER_ATTACK:	sentryUnderAttackTaskUpdate();	break;
				case GIMBAL_PATROL :		sentryPatrolTaskUpdate();		break;						
			}	
		}break;
		//����̨
		case GIMBAL_ABOVE:{
			switch(sentryAtClData.gimbalTask){
				case GIMBAL_ANTIMISSILE:	sentryAntimissileTaskUpdate();	break;
				case GIMBAL_ATTACK :		sentryAttackTaskUpdate();		break;
				case GIMBAL_UNTER_ATTACK:	sentryUnderAttackTaskUpdate();	break;
				case GIMBAL_PATROL :		sentryPatrolTaskUpdate();		break;						
			}	
		}break;
	}
}

/*�ڱ������˶�����*/
static void autoChassisMove(void){
	//��ȡ��紫������Ϣ
	readChassisSensorData();
	sentryChasissModeUpdate();
	switch(sentryAtClData.chassisTask){
		case CHASSIS_NORMAL:{
			//���̼���
			sentryChassisAccelerate();
			//����˶�
			sentryRandomMove();
			//���̷�ײ����
			sentryCrashproof();
			//���̸����˶�
			sentryMoveSpeed(SPEEDHIGH);
			break;
		}
		case CHASSIS_UNTER_ATTACK:{	  
			break;
		}
		case CHASSIS_ANTIMISSILE:{
			sentryAntimissileMove();
			break;
		}		
	}
}

//1000ms�͸�TX2��һ������ ��ֹ��TX2ʧȥ��ϵ.
static void communicationWithManifold(void){
	if( !(getcontrolData()->loops % 1000) ){														
		visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);
	}
}

static void sentryAutoTaskInit(void) {
	digitalClan(&sentryAtClData.visionSchedule);
	digitalClan(&sentryAtClData.attackMode);
	sentryAtClData.randomStep = 0;
	sentryAutoData.schedule = 1; 
	sentryDeformingData.direction = 1;
	sentryAtClData.angleSave = getGimbalData()->yawAngleSave;
	sentryAutoData.currentTask = SENTRY_PATROL;
	gimbalSwitch(ENABLE);
	chassisSwitch(ENABLE);
	sentryChassisIinit();
}

static void sentryGimbalTypeJudge(void){
	if(ROBOT == SENTRY_ID){
		sentryAtClData.gimbalType = GIMBAL_BELOW;
	}
	else if(ROBOT == SMALLGIMBAL_ID){
		sentryAtClData.gimbalType = GIMBAL_ABOVE;
	}
}

static void sentryAutoTaskRelease(void){
	digitalClan(&sentryAtClData.speedFloag);
	digitalClan(&getchassisData()->landingSpeedy);
	digitalClan(&sentryDeformingData.direction);
	digitalClan(&sentryAtClData.randomStep);
	autoDataInit(&sentryAutoData);
}

void sentryAutoTaskUpdate(void){
	//KM��RCģʽ�³�ʼ��
	if( (getrobotMode() == MODE_KM) && (!sentryAtClData.init) ){
		sentryAutoTaskInit();
		sentryGimbalTypeJudge();		
		sentryAtClData.init = true;
	}
	else if( !(getrobotMode() == MODE_KM) ){
		sentryAtClData.init = false;
	}
    //����ģʽΪȫ�Զ�
	if(getrobotMode() == MODE_KM /*&& (judgeData.extGameRobotState.remain_HP != 0 )*/ && sentryAtClData.init){
		srand(getcontrolData()->loops);
		//ģʽˢ��
		sentryModeUpdate();
		//�Զ�������̨����
		autoGimbalMove();
//		getchassisData()->landingSpeedy	= 0.0f;	
		//�Զ�������̿���
        autoChassisMove(); 
		communicationWithManifold();		    //����ʶ����Ϣ��TX2
	}
	else{
		sentryAutoTaskRelease();
	}        
}
