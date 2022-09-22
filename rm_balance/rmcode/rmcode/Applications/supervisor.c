#include "application.h"
#include "motorHandle.h"
#include "Driver_imuSC.h"
#include "supercapacitor.h"
supervisorStruct_t supervisorData;

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

//��������Ƿ����� ÿ��������loop+1
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

//���ڿ����л�����״̬��
void supervisorArmedSwitch(uint8_t valve){														
	if(valve){
        //����  ����0
		supervisorData.state &= ~STATE_DISARMED;													
        //����
		supervisorData.state |= STATE_ARMED;															
	}
	else{
        //����		����0
		supervisorData.state &= ~STATE_ARMED;															
        //�л�������״̬
		supervisorData.state |= STATE_DISARMED;														
	}
}

//�����л����״̬��״̬��	
void supervisorStateSwitch(uint16_t state,uint8_t valve){																																		
	if (valve)
		supervisorData.state |= state;
	else
		supervisorData.state &= ~state;
}

//MAGУ׼ָ��   ��TQH:������У׼��
void supervisorMagCali(void){																					
	supervisorStateSwitch(STATE_MAGCALI,ENABLE);
}

//IMUУ׼ָ��   (TQH:���Դ�����)
void supervisorImuCali(uint8_t accTare){															
	if(accTare)
		supervisorData.imuCali = CALI_GYO_ACC;
	else
		supervisorData.imuCali = CALI_ONLY_GYO;
}

//����RGB��˸״̬
void supervisorLedSwitch(void){				
	switch(BOARD_TYPE){
		case BOARD_CHASSIS:{
			if(supervisorData.state & STATE_TRANS_ERROR){
				//ͨ�Ŷ�ʧ��
				supervisorData.ledState = LED_DOUBLE_TRANS; //������Ÿ�ֵ	
			 }
			else{
				//����������
				supervisorData.ledState = LED_WORKINGORDER;                     	
			}
		}
		break;
		case BOARD_CONTROL:{
			if(!getcontrolData()->chassisBoardDataFinish){
				//��̨���س�ʼ��ʧ��
				supervisorData.ledState = LED_CONTROLINIT_FAIL;											
			}
			else if(supervisorData.state & STATE_SENSOR_IMU_ERROR){
				//��̨���������
				supervisorData.ledState = LED_HARDWORK_FAIL;											
			}
			else if(supervisorData.state & STATE_IMUCALI){
				//IMUУ׼������
				supervisorData.ledState = LED_IMU_CALI;                               	
			}
			else if(supervisorData.gimbalCaliReset){														
				//��̨У׼
				supervisorData.ledState = LED_GIMBAL_CALI;												
			}
			else if(supervisorData.state & STATE_RADIO_LOSS){
				//ң������ʧ������
				supervisorData.ledState = LED_RADIO_LOSS;    		
			}
			else if(supervisorData.state & STATE_TRANS_ERROR){
				//ͨ�Ŷ�ʧ��
				supervisorData.ledState = LED_DOUBLE_TRANS; 
			}
			else{
				//����������
				supervisorData.ledState = LED_WORKINGORDER;                     	
			}
		}
		break;
        
        //LCM: ������RGB��Ч
        case BOARD_OTHER : {
            
			if(supervisorData.state & STATE_RADIO_LOSS){
				//ң������ʧ������
				supervisorData.ledState = LED_RADIO_LOSS;    		
			}
			else if(supervisorData.state & STATE_TRANS_ERROR){
				//ͨ�Ŷ�ʧ��
				supervisorData.ledState = LED_DOUBLE_TRANS; 
			}
			else{
				//����������
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
		//����
		
	}
	else{
	
	}
}
//����У��
static void supervisorControlTransferError(){
	controlSerial.serialError.intervalNum = controlSerial.serialError.errorCount.u32_temp - controlSerial.serialError.lastErrorCount;
	controlSerial.serialError.lastErrorCount = controlSerial.serialError.errorCount.u32_temp;
    
    //LCM: UAVû�е��̰壬����Ҫ˫��ͨ��
    if(ROBOT == UAV_ID || ROBOT == SMALLGIMBAL_ID) {
        supervisorStateSwitch(STATE_TRANS_ERROR,DISABLE);
        digitalClan(&controlSerial.serialError.waitingConnect);
    }
    else {
        if(!controlSerial.serialError.intervalNum){
            supervisorStateSwitch(STATE_TRANS_ERROR,ENABLE);   
            digitalLo(&getcontrolData()->chassisBoardDataFinish);
            //����
            //�ȴ�����
            digitalIncreasing(&controlSerial.serialError.waitingConnect);
        }
        else{
            supervisorStateSwitch(STATE_TRANS_ERROR,DISABLE);
            digitalClan(&controlSerial.serialError.waitingConnect);
        }
    }
    
	//����ʱ�䳬��200ms
	if(controlSerial.serialError.waitingConnect > LOST_TIME){
		//�¿�
		setSuspendRobotMode(getrobotMode());
		setRobotMode(MODE_RELAX);
		//ֹͣ�����������
		/*TQH*/
//		IWDG_Init();		//1s��  reset
		/**/
	}
	else{
		//�ص���һ��״̬
//		setRobotMode(getSuspendrobotMode());
		digitalClan(&chassisSerial.serialError.waitingConnect);
		
	}
}

//���̽���ͨ��У��
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



//��ⶪ��
void supervisorRadioLoss(void){																				
	remoteControlData.rcError.intervalNum = remoteControlData.rcError.errorCount.u32_temp - remoteControlData.rcError.lastErrorCount;
	remoteControlData.rcError.lastErrorCount = remoteControlData.rcError.errorCount.u32_temp;
	if(!remoteControlData.rcError.intervalNum){
        //��⵽����
		supervisorStateSwitch(STATE_RADIO_LOSS,ENABLE);                  	
		supervisorData.beepState = MUSIC_RADIO_LOSS;
	}
	else{
        //����޶���
		supervisorStateSwitch(STATE_RADIO_LOSS,DISABLE);                  
		digitalLo(&controlTransData.otherRcReadly);
	}
}

//������ϵͳ
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

//����Ӿ���
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

//����������Ƿ������ݷ���  
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

//��⹦�ʰ�
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

//��ⳬ�������Ƿ������ݷ���
void supervisorSupercapacitorError(void){
  getcapacitorData()->intervalNum = getcapacitorData()->CapacitorErrorCount - getcapacitorData()->lastErrorCount;
	getcapacitorData()->lastErrorCount = getcapacitorData()->CapacitorErrorCount;
	if(!getcapacitorData()->lastErrorCount)
		supervisorStateSwitch(STATE_CAPACITANCE_ERROR,ENABLE);
	else
		supervisorStateSwitch(STATE_CAPACITANCE_ERROR,DISABLE);
}

//�����������Ƿ�����
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

//Flash����
void supervisorFlash(void){	   												
    //�����Ҫ��flash���в�����ֱ�ӽ�supervisorData.flashSave����һ�μ���								
	if(supervisorData.flashSave){                                       
        //��⵽��Ҫ����Flash		����������ʾ��
		supervisorData.beepState = MUSIC_PARAMCALI;								
        //д��flash				
		configFlashWrite();  
    if(BOARD_TYPE == BOARD_CONTROL)  getchassisData()->chassisFlash = 1;		
        //д��TF��PID����
		parameterWriteDataFormFlash(robotConfigData.typeOfRobot);					
		digitalLo(&supervisorData.flashSave);                             	
	} 	
}

//��̨����ֵУ׼
void supervisorGimbalCali(void){												
	getConfigData()->pitchCenter = motorHandleClass.Encoder(&commandoMotorConfig[PITCHMOTOR]);
	getConfigData()->yawCenter = motorHandleClass.Encoder(&commandoMotorConfig[YAWMOTOR]);
	getConfigData()->rollCenter = motorHandleClass.Encoder(&commandoMotorConfig[ROLLMOTOR]);
	getchassisData()->yawCenterSave = getConfigData()->yawCenter;
	digitalHi(&supervisorData.flashSave);
}

//����������У׼
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
					//IMUУ׼
					if(Loops[IMU] >= WAITLOOPS && Digital[IMU]!= High){
                        //ң���������е�У׼���ǲ������ٶȼ�У׼��	
						supervisorImuCali(DISABLE);																					
						supervisorData.beepState = MUSIC_IMUCALI;
						digitalHi(&Digital[IMU]);
						digitalClan(&Loops[IMU]);
					}
					else if(wirelessData.imuCalibrate){
                        //IMUУ׼
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
			
			
#if UAV_SBUS        //RM��																														
			if(((RC_ROLL < -CALI_RC_VALUE) && (RC_PITCH > CALI_RC_VALUE)) \
				|| (wirelessData.magCalibrate)){
#else
			if((((RC_TRANSVERSE < -CALI_RC_VALUE) && (RC_LONGITUDINAL > CALI_RC_VALUE)) \
				|| (wirelessData.magCalibrate))){
#endif
				if(!supervisorData.busyState){	
					//MAGУ׼
					if(Loops[MAG] >= WAITLOOPS && Digital[MAG]!= High){
                        //MAGУ׼ָ��
						supervisorMagCali();																				
						supervisorData.beepState = MUSIC_MAGCALI;
						digitalHi(&Digital[MAG]);
						digitalClan(&Loops[MAG]);
					}
					else if(wirelessData.magCalibrate){
                        //������У׼
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
                        //����flash
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
                        //��������ʶ��
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
                        //���ذ�ID����
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
                        //��̨����ֵУ׼
						digitalHi(&supervisorData.gimbalCaliReset);									
						digitalHi(&Digital[GIMBAL_CALI]);
						digitalClan(&Loops[GIMBAL_CALI]);
					}
				}
			}
			else{
				if(supervisorData.gimbalCaliReset){
                    //ֻ�����ɿ�ҡ�˺�Ż�ִ��У׼��̨����
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
                //У׼������ҡ����ȫ�ɿ��ſ�ʼ				
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
#endif          //У׼������ҡ����ȫ�ɿ��ſ�ʼ
				digitalLo(&Digital[IMU]);
				digitalLo(&Digital[MAG]);
				digitalLo(&Digital[FLASH_OPERATION]);
				digitalLo(&Digital[DEFINE_ROBOT]);
				digitalLo(&Digital[YAW_CALI]);
				digitalLo(&Digital[GIMBAL_CALI]);
                digitalLo(&Digital[DEFINE_ID]);
			}

	}
				
//����ģ����
void static supervisorChassisError(){
	//ͨ����·���
	supervisorChassisTransferError();
	supervisorSlaveimuError();
	if(!(supervisorData.loops % 10)){
		//������ϵͳ����
		supervisorJudgeError_chassis();	
		//��⹦�ʿ���
		if(robotConfigData.robotDeviceList & DEVICE_CURRENT){
			supervisorCurrentError();	
		} 											
	}
}
//����ģ����
void static supervisorControlError(){
	//ң��������
	dbusFunction();
	//�Ӿ�ͨ����·���
	supervisorTX2TransferError();

    //����ͨ����·���
    supervisorControlTransferError();
	//�������ݼ��
	controlDeviceConfirm(DEVICE_CAPACITANCE,supervisorSupercapacitorError);
	//��̨����
	supervisorSlaveimuError();
	/**/
	
	/**/
	if(!(supervisorData.loops % 10)){
        //������																		
		supervisorMotorError();		
		//����ϵͳ���
		supervisorJudgeError_gimbal();
		//����״̬��⣬1Hz
		supervisorRadioLoss();	
		//�Ӿ��˼��
		controlDeviceConfirm(DEVICE_VISION,supervisorVisionError);											
	}
	//�ڽ���״̬��
	/*-- ������״̬�µ�״̬����� --*/
	if(supervisorData.state & STATE_ARMED){														
		//����Ȩ���		����ڵײ�������
		if(RC_MODE == RCSW_BOTTOM){		  																
			//������״̬�л�������״̬
			supervisorArmedSwitch(DISABLE);														
			//��������ʾ��		
			supervisorData.beepState = MUSIC_DISARMED;										
		}
	}/*-- �ڽ���״̬�µ�״̬����� --*/
	else if(supervisorData.state & STATE_DISARMED){										
			if(!(supervisorData.state & STATE_MAGCALI) \
				&& !(supervisorData.state & STATE_IMUCALI) \
				&& (robotConfigData.distinguishState != ROBOT_BEING_IDENTIFIED) \
				&& (supervisorData.gimbalCaliReset != ENABLE)){
                //ִ�и���У׼���봦�ڲ�æµ����ID��״̬
				supervisorData.busyState = false;															
			}
			else{
				supervisorData.busyState = true;	
			}
            //��������ɨ��
			if(RC_MODE == RCSW_MID || RC_MODE== RCSW_TOP){									
				supervisorArmedSwitch(ENABLE);
				supervisorData.beepState=MUSIC_ARMED;												  
				supervisorData.ArmSwitch=ENABLE;
				digitalClan(&supervisorData.armTime);
			}
		}
	if(robotConfigData.typeOfRobot == NO_ID){
        //û��ID��һֱ��
		supervisorData.beepState = MUSIC_NO_ID;													
	}
	
}

//LCM: ���������
static void supervisorOtherError(void) {
	//ң��������
	dbusFunction();
	if(!(supervisorData.loops % 10)){
		//����״̬��⣬1Hz//����ϵͳ���
		supervisorRadioLoss();											
	}
	//�ڽ���״̬�� 
	/*-- ������״̬�µ�״̬����� --*/
	if(supervisorData.state & STATE_ARMED){														
		//����Ȩ���		����ڵײ�������
		if(RC_MODE == RCSW_BOTTOM){		  																
			//������״̬�л�������״̬
			supervisorArmedSwitch(DISABLE);														
			//��������ʾ��		
			supervisorData.beepState = MUSIC_DISARMED;										
		}
	}/*-- �ڽ���״̬�µ�״̬����� --*/
	else if(supervisorData.state & STATE_DISARMED){										
			if(!(supervisorData.state & STATE_MAGCALI) \
				&& !(supervisorData.state & STATE_IMUCALI) \
				&& (robotConfigData.distinguishState != ROBOT_BEING_IDENTIFIED) \
				&& (supervisorData.gimbalCaliReset != ENABLE)){
                //ִ�и���У׼���봦�ڲ�æµ����ID��״̬
				supervisorData.busyState = false;															
			}
			else{
				supervisorData.busyState = true;	
			}
            //��������ɨ��
			if(RC_MODE == RCSW_MID || RC_MODE== RCSW_TOP){									
				supervisorArmedSwitch(ENABLE);
				supervisorData.beepState=MUSIC_ARMED;												  
				supervisorData.ArmSwitch=ENABLE;
				digitalClan(&supervisorData.armTime);
			}
		}
	if(robotConfigData.typeOfRobot == NO_ID){
        //û��ID��һֱ��
		supervisorData.beepState = MUSIC_NO_ID;													
	}	
}

//ȫ�ּ��
void static supervisorErrorRobot(){
	boardTypeConfirm(BOARD_CHASSIS,supervisorChassisError);
	boardTypeConfirm(BOARD_CONTROL,supervisorControlError);
    //LCM: ��������
    boardTypeConfirm( BOARD_OTHER , supervisorOtherError );    
}
	
void supervisorUpdateTask(void *Parameters){
    //��ȡ����ϵͳ���е�ʱ�ӽ�����
	TickType_t xLastWakeTime = xTaskGetTickCount();											
	while(true){
        //������ʱ����������ʱ������
		vTaskDelayUntil(&xLastWakeTime,SUPER_STACK_PERIOD);	
//        //��������Ƿ�����
//		supervisorTaskCheck();			
		//������RGB�����������
		warningUpdate();		
        //Flashд����										
		supervisorFlash();                                                
        //tf�����
		tFCardUpdate();																											
		//������ɫLED״̬�л�	���õƵļ��ָʾ
		supervisorErrorRobot();
		//˫�ؼ�ػ�
		supervisorLedSwitch();		
		digitalIncreasing(&supervisorData.loops);
	}
}

//��������ʼ������LEDһ�����ڼ��״̬��
void supervisorInit(void){
	supervisorArmedSwitch(DISABLE);
	sightClass.Init();
	//SK6812������ʼ��
	SK6812Config();
	supervisorData.taskEvent[SUPERVISOR_TASK] = xTaskCreate(supervisorUpdateTask, \
                                                            "SUPE", \
                                                            SPUER_STACK_SIZE, \
                                                            NULL,SPUER_PRIORITY, \
                                                            &supervisorData.xHandleTask);
    usbVCP_Printf("supervisorInit Successfully \r\n");
}



