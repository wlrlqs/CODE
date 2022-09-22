#include "application.h"
#include "driver.h"
#include "vision.h"
#include "motorHandle.h"
#include "broadcast_demo.h"

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

controlStruct_t controlData;
robotModeStruct_t robotMode = MODE_RELAX;			//�������
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

//�˺������ڻ�ȡȫ�ֿ���ģʽ��Ϣ
void getGlobalMode(void){													
#ifndef SHIELD_JUDGE

#endif
	if(supervisorData.state & STATE_RADIO_LOSS){
        //�ϵ���ºú󶪿ؾ�STOPģʽ
		robotMode = MODE_STOP;               		  
	}
	else{
		switch(BOARD_TYPE){
			case BOARD_CONTROL:
				switch(robotMode){
					//�����ǰΪMODE_INIT
					case MODE_INIT :{ 									
						//���SW2����͵�ʱ	  
						if(RC_MODE == RCSW_BOTTOM){					
							//����MODE_RELAX  
							robotMode = MODE_RELAX;             
						}
						//�����̨������� ����ģʽ�л�
						else if(getGimbalData()->initFinishFlag){	  
							digitalLo(&getGimbalData()->initFinishFlag);
							//��ȡң����sw2��״̬	
							robotMode = (robotModeStruct_t)ControlMode(); 		
						}
					}break;
					//�����ǰΪ����ģʽ 
					case MODE_KM :        								  				
					//�����ǰΪҡ��ģʽ																		
					case MODE_RC :{ 										
						//���SW2����͵�ʱ	  			
						if(RC_MODE == RCSW_BOTTOM){					 
							//����ģʽΪRELAXģʽ	 
							robotMode = MODE_RELAX;             
						}
						else {
							//������Խ���ģʽ�л�	��ȡSW2��״̬
							robotMode = (robotModeStruct_t)ControlMode();
						}
					}break;
					//�����ǰ��⵽ģʽΪ����ģʽ��˵���ּ�⵽ң��������Ȼ�������в������û�ж���
					case MODE_STOP :{ 								
						//��⵽ң�����л�����ʼ��ģʽ		  
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
            
            //LCM: ������ ģʽ����
            case BOARD_OTHER : {
				switch(robotMode){
					//�����ǰΪMODE_INIT
					case MODE_INIT :{ 									
						//���SW2����͵�ʱ	  
						if(RC_MODE == RCSW_BOTTOM){					
							//����MODE_RELAX  
							robotMode = MODE_RELAX;             
						}
						else {	 
							//��ȡң����sw2��״̬	
							robotMode = (robotModeStruct_t)ControlMode(); 		
						}
					}break;
					//�����ǰΪ����ģʽ 
					case MODE_KM :        								  				
					//�����ǰΪҡ��ģʽ																		
					case MODE_RC :{ 										
						//���SW2����͵�ʱ	  			
						if(RC_MODE == RCSW_BOTTOM){					 
							//����ģʽΪRELAXģʽ	 
							robotMode = MODE_RELAX;             
						}
						else {
							//������Խ���ģʽ�л�	��ȡSW2��״̬
							robotMode = (robotModeStruct_t)ControlMode();
						}
					}break;
					//�����ǰ��⵽ģʽΪ����ģʽ��˵���ּ�⵽ң��������Ȼ�������в������û�ж���
					case MODE_STOP :{ 								
						//��⵽ң�����л�����ʼ��ģʽ		  
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
//���ư����͸��º���
void boardTypeConfirm(uint16_t boardFlag,DeviceActivation_t *boardType){
	if(BOARD_TYPE & boardFlag){
		boardType();
	}
}
//��ʼ������º���
void controlDeviceConfirm(uint16_t deviceFlag,DeviceActivation_t *deviceFunction){
	if(robotConfigData.robotDeviceList & deviceFlag){
		deviceFunction();
	}
}
//���̰���Ƴ�ʼ��
static void congfigChassisInit(void){
	//���̳�ʼ��
	chassisInit();				
    //���ױ��γ�ʼ�� 								
	controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingInit);
}
//��̨���Ƴ�ʼ��
static void congtrolGlobalInit(void){
	//��̨��ʼ��
	controlDeviceConfirm(DEVICE_GIMBAL,gimbalInit);																														
	//���ױ��γ�ʼ�� 								
	controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingInit);							
	//���䲹��������ʼ��
	controlDeviceConfirm(DEVICE_SHOOT,shootInit);                               
	//�Ӿ�������Ϣ��ʼ��                             
	visionSendDataInit();                           					
	//������ʼ��							
	pneumaticInit();
	//�Զ������ʼ��
	autoTaskInit();
	//�����ʼ��
//	servoInitClass.Init();
	controlData.temperatureFaultState = false;
	getinfantryAutoData()->rotateEnb = false;
	getinfantryAutoData()->enbStart = false;
	getinfantryAutoData()->enbStep = 0;
	getinfantryAutoData()->rcYaw = &getGimbalData()->yawMotorAngle;
	//������Ϣ�������
	//getcontrolData()->boardDataFinish = INIT_STATE;
}

//LCM: �������豸��ʼ��
static void otherDeviceInit() {
    
}

//LCM: �������ʼ��
static void controlOtherInit(void) {
	//�Զ������ʼ��
	autoTaskInit();
    otherDeviceInit();
    
	controlData.temperatureFaultState = false;
	getinfantryAutoData()->rotateEnb = false;
	getinfantryAutoData()->enbStart = false;
	getinfantryAutoData()->enbStep = 0;
	getinfantryAutoData()->rcYaw = &getGimbalData()->yawMotorAngle;
	//������Ϣ�������
	//getcontrolData()->boardDataFinish = INIT_STATE;    
}

static void controlRobotInit(){
	//���ذ���ع��ܳ�ʼ��
	boardTypeConfirm(BOARD_CONTROL,congtrolGlobalInit);
	//���̰���ع��ܳ�ʼ��
	boardTypeConfirm(BOARD_CHASSIS,congfigChassisInit);
    //LCM: ������ ��ع��ܳ�ʼ��
    boardTypeConfirm( BOARD_OTHER , controlOtherInit );    
}

static void controlChassisUpdata(void){
	if(!(controlData.loops % 2)){	
			if(judgeData.initFlag)
				//����ϵͳ���ݸ���
				judgeTask();																											
	}
	//���̸���
	chassisUpdate();
	//CAN���͸���
    if(getConfigData()->robotType){
        canSendUpdate();	
    }
}

static void controlControlUpdata(void){
	if(!(controlData.loops % 2)){	//DCQ: �o�˙C�οز���ϵ�y���Ք���
			if(judgeData.initFlag)
				//����ϵͳ���ݸ���
				judgeTask();																											
	}
	if(remoteControlData.initFlag)
		//ң�������ݸ���
		rcUpdateTask();																											
	if(controlData.dataInitFlag){
		//������´���	
		motorTemperatureControl();		
		//��ȡ������ģʽ
		getGlobalMode();               
		//��ȡĦ����ģʽ	     																
        getShootMode();									
	}						  	
	//�Զ�����ִ��
	autoTaskUpdate();	
	//���ط���������
	/********�����*******/
	//���ױ��θ���	
	controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingUpdate);		
	//��̨����			
	controlDeviceConfirm(DEVICE_GIMBAL,gimbalUpdate);															
	//�����������
	controlDeviceConfirm(DEVICE_SHOOT,shootUpdate);			
	//CAN���͸���
	controlDeviceConfirm(DEVICE_CANSEND,canSendUpdate);	
	//4msһ�εĿ���
	if(!(controlData.loops % 2)){																				
		//������������
		controlDeviceConfirm(DEVICE_SUPPLY,supplyUpdate);													
	}
	getGimbalData()->lastCtrlMode =	getGimbalData()->ctrlMode;	
	lastRobotMode = robotMode;
	
}

//LCM: ���������
static void controlOtherUpdate(void) {
	if(remoteControlData.initFlag)
		//ң�������ݸ���
		rcUpdateTask();																											
	if(controlData.dataInitFlag){
		//��ȡ������ģʽ
		getGlobalMode();               								
	}						  	
	//�Զ�����ִ��
	autoTaskUpdate();
    //LCM: ���ڷ���ܸ���
    controlDeviceConfirm( DEVICE_MISSILE , missileUpdate );
	//LCM: ���ڷ����CAN���͸���
	controlDeviceConfirm( DEVICE_MISSILE , missileCanSend );
    
	getGimbalData()->lastCtrlMode =	getGimbalData()->ctrlMode;	
	lastRobotMode = robotMode;	
}

void controlSetup(void){
	//�˻����ƽ����ĳ�ʼ��
	rcInit();
	//�Ӿ�����
	visionInit();
    //������ñ��ʼ��
	motorSeverInitClass.Init();
    //CAN��ʼ��
	canSendInit();
	if(ROBOT == UAV_ID){
		//����ϵͳ��ʼ��
	jugdeInit();
	}
	//�ܿ��Ƴ�ʼ��
	controlInit();
    //��̨���ʼ�����
    //digitalHi(&robotConfigData.id_set);
    digitalHi(&getcontrolData()->controlBoardDataFinish);
}
//���̰��ϵ��ʼ��
void chassisSetup(void){
    //�¼���־��
	xEventGroupWaitBits(taskInitData.eventGroups,CONTROL_RES_OK,pdTRUE,pdTRUE,portMAX_DELAY);
    //������ID��������ʶ��
    robotBoardConfig();
	//����ϵͳ��ʼ��
	jugdeInit();
    //������ñ��ʼ��
	motorSeverInitClass.Init();
    //CAN��ʼ��
	canSendInit();
	//ADC�ɼ���ʼ��											
	adcClass.Init(); 
	//�ܿ��Ƴ�ʼ��
	controlInit();
	//��ʼ�����
	digitalHi(&getcontrolData()->chassisBoardDataFinish);
}

//LCM:  �������ʼ��
void otherSetup( void ) {
	//�˻����ƽ����ĳ�ʼ��
	rcInit();
    //CAN��ʼ��
	canSendInit();
	//�ܿ��Ƴ�ʼ��
	controlInit();
    //��̨���ʼ�����
    //digitalHi(&robotConfigData.id_set);   
    digitalHi(&getcontrolData()->controlBoardDataFinish);    
}

static void controlRobotUpdata(){
    super_capacitorTask();
	boardTypeConfirm(BOARD_CHASSIS,controlChassisUpdata);
	boardTypeConfirm(BOARD_CONTROL,controlControlUpdata);
    //LCM:  ���������
    boardTypeConfirm( BOARD_OTHER , controlOtherUpdate );   
}

void controlUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(true){	
		vTaskDelayUntil(&xLastWakeTime,CONTROL_PERIOD);
		//imu�����������򲻼���ִ��
//		if(supervisorData.state & STATE_SENSOR_ERROR)													
//			continue;
//		if(!supervisorData.taskEvent[SUPERVISOR_TASK])
//			continue;		
		//�����ڳɹ���ȡ����������Ϣ�����ִ�г�ʼ��
		if(robotConfigData.distinguishState == ROBOT_COMPLETE_IDENTIFIED){					
			//���п���ȫ����ʼ�����������еĽṹ���г�ʼ��
			controlRobotInit();																								
			//���ͷ�ֹ�ظ���ʼ������ֹ�ظ���ID
			robotConfigData.distinguishState = ROBOT_NO_NEED_TO_IDENTIFY;																															
			digitalHi(&controlData.dataInitFlag);
		}
		//���ư���࣬������
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


	

