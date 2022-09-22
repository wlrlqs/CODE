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
changeChassisTaskStruct_t changeChassisTaskData;         //���ĵ�������
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
	//���Ϊ������
	infantryAutoData.taskState = EXECUTING;								
	//��ִ�����м�һ		
	digitalIncreasing(&infantryAutoData.schedule);	
}
//�������/�����������

void infantryAutomaticAimUpdate(void){		
	//��schedule�����ӡ����ٺ��ж��������
	switch(infantryAutoData.schedule){													
		case 1: gimbalSwitch(DISABLE);												
            chassisSwitch(DISABLE);
				//����TX2����
				visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);
        digitalIncreasing(&(infantryAutoData.schedule));	
            break;
		case 2: gimbalSwitch(ENABLE);	
            shootVisionInit();
            //����ģʽ����//�Ҽ� ��Ԥ�д��
            if(KB_PJEJUDGMENT){															
                getvisionData()->prejudgFlag = true;	
                getvisionData()->manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE * 0.5f;
                getvisionData()->manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
            }
            //��� ����Ԥ�д��
            else{																							
                getvisionData()->prejudgFlag = false;											
                digitalClan(&getvisionData()->manualPitchBias);
                digitalClan(&getvisionData()->manualYawBias);
            }
									
            if(infantryAutoData.breakSign){
                //����д�����������
                infantryAutoData.schedule = 99;									
                getvisionData()->prejudgFlag = false;
                shootDataReset();	
            }			
            break; 
		//ֻ�е���99ʱ�����˳�
		case 99: infantryAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						 
		//��������б���û�н��ȣ����������
		default: infantryAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;
}

//�������/�����������
void infantryAutomaticBuffUpdate(void){										
	static shootMode_e originalShootMode = MANUAL_SINGLE;
	//��schedule�����ӡ����ٺ��ж��������
	switch(infantryAutoData.schedule){													
		case 1: gimbalSwitch(DISABLE);
            chassisSwitch(DISABLE);
            originalShootMode = getshootData()->shootMode_17mm;
			if(infantryAutoData.currentTask == INFANTRY_ACTIVE_BUFF)
				//����TX2����
				visionSendDataUpdate(TX2_DISTINGUISH_BUFF,SMALL_BULLET);
			else
				visionSendDataUpdate(TX2_DISTINGUISH_BIGBUFF,SMALL_BULLET);
            digitalIncreasing(&(infantryAutoData.schedule));	
            break;
		case 2: gimbalSwitch(ENABLE);	
            chassisSwitch(ENABLE);
			//������ͣ
			digitalHi(&getchassisData()->suspend_chassis);					
            shootVisionInit();
            getshootData()->shootMode_17mm = MANUAL_SINGLE;
            //����ģʽ����//���
            if(KB_NO_PJEJUDGMENT){														
                getvisionData()->prejudgFlag = false;														
            }
            
            //����W/S/A/D�������ģʽ
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
                //����д�����������
                infantryAutoData.schedule = 99;
				digitalLo(&getchassisData()->suspend_chassis);
                getvisionData()->prejudgFlag = false;
                getshootData()->shootMode_17mm = originalShootMode;
                shootDataReset();	
            }			
            break; 
		//ֻ�е���99ʱ�����˳�
		case 99: infantryAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						 
		//��������б���û�н��ȣ����������
		default: infantryAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;
}

//�ӵ�����������£�������
void infantryBulletTransferUpdate(void){									
	//����д�ϣ��жϣ�
	if(infantryAutoData.breakSign){		
		//��ת�����ٶ�����ֵ				
		getshootData()->supplyRef = 2000;		
		infantryAutoData.taskDuration = 62.1f;
		//ֱ���������һ��
		infantryAutoData.schedule = 4;	
		getshootData()->supplyPlaceFlag = supply_close;
		digitalLo(&infantryAutoData.breakSign) ; 
	}
	//��B������
	if((getshootData()->supplyNumFlag != 1) && infantryAutoData.taskDuration <60.0f ){ 
		getshootData()->supplyNumFlag = 1;
		infantryAutoData.taskDuration = 0.0f;
		infantryAutoData.schedule = 2;
	}
	//��schedule�����ӡ����ٺ��ж��������
	switch(infantryAutoData.schedule){
		case 1:
			getshootData()->supplyRef = 80;
			break;
		case 2: 
			if(infantryAutoData.taskDuration < 0.3f){
				getshootData()->supplyRef = -2000;
				getshootData()->supplyPlaceFlag = supply_move;
				//���ٹ��̺�����ٶȶ�������ԭ����40%
				getchassisData()->speedLimit = 0.4f;								
			}
			else{
				getshootData()->supplyRef = -80;
				getshootData()->supplyPlaceFlag=supply_open;
				digitalIncreasing(&infantryAutoData.schedule);
			}
			break;
		case 3: 
			//����ĳ���30��
			if(infantryAutoData.taskDuration > 62.0f){		
				getshootData()->supplyRef = 2000;
				getshootData()->supplyPlaceFlag = supply_move;
				digitalIncreasing(&infantryAutoData.schedule);
			}
			 break;
		case 4:
			//������ԭ����ֵ
			getchassisData()->speedLimit = 1.0f;				
			//2����ʱ��ر�	  
			if(infantryAutoData.taskDuration > 62.3f){			
				getshootData()->supplyRef = 80;
				getshootData()->supplyPlaceFlag = supply_close;
				infantryAutoData.schedule = 99;
			}
			break;
		//ֻ�е���99ʱ�����˳�
		case 99: 
			infantryAutoData.taskState = END_OF_EXECUTION; 
		  getshootData()->supplyNumFlag = 0;
		break;						
		//��������б���û�н��ȣ����������
		default: infantryAutoData.taskState = EXECUTION_ERROR; break;							
	}	
	//��ʱ
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;	
}

void infantryAviodUpdate(void){													 
	static int8_t sign = 1;
	//Ť����С����
	if(infantryAutoData.avoidTask){                    //����E��ʱ��ʹ��
		static uint8_t lastKeyE;
		if(!lastKeyE && !infantryAutoData.aviodFlag && getrobotMode() != MODE_RELAX){																	
				if(PRESS_E){
					infantryAutoData.aviodFlag = 1;
					//��һ����ת�ı�ת��
				}
			}
		else{ 
			//�ٴΰ��½����ת
			if(lastKeyE && infantryAutoData.aviodFlag){
				if(PRESS_E || TASK_PRESS_F)
					infantryAutoData.aviodFlag = 0;
			}
			//�����Զ����С����
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
	//С������ת���
	static uint8_t lastKeySate,kmRotateFlag,rcRotateFlag,rotateDirection;
		if(RC_ROTATE > 100)
				rcRotateFlag = 1;
			else
				rcRotateFlag = 0;
		if(RC_ROTATE < -100)
				infantryAutoData.aviodFlag = 1;
			else
				infantryAutoData.aviodFlag = 0;		
			//һ��������ת                                ���¿�
			if(!lastKeySate && !kmRotateFlag && getrobotMode() != MODE_RELAX){																	
				if(PRESS_Q){
					kmRotateFlag = 1;
					//��һ����ת�ı�ת��
					infantryAutoData.km_rotate = 1;
					rotateDirection = !rotateDirection;
				}
			}
			else{ 
				//�ٴΰ��½����ת
				if(!lastKeySate && kmRotateFlag){
					if(PRESS_Q || TASK_PRESS_F)
						kmRotateFlag = 0;
						infantryAutoData.km_rotate = 0;
				}
				//�����Զ����С����
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
				
				//������תʱ��
				infantryAutoData.rotateTime += RTIMEFEQ;
				
				//������ת�ٶȱ�
				getchassisData()->rotate = (f32_t)0.6*sinf(infantryAutoData.rotateTime)+1.5f;
				
				//�ٶȱ��޷�
				getchassisData()->rotate = SINF_RUDD(getchassisData()->rotate);
				
#if ROTATE_ORIENT
				//�������ݽ���ָ��
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
//���ݽ�����ָ̨��
void rotateOff(void){
	int16_t rotateCode;
	//������ƫλ
	rotateCode = motorHandleClass.Encoder(&commandoMotorConfig[YAWMOTOR]) - getConfigData()->yawCenter;
	//��������
	if(T_OR_F0(rotateCode)) rotateCode = 16384 + rotateCode;
	//ָ��
	if(infantryAutoData.rotateEnb){
		//ָ��ֻ��������������������
		if(!infantryAutoData.rotateFlag){
			switch(infantryAutoData.enbStep){
				case 0:{
					//��̨YAW��ǰλ��λ��ǰװ�װ��90�㷶Χ��
					if(rotateCode <= 4096 || rotateCode > 12288){
						//ǹ��ָ��ͷ
						getGimbalData()->chassisChange = getConfigData()->yawCenter;
						getchassisData()->roDir = 0x00;
					}
					//��̨YAW��ǰλ��λ�ں�װ�װ��90�㷶Χ��
					else if(rotateCode > 4096 && rotateCode <= 12288){
						//ǹ��ָ��β
						getGimbalData()->chassisChange = getConfigData()->yawCenter + 8192;
						getchassisData()->roDir = 0x01;
					}
					digitalIncreasing(&infantryAutoData.enbStep);
				}break;
				case 1:{
					//���ǹ��ָ��ͷ�������˶�
					if(getGimbalData()->chassisChange == getConfigData()->yawCenter){
						getchassisData()->direction = 1;
					}
					else{
					//ǹ��ָ��β�����˶�
						getchassisData()->direction = -1;
					}
					infantryAutoData.enbStep = 99;
				}break;
				case 99:{
					//ָ���,ֻ�����´ο������ݺ�����ٴο���
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



//�ڱ�����ʼ�͸���ʱ�Զ�����Ħ���֣�����ͨ�������Ҽ�����Ħ����
void infantryChangeFricState(void){
	//������ڱ���״̬������Ѫ����Ϊ0
	if((judgeData.extGameRobotState.remain_HP != 0) && (TASK_PRESS_V ==0)){
		//���Ħ���ִ��ڹر�״̬�����Ħ���֣�����ر�Ħ����
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
				sign = 1;    //��������
				digitalIncreasing(&infantryAutoData.schedule);
			}
		  if(PRESS_C){ 
				sign = -1;    //��������
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
		//ֻ�е���99ʱ�����˳�
		case 99: infantryAutoData.taskState = END_OF_EXECUTION; break;
				//��������б���û�н��ȣ����������
		default: infantryAutoData.taskState = EXECUTION_ERROR; break;	
	}		
}

//����������̨����
void separate_gimbal_chassiss(void){
     if(PRESS_C){
	    infantryAutoData.separate_flag = 1;
	 }
      else{
	    infantryAutoData.separate_flag = 0;
	  }	 
}

void infantryAutoTaskUpdate(void){
		//������ڱ���״̬�����Ҹոո���ʱ
	if((judgeData.extGameRobotState.remain_HP!=0) && (infantryFricState.infantryFricNumFlag==1)){
		//����ʱִ�У��´θ���֮ǰ������ִ�и�if������
		infantryFricState.infantryFricNumFlag = 0;
		//��Ħ����
		infantryFricState.infantryFricStateFlag = 1;
	}
	//������ڱ���״̬����������ʱ
	else if((judgeData.extGameRobotState.remain_HP==0) && (infantryFricState.infantryFricNumFlag==0)){
		//���������ȴ�����ʱ��Ħ����
		infantryFricState.infantryFricNumFlag = 1;
		//�ر�Ħ����
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
			infantryAutoData.currentTask = INFANTRY_AUTOMATIC_AIM;  //�������� �ֶ�ģʽ
		}
	if(infantryAutoData.currentTask != INFANTRY_ACTIVE_BUFF || infantryAutoData.currentTask != INFANTRY_ACTIVE_B_BUFF){
		infantryAviodUpdate(); //�������  ��INFANTRY_MANUAL���ֶ�ģʽ
	
#if ROTATE_ORIENT
		rotateOff();
#endif
	}
	else{
		//���ʱ�������
		getchassisData()->chaseRef = 0.0f;
		infantryAutoData.aviodFlag = false;
		digitalClan(&infantryAutoData.avoidSchedule);				
		infantryAutoData.avoidTask = DISABLE;		
	}					
  //����Ħ����
  if(keyBoardCtrlData.rkSta == KEY_PRESS_ONCE){
		infantryChangeFricState();
	}
	
	//�����п�ִ������
	if(infantryAutoData.currentTask 
		!= INFANTRY_MANUAL){		
		if(infantryAutoData.currentTask != INFANTRY_BULLET_TRANSFER \
			&& infantryAutoData.lastTask == INFANTRY_BULLET_TRANSFER){
			getshootData()->supplyRef = 80;
		}
		//�������ոտ�ʼִ��   δִ�У�UNEXECUTED��
		if(infantryAutoData.taskState == UNEXECUTED){					
			//ִ�п�ʼ����
			infantryTaskBegin();																
		}
		//�����ִ����
		else if(infantryAutoData.taskState == EXECUTING){		
			//��ǰ����
			switch(infantryAutoData.currentTask){								
				//������׼
				case INFANTRY_AUTOMATIC_AIM:  {										
					infantryAutomaticAimUpdate();
					break;
				}
				//������
				case INFANTRY_ACTIVE_BUFF:
				case INFANTRY_ACTIVE_B_BUFF:{											
					infantryAutomaticBuffUpdate();
					break;
				}
				//�ӵ�����
				case INFANTRY_BULLET_TRANSFER: {									
					infantryBulletTransferUpdate();
					break;
				}
				//������
				case INFANTRY_CHANGE_SHOOT_RATE:
					infantryShootTaskUpdate();
				break;
				//��������������ֱ�����³�ʼ���ṹ��
				default: {																				
					autoDataInit(&infantryAutoData); 
					break;
				}
			}
		}
		//���ִ����ϻ�ִ�д��������³�ʼ���ṹ��
		else{																									
			autoDataInit(&infantryAutoData);
		}
	}
	//������һ�̵�����
	infantryAutoData.lastTask = infantryAutoData.currentTask;
}

