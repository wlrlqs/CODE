#include "config.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "deforming.h"
#include "vision.h"
#include "auto_tank.h"
#include "rc.h"
#include "DRIVER_VL53L0X.h"
#include "keyboard.h"
#include "judge.h"
AutoTaskStruct_t tankAutoData;
ftankfricFlagStruct_t ftankFricState;
AutoTaskStruct_t* getTankAutoData(){
        return &tankAutoData;
}
ftankfricFlagStruct_t* getftankFricState(void){
	  return &ftankFricState;
}

void tankTaskBegin(void){
	//���Ϊ������
	tankAutoData.taskState = EXECUTING;						
	//��ִ�����м�һ		
	digitalIncreasing(&tankAutoData.schedule);					
}
//�ڱ�����ʼ�͸���ʱ�Զ�����Ħ���֣�����ͨ�������Ҽ�����Ħ����
void ftankChangeFricState(void){
	//������ڱ���״̬������Ѫ����Ϊ0
	if((judgeData.extGameRobotState.remain_HP != 0) ){
		//���Ħ���ִ��ڹر�״̬�����Ħ���֣�����ر�Ħ����
		if(ftankFricState.ftankFricStateFlag == 0)  
			ftankFricState.ftankFricStateFlag = 1;
		else ftankFricState.ftankFricStateFlag = 0;
	}
}

/* p_tank Hand over to auxiliary */
void p_tankBulletAroundUpdate(void){
    switch(tankAutoData.schedule){                                              //��schedule�����ӡ����ٺ��ж��������
        case 0:{                     
            if(tankAutoData.keyboard_X_Flag){
                digitalIncreasing(&tankAutoData.schedule);
                digitalLo(&tankAutoData.keyboard_X_Flag);
            }
            else if(TASK_PRESS_X){
                digitalHi(&tankAutoData.keyboard_X_Flag);
            }
        }break;
        case 1:{
					getshootData()->bigsupplySpeedOut = -3500;
					digitalIncreasing(&tankAutoData.loadtime);
                if(tankAutoData.loadtime >  0X490){
									digitalClan(&tankAutoData.loadtime);
									digitalIncreasing(&tankAutoData.schedule);
								}
        }break;
        case 2:{
            if(TASK_PRESS_X){
						getshootData()->bigsupplySpeedOut = 3500;
                    PAout(2)=0;
                    PAout(3)=0;
                if(tankAutoData.loadtime >  0x090){
								digitalSet(&tankAutoData.schedule,99);
									digitalClan(&tankAutoData.loadtime);
                    digitalLo(&tankAutoData.keyboard_X_Flag);
                }
                digitalIncreasing(&tankAutoData.loadtime);
            }
//            else if(TASK_PRESS_X){
//                digitalHi(&tankAutoData.keyboard_X_Flag);
//            }
        }break;

                
        case 99:{
            digitalClan(&tankAutoData.schedule);
            digitalLo(&tankAutoData.keyboard_X_Flag);
            digitalClan(&tankAutoData.loadtime);
            digitalLo(&P_HERO_42_LID);       //�رմ󵯲�
						getshootData()->bigsupplySpeedOut = -3500;
        break;
        }

    }
}

/* p_tank mortar */
void p_tankMortarUpdate(void){
	//����д�ϣ��жϣ�
	if(tankAutoData.breakSign){													
		//ֱ���������һ��
		tankAutoData.schedule = 4;												
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_CTRL && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 3;
	}
	//��schedule�����ӡ����ٺ��ж��������
	switch(tankAutoData.schedule){											
		case 1: {
            gimbalStopSwitch(DISABLE);
			gimbalSwitch(DISABLE);
			digitalIncreasing(&tankAutoData.schedule);
		} break;
		case 2:{ 		 
			if(getGimbalData()->initFinishFlag){
			digitalHi(&getGimbalData()->followLock);
			gimbalSwitch(ENABLE);
			getchassisData()->ctrlMode = CHASSIS_STOP;
			digitalIncreasing(&tankAutoData.schedule);
			}
		} break;
		case 3:{ 	
			//�Ȼ���ģʽά��20s
			if(tankAutoData.taskDuration < 30.0f){			
			visionMortar();
				//�Ҽ���Ԥ�д��	
				if(KB_PJEJUDGMENT){																
					getvisionData()->manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
					getvisionData()->manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE;
				}
			}
			else{
				digitalIncreasing(&tankAutoData.schedule);
			}
		} break;
		case 4: {
			gimbalStopSwitch(DISABLE);
			gimbalSwitch(DISABLE);
			getvisionData()->prejudgFlag = false;	
			shootDataReset();	
			digitalLo(&getGimbalData()->followLock);
			digitalClan(&getvisionData()->manualPitchBias);
			digitalClan(&getvisionData()->manualYawBias);
			tankAutoData.schedule = 99;
		}  break;			 
		//ֻ�е���99ʱ�����˳�
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						
		//��������б���û�н��ȣ����������
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}



/* p_tank Hand over to depot */
void p_tankBulletSupplyUpdate(void){
	//����д�ϣ��жϣ�
	if(tankAutoData.breakSign){													
		//ֱ���������һ��
		tankAutoData.schedule = 3;												
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_R && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 1;
	}
	//��schedule�����ӡ����ٺ��ж��������
	switch(tankAutoData.schedule){											
		case 1: 
			if(tankAutoData.taskDuration < 1.0f){
				//����С����
				digitalHi(&P_HERO_17_LID);      
				//���ٹ��̺�����ٶȶ�������ԭ����50%
				getchassisData()->speedLimit = 0.5f;								
			}
			else{
				digitalIncreasing(&tankAutoData.schedule);
			}
			break;
		case 2: 	
			//30������ɲ���
			if(tankAutoData.taskDuration > 30.0f){			
				digitalIncreasing(&tankAutoData.schedule);
			}
			break;
		case 3:
			digitalLo(&P_HERO_42_LID);      						//�رմ󵯲�
			digitalLo(&P_HERO_17_LID);      						//�ر�С����
			getchassisData()->speedLimit = 1.0f;							//������ԭ����ֵ
			tankAutoData.schedule = 99;
			break;
		//ֻ�е���99ʱ�����˳�
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;	
		//��������б���û�н��ȣ����������
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}



/*p_tank Switching in 17mm bullet and 42mm bullet */
void p_tankChangeHitModeUpdate(void){									
	if(TASK_PRESS_Z ){
		digitalTogg(&getshootData()->p_tankshootmode);
	}
}

/*p_tank automatic aiming */
void p_tankAutomaticAimUpdate(void){										
	//��schedule�����ӡ����ٺ��ж��������
	switch(tankAutoData.schedule){											
		case 1: gimbalSwitch(DISABLE);												
				chassisSwitch(DISABLE);
				shootVisionInit();
				//����TX2����
				visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,BIG_BULLET);							
				digitalIncreasing(&(tankAutoData.schedule));	
				break;
		case 2: gimbalSwitch(ENABLE);	
						//����ģʽ����//�Ҽ���Ԥ�д��
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
	
						if(tankAutoData.breakSign){
							//����д�����������
							tankAutoData.schedule = 99;							
							getvisionData()->prejudgFlag = false;	
							shootDataReset();	
						}			
						break;
		//ֻ�е���99ʱ�����˳�
		case 99: tankAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  
		//��������б���û�н��ȣ����������
		default: tankAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/*p_tank automatic Aviod */
void p_tankAviodUpdate(void){
	static uint8_t avoidTurn = 0; 
	if(tankAutoData.avoidTask){
		if(avoidTurn){
			if(getchassisData()->chaseRef < AVOID_RANGE)
				getchassisData()->chaseRef += AVOID_RATE;
			else if(getchassisData()->chaseRef > AVOID_RANGE){
				getchassisData()->chaseRef -= AVOID_RATE;
				avoidTurn = 0;
			}
		}
		else{
			if(getchassisData()->chaseRef > -AVOID_RANGE)
				getchassisData()->chaseRef -= AVOID_RATE;
			else if(getchassisData()->chaseRef < -AVOID_RANGE){
				getchassisData()->chaseRef += AVOID_RATE;
				avoidTurn = 1;
			}
		}
		//�����ʱ����6.0s��ʱ�䣬�򽫵�ǰ�������
		if(tankAutoData.aviodDuration > R_TIME){					
			getchassisData()->chaseRef = 0.0f;
			tankAutoData.avoidTask = DISABLE;
		}	
		tankAutoData.aviodDuration += AUTO_TASK_DURATION;	
	}
	else{	
	//��������ת���
	        static uint8_t lastKeySate,kmRotateFlag,rcRotateFlag,rotateDirection;
		       if(RC_ROTATE > 100)
				  rcRotateFlag = 1;
			   else
				rcRotateFlag = 0;
			   //һ��������ת
			   if(!lastKeySate && !kmRotateFlag){																	
				    if(PRESS_Q){
					    kmRotateFlag = 1;
					    //��һ����ת�ı�ת��
					    rotateDirection = !rotateDirection;
				    }
			   }
			   else{
				//�ٴΰ��½����ת
				    if(!lastKeySate && kmRotateFlag){
					     if(PRESS_Q || TASK_PRESS_F)
						kmRotateFlag = 0;
				     }
				      //�����Զ����С����
				    if(tankAutoData.closeRotate){
					    kmRotateFlag = 0;
					    tankAutoData.closeRotate = false;
				     }
			  }
			  lastKeySate = PRESS_Q;
			
			   if(kmRotateFlag || rcRotateFlag){
				   if(rotateDirection)
				       getchassisData()->chaseRef -= (AVOID_RATE * 4);
				   else
					   getchassisData()->chaseRef += (AVOID_RATE * 4);
				       tankAutoData.rotateFlag = true;
				       //������תʱ��
				       tankAutoData.rotateTime += RTIMEFEQ;
				        //������ת�ٶȱ�
				       getchassisData()->rotate = (f32_t)0.6*sinf(tankAutoData.rotateTime)+1.2f;
				       //�ٶȱ��޷�
				       getchassisData()->rotate = SINF_RUDD(getchassisData()->rotate);
			   }
	
			else{
				getchassisData()->chaseRef = 0.0f;
				tankAutoData.rotateFlag = false;
				getchassisData()->rotate = 1.0f;
				digitalClan(&tankAutoData.rotateTime);
			}
	}
}

/*p_tank Suicide fire*/
void p_tankSuicideFireUpdate(void){
	if(PRESS_C){
		digitalHi(&getshootData()->suicideFireFlag);
	}
	else{
		digitalLo(&getshootData()->suicideFireFlag);
	}
}
/* p_tank Autotask list*/
AutotankStruct_t tankAutoSerial = {
	p_tankBulletAroundUpdate,
	p_tankMortarUpdate,
	p_tankBulletSupplyUpdate,
	p_tankChangeHitModeUpdate,
	p_tankAutomaticAimUpdate,
	p_tankAviodUpdate,
	p_tankSuicideFireUpdate,
	p_tankSuicideFireUpdate,
};
void p_tankAutoTaskUpdate(void){
     super_capacitorTask();
	//������񣨺Ͳ������������ͬ��
	tankAutoSerial.p_tank_q_task();																	
	//��ɱ����ģʽ����
	tankAutoSerial.p_tank_c_task();													                                   
	//�����п�ִ������  TANK_MANUAL���ֶ�)
	if(tankAutoData.currentTask != TANK_MANUAL){										
		//�������ոտ�ʼִ��
		if(tankAutoData.taskState == UNEXECUTED){										
			//ִ�п�ʼ����
			tankTaskBegin();																
		}
		//�����ִ����
		else if(tankAutoData.taskState == EXECUTING){		
			switch(tankAutoData.currentTask){

				//42mm��ҩ����
				case TANK_BULLET_TRANSFER: {	
					tankAutoSerial.p_tank_x_task();
					break;
				}
				//�Ȼ���ģʽ
				case TANK_MORTAR: {														
					tankAutoSerial.p_tank_ctrl_task();
					break;
				}
				//17mm��ҩ����վ����
				case TANK_BULLET_SUPPLY: {									   
					tankAutoSerial.p_tank_r_task();
					break;
				}
				//�л����ģʽ
				case TANK_CHANGE_HIT_MODE: {									 
					tankAutoSerial.p_tank_z_task();
					break;
				}	
				//������׼
				case TANK_AUTOMATIC_AIM: {										
					tankAutoSerial.p_tank_v_task();
					break;
				}
				//��������������ֱ�����³�ʼ���ṹ��
				default: {																	
					//����������			
					autoDataInit(&tankAutoData); 										
					break;
				}
			}
		}
		//���ִ����ϻ�ִ�д���������
		else{																									
			autoDataInit(&tankAutoData);
		}
	}
}
/******************************************************/
/*****This is an boundary between two type of tank*****/
/******************************************************/
void f_tankTaskBegin(void){
	//���Ϊ������
	tankAutoData.taskState = EXECUTING;						
	//��ִ�����м�һ		
	digitalIncreasing(&tankAutoData.schedule);					
}
/* �������&����������� */
void f_tankAutomaticAimUpdate(void){										
	//��schedule�����ӡ����ٺ��ж��������
	switch(tankAutoData.schedule){											
		case 1: gimbalSwitch(DISABLE);												
				chassisSwitch(DISABLE);
				shootVisionInit();
				//����TX2����
				visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,BIG_BULLET);							
				digitalIncreasing(&(tankAutoData.schedule));	
				break;
		case 2: gimbalSwitch(ENABLE);	
						//����ģʽ����//�Ҽ���Ԥ�д��
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
	
						if(tankAutoData.breakSign){
							//����д�����������
							tankAutoData.schedule = 99;							
							getvisionData()->prejudgFlag = false;	
							shootDataReset();	
						}			
						break;
		//ֻ�е���99ʱ�����˳�
		case 99: tankAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  
		//��������б���û�н��ȣ����������
		default: tankAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* �Զ���ͷ42mm��ҩ����������� */
void f_tankBulletAroundUpdate(void){
 if(tankAutoData.breakSign){
	 tankAutoData.schedule = 4;
	 digitalLo(&tankAutoData.breakSign);
 }
 
    switch(tankAutoData.schedule){                                              //��schedule�����ӡ����ٺ��ж��������
        case 1: 
		   digitalHi(&tankAutoData.get_pill_Flag);
           getshootData()->supplyRef = -2000;
           getGimbalData()->chassisChange = getConfigData()->yawCenter + 8192;
		   getchassisData()->direction = -1;
		   digitalIncreasing(&tankAutoData.schedule);
		   break;
        case 2:	
		   if(tankAutoData.taskDuration > 1.0f){				  
			   getshootData()->supplyRef = -150;
			   digitalIncreasing(&tankAutoData.schedule);
		   }
           break;
        case 3:
            if(TASK_PRESS_R){
                digitalIncreasing(&tankAutoData.schedule);
			} 		
            break;
        case 4:    
            getshootData()->supplyRef = 2000; 
			getGimbalData()->chassisChange = getConfigData()->yawCenter;
		    getchassisData()->direction = 1;
		    digitalLo(&tankAutoData.get_pill_Flag);
            digitalClan(&tankAutoData.taskDuration);	
         	digitalIncreasing(&tankAutoData.schedule);	
            break;
        case 5:
		    if(tankAutoData.taskDuration > 1.0f){		
                getshootData()->supplyRef = 150;
 		        digitalSet(&tankAutoData.schedule,99);
			}
			break;
		case 99:			
			tankAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);	    
		    break;						 
		//��������б���û�н��ȣ����������
		default: 
			tankAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);
		    break;	
    }
	tankAutoData.taskDuration+=AUTO_TASK_DURATION;	
}

/* ҡ�ڶ���ӵ�������� */
void f_tankAviodUpdate(void){
	static uint8_t avoidTurn = 0;
	//Ť����С����
	if(tankAutoData.avoidTask){
		switch(tankAutoData.avoidSchedule){
			case 0:
				// ������45�Ƚ�ӭ��
				if(getchassisData()->chaseRef < AVIOD_INITIAL_ANGEL)				
					getchassisData()->chaseRef += AVOID_RATE;
				else
					digitalIncreasing(&tankAutoData.avoidSchedule);
				break;
			case 1:
				if(avoidTurn){
					//120
					if(getchassisData()->chaseRef < (AVIOD_INITIAL_ANGEL+AVOID_RANGE))			
						getchassisData()->chaseRef += AVOID_RATE;
					else if(getchassisData()->chaseRef >= (AVIOD_INITIAL_ANGEL+AVOID_RANGE)){
						getchassisData()->chaseRef -= AVOID_RATE;
						digitalLo(&avoidTurn);
					}
				}
				else{
					//-30
					if(getchassisData()->chaseRef > (AVIOD_INITIAL_ANGEL-AVOID_RANGE))			
						getchassisData()->chaseRef -= AVOID_RATE;
					else if(getchassisData()->chaseRef <= (AVIOD_INITIAL_ANGEL-AVOID_RANGE)){
						getchassisData()->chaseRef += AVOID_RATE;
						digitalHi(&avoidTurn);
					}
				}
			 break;
		}
		if(tankAutoData.breakSign){							 
			getchassisData()->chaseRef = 0.0f;
			tankAutoData.aviodFlag = false;
			//��Ť���������ܱ�֤�´�Ť����45�Ƚ�ӭ��
			digitalClan(&tankAutoData.avoidSchedule);							 
			tankAutoData.avoidTask = DISABLE;
			digitalLo(&tankAutoData.breakSign);
		}	
	}
	else{	
	//��������ת���
	        static uint8_t lastKeySate,kmRotateFlag,rcRotateFlag,rotateDirection;
		       if(RC_ROTATE > 100)
				  rcRotateFlag = 1;
			   else
				rcRotateFlag = 0;
			   //һ��������ת
			   if(!lastKeySate && !kmRotateFlag){																	
				    if(PRESS_Q){
					    kmRotateFlag = 1;
					    //��һ����ת�ı�ת��
					    rotateDirection = !rotateDirection;
				    }
			   }
			   else{
				//�ٴΰ��½����ת
				    if(!lastKeySate && kmRotateFlag){
					     if(PRESS_Q || TASK_PRESS_F)
						kmRotateFlag = 0;
				     }
				      //�����Զ����С����
				    if(tankAutoData.closeRotate){
					    kmRotateFlag = 0;
					    tankAutoData.closeRotate = false;
				     }
			  }
			  lastKeySate = PRESS_Q;
			
			   if(kmRotateFlag || rcRotateFlag){
				   if(rotateDirection)
				       getchassisData()->chaseRef -= (AVOID_RATE * 4);
				   else
					   getchassisData()->chaseRef += (AVOID_RATE * 4);
				       tankAutoData.rotateFlag = true;
				       //������תʱ��
				       tankAutoData.rotateTime += RTIMEFEQ;
				        //������ת�ٶȱ�
				       getchassisData()->rotate = (f32_t)0.6*sinf(tankAutoData.rotateTime)+1.2f;
				       //�ٶȱ��޷�
				       getchassisData()->rotate = SINF_RUDD(getchassisData()->rotate);
			   }
	
			else{
				getchassisData()->chaseRef = 0.0f;
				tankAutoData.rotateFlag = false;
				getchassisData()->rotate = 1.0f;
				digitalClan(&tankAutoData.rotateTime);
			}
	}
}

///* ���ٵ�ͷ������� */
//void tankTurnAroundUpdate(void){
//	//����д�ϣ��жϣ�
//	if(tankAutoData.breakSign){													
//		//ֱ���������һ��
//		tankAutoData.schedule = 3;												
//		digitalLo(&tankAutoData.breakSign);
//	}
//	//��schedule�����ӡ����ٺ��ж��������
//	switch(tankAutoData.schedule){											
//		case 1:{
//			//������
//			autoTaskData->fastSeed = 1;									
//			//������ת180��
//			getGimbalData()->yawAngleRef -= 180.0f;						
//			digitalIncreasing(&tankAutoData.schedule);
//		}
//		break;
//		case 2:{
//			if(tankAutoData.taskDuration > 0.5f){
//				//3������ɵ�ͷ
//				if(tankAutoData.taskDuration < 3.0f){			
//					//��ͷ���
//					if(getGimbalData()->yawAngleFbd > getGimbalData()->yawAngleRef - 5.0f || getGimbalData()->yawAngleFbd < getGimbalData()->yawAngleRef + 5.0f){    
//						autoTaskData->fastSeed = 0;
//						tankAutoData.schedule = 99;
//					}
//				}
//				else
//					digitalIncreasing(&tankAutoData.schedule);
//			}
//		}		
//		break;
//		case 3:
//			//����ֹͣ��ת
//			getGimbalData()->yawAngleRef = getGimbalData()->yawAngleFbd;					
//			autoTaskData->fastSeed = 0;
//			tankAutoData.schedule = 99;
//			break;
//		//ֻ�е���99ʱ�����˳�
//		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						
//		//��������б���û�н��ȣ����������
//		default: tankAutoData.taskState = EXECUTION_ERROR; break;							
//	}
//	tankAutoData.taskDuration += AUTO_TASK_DURATION;
//}
/* ��ɱ����ģʽ���� */
void tankSuicideFireUpdate(void){
	if(PRESS_C){
		digitalHi(&getshootData()->suicideFireFlag);
	}
	else{
		digitalLo(&getshootData()->suicideFireFlag);
	}
}
void f_tankSuicideFireUpdate(void){
	if(PRESS_C){
		digitalHi(&getshootData()->suicideFireFlag);
	}
	else{
		digitalLo(&getshootData()->suicideFireFlag);
	}
}

void f_tankAutoTaskUpdate(void){
    		//������ڱ���״̬�����Ҹոո���ʱ
	if((judgeData.extGameRobotState.remain_HP!=0) && (ftankFricState.ftankFricNumFlag==1)){
		//����ʱִ�У��´θ���֮ǰ������ִ�и�if������
		ftankFricState.ftankFricNumFlag = 0;
		//��Ħ����
		ftankFricState.ftankFricStateFlag = 1;
	}
	//������ڱ���״̬����������ʱ
	else if((judgeData.extGameRobotState.remain_HP==0) && (ftankFricState.ftankFricNumFlag==0)){
		//���������ȴ�����ʱ��Ħ����
		ftankFricState.ftankFricNumFlag = 1;
		//�ر�Ħ����
		ftankFricState.ftankFricStateFlag = 0;	
	}
      //����Ħ����
  if(keyBoardCtrlData.rkSta == KEY_PRESS_ONCE){
		ftankChangeFricState();
	}
     super_capacitorTask();
	//������񣨺Ͳ������������ͬ��
	f_tankAviodUpdate();																	
	//��ɱ����ģʽ����
	f_tankSuicideFireUpdate();													                                   
	//�����п�ִ������  TANK_MANUAL���ֶ�)
	if(tankAutoData.currentTask != TANK_MANUAL){										
		//�������ոտ�ʼִ��
		if(tankAutoData.taskState == UNEXECUTED){										
			//ִ�п�ʼ����
			f_tankTaskBegin();																
		}
		//�����ִ����
		else if(tankAutoData.taskState == EXECUTING){		
			switch(tankAutoData.currentTask){
				//������׼
				case F_TANK_AUTOMATIC_AIM: 										
				   f_tankAutomaticAimUpdate();			      
				   break;
				//42mm��ҩ����
				case F_TANK_BULLET_TRANSFER:					   
				   f_tankBulletAroundUpdate();					
				   break;
				//��������������ֱ�����³�ʼ���ṹ��
				default: 																	
					//����������			
				   autoDataInit(&tankAutoData); 															
				   break;
		    }
		}
		//���ִ����ϻ�ִ�д���������
		else{																									
			autoDataInit(&tankAutoData);
		}
	}
}

