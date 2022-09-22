#include "auto_uav.h"
#include "gimbal.h"
#include "shoot.h"
#include "vision.h"
#include "rc.h"
#include "config.h"
#include "imu.h"
AutoTaskStruct_t uavAutoData;

AutoTaskStruct_t* getuavAutoData(){
    return &uavAutoData;
}

void uavTaskBegin(void){
    //���Ϊ������
	uavAutoData.taskState = EXECUTING;											
    //��ִ�����м�һ
	digitalIncreasing(&uavAutoData.schedule);								
}

//�������/�����������
void uavAutomaticAimUpdate(void){													
	switch(uavAutoData.schedule){														
		case 1: gimbalSwitch(DISABLE);												
            chassisSwitch(DISABLE);
            //����TX2����
            visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);							
            digitalIncreasing(&(uavAutoData.schedule));	
            break;
		case 2: gimbalSwitch(ENABLE);	
            shootVisionInit();
            
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
								
            if(uavAutoData.breakSign){
                //����д�����������
                uavAutoData.schedule = 99;									
                getvisionData()->prejudgFlag = false;
                shootDataReset();	
            }			
            break;
            //��������б���û�н��ȣ����������
		case 99: uavAutoData.taskState = END_OF_EXECUTION; break;						
		default: uavAutoData.taskState = EXECUTION_ERROR; break;						
	}
	uavAutoData.taskDuration += AUTO_TASK_DURATION;
}

void uavAutoTaskUpdate(void){
    //�����п�ִ������
	if(uavAutoData.currentTask != UAV_MANUAL){							
        //�������ոտ�ʼִ��
		if(uavAutoData.taskState == UNEXECUTED){							
            //ִ�п�ʼ����
			uavTaskBegin();																			
		}
        //�����ִ����
		else if(uavAutoData.taskState == EXECUTING){					
			switch(uavAutoData.currentTask){
				case UAV_AUTOMATIC_AIM: {
					uavAutomaticAimUpdate();
					break;
				}
                //��������������ֱ�����³�ʼ���ṹ��
				default: {																				
					autoDataInit(&uavAutoData); 
					break;
				}
			}
		}
        //���ִ����ϻ�ִ�д���������
		else{																									
			autoDataInit(&uavAutoData);
		}
	}
}



