#include "bsp.h"
#include "supervisor.h"
#include "parameter.h"
#include "config.h"
#include "type_robot.h"
#include "tf_card_parameter.h"

parameterStruct_t parameterRunData;

void tFCardUpdate(void){
	parameterRunData.TFInsertState = TFCARD_INSERT_IO;
	if(parameterRunData.TFInsertLastState != parameterRunData.TFInsertState)
		parameterRunData.TFInsertFlag = ENABLE;
	//����������
	if(parameterRunData.TFInsertState == TFCARD_INSERT){								
		supervisorData.tfState = ENABLE;				
		//�ղ���������
		if(parameterRunData.TFInsertFlag){															
			//����Ƿ����	
			parameterRunData.TFError = tfFATFS_Init();											
			parameterRunData.TFInsertFlag = DISABLE;
			//�������ݵ�parameter
			parameterReadDataFromTFCard(robotConfigData.typeOfRobot);					
		}
	}
	else{
		supervisorData.tfState = DISABLE;
		if(parameterRunData.TFInsertFlag)
			//���������
			parameterRunData.TFError = DISABLE;															
	}
	parameterRunData.TFInsertLastState = parameterRunData.TFInsertState;
}

//��flash�е�PID����д��TF���еĲ����ļ�
uint8_t parameterWriteDataFormFlash(uint8_t robotId){               
	uint8_t answer;
	if(robotId != NO_ID){
		answer = tfOverwrite(robotId,getConfigData);
		return answer;
	}
	else
		//д��ʧ��
		return 1;																													
}

//��TF���ж�ȡPID����
uint8_t parameterReadDataFromTFCard(uint8_t robotId){              
	uint8_t answer;
	if(robotId != NO_ID){
		answer = tfOverread(robotId,getConfigData);    
		return answer;
	}
	else
		//д��ʧ��
		return 1;																													
}

