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
	//插入的情况下
	if(parameterRunData.TFInsertState == TFCARD_INSERT){								
		supervisorData.tfState = ENABLE;				
		//刚插入的情况下
		if(parameterRunData.TFInsertFlag){															
			//检测是否出错	
			parameterRunData.TFError = tfFATFS_Init();											
			parameterRunData.TFInsertFlag = DISABLE;
			//读出数据到parameter
			parameterReadDataFromTFCard(robotConfigData.typeOfRobot);					
		}
	}
	else{
		supervisorData.tfState = DISABLE;
		if(parameterRunData.TFInsertFlag)
			//出错检测归零
			parameterRunData.TFError = DISABLE;															
	}
	parameterRunData.TFInsertLastState = parameterRunData.TFInsertState;
}

//把flash中的PID参数写入TF卡中的参数文件
uint8_t parameterWriteDataFormFlash(uint8_t robotId){               
	uint8_t answer;
	if(robotId != NO_ID){
		answer = tfOverwrite(robotId,getConfigData);
		return answer;
	}
	else
		//写入失败
		return 1;																													
}

//从TF卡中读取PID参数
uint8_t parameterReadDataFromTFCard(uint8_t robotId){              
	uint8_t answer;
	if(robotId != NO_ID){
		answer = tfOverread(robotId,getConfigData);    
		return answer;
	}
	else
		//写入失败
		return 1;																													
}

