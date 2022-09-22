#include "parameter.h"
#include "pid.h"
#include "config.h"

pidStruct_t *pidInit(systemConfigPID_t *PIDConfigData)
{
	pidStruct_t *pid;              //     pidָ�����
    
	pid = (pidStruct_t *)aqDataCalloc(1, sizeof(pidStruct_t));

	pid->pMax = &PIDConfigData->PID_PM;
	pid->iMax = &PIDConfigData->PID_IM;
	pid->dMax = &PIDConfigData->PID_DM;
	pid->oMax = &PIDConfigData->PID_OM;
	pid->pGain = &PIDConfigData->PID_P;
	pid->iGain = &PIDConfigData->PID_I;
	pid->dGain = &PIDConfigData->PID_D;
	pid->fGain = &PIDConfigData->PID_F;
	pid->pTrim = NULL;
	pid->iTrim = NULL;
	pid->dTrim = NULL;
	pid->fTrim = NULL;

	return pid;
}

f32_t pidUpdate(pidStruct_t *pid, f32_t setpoint, f32_t position,f32_t Dt){    //����Ϊ����ȫ΢��PID�����㷨��p��iͬ����PIDһ�£�ֻ����΢�ּ�����һ��һ�׹��Ի��ڣ���ͨ�˲���������ϵͳ���ܡ�
	f32_t error;
	f32_t p = *pid->pGain;       
	f32_t i = *pid->iGain;
    //�����ַ�������룬��ȡ��ַ�ڵ�ֵ��û����ȡ1	
	f32_t d = (pid->dGain) ? *pid->dGain : 0.0f;
	f32_t f = (pid->fGain) ? *pid->fGain : 1.0f;														
	//����Ѿ�������ϵͳ
	if(pid->differential)																			
        //��ַ��̼���								
		setpoint = differentialCal(pid->differential,setpoint);									

	error = setpoint - position;            //����ƫ��=Ԥ��-��ǰֵ
    //���������
                              
	pid->pTerm_1 = p * error;
	if (pid->pTerm_1 > *pid->pMax) {
		pid->pTerm_1 = *pid->pMax;
	}
	else if (pid->pTerm_1 < -*pid->pMax) {
		pid->pTerm_1 = -*pid->pMax;              //���-�ţ��Ǳ�ʾmin����ͬ��
	}																																					
    //���ʵ��ļ��޼������״̬          
	pid->iState += error;                //  ����ۼ�    
	pid->iTerm_1 = i * pid->iState * Dt;                  //��kʱ�̵����  iTerm_1 = ki * ��ei   ,��ei=iState * Dt     ����ͬ��
	if (pid->iTerm_1 > *pid->iMax) {       
		pid->iTerm_1 = *pid->iMax;
		pid->iState = pid->iTerm_1 / (i*Dt);
	}
	else if (pid->iTerm_1 < -*pid->iMax) {     
		pid->iTerm_1 = -*pid->iMax;
		pid->iState = pid->iTerm_1 / (i*Dt);
	}
    //΢��
	// derivative																															
    //�������΢����
	if (pid->dGain) {																													
		error = -position;       
        //�ڴ˴�ȥ��ʱ��ϵ��
		pid->dTerm_1 = (d * f) * (error - pid->dState);		   									//��ͨ�˲���Ʒ������һ��Ϊ0.707~1
		pid->dState += f * (error - pid->dState);
		if (pid->dTerm_1 > *pid->dMax) {
				pid->dTerm_1 = *pid->dMax;
		}
		else if (pid->dTerm_1 < -*pid->dMax) {
				pid->dTerm_1 = -*pid->dMax;
		}
	}
	else {                         
        //��������΢�����ȡ0
		pid->dTerm_1 = 0.0f;																										
	}
//
	pid->pv_1 = position;
	pid->sp_1 = setpoint;
	pid->co_1 = pid->pTerm_1 + pid->iTerm_1 + pid->dTerm_1;
	
    //��PID����޷�
	if (pid->co_1 > *pid->oMax) {																							
		pid->co_1 = *pid->oMax;
	}
	else if (pid->co_1 < -*pid->oMax) {
		pid->co_1 = -*pid->oMax;
	}
	return pid->co_1;                                        //�������� ���Ʋ���
}
     
void pidZeroIntegral(pidStruct_t *pid, f32_t pv, f32_t iState){      //0����
	if (*pid->iGain != 0.0f)
		pid->iState = iState / *pid->iGain;
	pid->dState = -pv;                 //ȥ��ʱ��ϵ��
	pid->sp_1 = pv;
	pid->co_1 = 0.0f;
	pid->pv_1 = pv;
	pid->pv_2 = pv;
}

void pidZeroState(pidStruct_t *pid){
	pid->setPoint = 0.0f;
	pid->dState = 0.0f;
	pid->iState = 0.0f;
	pid->co_1 = 0.0f;
	pid->pv_1 = 0.0f;
	pid->sp_1 = 0.0f;
	pid->pTerm_1 = 0.0f;
	pid->iTerm_1 = 0.0f;
	pid->dTerm_1 = 0.0f;
}


