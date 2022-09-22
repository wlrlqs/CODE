#ifndef __PID_H
#define __PID_H

#include "util.h"
#include "differential_calculation.h"
#include "config.h"

typedef struct {
								// Last setpoint 											��������ֵ
	f32_t 						setPoint;		
								// Last position input										����״̬����
	f32_t 						dState;		
								// Integrator state   										����״̬
	f32_t 						iState;		
								// integral gain											��������     ki
	f32_t 						*iGain;		
								// proportional gain										��������    kp
	f32_t 						*pGain;		
								// derivative gain											΢������     kd
	f32_t 						*dGain;		
								// low pass filter factor (1 - pole) for derivative gain	����΢������ĵ�ͨ�˲�������
	f32_t 						*fGain;
	f32_t 						*pMax, *iMax, *dMax, *oMax;
								// pointers to radio trim channels (or NULL)  �趨ֵ����λ��ָ�����ߵ�΢��ͨ����ָ�루���ֵ��
	int16_t 					*pTrim, *iTrim, *dTrim, *fTrim;	
	f32_t 						pv_1, pv_2;
	f32_t 						co_1;
	f32_t 						pTerm_1;        //�������
	f32_t 						iTerm_1;        //�������
	f32_t 						dTerm_1;        //΢�����
	f32_t 						sp_1;
	
	differentialDataStruct_t 	*differential;       //���
} pidStruct_t;

typedef struct {
	f32_t dataFbd;
    f32_t dataRef;
    f32_t dataOut;
} pidData_t;

pidStruct_t *pidInit(systemConfigPID_t *PIDConfigData);   
f32_t pidUpdate(pidStruct_t *pid, f32_t setpoint, f32_t position,f32_t Dt);    //PID�ջ�����
void pidZeroIntegral(pidStruct_t *pid, f32_t pv, f32_t iState);
void pidZeroState(pidStruct_t *pid);

#endif


