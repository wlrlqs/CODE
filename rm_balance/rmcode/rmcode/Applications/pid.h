#ifndef __PID_H
#define __PID_H

#include "util.h"
#include "differential_calculation.h"
#include "config.h"

typedef struct {
								// Last setpoint 											最后的设置值
	f32_t 						setPoint;		
								// Last position input										最后的状态输入
	f32_t 						dState;		
								// Integrator state   										积分状态
	f32_t 						iState;		
								// integral gain											积分增益     ki
	f32_t 						*iGain;		
								// proportional gain										比例增益    kp
	f32_t 						*pGain;		
								// derivative gain											微分增益     kd
	f32_t 						*dGain;		
								// low pass filter factor (1 - pole) for derivative gain	用于微分增益的低通滤波器因子
	f32_t 						*fGain;
	f32_t 						*pMax, *iMax, *dMax, *oMax;
								// pointers to radio trim channels (or NULL)  设定值、定位器指向无线电微调通道的指针（或空值）
	int16_t 					*pTrim, *iTrim, *dTrim, *fTrim;	
	f32_t 						pv_1, pv_2;
	f32_t 						co_1;
	f32_t 						pTerm_1;        //比例输出
	f32_t 						iTerm_1;        //积分输出
	f32_t 						dTerm_1;        //微分输出
	f32_t 						sp_1;
	
	differentialDataStruct_t 	*differential;       //差分
} pidStruct_t;

typedef struct {
	f32_t dataFbd;
    f32_t dataRef;
    f32_t dataOut;
} pidData_t;

pidStruct_t *pidInit(systemConfigPID_t *PIDConfigData);   
f32_t pidUpdate(pidStruct_t *pid, f32_t setpoint, f32_t position,f32_t Dt);    //PID闭环控制
void pidZeroIntegral(pidStruct_t *pid, f32_t pv, f32_t iState);
void pidZeroState(pidStruct_t *pid);

#endif


