#include "parameter.h"
#include "pid.h"
#include "config.h"

pidStruct_t *pidInit(systemConfigPID_t *PIDConfigData)
{
	pidStruct_t *pid;              //     pid指针变量
    
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

f32_t pidUpdate(pidStruct_t *pid, f32_t setpoint, f32_t position,f32_t Dt){    //以下为不完全微分PID控制算法：p和i同数字PID一致，只是在微分加入了一个一阶惯性环节（低通滤波器）改善系统性能。
	f32_t error;
	f32_t p = *pid->pGain;       
	f32_t i = *pid->iGain;
    //如果地址存在输入，则取地址内的值，没有则取1	
	f32_t d = (pid->dGain) ? *pid->dGain : 0.0f;
	f32_t f = (pid->fGain) ? *pid->fGain : 1.0f;														
	//如果已经加入差分系统
	if(pid->differential)																			
        //差分方程计算								
		setpoint = differentialCal(pid->differential,setpoint);									

	error = setpoint - position;            //输入偏差=预期-当前值
    //计算比例项
                              
	pid->pTerm_1 = p * error;
	if (pid->pTerm_1 > *pid->pMax) {
		pid->pTerm_1 = *pid->pMax;
	}
	else if (pid->pTerm_1 < -*pid->pMax) {
		pid->pTerm_1 = -*pid->pMax;              //添加-号，是表示min（下同）
	}																																					
    //用适当的极限计算积分状态          
	pid->iState += error;                //  误差累加    
	pid->iTerm_1 = i * pid->iState * Dt;                  //第k时刻的输出  iTerm_1 = ki * Σei   ,Σei=iState * Dt     （下同）
	if (pid->iTerm_1 > *pid->iMax) {       
		pid->iTerm_1 = *pid->iMax;
		pid->iState = pid->iTerm_1 / (i*Dt);
	}
	else if (pid->iTerm_1 < -*pid->iMax) {     
		pid->iTerm_1 = -*pid->iMax;
		pid->iState = pid->iTerm_1 / (i*Dt);
	}
    //微分
	// derivative																															
    //如果存在微分项
	if (pid->dGain) {																													
		error = -position;       
        //在此处去除时间系数
		pid->dTerm_1 = (d * f) * (error - pid->dState);		   									//低通滤波的品质因数一般为0.707~1
		pid->dState += f * (error - pid->dState);
		if (pid->dTerm_1 > *pid->dMax) {
				pid->dTerm_1 = *pid->dMax;
		}
		else if (pid->dTerm_1 < -*pid->dMax) {
				pid->dTerm_1 = -*pid->dMax;
		}
	}
	else {                         
        //不存在则微分输出取0
		pid->dTerm_1 = 0.0f;																										
	}
//
	pid->pv_1 = position;
	pid->sp_1 = setpoint;
	pid->co_1 = pid->pTerm_1 + pid->iTerm_1 + pid->dTerm_1;
	
    //给PID输出限幅
	if (pid->co_1 > *pid->oMax) {																							
		pid->co_1 = *pid->oMax;
	}
	else if (pid->co_1 < -*pid->oMax) {
		pid->co_1 = -*pid->oMax;
	}
	return pid->co_1;                                        //返回三者 控制参数
}
     
void pidZeroIntegral(pidStruct_t *pid, f32_t pv, f32_t iState){      //0积分
	if (*pid->iGain != 0.0f)
		pid->iState = iState / *pid->iGain;
	pid->dState = -pv;                 //去除时间系数
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


