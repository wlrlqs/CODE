#ifndef __AUTO_TASK_H
#define __AUTO_TASK_H


#include "stdbool.h"
#include "bsp.h"
#include "Util.h"
#include "supercapacitor.h"

#define R_TIME 6.0f				//躲避时间设置为6s
#define AVOID_RANGE 45.0f
#define AVOID_RATE	0.40f
#define AUTO_TASK_DURATION	0.002f
#define AVIOD_INITIAL_ANGEL 45.0f

#define TASK_PRESS_C (remoteControlData.dt7Value.keyBoard.bit.C)
#define TASK_PRESS_E (remoteControlData.dt7Value.keyBoard.bit.E)
#define TASK_PRESS_F (remoteControlData.dt7Value.keyBoard.bit.F)
#define TASK_PRESS_G (remoteControlData.dt7Value.keyBoard.bit.G)
#define TASK_PRESS_Q (remoteControlData.dt7Value.keyBoard.bit.Q)
#define TASK_PRESS_R (remoteControlData.dt7Value.keyBoard.bit.R)
#define TASK_PRESS_V (remoteControlData.dt7Value.keyBoard.bit.V)
#define TASK_PRESS_X (remoteControlData.dt7Value.keyBoard.bit.X)
#define TASK_PRESS_Z (remoteControlData.dt7Value.keyBoard.bit.Z)
#define TASK_PRESS_CTRL (remoteControlData.dt7Value.keyBoard.bit.CTRL)
#define TASK_PRESS_SHIFT (remoteControlData.dt7Value.keyBoard.bit.SHIFT)


#define PRESS_B (remoteControlData.dt7Value.keyBoard.bit.B)
#define PRESS_C (remoteControlData.dt7Value.keyBoard.bit.C)
#define PRESS_E (remoteControlData.dt7Value.keyBoard.bit.E)
#define PRESS_G (remoteControlData.dt7Value.keyBoard.bit.G)
#define PRESS_Q (remoteControlData.dt7Value.keyBoard.bit.Q)
#define PRESS_R (remoteControlData.dt7Value.keyBoard.bit.R)
#define PRESS_X (remoteControlData.dt7Value.keyBoard.bit.X)
#define PRESS_Z (remoteControlData.dt7Value.keyBoard.bit.Z)
#define PRESS_V (remoteControlData.dt7Value.keyBoard.bit.V)
#define PRESS_SHIFT (remoteControlData.dt7Value.keyBoard.bit.SHIFT)
#define PRESS_CTRL (remoteControlData.dt7Value.keyBoard.bit.CTRL)

enum{
	X_TASK = 1,					    //1
	CTRL_TASK,					    //2
	R_TASK,							//3
	Z_TASK,							//4
	V_TASK,							//5
	Q_TASK,							//6
	G_TASK,							//7
	C_TASK,							//8
};

enum taskStateList{
	UNEXECUTED = 0,								//未执行
	EXECUTING,									//执行中
	END_OF_EXECUTION,							//执行完毕
	EXECUTION_ERROR								//执行错误	
};

enum autoDeviceList{						    //自动任务装置列表
	AUTO_NO_DEVICE = 0x00,
	AUTO_DEVICE_GIMBAL = 0x01,
	AUTO_DEVICE_CHASSIS = 0x02,
	AUTO_DEVICE_SHOOT = 0x04,
	AUTO_DEVICE_DEFORMING = 0x08
};

typedef struct{
	uint8_t gimbalAutoSate;
	uint8_t chassisAutoState;
	uint8_t shootAutoState;
	uint8_t deformingAutoState;
}AutoDeviceStruct_t;

typedef struct{
	bool vol_limit;
}forceSwitch_t;

typedef struct{
	AutoDeviceStruct_t autoDeviceData;					//任务激活装置
	uint8_t avoidSchedule;                              //躲避任务45度角迎敌
	bool    rotateFlag;			                        //小陀螺旋转标志
	bool    aviodFlag;                                  //45度扭腰标志
	bool    closeRotate;                                //关闭小陀螺
	bool	changeFlag;
	bool    frontClawLockFlag;                          //工程底盘小爪子锁定标志位
	bool    reversingLockFlag;
	uint8_t capSchedule;                                //电容任务进度
	forceSwitch_t  forceSwitch;                         //强制切换
	int8_t mineralnumber;                              //当前矿石数量
	uint8_t breakSign;						            //任务中断标记
	uint8_t visionMode;						            //视觉模式
	uint8_t currentTask;					            //当前任务
	uint8_t lastTask;							        //上一刻的任务
	uint8_t schedule;							        //当前进度
	uint8_t avoidTask;						            //躲避模式
	f32_t aviodDuration;					            //躲避持续时间
	f32_t taskDuration;						            //持续时间
	f32_t keyPressDuration;				                //按键持续时间
	uint8_t taskState;						            //任务状态
	uint8_t lastSchedule;					            //当前进度	
	uint8_t fastSeed;							        //置位满速
	uint16_t countTimes;					            //累计判断次数
	uint8_t grabSchedule;								//抓弹序列
	uint8_t exchangeSchedule;                           //兑换序列
	uint8_t frontClawResetSchedule;                     //底盘爪子复位序列
	uint32_t grabCountTimes;							//抓弹计时
	uint32_t exchangeCountTimes;						//兑换计时	
	uint32_t frontClawResetCountTimes;					//爪子复位计时		
	bool auxiliaryResourseFlag;                         
	uint8_t waitTime;                  //
	bool keyRelease;
	f32_t rotateTime;               //小陀螺旋转时间
	bool rotateEnb;
	uint8_t enbStep;	
	bool enbStart;
	bool keyboard_X_Flag;
	f32_t *rcYaw;
	uint16_t loadtime;
	bool get_pill_Flag;
	bool separate_flag;               					//普步底盘分离云台标志位
	u8 chassis_fast;
	uint8_t	roDir;									//方向选择
	u8	km_rotate;									//键鼠小陀螺标志位
	u8	death_live;									//死亡标志位
	u8	avoid_lastKey;					
}AutoTaskStruct_t;


extern AutoTaskStruct_t *autoTaskData;
void gimbalSwitch(uint8_t active);
void chassisSwitch(uint8_t active);
void shootSwitch(uint8_t active);
void deformingSwitch(uint8_t active);
void autoDataInit(AutoTaskStruct_t *autoContrlData);
void autoTaskUpdate(void);
void autoTaskInit(void);

#endif
