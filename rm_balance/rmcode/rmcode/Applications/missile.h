#ifndef __MISSILE_H
#define __MISSILE_H

enum {	//步进电机的运动方向
	STEP_YAW_STOP = 0,
	STEP_YAW_CW,
	STEP_YAW_CCW,
	STEP_PITCH_STOP,
	STEP_PITCH_UP,
	STEP_PITCH_DOWN,
};

enum {	//KM模式下的打击目标
	TARGET_NULL = 0,
	TARGET_BASE,
	TARGET_OUTPOST,
};

enum {	//当前控制状态
	RELX_CONTROL = 0,			//遥控器控制 下控状态
	RC_CONTROL_MOVE,			//遥控器控制 步进电机与直流电机
	RC_CONTROL_TRIGGER,			//遥控器控制 舵机（测试阶段使用）
	RC_CONTROL_AUTOLOAD,		//自动控制(自动上膛)
	KM_CONTROL,					//键鼠控制
    KM_CALI,                    //键鼠模式下校准基地和前哨站位置
};

enum {	//直流电机手动控制时发送的指令
	DC_RC_STOP = 0,
	DC_RC_FORWARD,
	DC_RC_BACK
};

enum {	//missileData的数据位
	CONTROL_STATE = 0,		//当前控制状态						举例：RC_CONTROL_MOVE
	STEP_RC_YAW_STATE,		//步进电机YAW在RC状态下的运动状态		举例：STEP_YAW_STOP
	STEP_RC_PITCH_STATE,	//步进电机PITCH在RC状态下的运动状态	举例：STEP_PITCH_STOP
	DC_RC_STATE,			//直流电机在RC状态下的运动状态			举例：DC_RC_STOP
	FIRE_STATE,				//对应要发射的导弹
	STEP_KM_TARGET,			//步进电机在KM状态下的打击目标			举例：TARGET_NULL
};

typedef struct{
	uint8_t missileSend[6];	// 0、当前控制状态 1、步进电机RC状态下的运动状态 2、直流电机在RC状态下的运动状态 3、对应要发射的导弹 4、步进电机在KM状态下的打击目标
} missileDataStruct_t;

void missileUpdate(void);
missileDataStruct_t* getMissileData(void);
void missileCanSend(void);
#endif
