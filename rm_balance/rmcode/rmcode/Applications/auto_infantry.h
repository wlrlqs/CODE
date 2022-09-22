#ifndef __AUTO_INFANTRY_H
#define __AUTO_INFANTRY_H

#include "auto_task.h"
#include "keyboard.h"

#define AUTOMATIC_AIM_DEVICE	0x05
#define ACTIVE_BUFF_DEVICE	0x05
#define BULLET_TRANSFER_DEVICE	0x03
#define AVIOD_DEVICE	0x02
#define RTIMEFEQ 0.0128f
//使用小陀螺换向
#define ROTATE_ORIENT 0

//舵机旋转
enum{
	SERVO_HORIZONTAL = 0,
	SERVO_VERTICAL,
};

enum{
      INFANTRY_MANUAL = 0,						//手动
	INFANTRY_CHANGE_SHOOT_RATE = 1,					//X :改射速
      INFANTRY_BULLET_TRANSFER = 3,				//R:子弹交接
      INFANTRY_ACTIVE_BUFF = 4,					//Z:击打神符
      INFANTRY_AUTOMATIC_AIM = 5,					//V:辅助瞄准
	  INFANTRY_ACTIVE_B_BUFF = 7,					//G:大符
	  
};
enum{
	ARM_STOP = 0,
	ARM_FORWARD,
	ARM_BACK,
};
//PY: 摩擦轮状态
typedef struct{
	//PY: 步兵摩擦轮自动开启标志
	uint8_t infantryFricNumFlag;
	//PY: 步兵摩擦轮状态标志
	uint8_t infantryFricStateFlag;
}fricFlagStruct_t;

typedef struct{
	f32_t taskChassisFbd;
	f32_t taskChassisRef;
}changeChassisTaskStruct_t;

typedef struct{
	uint8_t change_finish;
	uint8_t change_chassis_step;
	uint8_t up_step_switch;
	uint8_t arm_direction;
}infantry_deformingStruct_t;
fricFlagStruct_t* getinfantryFricState(void);
AutoTaskStruct_t* getinfantryAutoData(void);
extern changeChassisTaskStruct_t changeChassisTaskData;
infantry_deformingStruct_t *get_infDeforming(void);
void infantryAutoTaskUpdate(void);
#endif

