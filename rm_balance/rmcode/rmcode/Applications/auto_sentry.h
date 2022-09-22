#ifndef __AUTO_SENTRY_H
#define __AUTO_SENTRY_H

#include "auto_task.h"

#define SPEEDHIGH 16000 
#define SPEEDLOW 8000
#define BASESPEED 12000
#define SENTRY_FRONT_ARMOR 0
#define SENTRY_BACK_ARMOR 1

#define X_RANGE 150
#define Y_RANGE 120
#define YAWADD 0.05f
#define PITCHADD 0.10f
#define TURNRATE 4.0f
#define YAW_MOTOR_ENCODER commandoMotorConfig[YAWMOTOR].motor_staus->motorEncoder  //��ǰ����ֵ
#define YAW_CENTER_ENCODER getConfigData()->yawCenter						//��ֵ����ֵ
#define YAW_MOTOR_INSTALL getInstallDirect(YAW_INSTALL_CONFIG, INSTALL_TURN)

//����̨ɨ��ʱ��
#define YAW_STAY_TIME 7
#define PITCH_STAY_TIME 15

//����̨ɨ�跶Χ
#define PITCH_PATROL_MINANGLE -40.0f
#define PITCH_PATROL_MAXANGLE -10.0f
#define YAW_PATRO_ANGLE 120.0f		//��װ�װ�Ϊ����
#define YAW_PATROL_MINANGLE 0 - YAW_PATRO_ANGLE/2
#define YAW_PATROL_MAXANGLE 0 + YAW_PATRO_ANGLE/2

//����̨ɨ��ʱ��
#define SMYAW_STAY_TIME 6
#define SMPITCH_STAY_TIME 15

//����̨ɨ�跶Χ
#define SMPITCH_PATROL_MINANGLE -32.0f
#define SMPITCH_PATROL_MAXANGLE -0.0f
#define SMALLGIMBALYAW_PATRO_ANGLE 200.0f	//��װ�װ�Ϊ����
#define SMYAW_PATROL_MINANGLE 0 - SMALLGIMBALYAW_PATRO_ANGLE/2
#define SMYAW_PATROL_MAXANGLE 0 + SMALLGIMBALYAW_PATRO_ANGLE/2

#define TARGETSIDE LEFTSIDE
#define LEFTSIDE 1
#define RIGHTSIDE -1

#define ANTIMISSILE_YAW_ANGLE -20.0f
#define ANTIMISSILE_PITCH_ANGLE 10.0f

typedef enum{
	SENTRY_MANUAL = 0,					    //�ֶ�
	SENTRY_PATROL,							//Ѳ��ģʽ
	SENTRY_ATTACK,							//����ģʽ
	SENTRY_UNTER_ATTACK,					//���ģʽ
	SENTRY_ANTIMISSILE
}sentryMode;

typedef enum{
	GIMBAL_MANUAL = 0,					    //�ֶ�
	GIMBAL_PATROL,							//Ѳ��ģʽ
	GIMBAL_ATTACK,							//����ģʽ
	GIMBAL_UNTER_ATTACK,					//���ģʽ
	GIMBAL_ANTIMISSILE
}gimbalMode;

typedef enum{
	CHASSIS_NORMAL = 0,
	CHASSIS_UNTER_ATTACK,
	CHASSIS_ANTIMISSILE	
}chassisMode;

enum{
	PATROL_MODE = 0,
	FOLLOW_MODE
};

typedef enum{
	GIMBAL_ABOVE = 0,
	GIMBAL_BELOW
}sentryGimbalType;

typedef struct{
	bool init;
	bool speedFloag;
	bool sentryInit;
	bool flagAddEncoder;
	bool flagReduceEncoder;
	sentryGimbalType gimbalType;
	u8 attackMode;
	u8 visionSchedule;
	u8 randomStep;
	u8 sentryLeft;
	u8 sentryRight;
	u8 chassisTask;
	u8 gimbalTask;
	
	u16 frontCenterAngle;
	u16 backCenterAngle;
	u16 randomRTTime;
	u16 randomStartTime;
	u16 randomValue;
	float angleSave;
	float yawRTMotorAngle;
	u32 lostTargetCNT;
	u32 yawCNT;
	u32 pitchCNT;

}stryAutoTaskStruct_t;

extern AutoTaskStruct_t sentryAutoData;
extern stryAutoTaskStruct_t sentryAtClData;

AutoTaskStruct_t* getsentryAutoData(void);

void sentryAutoTaskUpdate(void);
void sentryChassisDataUpdate(void);
#endif
