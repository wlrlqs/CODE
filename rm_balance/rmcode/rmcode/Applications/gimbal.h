#ifndef __GIMBAL_H
#define __GIMBAL_H
#include "BSP.h"
#include "Util.h"
#include "pid.h"
#include "clockcount.h"
#include "stdbool.h"

#define GIMBAL_TRACK 1
#define GIMBAL_POS 0

#define INSTALL_ENCODER	0
#define INSTALL_TURN	1

#define YAW_SPEED_SINGLE 0

typedef enum
{
  GIMBAL_RELAX = 0,
  GIMBAL_STOP,
  GIMBAL_INIT,
  GIMBAL_NORMAL,//F
  GIMBAL_TRACK_ARMOR,//B
  GIMBAL_SUPPLY_MODE,//R
  GIMBAL_SHOOT_BUFF,//V

} gimbalMode_e;

enum{
	NO_AIM = 0,
	TAKE_AIM,
};

typedef struct
{
	gimbalMode_e 		ctrlMode;
	gimbalMode_e 		lastCtrlMode;
						//���̽Ƕ�
	f32_t 				yawMotorAngle;        
	f32_t 				pitchMotorAngle;
	f32_t 				rollMotorAngle;
						//�����ǽǶ�
	f32_t 				yawGyroAngle;         
	f32_t 				pitchGyroAngle;
	f32_t 				rollGyroAngle;
						//����ֹͣ�Ƕ�
    f32_t 				yawAngleStop;         
	f32_t 				pitchAngleStop;
	f32_t 				rollAngleStop;
						//����ֹͣ�Ƕ�����
	f32_t 				yawAngleStopSet;      
	f32_t 				pitchAngleStopSet;
	f32_t 				rollAngleStopSet;
						//����ģʽ�л�����Ƕ�
	f32_t 				yawAngleSave;         
	f32_t 				pitchAngleSave;
	f32_t 				rollAngleSave;
						//�ٶ�����
	f32_t 				yawSpeedRef;
	f32_t 				pitchSpeedRef;
	f32_t 				rollSpeedRef;
						//�Ƕ�����
	f32_t 				yawAngleRef;
	f32_t 				pitchAngleRef;
	f32_t 				rollAngleRef;
						//�ٶȷ���
	f32_t 				yawSpeedFbd;
	f32_t 				pitchSpeedFbd;
	f32_t 				rollSpeedFbd;
						//�Ƕȷ���
	f32_t 				yawAngleFbd;
	f32_t 				pitchAngleFbd;
	f32_t 				rollAngleFbd;
						//�ٶ�PID���
	f32_t 				yawSpeedOut;
	f32_t 				pitchSpeedOut;
	f32_t 				rollSpeedOut;
						//�Ƕ�PID���
	f32_t 				yawAngleOut;
	f32_t 				pitchAngleOut;
	f32_t 				rollAngleOut;
						//���������ֵ
	f32_t 				chassisChange;
						//С��̨�ٶ�����
	f32_t 				slaveGimbalPitchRef;
	
	uint8_t 			autoMode;
	uint8_t 			angleCycleStep;
	
	errorScanStruct_t 	gimbalError[2];
						//yaw��PID�ṹ��
	pidStruct_t 		*yawSpeedPID;
	pidStruct_t 		*yawAnglePID;
						//pitch��PID�ṹ��
	pidStruct_t 		*pitchSpeedPID;
	pidStruct_t 		*pitchAnglePID;
						//roll��PID�ṹ��
	pidStruct_t 		*rollSpeedPID;
	pidStruct_t 		*rollAnglePID;
						//������ɱ�־
	volatile uint8_t 	initFinishFlag;
	uint8_t 			motorFlag;
	uint8_t         	low_sensor;//���մ�����
	bool 				followLock;
	f32_t 				intervalTime;
	double 				time[2];
	//���
	u8					logical_difference;
	f32_t				yawEncoder;// yaw��������ֵ
}gimbalStruct_t;

gimbalStruct_t* getGimbalData(void);
int8_t getInstallDirect(uint8_t installPara,bool type);
void gimbalUpdate(void);
void gimbalInit(void);
void gimbalRampInit(void);
void fullReverse(void);
static void gimbalInitHandle(void);
static void gimbalFollowHandle(void);
static void gimbalStopHandle(void);
static void gimbalRelaxHandle(void);
void gimbalStopSwitch(uint8_t active); 

#endif
