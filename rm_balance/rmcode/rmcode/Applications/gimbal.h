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
						//码盘角度
	f32_t 				yawMotorAngle;        
	f32_t 				pitchMotorAngle;
	f32_t 				rollMotorAngle;
						//陀螺仪角度
	f32_t 				yawGyroAngle;         
	f32_t 				pitchGyroAngle;
	f32_t 				rollGyroAngle;
						//码盘停止角度
    f32_t 				yawAngleStop;         
	f32_t 				pitchAngleStop;
	f32_t 				rollAngleStop;
						//码盘停止角度设置
	f32_t 				yawAngleStopSet;      
	f32_t 				pitchAngleStopSet;
	f32_t 				rollAngleStopSet;
						//用于模式切换保存角度
	f32_t 				yawAngleSave;         
	f32_t 				pitchAngleSave;
	f32_t 				rollAngleSave;
						//速度期望
	f32_t 				yawSpeedRef;
	f32_t 				pitchSpeedRef;
	f32_t 				rollSpeedRef;
						//角度期望
	f32_t 				yawAngleRef;
	f32_t 				pitchAngleRef;
	f32_t 				rollAngleRef;
						//速度反馈
	f32_t 				yawSpeedFbd;
	f32_t 				pitchSpeedFbd;
	f32_t 				rollSpeedFbd;
						//角度反馈
	f32_t 				yawAngleFbd;
	f32_t 				pitchAngleFbd;
	f32_t 				rollAngleFbd;
						//速度PID输出
	f32_t 				yawSpeedOut;
	f32_t 				pitchSpeedOut;
	f32_t 				rollSpeedOut;
						//角度PID输出
	f32_t 				yawAngleOut;
	f32_t 				pitchAngleOut;
	f32_t 				rollAngleOut;
						//保存编码器值
	f32_t 				chassisChange;
						//小云台速度期望
	f32_t 				slaveGimbalPitchRef;
	
	uint8_t 			autoMode;
	uint8_t 			angleCycleStep;
	
	errorScanStruct_t 	gimbalError[2];
						//yaw轴PID结构体
	pidStruct_t 		*yawSpeedPID;
	pidStruct_t 		*yawAnglePID;
						//pitch轴PID结构体
	pidStruct_t 		*pitchSpeedPID;
	pidStruct_t 		*pitchAnglePID;
						//roll轴PID结构体
	pidStruct_t 		*rollSpeedPID;
	pidStruct_t 		*rollAnglePID;
						//回中完成标志
	volatile uint8_t 	initFinishFlag;
	uint8_t 			motorFlag;
	uint8_t         	low_sensor;//弹舱传感器
	bool 				followLock;
	f32_t 				intervalTime;
	double 				time[2];
	//差分
	u8					logical_difference;
	f32_t				yawEncoder;// yaw轴电机码盘值
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
