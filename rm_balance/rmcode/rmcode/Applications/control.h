#ifndef __CONTROL_H
#define __CONTROL_H

#include "Util.h"
#include "pid.h"
#include "chassis.h"
#include "deforming.h"
#include "gimbal.h"
#include "shoot.h"
#include "cansend.h" 
#include "supercapacitor.h"
#define SHIELD_JUDGE

#define CONTROL_PRIORITY 5
#define CONTROL_PERIOD   2
#define CONTROL_STACK_SIZE 512
#define CONTROL_MIN_YAW_OVERRIDE	1000       		//在舵向摇杆摇动后保持最后期望1s


//#define CHASSIS_RES_OK	(1 << 0)
#define CONTROL_RES_OK	(1 << 0)

typedef void DeviceActivation_t(void);

typedef enum {
	//初始化模式
	MODE_INIT  = 0,
	//键鼠控制	
	MODE_KM,
	//解除控制权
	MODE_RELAX,	  		    
	//摇杆控制
	MODE_RC,	  			
	//丢控停止模式
	MODE_STOP,    		    
}robotModeStruct_t;

enum {
	//识别神符
	BUFF_MODE = 4,
	//识别装甲板
	AIM_MODE = 5,
};

enum {
	NOT_FINISH_STATE = 0,
	FINISH_STATE,
};

typedef struct {
	TaskHandle_t 			xHandleTask;
	uint8_t 				dataInitFlag;
	bool 					temperatureFaultState;
	uint32_t 				loops;
	uint8_t					chassisBoardControlMode;
	uint8_t					chassisBoardDataFinish;
    uint8_t					controlBoardDataFinish;
} controlStruct_t;

controlStruct_t* getcontrolData(void);
robotModeStruct_t getrobotMode(void);
robotModeStruct_t getlastrobotMode(void);
robotModeStruct_t getSuspendrobotMode(void);
void setRobotMode(robotModeStruct_t setMode);
void setSuspendRobotMode(robotModeStruct_t setMode);
extern robotModeStruct_t robotMode;			
extern robotModeStruct_t lastRobotMode;
extern f32_t controlGYRO[3];
extern f32_t controlRollTargetRateTest,controlPitchTargetRateTest,controlYawTargetRateTest;
void boardTypeConfirm(uint16_t boardFlag,DeviceActivation_t *boardType);
void controlDeviceConfirm(uint16_t deviceFlag,DeviceActivation_t *deviceFunction);
void controlUpdateTask(void *Parameters);
void controlInit(void);
void controlSetup(void);
void chassisSetup(void);
#endif


