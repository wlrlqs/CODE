#ifndef __SUPERVISOR_H
#define __SUPERVISOR_H

#include "stm32f4xx.h"
#include "NavAndData.h"
#include "stdbool.h"	

#define SPUER_PRIORITY          10
#define SPUER_STACK_SIZE        512
#define SUPER_STACK_PERIOD      100

#define SPUER_CHECK_FREQ	    10
#define WAITLOOPS               20

#define CALI_RC_VALUE	        490
#define LOST_TIME				100
enum {
	CALI_ONLY_GYO = 0x01,
	CALI_GYO_ACC  = 0x03
};

enum SupervisorConfig{
	IMU = 0,
	MAG,
	FLASH_OPERATION,
	YAW_CALI,
	DEFINE_ROBOT,
	DEFINE_ID,
	GIMBAL_CALI,
	CONFIG_LIST
};

enum {
	TASK_REGULAR = 1,
	TASK_FAULT,
	TASK_SUSPEND
};
       
enum {
	VISION_TASK = 0,   //视觉
	CONTROL_TASK,      //控制
	WIRELESS_TASK,
	SUPERVISOR_TASK,
	CHASSIS_RECODER_TASK,
	CHASSIS_START_TASK,
	TRANSFER_REC_TASK,
	LIST_OF_TASK
};

typedef struct {
	TaskHandle_t 	xHandleTask;
	f32_t 			soc;
	f32_t 			flightTime;		   			 		// seconds
	f32_t 			flightSecondsAvg;	    		        // avg flight time seconds for every percentage of SOC
	f32_t 			flightTimeRemaining;	                // seconds
	uint32_t 		armTime;
	uint32_t 		lastGoodRadioMicros;
	f32_t 			vInLPF;
	bool 			busyState;
	uint16_t 		state;
	uint8_t 		diskWait;
	uint8_t 		configRead;
	uint8_t 		beepState;
	uint8_t 		ledState;
	uint8_t 		ArmSwitch;
	uint8_t 		flashSave;
	uint8_t 		tfState;
	f32_t 			tempStd;
	uint8_t 		imuCali;
	uint8_t 		gimbalCaliReset;
	BaseType_t 		taskEvent[LIST_OF_TASK];      //任务时间
	uint8_t 		taskState[LIST_OF_TASK];       // 任务状态
	uint32_t 		loops;
} supervisorStruct_t;

enum supervisorStates {
	//初始化状态
	STATE_INITIALIZING	= 0x0000,														
	//磁力计校准状态
	STATE_MAGCALI		= 0x0001,																	
	//上锁
	STATE_DISARMED		= 0x0002,																
	//解锁
	STATE_ARMED			= 0x0004,																	
	//IMU校准
	STATE_IMUCALI		= 0x0008,																	
	//丢失
	STATE_RADIO_LOSS	= 0x0010,															
	//传感器错误
	STATE_SENSOR_IMU_ERROR	= 0x0020,														
	//电池
	STATE_LOW_BATTERY	= 0x0040,															
	//裁判系统错误
	STATE_JUDGE_ERROR	= 0x0080,															
	//视觉端错误
	STATE_VISION_ERROR 	= 0x0100,														
	//功率板错误
	STATE_CURRENT_ERROR = 0x0200,														
	//电机错误
	STATE_MOTOR_ERROR	= 0x0400,			
	//通信错误
	STATE_TRANS_ERROR	= 0x0800,
	//超级电容接受信息错误
	STATE_CAPACITANCE_ERROR = 0x1000,
	
};

extern uint8_t n;

void supervisorTaskCheck(void);
void supervisorStateSwitch(uint16_t state,uint8_t valve); 
void supervisorImuCali(uint8_t accTare);
void supervisorGimbalCali(void);
void supervisorInit(void);

void  IWDG_Init(void); //TQH
void  IWDG_Feed(void);	//TQH

extern supervisorStruct_t supervisorData __attribute__((section(".ccm")));
	
#endif


