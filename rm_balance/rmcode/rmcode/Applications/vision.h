#ifndef __VISION_H
#define __VISION_H

#include "bsp.h"
#include "Driver_Slave_Sensor.h"
#include "shoot.h"
#include "util.h"
#include "stdbool.h"
#include "auto_infantry.h"
#include "auto_tank.h"

#define VISION_PRIORITY	  	                6
#define VISION_STACK_SIZE	                1024
#define VISION_PERIOD                       4

#define ROBOT_ID  judgeData.extGameRobotState.robot_id

#define MM_TO_M_CONVERSION(p)               (f32_t)(*p) / 1000.0f
#define M_TO_MM_CONVERSION(p)	            (f32_t)(*p) * 1000.0f

#define ANGLE_TO_RADIAN(p)                  *p * PI / 180.0f
#define RADIAN_TO_ANGLE(p)                  *p * 180.0f / PI

#define SMALL_MANUAL_SINGLE		            26
#define SMALL_MANUAL_CONTINUOUS             20
#define SMALL_AUTO_CONTINUOUS 	            16

#define MANUAL_PREJUDG_SCALE                0.1f

#define VISION_STORED_LENGTH                50	

typedef enum{
INFANTRY_BARREL = 0,//17mm小枪管
	P_TANK_BARREL,
	F_TANK_BARREL,
	OLD_SENTRY_BARREL,
	OLD_SMG_BARREL,
	NEW_SENTRY_BARREL,
	NEW_SMG_BARREL,
	AUXILIARY_BARREL,
	UAV_BARREL,
}carType_e;

typedef enum{
	TX2_STOP = 0,							                    //停止工作		
	TX2_DISTINGUISH_ARMOR = 1,		                                //识别装甲板
	TX2_DISTINGUISH_BUFF = 6, 			                            //识别小符
	TX2_DISTINGUISH_BIGBUFF = 3,									//识别大符
	TX2_KILL_SENTRY = 4,											//杀哨兵
}visionWorkMode_e;

typedef enum{
	SMALL_BULLET = 0,
	BIG_BULLET
} bulletType_e;

typedef enum{
	ENEMY_RED = 0,
	ENEMY_BLUE
} enemyType_e;

typedef enum {
	NO_STATE = 0,
	W_STATE = 1,
	S_STATE = 2,
	A_STATE = 4,
	D_STATE = 8,
}buffKey_e;

typedef struct {
	TaskHandle_t xHandleTask;	
	/*		TX2回传数据		*/	
    formatTrans32Struct_t yawData;
    formatTrans32Struct_t pitchData;
	uint8_t distingushState;
	bool captureFlag;											//已识别到目标
	bool sameTargetFlag;										//同一目标指令
	bool getOrderFlag;											//TX2处于工作状态标志
	formatTrans16Struct_t CNTR; 								//LCM: TX2回传的惯导UKF计数位，可作为通讯状态检查 visionErrorCount ，可用于角度值匹配
																//LCM: 其实就是32发给TX2的 serialBuffer->CNTR
	
	/*		CNTR 相关变量	*/
	uint16_t lastCNTR;
	uint16_t CNTR_DataStored[VISION_STORED_LENGTH];		        //惯导UKF计数位 CNTR 历史储存值
//	f32_t pitchDataStored[VISION_STORED_LENGTH];				//pitch轴角度历史存储值 
//	f32_t yawDataStored[VISION_STORED_LENGTH];
	uint8_t storedIndex;										//LCM: 上面三个值的下标计数值，最大容量为 VISION_STORED_LENGTH
	uint32_t visionErrorCount;							        //LCM: 与TX2通讯状态检查，直接使用CNTR
	uint32_t visionLastErrorCount;	
	uint32_t intervalNum;										//LCM: visionErrorCount 与 visionLastErrorCount 的差值，若为0则说明与TX2通讯出现问题		
	
	/*		致TX2	 */
	shootMode_e fireMode;								        //开火模式，从主控接收，发送到tx2
	visionWorkMode_e distinguishMode;					                //视觉工作模式，从主控接收，发送到tx2 
	bulletType_e bullet;								        //子弹类型，从主控接收，发送到tx2
	uint8_t enemyType;									        //敌方颜色， 从主控接收，发送到tx2
	carType_e carType;									     	//LCM: 发送给TX2的车辆类型    
	
	/*		shoot	*/
	uint8_t shootSpeed;											//LCM: 获取当前射速（本质是参数，需要测量）

	float judgeShootSpeed;	              //PY: 获取裁判系统返回的实际射速

	/*		云台角度真值		*/
	f32_t yawReal;												//LCM: 陀螺仪传来的角度数据，随视觉任务 visionUpdateTask 更新
	f32_t pitchReal;
	
	/*		视觉控云台变量		*/
	f32_t yawCmd;												//LCM: 最终输出的目标角度（相对于回中值的相对角度）
	f32_t pitchCmd;										        
	f32_t yawCmdBuff;									       	//LCM：该变量进行各种处理后赋给 yawCmd 	
	f32_t pitchCmdBuff;
	f32_t manualYawBias;										//LCM: 在自瞄状态中，操作手依然拥有一定的云台控制权，该变量代表削弱后的操作手控制指令
	f32_t manualPitchBias;										
    
	/*标志位*/
	bool initFlag;												//LCM: 视觉初始化完成（置true后并没有被用到）
	bool prejudgFlag;											//LCM: 检测开启自瞄模式时是否使用鼠标右键，若使用则在自瞄状态中，操作手依然拥有一定的云台控制权（见 manualYawBias ）
	bool cailSuccess;											//LCM: 在 visionUpdateTask 刷新中检测是否有新一帧数据包。若有则置true，无则置false。只有为true时cmd才可以成为云台期望


	/*	迫击炮模式下的变量	*/	
	f32_t mortarPitCmd;
	f32_t mortarYawCmd;
	f32_t mortarPitCmdBuff;
	f32_t mortarYawCmdBuff;

	/*		杂物		*/
	bool miniPTZEnableAim;	
	uint32_t loops;	
	uint8_t buff_mod;			//LCM: 打符模式下 W S A D 微调标志位 见 buffKey_e 枚举类型
	
} visionStruct_t;

visionStruct_t* getvisionData(void);
void visionStoreHistoricalData(f32_t pitch, f32_t yaw, uint16_t CNTR);
void visionSendDataUpdate(uint8_t workMode,uint8_t bullet);
void visionSendDataInit(void);
void identifyCamp(void);
void visionInit(void);
void shootVisionInit(void);
void visionFireFbdUpdata(uint8_t * shootFlag);
void visionMortar(void);

#endif


