#ifndef __SHOOT_H
#define __SHOOT_H
#include "BSP.h"
#include "Driver.h"
#include "Util.h"
#include "pid.h"
#include "Driver_RMMotor.h"
#include "control.h"
#include "type_can_isp.h"

#define FricMotor_L TIM9->CCR1
#define FricMotor_R TIM9->CCR2

#define BULLET_MONITOR_FLAG					shootData.bulletMonitorFlag

#define LASER_GPIO									BSP_GPIOE11
#define LASER_ON									PEout(11)=1
#define LASER_OFF									PEout(11)=0
#define SHOOTER_SWITCH_GPIO							BSP_GPIOB9
		
#define SHOOTER_SWITCH					GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)
#define TANK_SHOOTER_SWITCH	         	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)// pneumaticData.pneumatic_3.readData[1].byte.g_temp 

#define SHOOT_HEAT(shootSpeed)			(f32_t)shootSpeed

#define MAIN_POKE					commandoMotorConfig[POKEMOTOR]
#define SLAVE_POKE					commandoMotorConfig[SLAVE_POKEMOTOR]

#define SHOOT_STALL_CURRENT				9000                     //发射矢量电流
#define POKE_SPEED_RATA					0.6f           
#define POKE_STATIC_SPEED				7300          //拨弹盘转速
//开火信号 
#define FIRE_FLAG_17MM 			         	shootData.fireFlag_17mm	         //开火标志位
#define FIRE_FLAG_42MM						shootData.fireFlag_42mm

//连续发射情况下发射子弹数目
#define SHOOT_TIMES_CONTINUOUS			3
//小子弹伤害
#define SMALL_BULLET_HARM				10
//大子弹伤害
#define BIG_BULLET_HARM					100	
//小子弹默认射速
#define BULLET_SPEED_DEFAULT    		26.0f			    				
#define SLAVE_POKE_SET					shootData.slave_pokeSpeedRef	//LCM: 受控于shootTrigger_17mm

#define MIN_SPEED                    0
#define MAX_SPEED                    5000
#define SNAIL_MOTOR                  DISABLE

//17mm热量
#define	__17MM_HEAT			10.0f
//42mm热量
#define __42MM_HEAT			100.0f
//无热量
#define __NO_HEAT			0.0f
//英雄保护热量
#define protectHeartTank             100										

//17mm射击速度限制
#define SPEED_17MM_LOW     15
#define SPEED_17MM_MID     18
#define SPEED_17MM_HIGH    22
#define SPEED_17MM_EXTREME 30
//42mm射击速度限制
#define SPEED_42MM_LOW		10
#define SPEED_42MM_MID		12
#define SPEED_42MM_HIGH		14
#define SPEED_42MM_EXTREME	16

//弹仓盖状态
enum{
	supply_move = 0,
	supply_close = 1,
	supply_open = 2,
};	
//射击模式
typedef enum{
	MANUAL_SINGLE = 0,
	MANUAL_CONTINUOUS,
	AUTO_CONTINUOUS,
}shootMode_e;      						

//射击状态
typedef enum{
	SAFE = 0,
	DANGEROUS,
}shootStatus_e;								

//电机状态
typedef enum{
	MOTOR_REVERSAL = -1,
	MOTOR_STOP = 0,
	MOTOR_FOWARD = 1,
}motorStatus_e;								

enum{
	TYPE_17MM = 1,
	TYPE_42MM = 2,
};

typedef struct{
						//时间监测
	TickType_t 			xLastWakeTime;	  
						//17mm射击模式
	shootMode_e 		shootMode_17mm;
						//17mm射击状态
	shootStatus_e 		shootStatus_17mm;
						//42mm射击状态
	shootStatus_e		shootStatus_42mm;
						//17mm射击安全模式
	shootStatus_e 		shootStatusMode_17mm;
	TaskHandle_t 		xHandleTaskshoot;	
	pidStruct_t 		*speedPID;
	pidStruct_t 		*testSpeedPID;
	pidStruct_t 		*fricWheelSpeedPID[2];
	pidStruct_t 		*slave_pokeSpeedPID;
	errorScanStruct_t 	fricWheelError[2];
	errorScanStruct_t 	lidError;
	errorScanStruct_t 	pokeError;
						//清弹标志位
	bool  				clearBulletFlag;      
						//拨弹盘速度期望值
	f32_t 				pokeSpeedRef;
						//从拨弹盘速度期望值
	f32_t				slave_pokeSpeedRef;
						//拨弹盘速度设定
	f32_t				pokeSpeedSet;
						//拨弹盘PID输出值
	f32_t 				pokeSpeedOut;
						//从拨弹盘PID输出值;
	f32_t 				slave_pokeSpeedOut;
						//17mm安全热量
	f32_t				safe_heat_17mm;
						//42mm安全热量
	f32_t				safe_heat_42mm;
						//射速限制
	uint16_t			speed_limit;
						//拨弹盘速度期望值
	f32_t 				bigPokeSpeedRef;  		
						//拨弹盘PID输出值
	f32_t 				bigPokeSpeedOut; 	  	
	uint8_t 			autoMode;			  
						//上次子弹射速
	f32_t 				shootSpeedLast;				
						//42mm摩擦轮转速设定
	uint16_t 			fricSpeedSet_42mm; 
						//17mm摩擦轮转速设定
	uint16_t 			fricSpeedSet_17mm;	
						//英雄车子弹射速期望值
	f32_t 				fricWheelSpeedRef[2];	
						//英雄车子弹射速PID输出值
	f32_t				fricWheelSpeedOut[2]; 				  
						//剩余手动开火次数
	uint16_t 			shootManualNumber; 
						//射击命令flag
	uint8_t 			fireDataInit;				
						//17mm射击开启指令
	uint8_t 			shootTrigger_17mm;
						//42mm射击开启指令
	uint8_t				shootTrigger_42mm;
						//射击触发标志
	uint8_t				shootTriggerReserve;
						//射击预计伤害
	f32_t 				shootHPRef;   		  	
	f32_t 				shooterJudgeLastHeat_17mm;
	f32_t 				shooterJudgeHeat_17mm;
	f32_t				shooterJudgeHeat_42mm;
	f32_t				shooterJudgeLastHeat_42mm;
						//17mm枪口热量
	f32_t 				shooterHeat_17mm;     
						//42mm枪口热量
	f32_t				shooterHeat_42mm;
						//17mm枪口热量每0.1秒冷却值
	f32_t 				shooterHeatCoolingPs_17mm;
						//42mm枪口热量每0.1秒冷却值
	f32_t				shooterHeatCoolingPs_42mm;
						//17mm枪口剩余热量
	f32_t 				shooterHeatRemain_17mm;
						//42mm枪口剩余热量
	f32_t				shooterHeatRemain_42mm;
						//下次射击等待时间
	TickType_t 			shootWaitTime;
	uint8_t 			monitorFlag;
						//摩擦轮开启标志
	uint8_t 			fricMotorFlag;		  
						//42mm子弹开火标志
	uint8_t 			fireFlag_42mm;		  
						//17mm子弹开火标志
	uint8_t 			fireFlag_17mm;
		          //弹舱盖运动标志
	uint8_t       supplyPlaceFlag;
	            //PY: 弹舱盖开关标志
	uint8_t       supplyNumFlag;
						//弹舱盖期望
	f32_t 				supplyRef;
						//从弹舱盖期望
	f32_t				slave_supplyRef;
						//弹舱盖PID
    pidStruct_t 		*supplySpeedPID;
						//弹舱盖PID输出值
	f32_t 				supplySpeedOut;
						//从弹舱盖PID输出值
	f32_t				slave_supplySpeedOut;
						//弹道填充标志
	bool 				ballisticFill;					
						//自杀开火标志
	uint8_t 			suicideFireFlag;
	f32_t 				bigsupplySpeedOut;
    pidStruct_t 		*shootsupplyPID;
    f32_t 				supplyTime[2];
    f32_t 				supplyIntervalTime;
						//拨弹堵转位
	int8_t 				pokeStall;		
	
	int8_t 				slave_pokeStall;		
						//42mm枪口热量
	f32_t 				shooterHeatTank; 	
	uint8_t 			bulletMonitorFlag;
	f32_t 				intervalTime;
	double 				time[2];
	uint32_t 			loops;
	uint16_t			waitTime;
	double 				timeing,lasttimez,gsTime;
	f32_t 				speeding;
	//步兵修改摩擦轮转速
	int16_t       infantry_add_fire_speed_low; 
	int16_t       infantry_add_fire_speed_mid; 
	int16_t       infantry_add_fire_speed_high; 
	int16_t       infantry_add_fire_speed_extreme; 
	int16_t				L_speed;
	int16_t				R_speed;
	u8						shootmode_ui;
    
	
	//气动英雄打弹相关变量
	uint16_t 			loadStep;
	bool 				bulletExistStop;
	uint32_t 			turntableStalltime;
	bool				bulletBuzyFlag;
	bool				bulletfreeFlag;
	bool				bulletFlag;
    bool			    p_tankshootmode;
	uint8_t				laser_close;
}shootStruct_t;

extern u8 infantryFricStateFlag;
extern u8 ftankFricStateFlag;
extern shootStruct_t* getshootData(void);

void getShootMode(void);
void supplyUpdate(void);
void shootDataReset(void);
void shooterHeatIncreasing(void);
void shooterHeatCooling(void);
void shooterHeatAbjust(void);
void shootUpdate(void);
void shootInit(void);	
void smallPokeSpeedRefChange(void);
void shootProtectTank(void);

#endif
