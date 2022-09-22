#ifndef __TYPE_ROBOT_H
#define __TYPE_ROBOT_H

#include "NavAndData.h"
#include "bsp.h"

#define TYPE_ROBOT_PRIORITY	                    12
#define TYPE_ROBOT_STACK_SIZE                   512
#define TYPE_ROBOT_PERIOD	                    10

#define BOARD_CHASSIS_LIST						0x0001
#define BOARD_CONTROL_LIST						0x0002

#define UAV_SBUS DISABLLE						//使能为正常SBUS,失能为RM版
#define INFANTRY_DEVICE_LIST	                0x17EF						
#define P_TANK_DEVICE_LIST	                    0x00FF
#define F_TANK_DEVICE_LIST                      0X17EF
#define AUXILIARY_DEVICE_LIST	                0x041F // 0x041F
#define AUXILIARY_SLAVE_DEVICE_LIST             0x0011
#define SENTRY_DEVICE_LIST	                    0x04FF
#define UAV_DEVICE_LIST	                        0x004B
#define SMALLGIMBAL_LIST                        0x041B
#define MISSILE_DEVICE_LIST                     0x0800  //LCM: 飞镖发射架设备名单

#define CHASSIS_NO_LIMIT	                    0.0f

#define INFANTR_CHASSIS_CURRENT					 64000.0f           //步兵底盘电流
#define INFANTR_POWER_LIMIT                     100.0f    //21年 步兵功率上限    W
#define INFANTR_WARNING_POWER                   50.0f     // 21年 步兵功率下限  
#define INFANTR_WARNING_POWER_BUFF              50.0f      //设定的缓冲能量警告值   J    
#define INFANTR_NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //无裁判系统时总电流限制   (C620规定)    rmp
#define INFANTR_JUDGE_TOTAL_CURRENT_LIMIT       34000.0f     //
#define INFANTR_ADD_POWER_CURRENT  				18000.0f       

#define TANK_CHASSIS_CURRENT					64000.0f
#define TANK_POWER_LIMIT                        120.0f
#define TANK_WARNING_POWER                      60.0f
#define TANK_WARNING_POWER_BUFF                 50.0f
#define TANK_NO_JUDGE_TOTAL_CURRENT_LIMIT       64000.0f
#define TANK_JUDGE_TOTAL_CURRENT_LIMIT          34000.0f
#define TANK_ADD_POWER_CURRENT  			    18000.0f 

#define SENTRY_CHASSIS_CURRENT					16000.0f
#define SENTRY_POWER_LIMIT                      30.0f
#define SENTRY_WARNING_POWER                    15.0f
#define SENTRY_WARNING_POWER_BUFF               190.0f
#define SENTRY_NO_JUDGE_TOTAL_CURRENT_LIMIT     16000.0f
#define SENTRY_JUDGE_TOTAL_CURRENT_LIMIT        16000.0f
#define SENTRY_ADD_POWER_CURRENT  			    2000.0f 

#define INFANTRY_PITCH_MIN_RANGE                -20.0f				    //步兵的最小俯仰角
#define INFANTRY_PITCH_MAX_RANGE                40.0f				    //步兵的最大俯仰角
#define TANK_PITCH_MIN_RANGE	                -25.0f					//英雄车的最小俯仰角
#define TANK_PITCH_MAX_RANGE	                40.0f					//英雄车的最大俯仰角
#define F_TANK_PITCH_MIN_RANGE	                -15.0f					//老英雄车的最小俯仰角
#define F_TANK_PITCH_MAX_RANGE	                30.0f					//老英雄车的最大俯仰角
#define AUXILIARY_PITCH_MIN_RANGE	            -30.0f			        //工程车的最小俯仰角
#define AUXILIARY_PITCH_MAX_RANGE               30.0f				    //工程车的最大俯仰角
#define SENTRY_PITCH_MIN_RANGE	                -45.0f				    //哨兵的最小俯仰角
#define SENTRY_PITCH_MAX_RANGE	                0.0f					//哨兵的最大俯仰角
#define UAV_PITCH_MIN_RANGE	                    -45.0f				    //无人机的最小俯仰角
#define UAV_PITCH_MAX_RANGE	                    10.0f					//无人机的最大俯仰角	
#define SMALLGIMBAL_PITCH_MIN_RANGE             -45.0f                  //小云台的最小俯仰角
#define SMALLGIMBAL_PITCH_MAX_RANGE             45.0f                   //小云台的最小俯仰角

#define CW_ENCODER 0x02     //正向 
#define CCW_ENCODER 0X00    //反向
#define CW_TURN	0x01        //顺时针
#define CCW_TURN 0x00       //逆时针         
                                            //CW顺时针   CCW逆时针
#define INFANTRY_PITCH_INSTALL		        CCW_ENCODER + CW_TURN
#define INFANTRY_YAW_INSTALL			    CCW_ENCODER + CW_TURN
#define TANK_PITCH_INSTALL				    CCW_ENCODER + CCW_TURN
#define TANK_YAW_INSTALL					CCW_ENCODER + CW_TURN
#define TANK_SLAVE_PITCH_INSTALL            CCW_ENCODER + CW_TURN
#define TANK_SLAVE_YAW_INSTALL		        CCW_ENCODER + CW_TURN
#define AUXILIARY_PITCH_INSTALL		        CW_ENCODER  + CW_TURN
#define AUXILIARY_YAW_INSTALL			    CW_ENCODER  + CW_TURN
#define SENTRY_PITCH_INSTALL			    CCW_ENCODER + CW_TURN
#define SENTRY_YAW_INSTALL				    CCW_ENCODER + CW_TURN
#define UAV_PITCH_INSTALL					CCW_ENCODER + CW_TURN
#define UAV_YAW_INSTALL						CCW_ENCODER + CW_TURN
#define SMALLGIMBAL_PITCH_INSTALL           CCW_ENCODER + CW_TURN
#define SMALLGIMBAL_YAW_INSTALL             CCW_ENCODER + CW_TURN

#define GPIO_SWITCH_1						BSP_GPIOE15
#define	GPIO_SWITCH_2						BSP_GPIOE14
#define	GPIO_SWITCH_3						BSP_GPIOE13
#define	GPIO_SWITCH_4						BSP_GPIOE12
#define SWITCH_1                            !GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15)
#define SWITCH_2                            !GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_14)
#define SWITCH_3                            !GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13)
#define SWITCH_4                            !GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12)

#define BOARD_TYPE							robotConfigData.boardType
enum{
	CAMP_BLUE = 0,
	CAMP_RED,
};

enum{
	NO_ID = 0,									//没有ID不会工作
	INFANTRY_ID,								//步兵
	P_TANK_ID,									//新英雄车
	AUXILIARY_ID,								//工程车
	SENTRY_ID,									//哨兵
	UAV_ID,										  //无人机
	SMALLGIMBAL_ID,             //小云台
	F_TANK_ID,									//老英雄车
    MISSILE_ID,                 //LCM: 飞镖发射架
    COUNT_ID,                   //LCM: ID总数    
};

enum{
	NO_WEAPONS = 0,
	SMALL_LAUNCHER,
	BIG_LAUNCHER,
	DOUBLE_LAUNCHER
};
enum boardTypeList{
	BOARD_CHASSIS			= 0x0001,      //底盘板
	BOARD_CONTROL			= 0x0002,     //云台板
  BOARD_OTHER             = 0X0004,   //LCM: 其他板
}; 
enum robotDeviceList{
	DEVICE_CANSEND 			     = 0x0001,			    //CAN通信
	DEVICE_GIMBAL 			     = 0x0002,				  //置高则代表激活云台
	DEVICE_SHOOT 			       = 0x0008,				  //置高则代表激活发射机构
	DEVICE_DEFORMING 		     = 0x0010,         	//置高则代表激活车身变形系统
	DEVICE_CURRENT 			     = 0x0020,			   	//置高则代表激活功率板
	DEVICE_VISION 			     = 0x0040,				  //置高则代表激活妙算
	DEVICE_WIFI 			       = 0x0080,					//置高则代表激活WIFI无线网络
	DEVICE_VIEW 			       = 0x0100,					//置高则代表激活视角切换机构
	DEVICE_SUPPLY 			     = 0x0200,					//置高则代表激活补弹装置
	DEVICE_BuiltInPokeMotor  = 0x0400,  			  //置高则代表拨弹电机内置
    DEVICE_MISSILE           = 0x0800,          //LCM： 置高则代表为 飞镖发射架    
	DEVICE_CAPACITANCE       = 0x1000,          //置高则代表安装超级电容
};

enum robotDistinguishSate{
	ROBOT_NO_NEED_TO_IDENTIFY = 0,
	ROBOT_BEING_IDENTIFIED,
	ROBOT_COMPLETE_IDENTIFIED
};

typedef struct{
	TaskHandle_t 	xHandleTask;
    uint8_t 		robotTypeFlag;                      //机器人类型启动标志位
	uint8_t 		typeOfRobot;
	uint8_t 		camp;
	uint8_t 		distinguishState;
	uint8_t 		distinguishLever;
	uint8_t 		distinguishLastLever;
	f32_t 			picthMinConstrain;
	f32_t 			picthMaxConstrain;
	uint16_t 		robotDeviceList;
	uint32_t 		loops;
	uint8_t			outfitNum;
	uint16_t		boardType;
	uint8_t			id_set;
}robotConfigStruct_t;
void robotBoardConfig(void);
uint8_t getRobotType(void);
void robotKeyInit(void);
void currentRobotParameterConfig(void);
void robotDistinguish(void);
void init_data_prepared(uint16_t boardType);
extern robotConfigStruct_t robotConfigData; 

#endif
