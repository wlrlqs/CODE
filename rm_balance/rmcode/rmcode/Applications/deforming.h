#ifndef __DEFORMING_H
#define __DEFORMING_H
#include "BSP.h"
#include "Util.h"
#include "pid.h"
#include "chassis.h"

/************
工程车登岛
*************/


/************
工程车抓弹
*************/
/* 
以下开关是对应实物动作、不是对应气缸的开关
爪子的打开对应气缸的关闭
爪子的关闭对应气缸的打开
*/							//工程改


//工程传感器触发限制
//    1     1     1     1
//   左开  左闭	 右开  右闭
#define PROTECT_LIMIT       0      
#define LEFT_ON 			0x09
#define RIGHT_ON			0x06
#define MID_ON				0x05
//打哨兵仰角
#define AUXILI_FIGHT_ANGLE	25.0f
#define REVERSING_GPIO							BSP_GPIOA1
#define REVERSING_FRONT							PAout(1)=0
#define REVERSING_BEHIND						PAout(1)=1
/************21赛季工程气动板****************/
/******************pneumatic_1********************/
//拖车机构
#define AUXILIARY_RESCUE_OUT            pneumaticData.pneumatic_1.sendData[1].byte.h_temp = 1
#define AUXILIARY_RESCUE_IN             pneumaticData.pneumatic_1.sendData[1].byte.h_temp = 0
//刷卡复活机构
#define AUXILIARY_SWIPE_OUT             pneumaticData.pneumatic_1.sendData[1].byte.g_temp = 1
#define AUXILIARY_SWIPE_IN              pneumaticData.pneumatic_1.sendData[1].byte.g_temp = 0

//拉线传感器
#define FIRST_HEIGHT_GET                260//小资源岛高度一260
#define SECOND_HEIGHT_GET               260//小资源岛高度二320 2.挡前哨战模式高度
#define THIRD_HEIGHT_GET                220//1.兑换站高度 
#define FOURTH_HEIGHT_GET               90//1.大资源岛高度一 2.有矿正常高度 
#define FIFTH_HEIGHT_GET                170//大资源岛高度二
#define SIXTH_HEIGHT_GET                130//1.地盘爪子升起高度
#define NORMAL_HEIGHT_GET               20//1.正常行走高度2.兑换第一块下降高度
#define HIGHESTHEIGHT_GET               335//最高高度

/******************pneumatic_2********************/
//X轴移动
//#define X_LEFT_GET                      pneumaticData.pneumatic_2.readData[0].byte.a_temp
//#define X_MIDDLE_GET                    pneumaticData.pneumatic_2.readData[0].byte.b_temp
//#define X_RIGHT_GET                     pneumaticData.pneumatic_2.readData[0].byte.c_temp
//Y轴移动
//#define Y_FRONT_GET                     pneumaticData.pneumatic_2.readData[0].byte.d_temp
//#define Y_BEHIND_GET                    pneumaticData.pneumatic_2.readData[0].byte.e_temp
//底盘小爪子相关操作
//爪子伸出与收回
//#define FRONT_CLAW_OUT                  pneumaticData.pneumatic_2.sendData[1].byte.c_temp=1
//#define FRONT_CLAW_IN                   pneumaticData.pneumatic_2.sendData[1].byte.c_temp=0
//爪子抬升与下降
//#define FRONT_CLAW_UP                   pneumaticData.pneumatic_2.sendData[1].byte.b_temp=1
//#define FRONT_CLAW_DOWN                 pneumaticData.pneumatic_2.sendData[1].byte.b_temp=0
//爪子张开与关闭
//#define FRONT_CLAW_ON                   pneumaticData.pneumatic_2.sendData[1].byte.a_temp=1
//#define FRONT_CLAW_OFF                  pneumaticData.pneumatic_2.sendData[1].byte.a_temp=0
////底盘小爪子位置
//#define FRONT_CLAW_RESET                pneumaticData.pneumatic_2.readData[1].byte.e_temp  
//前伸机构
#define Y_FRONT                        	pneumaticData.pneumatic_2.sendData[1].byte.d_temp=1
#define Y_BACK                       	pneumaticData.pneumatic_2.sendData[1].byte.d_temp=0
/******************pneumatic_3********************/
//爪子张开关闭        
#define PAW_ON                          pneumaticData.pneumatic_3.sendData[0].byte.g_temp=0
#define PAW_OFF                         pneumaticData.pneumatic_3.sendData[0].byte.g_temp=1
//爪子伸出与收回   
#define PAW_OUT                         pneumaticData.pneumatic_3.sendData[0].byte.h_temp=1
#define PAW_IN                          pneumaticData.pneumatic_3.sendData[0].byte.h_temp=0
//矿石一拉紧与放开(带杆气缸)  
#define ORE1_ON                        	pneumaticData.pneumatic_3.sendData[1].byte.a_temp=1
#define ORE1_OFF                        pneumaticData.pneumatic_3.sendData[1].byte.a_temp=0
//矿石一拉出与收回
#define ORE1_OUT                       	pneumaticData.pneumatic_3.sendData[0].byte.f_temp=1
#define ORE1_IN                        pneumaticData.pneumatic_3.sendData[0].byte.f_temp=0

/**************2020赛季**********************************/

//升降判断开关
#define UPPER_ELEC_SWITCH_UP1           pneumaticData.pneumatic_1.readData[0].byte.f_temp
#define UPPER_ELEC_SWITCH_DOWN1			pneumaticData.pneumatic_1.readData[0].byte.g_temp
#define UPPER_ELEC_SWITCH_UP2           1
#define UPPER_ELEC_SWITCH_DOWN2			1//pneumaticData.pneumatic_1.readData[0].byte.c_temp
#define UPPER_ELEC_SWITCH_UP3           1//pneumaticData.pneumatic_1.read1[4]
#define UPPER_ELEC_SWITCH_DOWN3			1//pneumaticData.pneumatic_1.readData[0].byte.c_temp
#define UPPER_ELEC_SWITCH_UP4           1//pneumaticData.pneumatic_1.read1[4]
#define UPPER_ELEC_SWITCH_DOWN4			1//pneumaticData.pneumatic_1.readData[0].byte.c_temp
//上身
#define UPPER_LEFT_ELEC_SWITCH_ON       pneumaticData.pneumatic_2.readData[0].byte.f_temp
#define UPPER_RIGHT_ELEC_SWITCH_ON      pneumaticData.pneumatic_2.readData[1].byte.e_temp
#define UPPER_LEFT_ELEC_SWITCH_OFF      pneumaticData.pneumatic_2.readData[0].byte.g_temp
#define UPPER_RIGHT_ELEC_SWITCH_OFF     pneumaticData.pneumatic_2.readData[1].byte.d_temp
#define UPPER_FRONT_ELEC_SWITCH_ON      pneumaticData.pneumatic_2.readData[1].byte.g_temp
#define UPPER_FRONT_ELEC_SWITCH_OFF     pneumaticData.pneumatic_2.readData[1].byte.f_temp

//上车身升降
#define UPPER_UP                        pneumaticData.pneumatic_1.sendData[0].byte.f_temp =1
#define UPPER_DOWN                      pneumaticData.pneumatic_1.sendData[0].byte.f_temp =0     

#define SENTRY_DEFORMING_PID shootData.speedPID
#define SENTRY_DEFORMING_MOVREF  5000
#define SENTRY_REMOTE_SPEED remoteControlData.chassisSpeedTarget.y
#define SENTRY_AUTO_SPEED getchassisData()->landingSpeedy
//tank 
//气缸
#define BIG_GUN_WEAPONS_ON              pneumaticData.pneumatic_1.sendData[0].byte.a_temp=1
#define BIG_GUN_WEAPONS_OFF             pneumaticData.pneumatic_1.sendData[0].byte.a_temp=0
//开火
#define BIG_GUN_ON                      pneumaticData.pneumatic_1.sendData[0].byte.b_temp=1
#define BIG_GUN_OFF                     pneumaticData.pneumatic_1.sendData[0].byte.b_temp=0
//准备上膛
#define GUN_AWAIT1                      !pneumaticData.pneumatic_1.readData[0].byte.a_temp   //大枪管光电开关
#define GUN_AWAIT2                      !pneumaticData.pneumatic_1.readData[0].byte.d_temp   //大枪管光电开关
//气缸状态(电磁传感器)
#define ELEC_MAG_SENSOR_FRONT           pneumaticData.pneumatic_1.readData[0].byte.b_temp    //前电磁传感器
#define ELEC_MAG_SENSOR_BEHIND          pneumaticData.pneumatic_1.readData[0].byte.c_temp    //后电磁传感器


typedef struct 
{
	//光电开关返回信号
	uint8_t 	optoelectronicRecive[8];						 	
    pidData_t 	elevatorSpeed[2];//升降机构
    pidData_t 	elevatorAngle[2];
	pidData_t   xSlidewaySpeed;
	pidData_t   ySlidewaySpeed;
	pidData_t   frontClawSpeed;
	pidData_t   reversingSpeed[2];
	
	pidStruct_t *elevatorSpeedPID[2];
	pidStruct_t *elevatorAnglePID[2];	
	pidStruct_t *xSlidewaySpeedPID;
	pidStruct_t *ySlidewaySpeedPID;
	pidStruct_t *frontClawSpeedPID;	
	pidStruct_t *reversingSpeedPID[2];	
	
    double 		time[2];
	f32_t 		intervalTime;
	uint8_t     elevatorSpeedLimitFlag;
	uint8_t heightstatebreak_Flag;
	uint8_t heightstatetrimming_Flag;	
	uint8_t X_statebreak_Flag;
	uint8_t Y_PlaceInitFinishFlag;//Y轴上电回中标志位
	uint8_t Y_statebreak_Flag;
	uint8_t X_PlaceJudgeFlag;
	uint8_t X_PlaceInitFinishFlag;//X轴上电回中标志位
	uint8_t AllInitFinishFlag;//所有机构复位标志位
	
}auxiliaryStruct_t; 

typedef struct 
{	
    bool protectFlag;
    int8_t direction;
	float dataOut;
	float deformRef;
	
}sentryDeformingStruct_t;   
enum{ 
	X_LEFT =1,
	X_MIDDLE,
	X_RIGHT,
	X_MALPOSITION
};
extern auxiliaryStruct_t auxiliaryDeformingData;
extern sentryDeformingStruct_t sentryDeformingData;	

//整车上升(包含登岛抓弹下岛全数据)
void GO_TO_HEIGHT(uint16_t HEIGHT_TO_GET);
void LEFT_TO_X(uint8_t X_TO_GET);
void RIGHT_TO_X(uint8_t X_TO_GET);
void ADVANCE_TO_Y(uint8_t Y_TO_GET);
void BACK_TO_Y(uint8_t Y_TO_GET);
uint8_t retuernToMiddle_X(void);
void mechaDeformingUpdate(void);
void mechaDeformingInit(void);
void grabAmmunitionUpdate(void);
uint8_t tankGrabForward(void);
uint8_t tankGrabBack(void);
uint8_t tankGrabThrow(void);
void setDeformingZero(void);
void motorPIDUpdate(void);
/********
步兵
********/
void up_step_control(uint8_t __switch,f32_t intervalTime);
void infantry_deformingTask(void);
/********
工程车
********/
void protectDeforming(void);
void chassisChaseSwitch(uint8_t isEnable);
void grabFirstMiddle(void);
void resSetPress(void);
#endif
