#ifndef __WARNING_H
#define __WARNING_H

#include "Util.h"
#include "Driver_SK6812.h"
#include "Driver_Beep.h"

#define WARNING_PRIORITY 8
#define WARNING_STACK_SIZE 64
#define WARNING_STACK_PERIOD 100

#define SK6812_GREEN  	sk6812Data.colorStd[COLOR_GREEN]
#define SK6812_RED	  	sk6812Data.colorStd[COLOR_RED]
#define SK6812_YELLOW 	sk6812Data.colorStd[COLOR_YELLOW]
#define SK6812_DARK		sk6812Data.colorStd[COLOR_DARK]
#define SK6812_BLUE		sk6812Data.colorStd[COLOR_BLUE]
#define SK6812_PINK		sk6812Data.colorStd[COLOR_PINK]
#define SK6812_WHITE	sk6812Data.colorStd[COLOR_WHITE]
#define SK6812_PURPLE	sk6812Data.colorStd[COLOR_PURPLE]

#define CAP_SOC       ((f32_t)(100 * (getpowerData() ->capVol - 17) / (getpowerData() ->capVolMax - 17)))

enum{
	RC_FAULT 			= 0x0001,//遥控器故障或丢失       第一颗
	JUDGE_FAULT 		= 0x0002,//裁判系统通信异常     第二颗
	SENSOR_FAULT 		= 0x0004,//IMU故障              第三颗
	MOTOR_FAULT 		= 0x0008,//电机故障             第四颗
	VISION_FAULT 		= 0x0010,//TX2通信故障          第五颗
	DOUBLE_TRANS_FAULT  = 0x0020,//CANFD故障	      第六颗
	POWER_LIMIT_FAULT   = 0x0040,//电容控制板故障   第七颗
};

typedef struct{
	//灯条灯珠状态(最大16颗灯珠),每1位表示1颗灯珠
	formatTrans16Struct_t 	lightBarsState;
	//最高级错误
	uint8_t 				highestFault;
	//最高级错误存在标志
	uint8_t 				highestFaultFlag;
	uint8_t 				highestFaultLoop;
	hsvColor_t 				displayColor;
	//点亮个数
	uint8_t 				displayNumber;
	//闪烁频率
	uint8_t 				blinkFrequency;
	uint8_t 				capSoc;
	uint32_t 				loops;
}warningStruct_t;

extern warningStruct_t warningData;

void warningUpdate(void);
void lightBarsStateUpdata(void);
uint8_t check_shooter_working(void);
uint8_t check_gimbal_working(void);
#endif

