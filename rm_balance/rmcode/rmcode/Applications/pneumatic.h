#ifndef __PNEUMATIC_H
#define __PNEUMATIC_H

#include "BSP.h"
#include "Util.h"
#include "stdint.h"
#include <stdbool.h>
#define P_HERO_FIRE			pneumaticData.pneumatic_1.send4[0]							//开火快排阀
#define P_HERO_LOAD			pneumaticData.pneumatic_1.send4[1]							//上膛气缸
#define P_HERO_42_LID		pneumaticData.pneumatic_1.send4[2]							//大弹仓盖
#define P_HERO_17_LID		pneumaticData.pneumatic_1.send4[3]							//小弹仓盖
#define P_HERO_TV			pneumaticData.pneumatic_1.send4[4]							//倒车影像
#define P_HERO_CCD			pneumaticData.pneumatic_1.send4[5]							//倒车摄像头开关

#define ID_PNEUMATIC    0x703																//气动板反馈

typedef union{
	uint8_t Flag_temp;
	struct{
		uint8_t firstHeight_Flag : 1;//小资源岛抓矿高度；底盘爪子上升高度
		uint8_t secondHeight_Flag : 1;//小资源岛抽矿高度
		uint8_t thirdHeight_Flag : 1;//大资源岛抓矿高度
		uint8_t fourthHeight_Flag : 1;//大资源岛抽矿高度
		uint8_t fifthHeight_Flag : 1;//兑换站高度
		uint8_t normalHeight_Flag : 1;//无作用高度
		uint8_t highestHeight_Flag : 1;//最高高度
		uint8_t free_Flag5 : 1;
		
	}byte;
}MillayFlagStruct_t;

typedef struct 
{
	uint8_t penumaticFlag;
	formatTrans8Struct_t sendData[2];
	uint8_t send1[6];
	uint8_t send2[6];
	uint8_t send3[6];
	uint8_t send4[6];
	uint8_t read1[6];
	//传感器
	formatTrans8Struct_t readData[2];
	MillayFlagStruct_t  Flag;
	uint8_t LmmSubtractRmm;
	uint16_t millayData_mm[2];//[0]为左侧//[1]为右侧
}pneumatic_state_data_t;

typedef struct 
{
	pneumatic_state_data_t pneumatic_1;
	pneumatic_state_data_t pneumatic_2;
	pneumatic_state_data_t pneumatic_3;
}pneumaticData_t;


void pneumaticInit(void);
void pneumatic_can1_sentData(uint32_t ID_CAN);
void pneumatic_can2_sentData(uint32_t ID_CAN);
void p_tank_can1_sentData(uint32_t ID_CAN);
void pneumatic_sendDataUpdate(void);
void pneumatic_readData(CanRxMsg *can_rx_msg);
extern pneumaticData_t	pneumaticData;

#endif


