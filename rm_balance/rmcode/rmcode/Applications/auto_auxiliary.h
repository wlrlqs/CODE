#ifndef __AUTO_AUXILIARY_H
#define __AUTO_AUXILIARY_H

#include "auto_task.h"

#define PRESS_W (remoteControlData.dt7Value.keyBoard.bit.W)
#define PRESS_A (remoteControlData.dt7Value.keyBoard.bit.A)
#define PRESS_S (remoteControlData.dt7Value.keyBoard.bit.S)
#define PRESS_D (remoteControlData.dt7Value.keyBoard.bit.D)

enum{
    AUXILIARY_MANUAL = 0,										//手动
	AUXILIARY_PROTECT_OUTPOST = 1,                              //X:工程车挡前哨站模式
    AUXILIARY_SMALL_RESOURSE = 8,					            //C:（+X）小资源岛三连抓
	AUXILIARY_BIG_RESOURSE = 4,								    //Z：（+X）大资源岛单抓	
    AUXILIARY_EXCHANGE = 5,                                     //V：(+R)兑换站兑换矿石
	AUXILIARY_RESCUE = 9,										//SHIFT：拖车
	AUXILIARY_SWIPE_CARD = 6,                                   //Q ：刷卡复活
	//AUXILIARY_FRONT_CLAW = 10,                                 //E:(+X)底盘小爪子
	
};
AutoTaskStruct_t* getauxiliaryAutoData(void);
void auxiliaryAutoTaskUpdate(void);

#endif
