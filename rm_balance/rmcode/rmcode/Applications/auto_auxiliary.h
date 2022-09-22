#ifndef __AUTO_AUXILIARY_H
#define __AUTO_AUXILIARY_H

#include "auto_task.h"

#define PRESS_W (remoteControlData.dt7Value.keyBoard.bit.W)
#define PRESS_A (remoteControlData.dt7Value.keyBoard.bit.A)
#define PRESS_S (remoteControlData.dt7Value.keyBoard.bit.S)
#define PRESS_D (remoteControlData.dt7Value.keyBoard.bit.D)

enum{
    AUXILIARY_MANUAL = 0,										//�ֶ�
	AUXILIARY_PROTECT_OUTPOST = 1,                              //X:���̳���ǰ��վģʽ
    AUXILIARY_SMALL_RESOURSE = 8,					            //C:��+X��С��Դ������ץ
	AUXILIARY_BIG_RESOURSE = 4,								    //Z����+X������Դ����ץ	
    AUXILIARY_EXCHANGE = 5,                                     //V��(+R)�һ�վ�һ���ʯ
	AUXILIARY_RESCUE = 9,										//SHIFT���ϳ�
	AUXILIARY_SWIPE_CARD = 6,                                   //Q ��ˢ������
	//AUXILIARY_FRONT_CLAW = 10,                                 //E:(+X)����Сצ��
	
};
AutoTaskStruct_t* getauxiliaryAutoData(void);
void auxiliaryAutoTaskUpdate(void);

#endif
