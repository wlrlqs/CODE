#ifndef __AUTO_UAV_H
#define __AUTO_UAV_H

#include "auto_task.h"

enum{
	UAV_MANUAL = 0,															//��̨���ֶ�����
	UAV_AUTOMATIC_AIM = 5,													//V��������׼
};

AutoTaskStruct_t* getuavAutoData(void);
void uavAutoTaskUpdate(void);

#endif
