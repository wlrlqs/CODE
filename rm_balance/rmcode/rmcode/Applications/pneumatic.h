#ifndef __PNEUMATIC_H
#define __PNEUMATIC_H

#include "BSP.h"
#include "Util.h"
#include "stdint.h"
#include <stdbool.h>
#define P_HERO_FIRE			pneumaticData.pneumatic_1.send4[0]							//������ŷ�
#define P_HERO_LOAD			pneumaticData.pneumatic_1.send4[1]							//��������
#define P_HERO_42_LID		pneumaticData.pneumatic_1.send4[2]							//�󵯲ָ�
#define P_HERO_17_LID		pneumaticData.pneumatic_1.send4[3]							//С���ָ�
#define P_HERO_TV			pneumaticData.pneumatic_1.send4[4]							//����Ӱ��
#define P_HERO_CCD			pneumaticData.pneumatic_1.send4[5]							//��������ͷ����

#define ID_PNEUMATIC    0x703																//�����巴��

typedef union{
	uint8_t Flag_temp;
	struct{
		uint8_t firstHeight_Flag : 1;//С��Դ��ץ��߶ȣ�����צ�������߶�
		uint8_t secondHeight_Flag : 1;//С��Դ�����߶�
		uint8_t thirdHeight_Flag : 1;//����Դ��ץ��߶�
		uint8_t fourthHeight_Flag : 1;//����Դ�����߶�
		uint8_t fifthHeight_Flag : 1;//�һ�վ�߶�
		uint8_t normalHeight_Flag : 1;//�����ø߶�
		uint8_t highestHeight_Flag : 1;//��߸߶�
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
	//������
	formatTrans8Struct_t readData[2];
	MillayFlagStruct_t  Flag;
	uint8_t LmmSubtractRmm;
	uint16_t millayData_mm[2];//[0]Ϊ���//[1]Ϊ�Ҳ�
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


