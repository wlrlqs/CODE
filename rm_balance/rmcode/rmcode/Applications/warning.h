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
	RC_FAULT 			= 0x0001,//ң�������ϻ�ʧ       ��һ��
	JUDGE_FAULT 		= 0x0002,//����ϵͳͨ���쳣     �ڶ���
	SENSOR_FAULT 		= 0x0004,//IMU����              ������
	MOTOR_FAULT 		= 0x0008,//�������             ���Ŀ�
	VISION_FAULT 		= 0x0010,//TX2ͨ�Ź���          �����
	DOUBLE_TRANS_FAULT  = 0x0020,//CANFD����	      ������
	POWER_LIMIT_FAULT   = 0x0040,//���ݿ��ư����   ���߿�
};

typedef struct{
	//��������״̬(���16�ŵ���),ÿ1λ��ʾ1�ŵ���
	formatTrans16Struct_t 	lightBarsState;
	//��߼�����
	uint8_t 				highestFault;
	//��߼�������ڱ�־
	uint8_t 				highestFaultFlag;
	uint8_t 				highestFaultLoop;
	hsvColor_t 				displayColor;
	//��������
	uint8_t 				displayNumber;
	//��˸Ƶ��
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

