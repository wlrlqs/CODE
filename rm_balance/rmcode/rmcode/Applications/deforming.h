#ifndef __DEFORMING_H
#define __DEFORMING_H
#include "BSP.h"
#include "Util.h"
#include "pid.h"
#include "chassis.h"

/************
���̳��ǵ�
*************/


/************
���̳�ץ��
*************/
/* 
���¿����Ƕ�Ӧʵ�ﶯ�������Ƕ�Ӧ���׵Ŀ���
צ�ӵĴ򿪶�Ӧ���׵Ĺر�
צ�ӵĹرն�Ӧ���׵Ĵ�
*/							//���̸�


//���̴�������������
//    1     1     1     1
//   ��  ���	 �ҿ�  �ұ�
#define PROTECT_LIMIT       0      
#define LEFT_ON 			0x09
#define RIGHT_ON			0x06
#define MID_ON				0x05
//���ڱ�����
#define AUXILI_FIGHT_ANGLE	25.0f
#define REVERSING_GPIO							BSP_GPIOA1
#define REVERSING_FRONT							PAout(1)=0
#define REVERSING_BEHIND						PAout(1)=1
/************21��������������****************/
/******************pneumatic_1********************/
//�ϳ�����
#define AUXILIARY_RESCUE_OUT            pneumaticData.pneumatic_1.sendData[1].byte.h_temp = 1
#define AUXILIARY_RESCUE_IN             pneumaticData.pneumatic_1.sendData[1].byte.h_temp = 0
//ˢ���������
#define AUXILIARY_SWIPE_OUT             pneumaticData.pneumatic_1.sendData[1].byte.g_temp = 1
#define AUXILIARY_SWIPE_IN              pneumaticData.pneumatic_1.sendData[1].byte.g_temp = 0

//���ߴ�����
#define FIRST_HEIGHT_GET                260//С��Դ���߶�һ260
#define SECOND_HEIGHT_GET               260//С��Դ���߶ȶ�320 2.��ǰ��սģʽ�߶�
#define THIRD_HEIGHT_GET                220//1.�һ�վ�߶� 
#define FOURTH_HEIGHT_GET               90//1.����Դ���߶�һ 2.�п������߶� 
#define FIFTH_HEIGHT_GET                170//����Դ���߶ȶ�
#define SIXTH_HEIGHT_GET                130//1.����צ������߶�
#define NORMAL_HEIGHT_GET               20//1.�������߸߶�2.�һ���һ���½��߶�
#define HIGHESTHEIGHT_GET               335//��߸߶�

/******************pneumatic_2********************/
//X���ƶ�
//#define X_LEFT_GET                      pneumaticData.pneumatic_2.readData[0].byte.a_temp
//#define X_MIDDLE_GET                    pneumaticData.pneumatic_2.readData[0].byte.b_temp
//#define X_RIGHT_GET                     pneumaticData.pneumatic_2.readData[0].byte.c_temp
//Y���ƶ�
//#define Y_FRONT_GET                     pneumaticData.pneumatic_2.readData[0].byte.d_temp
//#define Y_BEHIND_GET                    pneumaticData.pneumatic_2.readData[0].byte.e_temp
//����Сצ����ز���
//צ��������ջ�
//#define FRONT_CLAW_OUT                  pneumaticData.pneumatic_2.sendData[1].byte.c_temp=1
//#define FRONT_CLAW_IN                   pneumaticData.pneumatic_2.sendData[1].byte.c_temp=0
//צ��̧�����½�
//#define FRONT_CLAW_UP                   pneumaticData.pneumatic_2.sendData[1].byte.b_temp=1
//#define FRONT_CLAW_DOWN                 pneumaticData.pneumatic_2.sendData[1].byte.b_temp=0
//צ���ſ���ر�
//#define FRONT_CLAW_ON                   pneumaticData.pneumatic_2.sendData[1].byte.a_temp=1
//#define FRONT_CLAW_OFF                  pneumaticData.pneumatic_2.sendData[1].byte.a_temp=0
////����Сצ��λ��
//#define FRONT_CLAW_RESET                pneumaticData.pneumatic_2.readData[1].byte.e_temp  
//ǰ�����
#define Y_FRONT                        	pneumaticData.pneumatic_2.sendData[1].byte.d_temp=1
#define Y_BACK                       	pneumaticData.pneumatic_2.sendData[1].byte.d_temp=0
/******************pneumatic_3********************/
//צ���ſ��ر�        
#define PAW_ON                          pneumaticData.pneumatic_3.sendData[0].byte.g_temp=0
#define PAW_OFF                         pneumaticData.pneumatic_3.sendData[0].byte.g_temp=1
//צ��������ջ�   
#define PAW_OUT                         pneumaticData.pneumatic_3.sendData[0].byte.h_temp=1
#define PAW_IN                          pneumaticData.pneumatic_3.sendData[0].byte.h_temp=0
//��ʯһ������ſ�(��������)  
#define ORE1_ON                        	pneumaticData.pneumatic_3.sendData[1].byte.a_temp=1
#define ORE1_OFF                        pneumaticData.pneumatic_3.sendData[1].byte.a_temp=0
//��ʯһ�������ջ�
#define ORE1_OUT                       	pneumaticData.pneumatic_3.sendData[0].byte.f_temp=1
#define ORE1_IN                        pneumaticData.pneumatic_3.sendData[0].byte.f_temp=0

/**************2020����**********************************/

//�����жϿ���
#define UPPER_ELEC_SWITCH_UP1           pneumaticData.pneumatic_1.readData[0].byte.f_temp
#define UPPER_ELEC_SWITCH_DOWN1			pneumaticData.pneumatic_1.readData[0].byte.g_temp
#define UPPER_ELEC_SWITCH_UP2           1
#define UPPER_ELEC_SWITCH_DOWN2			1//pneumaticData.pneumatic_1.readData[0].byte.c_temp
#define UPPER_ELEC_SWITCH_UP3           1//pneumaticData.pneumatic_1.read1[4]
#define UPPER_ELEC_SWITCH_DOWN3			1//pneumaticData.pneumatic_1.readData[0].byte.c_temp
#define UPPER_ELEC_SWITCH_UP4           1//pneumaticData.pneumatic_1.read1[4]
#define UPPER_ELEC_SWITCH_DOWN4			1//pneumaticData.pneumatic_1.readData[0].byte.c_temp
//����
#define UPPER_LEFT_ELEC_SWITCH_ON       pneumaticData.pneumatic_2.readData[0].byte.f_temp
#define UPPER_RIGHT_ELEC_SWITCH_ON      pneumaticData.pneumatic_2.readData[1].byte.e_temp
#define UPPER_LEFT_ELEC_SWITCH_OFF      pneumaticData.pneumatic_2.readData[0].byte.g_temp
#define UPPER_RIGHT_ELEC_SWITCH_OFF     pneumaticData.pneumatic_2.readData[1].byte.d_temp
#define UPPER_FRONT_ELEC_SWITCH_ON      pneumaticData.pneumatic_2.readData[1].byte.g_temp
#define UPPER_FRONT_ELEC_SWITCH_OFF     pneumaticData.pneumatic_2.readData[1].byte.f_temp

//�ϳ�������
#define UPPER_UP                        pneumaticData.pneumatic_1.sendData[0].byte.f_temp =1
#define UPPER_DOWN                      pneumaticData.pneumatic_1.sendData[0].byte.f_temp =0     

#define SENTRY_DEFORMING_PID shootData.speedPID
#define SENTRY_DEFORMING_MOVREF  5000
#define SENTRY_REMOTE_SPEED remoteControlData.chassisSpeedTarget.y
#define SENTRY_AUTO_SPEED getchassisData()->landingSpeedy
//tank 
//����
#define BIG_GUN_WEAPONS_ON              pneumaticData.pneumatic_1.sendData[0].byte.a_temp=1
#define BIG_GUN_WEAPONS_OFF             pneumaticData.pneumatic_1.sendData[0].byte.a_temp=0
//����
#define BIG_GUN_ON                      pneumaticData.pneumatic_1.sendData[0].byte.b_temp=1
#define BIG_GUN_OFF                     pneumaticData.pneumatic_1.sendData[0].byte.b_temp=0
//׼������
#define GUN_AWAIT1                      !pneumaticData.pneumatic_1.readData[0].byte.a_temp   //��ǹ�ܹ�翪��
#define GUN_AWAIT2                      !pneumaticData.pneumatic_1.readData[0].byte.d_temp   //��ǹ�ܹ�翪��
//����״̬(��Ŵ�����)
#define ELEC_MAG_SENSOR_FRONT           pneumaticData.pneumatic_1.readData[0].byte.b_temp    //ǰ��Ŵ�����
#define ELEC_MAG_SENSOR_BEHIND          pneumaticData.pneumatic_1.readData[0].byte.c_temp    //���Ŵ�����


typedef struct 
{
	//��翪�ط����ź�
	uint8_t 	optoelectronicRecive[8];						 	
    pidData_t 	elevatorSpeed[2];//��������
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
	uint8_t Y_PlaceInitFinishFlag;//Y���ϵ���б�־λ
	uint8_t Y_statebreak_Flag;
	uint8_t X_PlaceJudgeFlag;
	uint8_t X_PlaceInitFinishFlag;//X���ϵ���б�־λ
	uint8_t AllInitFinishFlag;//���л�����λ��־λ
	
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

//��������(�����ǵ�ץ���µ�ȫ����)
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
����
********/
void up_step_control(uint8_t __switch,f32_t intervalTime);
void infantry_deformingTask(void);
/********
���̳�
********/
void protectDeforming(void);
void chassisChaseSwitch(uint8_t isEnable);
void grabFirstMiddle(void);
void resSetPress(void);
#endif
