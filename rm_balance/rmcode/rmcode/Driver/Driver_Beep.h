#ifndef __DRIVER_BEEP_H
#define __DRIVER_BEEP_H

#include "stdbool.h"
#include "stm32f4xx.h"
#include "driver.h"
/* �����ͺ�ѡ��ͬʱֻ�ܶ���һ�� */
//2019��һ�����أ�ȫ�ӿ�
//#define USE_2019_A	
//2019�ڶ������أ�������CANΪ���̣�����ADC�ӿ�
//#define USE_2019_B			
#define USE_2020

#ifdef USE_2019_A
#define BEEP_LENGTH 50
#define BEEP_MAX_NOISE 50
#define BEEP_TIM TIM4
#define BEEP_PORT BEEP_TIM->CCR2
#define BEEP_GPIO BSP_GPIOD13

#define LED_R_GPIO BSP_GPIOD4
#define LED_G_GPIO BSP_GPIOD5
#define LED_B_GPIO BSP_GPIOD6

#define LED_R PDout(4)
#define LED_G PDout(5)
#define LED_B PDout(6)
#endif

#ifdef USE_2019_B
#define BEEP_LENGTH 100
#define BEEP_MAX_NOISE 100
#define BEEP_TIM TIM12
#define BEEP_PORT BEEP_TIM->CCR1
#define BEEP_GPIO BSP_GPIOD13
#define LED_R_GPIO BSP_GPIOD4
#define LED_G_GPIO BSP_GPIOD5
#define LED_B_GPIO BSP_GPIOD6

#define LED_R PDout(4)
#define LED_G PDout(5)
#define LED_B PDout(6)
#endif

#ifdef USE_2020
#define BEEP_LENGTH 30
#define BEEP_MAX_NOISE 30
#define BEEP_TIM TIM2
#define BEEP_PORT BEEP_TIM->CCR1
#define BEEP_GPIO BSP_GPIOA15

#define LED_R_GPIO BSP_GPIOB12
#define LED_G_GPIO BSP_GPIOB13
#define LED_B_GPIO BSP_GPIOB15

#define LED_R PBout(12)
#define LED_G PBout(13)
#define LED_B PBout(15)


#endif

#define BEEP_PRESCALER 180
#define BEEP_MIN_NOISE 0
#define BEEP_NOTE_MIN 10
#define BEEP_NOTE_MAX 32768 
#define BEEP_PSC BEEP_TIM->PSC
#define MUSIC_MAX_LENGHT 48

#define LED_SLOW 10
#define LED_NORMAL 4
#define LED_FAST 1
#define LED_LIST 11



#define	ID_0x0101 7
#define ID_0x0102 8
#define ID_0x0103 9
#define ID_0x0104 10
#define ID_0x0105 11
/*��������*/
enum{			
	QUIET=0,
	//������ʾ��
	MUSIC_ARMED,
	//������ʾ��
	MUSIC_DISARMED,	
	//IMUУ׼��ʾ��
	MUSIC_IMUCALI,
	//����������ʾ��
	MUSIC_PARAMCALI,
	//MAGУ׼
	MUSIC_MAGCALI,	
	//ʧ��
	MUSIC_RADIO_LOSS,
	//����
	MUSIC_TYPE_INFANTRY,
	//Ӣ��
	MUSIC_TYPE_TANK,			
	//���̳�
	MUSIC_TYPE_AUXILIARY,	
	//�ڱ�
	MUSIC_TYPE_SENTRY,		
	//���˻�
	MUSIC_TYPE_UAV,				
	//С��̨
	MUSIC_TYPE_SMALLGIMBAL,
	//��ID
	MUSIC_NO_ID,					
	//�͵�ѹ
	MUSIC_LOWPOWER,				
	//�������
	MUSIC_HIGHTEMPERATURE,	
	MUSIC_LIST
};
/*����*/
enum{     
	BASS_DO=0,
	BASS_RE,
	BASS_MI,
	BASS_FA,
	BASS_SOL,
	BASS_LA,
	BASS_SI,
	ALTO_DO,
	ALTO_RE,
	ALTO_MI,
	ALTO_FA,
	ALTO_SOL,
	ALTO_LA,
	ALTO_SI,
	HIGH_DO,
	HIGH_RE,
	HIGH_MI,
	HIGH_FA,
	HIGH_SOL,
	HIGH_LA,
	HIGH_SI,
};

enum{
	LED_CONTROLINIT_FAIL = 1,   
	LED_HARDWORK_FAIL,          
	LED_RADIO_LOSS,             
	LED_MAG_CALI,               
	LED_WORKINGORDER,           
	LED_RESERVE_1,              
	LED_IMU_CALI,               
	LED_GIMBAL_CALI,            
	LED_RESERVE_2,              
	LED_DEFINE_ID,
	LED_DOUBLE_TRANS
};

enum{
	LED_DISWORK=0,
	LED_WORK_RED,			//0000 0001    
	LED_WORK_GREEN,			//0000 0010
	LED_WORK_YELLOW,		//0000 0011
	LED_WORK_BLUE,			//0000 0100	
	LED_WORK_PINK,			//0000 0101														
	LED_WORK_CYAN,			//0000 0110  ��ɫ
	LED_WORK_WHITE			//0000 0111
};
enum{
	LED_ENABLE=0,
	LED_DISABLE	
};
/*����*/
#define	N_NOISE 0
#define L_NOISE 50
#define	H_NOISE 100
#pragma pack(1)
typedef struct {
	uint16_t note[MUSIC_MAX_LENGHT];
	uint16_t volume[MUSIC_MAX_LENGHT];
	uint16_t lenght;
} BeepSound_t;

typedef struct {
	uint16_t colour;
	uint16_t frequency;
} LedData_t;

typedef struct{
	void (*beep) (uint16_t commandState);
	void (*led)  (uint16_t commandState);
}appSightFuncClass;

#pragma pack()
extern appSightFuncClass appSightClass;
#endif

