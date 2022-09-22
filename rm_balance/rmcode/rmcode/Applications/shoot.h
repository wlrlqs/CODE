#ifndef __SHOOT_H
#define __SHOOT_H
#include "BSP.h"
#include "Driver.h"
#include "Util.h"
#include "pid.h"
#include "Driver_RMMotor.h"
#include "control.h"
#include "type_can_isp.h"

#define FricMotor_L TIM9->CCR1
#define FricMotor_R TIM9->CCR2

#define BULLET_MONITOR_FLAG					shootData.bulletMonitorFlag

#define LASER_GPIO									BSP_GPIOE11
#define LASER_ON									PEout(11)=1
#define LASER_OFF									PEout(11)=0
#define SHOOTER_SWITCH_GPIO							BSP_GPIOB9
		
#define SHOOTER_SWITCH					GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)
#define TANK_SHOOTER_SWITCH	         	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)// pneumaticData.pneumatic_3.readData[1].byte.g_temp 

#define SHOOT_HEAT(shootSpeed)			(f32_t)shootSpeed

#define MAIN_POKE					commandoMotorConfig[POKEMOTOR]
#define SLAVE_POKE					commandoMotorConfig[SLAVE_POKEMOTOR]

#define SHOOT_STALL_CURRENT				9000                     //����ʸ������
#define POKE_SPEED_RATA					0.6f           
#define POKE_STATIC_SPEED				7300          //������ת��
//�����ź� 
#define FIRE_FLAG_17MM 			         	shootData.fireFlag_17mm	         //�����־λ
#define FIRE_FLAG_42MM						shootData.fireFlag_42mm

//������������·����ӵ���Ŀ
#define SHOOT_TIMES_CONTINUOUS			3
//С�ӵ��˺�
#define SMALL_BULLET_HARM				10
//���ӵ��˺�
#define BIG_BULLET_HARM					100	
//С�ӵ�Ĭ������
#define BULLET_SPEED_DEFAULT    		26.0f			    				
#define SLAVE_POKE_SET					shootData.slave_pokeSpeedRef	//LCM: �ܿ���shootTrigger_17mm

#define MIN_SPEED                    0
#define MAX_SPEED                    5000
#define SNAIL_MOTOR                  DISABLE

//17mm����
#define	__17MM_HEAT			10.0f
//42mm����
#define __42MM_HEAT			100.0f
//������
#define __NO_HEAT			0.0f
//Ӣ�۱�������
#define protectHeartTank             100										

//17mm����ٶ�����
#define SPEED_17MM_LOW     15
#define SPEED_17MM_MID     18
#define SPEED_17MM_HIGH    22
#define SPEED_17MM_EXTREME 30
//42mm����ٶ�����
#define SPEED_42MM_LOW		10
#define SPEED_42MM_MID		12
#define SPEED_42MM_HIGH		14
#define SPEED_42MM_EXTREME	16

//���ָ�״̬
enum{
	supply_move = 0,
	supply_close = 1,
	supply_open = 2,
};	
//���ģʽ
typedef enum{
	MANUAL_SINGLE = 0,
	MANUAL_CONTINUOUS,
	AUTO_CONTINUOUS,
}shootMode_e;      						

//���״̬
typedef enum{
	SAFE = 0,
	DANGEROUS,
}shootStatus_e;								

//���״̬
typedef enum{
	MOTOR_REVERSAL = -1,
	MOTOR_STOP = 0,
	MOTOR_FOWARD = 1,
}motorStatus_e;								

enum{
	TYPE_17MM = 1,
	TYPE_42MM = 2,
};

typedef struct{
						//ʱ����
	TickType_t 			xLastWakeTime;	  
						//17mm���ģʽ
	shootMode_e 		shootMode_17mm;
						//17mm���״̬
	shootStatus_e 		shootStatus_17mm;
						//42mm���״̬
	shootStatus_e		shootStatus_42mm;
						//17mm�����ȫģʽ
	shootStatus_e 		shootStatusMode_17mm;
	TaskHandle_t 		xHandleTaskshoot;	
	pidStruct_t 		*speedPID;
	pidStruct_t 		*testSpeedPID;
	pidStruct_t 		*fricWheelSpeedPID[2];
	pidStruct_t 		*slave_pokeSpeedPID;
	errorScanStruct_t 	fricWheelError[2];
	errorScanStruct_t 	lidError;
	errorScanStruct_t 	pokeError;
						//�嵯��־λ
	bool  				clearBulletFlag;      
						//�������ٶ�����ֵ
	f32_t 				pokeSpeedRef;
						//�Ӳ������ٶ�����ֵ
	f32_t				slave_pokeSpeedRef;
						//�������ٶ��趨
	f32_t				pokeSpeedSet;
						//������PID���ֵ
	f32_t 				pokeSpeedOut;
						//�Ӳ�����PID���ֵ;
	f32_t 				slave_pokeSpeedOut;
						//17mm��ȫ����
	f32_t				safe_heat_17mm;
						//42mm��ȫ����
	f32_t				safe_heat_42mm;
						//��������
	uint16_t			speed_limit;
						//�������ٶ�����ֵ
	f32_t 				bigPokeSpeedRef;  		
						//������PID���ֵ
	f32_t 				bigPokeSpeedOut; 	  	
	uint8_t 			autoMode;			  
						//�ϴ��ӵ�����
	f32_t 				shootSpeedLast;				
						//42mmĦ����ת���趨
	uint16_t 			fricSpeedSet_42mm; 
						//17mmĦ����ת���趨
	uint16_t 			fricSpeedSet_17mm;	
						//Ӣ�۳��ӵ���������ֵ
	f32_t 				fricWheelSpeedRef[2];	
						//Ӣ�۳��ӵ�����PID���ֵ
	f32_t				fricWheelSpeedOut[2]; 				  
						//ʣ���ֶ��������
	uint16_t 			shootManualNumber; 
						//�������flag
	uint8_t 			fireDataInit;				
						//17mm�������ָ��
	uint8_t 			shootTrigger_17mm;
						//42mm�������ָ��
	uint8_t				shootTrigger_42mm;
						//���������־
	uint8_t				shootTriggerReserve;
						//���Ԥ���˺�
	f32_t 				shootHPRef;   		  	
	f32_t 				shooterJudgeLastHeat_17mm;
	f32_t 				shooterJudgeHeat_17mm;
	f32_t				shooterJudgeHeat_42mm;
	f32_t				shooterJudgeLastHeat_42mm;
						//17mmǹ������
	f32_t 				shooterHeat_17mm;     
						//42mmǹ������
	f32_t				shooterHeat_42mm;
						//17mmǹ������ÿ0.1����ȴֵ
	f32_t 				shooterHeatCoolingPs_17mm;
						//42mmǹ������ÿ0.1����ȴֵ
	f32_t				shooterHeatCoolingPs_42mm;
						//17mmǹ��ʣ������
	f32_t 				shooterHeatRemain_17mm;
						//42mmǹ��ʣ������
	f32_t				shooterHeatRemain_42mm;
						//�´�����ȴ�ʱ��
	TickType_t 			shootWaitTime;
	uint8_t 			monitorFlag;
						//Ħ���ֿ�����־
	uint8_t 			fricMotorFlag;		  
						//42mm�ӵ������־
	uint8_t 			fireFlag_42mm;		  
						//17mm�ӵ������־
	uint8_t 			fireFlag_17mm;
		          //���ո��˶���־
	uint8_t       supplyPlaceFlag;
	            //PY: ���ոǿ��ر�־
	uint8_t       supplyNumFlag;
						//���ո�����
	f32_t 				supplyRef;
						//�ӵ��ո�����
	f32_t				slave_supplyRef;
						//���ո�PID
    pidStruct_t 		*supplySpeedPID;
						//���ո�PID���ֵ
	f32_t 				supplySpeedOut;
						//�ӵ��ո�PID���ֵ
	f32_t				slave_supplySpeedOut;
						//��������־
	bool 				ballisticFill;					
						//��ɱ�����־
	uint8_t 			suicideFireFlag;
	f32_t 				bigsupplySpeedOut;
    pidStruct_t 		*shootsupplyPID;
    f32_t 				supplyTime[2];
    f32_t 				supplyIntervalTime;
						//������תλ
	int8_t 				pokeStall;		
	
	int8_t 				slave_pokeStall;		
						//42mmǹ������
	f32_t 				shooterHeatTank; 	
	uint8_t 			bulletMonitorFlag;
	f32_t 				intervalTime;
	double 				time[2];
	uint32_t 			loops;
	uint16_t			waitTime;
	double 				timeing,lasttimez,gsTime;
	f32_t 				speeding;
	//�����޸�Ħ����ת��
	int16_t       infantry_add_fire_speed_low; 
	int16_t       infantry_add_fire_speed_mid; 
	int16_t       infantry_add_fire_speed_high; 
	int16_t       infantry_add_fire_speed_extreme; 
	int16_t				L_speed;
	int16_t				R_speed;
	u8						shootmode_ui;
    
	
	//����Ӣ�۴���ر���
	uint16_t 			loadStep;
	bool 				bulletExistStop;
	uint32_t 			turntableStalltime;
	bool				bulletBuzyFlag;
	bool				bulletfreeFlag;
	bool				bulletFlag;
    bool			    p_tankshootmode;
	uint8_t				laser_close;
}shootStruct_t;

extern u8 infantryFricStateFlag;
extern u8 ftankFricStateFlag;
extern shootStruct_t* getshootData(void);

void getShootMode(void);
void supplyUpdate(void);
void shootDataReset(void);
void shooterHeatIncreasing(void);
void shooterHeatCooling(void);
void shooterHeatAbjust(void);
void shootUpdate(void);
void shootInit(void);	
void smallPokeSpeedRefChange(void);
void shootProtectTank(void);

#endif
