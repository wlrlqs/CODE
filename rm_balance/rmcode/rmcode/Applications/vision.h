#ifndef __VISION_H
#define __VISION_H

#include "bsp.h"
#include "Driver_Slave_Sensor.h"
#include "shoot.h"
#include "util.h"
#include "stdbool.h"
#include "auto_infantry.h"
#include "auto_tank.h"

#define VISION_PRIORITY	  	                6
#define VISION_STACK_SIZE	                1024
#define VISION_PERIOD                       4

#define ROBOT_ID  judgeData.extGameRobotState.robot_id

#define MM_TO_M_CONVERSION(p)               (f32_t)(*p) / 1000.0f
#define M_TO_MM_CONVERSION(p)	            (f32_t)(*p) * 1000.0f

#define ANGLE_TO_RADIAN(p)                  *p * PI / 180.0f
#define RADIAN_TO_ANGLE(p)                  *p * 180.0f / PI

#define SMALL_MANUAL_SINGLE		            26
#define SMALL_MANUAL_CONTINUOUS             20
#define SMALL_AUTO_CONTINUOUS 	            16

#define MANUAL_PREJUDG_SCALE                0.1f

#define VISION_STORED_LENGTH                50	

typedef enum{
INFANTRY_BARREL = 0,//17mmСǹ��
	P_TANK_BARREL,
	F_TANK_BARREL,
	OLD_SENTRY_BARREL,
	OLD_SMG_BARREL,
	NEW_SENTRY_BARREL,
	NEW_SMG_BARREL,
	AUXILIARY_BARREL,
	UAV_BARREL,
}carType_e;

typedef enum{
	TX2_STOP = 0,							                    //ֹͣ����		
	TX2_DISTINGUISH_ARMOR = 1,		                                //ʶ��װ�װ�
	TX2_DISTINGUISH_BUFF = 6, 			                            //ʶ��С��
	TX2_DISTINGUISH_BIGBUFF = 3,									//ʶ����
	TX2_KILL_SENTRY = 4,											//ɱ�ڱ�
}visionWorkMode_e;

typedef enum{
	SMALL_BULLET = 0,
	BIG_BULLET
} bulletType_e;

typedef enum{
	ENEMY_RED = 0,
	ENEMY_BLUE
} enemyType_e;

typedef enum {
	NO_STATE = 0,
	W_STATE = 1,
	S_STATE = 2,
	A_STATE = 4,
	D_STATE = 8,
}buffKey_e;

typedef struct {
	TaskHandle_t xHandleTask;	
	/*		TX2�ش�����		*/	
    formatTrans32Struct_t yawData;
    formatTrans32Struct_t pitchData;
	uint8_t distingushState;
	bool captureFlag;											//��ʶ��Ŀ��
	bool sameTargetFlag;										//ͬһĿ��ָ��
	bool getOrderFlag;											//TX2���ڹ���״̬��־
	formatTrans16Struct_t CNTR; 								//LCM: TX2�ش��Ĺߵ�UKF����λ������ΪͨѶ״̬��� visionErrorCount �������ڽǶ�ֵƥ��
																//LCM: ��ʵ����32����TX2�� serialBuffer->CNTR
	
	/*		CNTR ��ر���	*/
	uint16_t lastCNTR;
	uint16_t CNTR_DataStored[VISION_STORED_LENGTH];		        //�ߵ�UKF����λ CNTR ��ʷ����ֵ
//	f32_t pitchDataStored[VISION_STORED_LENGTH];				//pitch��Ƕ���ʷ�洢ֵ 
//	f32_t yawDataStored[VISION_STORED_LENGTH];
	uint8_t storedIndex;										//LCM: ��������ֵ���±����ֵ���������Ϊ VISION_STORED_LENGTH
	uint32_t visionErrorCount;							        //LCM: ��TX2ͨѶ״̬��飬ֱ��ʹ��CNTR
	uint32_t visionLastErrorCount;	
	uint32_t intervalNum;										//LCM: visionErrorCount �� visionLastErrorCount �Ĳ�ֵ����Ϊ0��˵����TX2ͨѶ��������		
	
	/*		��TX2	 */
	shootMode_e fireMode;								        //����ģʽ�������ؽ��գ����͵�tx2
	visionWorkMode_e distinguishMode;					                //�Ӿ�����ģʽ�������ؽ��գ����͵�tx2 
	bulletType_e bullet;								        //�ӵ����ͣ������ؽ��գ����͵�tx2
	uint8_t enemyType;									        //�з���ɫ�� �����ؽ��գ����͵�tx2
	carType_e carType;									     	//LCM: ���͸�TX2�ĳ�������    
	
	/*		shoot	*/
	uint8_t shootSpeed;											//LCM: ��ȡ��ǰ���٣������ǲ�������Ҫ������

	float judgeShootSpeed;	              //PY: ��ȡ����ϵͳ���ص�ʵ������

	/*		��̨�Ƕ���ֵ		*/
	f32_t yawReal;												//LCM: �����Ǵ����ĽǶ����ݣ����Ӿ����� visionUpdateTask ����
	f32_t pitchReal;
	
	/*		�Ӿ�����̨����		*/
	f32_t yawCmd;												//LCM: ���������Ŀ��Ƕȣ�����ڻ���ֵ����ԽǶȣ�
	f32_t pitchCmd;										        
	f32_t yawCmdBuff;									       	//LCM���ñ������и��ִ���󸳸� yawCmd 	
	f32_t pitchCmdBuff;
	f32_t manualYawBias;										//LCM: ������״̬�У���������Ȼӵ��һ������̨����Ȩ���ñ�������������Ĳ����ֿ���ָ��
	f32_t manualPitchBias;										
    
	/*��־λ*/
	bool initFlag;												//LCM: �Ӿ���ʼ����ɣ���true��û�б��õ���
	bool prejudgFlag;											//LCM: ��⿪������ģʽʱ�Ƿ�ʹ������Ҽ�����ʹ����������״̬�У���������Ȼӵ��һ������̨����Ȩ���� manualYawBias ��
	bool cailSuccess;											//LCM: �� visionUpdateTask ˢ���м���Ƿ�����һ֡���ݰ�����������true��������false��ֻ��Ϊtrueʱcmd�ſ��Գ�Ϊ��̨����


	/*	�Ȼ���ģʽ�µı���	*/	
	f32_t mortarPitCmd;
	f32_t mortarYawCmd;
	f32_t mortarPitCmdBuff;
	f32_t mortarYawCmdBuff;

	/*		����		*/
	bool miniPTZEnableAim;	
	uint32_t loops;	
	uint8_t buff_mod;			//LCM: ���ģʽ�� W S A D ΢����־λ �� buffKey_e ö������
	
} visionStruct_t;

visionStruct_t* getvisionData(void);
void visionStoreHistoricalData(f32_t pitch, f32_t yaw, uint16_t CNTR);
void visionSendDataUpdate(uint8_t workMode,uint8_t bullet);
void visionSendDataInit(void);
void identifyCamp(void);
void visionInit(void);
void shootVisionInit(void);
void visionFireFbdUpdata(uint8_t * shootFlag);
void visionMortar(void);

#endif


