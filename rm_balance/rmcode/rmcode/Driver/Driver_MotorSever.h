#ifndef __DRIVER_MOTORSEVER_H
#define __DRIVER_MOTORSEVER_H
#include "driver.h"
#include "canSend.h"
#include "config.h"
#include "shoot.h"
#include "chassis.h"
#include "gimbal.h"

#define NO_IDENTIFIER 	0x00
#define NO_TYPE 		NULL
#define NO_INSTURN 		NO_TYPE
#define STANDBY			0
#define DEAFULT_NET		NET1
#define DEAFULT_CAN_NET CAN1_0X1FF_NET
#define DEAFULT_FREQ	INCREASING_500HZ
#define NO_PARAMETER	0

#define CONTROL_NET		NET1
#define CHASSIS_NET		NET2

/*闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�兘鏁撻�?銈嗗閼婚亶鏁撳鏃傤暜閹�?�兘鏁撻�?銈嗗闁跨喐�?�?敂鍏煎闁跨喐鏋婚幏鐑芥晸閿燂拷*/
#define ARM_WHEEL			OUTFIT1
#define SYNCHRONOUS_WHEEL	OUTFIT2
/*闁跨喐鏋婚幏鐤闁跨喐顢欑喊澶�?闁跨喐鏋�?�幏鐑芥晸閹恒儻璁ｉ幏鐑芥晸閺傘倖瀚归柨鐕傛�??*/
#define HERO_FRICLMOTOR		OUTFIT1
#define HERO_FRICRMOTOR		OUTFIT2
#define SLAVE_POKEMOTOR		OUTFIT3
#define SLAVE_SUPPLY		OUTFIT4
/*闁跨喐鏋婚幏鐑芥晸閺佹瑧顣幏鐑芥晸閺傘倖瀚归柨鐔稿�?�閿斿吋瀚归柨鐔告灮閹�?�兘鏁撻敓锟�?*/
#define AUX_CLAW       OUTFIT1
#define AUX_L_ELE      OUTFIT2
#define AUX_R_ELE      OUTFIT3
//#define AUX_X_SWY      OUTFIT4
//#define AUX_Y_SWY      OUTFIT5
#define AUX_L_REV      OUTFIT4
#define AUX_R_REV      OUTFIT5
/*闁跨喕濡幉瀣闁跨喐鏋�?�幏鐑芥晸閺傘倖瀚归柨鐔稿�?�閿斿吋瀚归柨鐔告灮閹�?�兘鏁撻敓锟�?*/
#define STRY_MOVE_DIRECT    OUTFIT1

enum {
	YAWMOTOR = 0,
	PITCHMOTOR,
	ROLLMOTOR,
	POKEMOTOR,    
	SUPPLYMOTOR,
	FRICLMOTOR,
    FRICRMOTOR,
	CHASSISMOTOR_RF,     //7
	CHASSISMOTOR_LF,
    CHASSISMOTOR_L,		//9
	CHASSISMOTOR_R,		//10
	CHASSISMOTOR_LB,
	CHASSISMOTOR_RB,
	OUTFIT1,
	OUTFIT2,
	OUTFIT3,
	OUTFIT4,
	OUTFIT5,
	USE_LIST,
};

enum {
	RF_WHEEL = 0,
	LF_WHEEL,
	LB_WHEEL,
	RB_WHEEL,
	NUM_OF_WHEEL,
};

enum {
	FRIC_L = 0,
	FRIC_R,
	DIR_OF_FRIC,
};

enum {
	OMNI_L = 0,
	OMNI_R,
	DIR_OF_OMNI,
};
enum {
	ELE_L = 0,
	ELE_R,
	DIR_OF_ELE,
};

enum {
	SWY_X = 0,
	SWY_Y,
	DIR_OF_SWY,
};

enum {
	REV_L = 0,
	REV_R,
	DIR_OF_REV,
};
enum {
	DRAG_L = 0,
	DRAG_R,
	DIR_OF_DRAG,
};
/**************ver2.0**************/
//闁跨喐鏋婚幏鐑芥晸鏉堝啰顣幏鐑芥晸閿燂�?
typedef enum{
	NO_MOTOR = 0,
	RM_MOTOR,
	GM_MOTOR,
	RMD_MOTOR,
	M1502A_MOTOR,
}motorType_e;
//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�兘鏁撻�?锛勵暜閹风兘鏁撻敓锟�?
typedef enum{
	INCREASING_500HZ = 0,
	INCREASING_250HZ_F,
	INCREASING_250HZ_B,
}motorIncreasing_e;
typedef enum{
	MOD_500HZ = 1,
	MOD_250HZ,
}motorMod_e;
//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹疯渹缍呴柨鐔告�?閹凤�?
typedef enum{
	ENCODER_13 = 8192,
	ENCODER_14 = 16384,
}encoderType_e;
//CAN闁跨喐鏋婚幏鐑芥晸閻ｅ瞼绱﹂柨鐔告灮閹�?�兘鏁撻柧鐗堝敾閹凤�?
typedef enum{
	CAN1_0X1FF_NET = 0,
	CAN1_0X200_NET,
	CAN1_0X2FF_NET,
	CAN1_0X141_NET,
	CAN1_0X142_NET,
	CAN2_0X1FF_NET,
	CAN2_0X200_NET,
	CAN2_0X2FF_NET,
	CAN2_0X141_NET,
	CAN2_0X142_NET,
	CAN_NET_LIST,
}CAN_NetType_e;
typedef enum{
	NET1 = 0,
	NET2,
	NET_LIST,
}Net_Code_e;
/***********************************/
#pragma pack(1)

typedef struct{
	f32_t *current[4];
}CAN_SendForm;

typedef struct{
    int8_t temperature;
    int16_t currunt;
    int16_t motorSpeed;
    uint16_t motorEncoder;
	uint8_t error;
	uint8_t mode;
}CAN_RevForm;
/**************ver2.0**************/
//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�兘鏁撻�?銈嗗闁跨喕绶濋敓锟�?
typedef struct{
	//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�兘鏁撻敓锟�?
	motorType_e 		motor_type;
	//闁跨喐鏋婚幏鐑芥晸閻欙紕顣幏鐑芥晸閿燂�?
	motorIncreasing_e 	motor_frequency;
	//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�兘鏁撻�?銈嗗闁跨喐鏋�?�幏锟�
	uint16_t			motor_encoder;
}motorLibraryStruct_t;
//闁跨喐鏋婚幏鐑芥晸閻欙紕顣幏铚傜矆闁跨喐鏋婚幏鐑芥晸閿燂拷
typedef struct{
	//loops increse
	uint8_t 			loopIncresing;
	//loops remainder
	uint8_t 			loopMod;	
}motorFreqStruct_t;
//CAN闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
typedef struct{
	CAN_NetType_e	can_net;
	Net_Code_e		board_net;
}canNetStruct_t;
/***********************************/
typedef struct  motorConfigStruct {
	//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�兘鏁撻�?銈嗗闁跨喕绶濋敓锟�?
	motorLibraryStruct_t		motor_lib;
	motorFreqStruct_t			motor_frequency_parameter;
	//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�兘鏁撻敓锟�?(CAN1,CAN2)
	CAN_TypeDef* 				motorType;
	//闁跨喐鏋婚幏鐑芥晸闁板灚鍞�?�幏鐤槕闁跨喐鏋�?�幏锟�
	uint16_t 					send_identifier;
	//闁跨喐鏋婚幏鐑芥晸缁夊憡鍞�?�幏鐤槕闁跨喐鏋�?�幏锟�
	uint16_t					receive_identifier;
	//CAN闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�兘鏁撻敓锟�?
	canNetStruct_t				can_net_parameter;
	//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔活潡閻ц�?�鍩￠敓锟�			
    CAN_RevForm*  				motor_staus;
	//闁跨喐鏋婚幏鐑芥晸闂�?泛鍩￠濠冨闁跨喐鏋�?�幏閿�??稏闁跨喐鏋�?�幏鐑芥晸閿燂拷
	void (*motorUpdate) 		(struct motorConfigStruct* motorConfig);
	//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�柉�?傞柨鐔告�?閹�?�柉�?�?柨鐔告灮閹�?�兘鏁撻敓锟�?
	void (*motorRev)			(struct motorConfigStruct* motorConfig,CanRxMsg *CanRevData);
	//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔稿�?�閿斿吋瀚归柨鐔告灮閹凤�?
	f32_t						*currentOut;
	//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹�?�兘鏁撻梼璺哄煛椤掑﹥瀚归柨鐔告灮閹�?�兘鏁撴�?鍖℃�?
	uint32_t 					errorCount;
}motorConfigStruct_t;

typedef struct {
	void (*Send) 	(void);
	void (*Receive) (motorConfigStruct_t *motorUsing,CanRxMsg *CanRevData);
}motorSeverFuncClass;

#pragma pack()
void M1502A_MotorUpdata(motorConfigStruct_t *motorConfig);
void M1502A_MotorReviece(motorConfigStruct_t *motorConfig,CanRxMsg *CanRevData);
void RMD_MotorUpdata(motorConfigStruct_t *motorConfig);
void RMD_MotorReviece(motorConfigStruct_t *motorConfig,CanRxMsg *CanRevData);
void RM_MotorUpdata(motorConfigStruct_t *motorConfig);
void RM_MotorReviece(motorConfigStruct_t *motorConfig,CanRxMsg *CanRevData);
void equipMotorCurrent(motorConfigStruct_t* motor,f32_t *current);
void motorCurrentDective(motorConfigStruct_t* motor);
void freqDective(motorConfigStruct_t* motor);
void encoderDective(motorConfigStruct_t* motor);
extern f32_t RMDcmd;
extern Net_Code_e board_net;
extern motorSeverFuncClass motorSeverClass;
extern motorConfigStruct_t commandoMotorConfig[USE_LIST];
#endif
