#ifndef __DRIVER_M1502AMOTOR_H
#define __DRIVER_M1502AMOTOR_H

#include "stm32f4xx.h"
#include "driver.h"
#include "Util.h"


enum {
	M1502A_VOLTAGE  = 0,
	M1502A_CURRENT  = 1,
	M1502A_SPEED    = 2,
	M1502A_POSITION = 3,
};

/*PY：M1502A电机CAN发送结构体*/
typedef struct {
	int16_t data[8];
}M1502ACANSendStruct_t;
/*PY：M1502A电机状态反馈结构体*/
typedef struct {
	uint16_t  motorSpeed;
	uint16_t  motorCurrent;
	uint16_t  motorDis;
	uint8_t   motorError;
	uint8_t   motorMode;
}M1502AStateStruct_t;

extern M1502ACANSendStruct_t M1502ASend;

void M1502A_MotorUpdata(motorConfigStruct_t *motorConfig);
void M1502A_MotorReviece(motorConfigStruct_t *motorConfig,CanRxMsg *CanRevData);
void M1502A_MotorInit(uint8_t mode);
void M1502A_MotorStop(void);
//void M1502AMotorOpenCtrl(CAN_TypeDef *CANx,uint16_t ID_CAN,M1502ACANSendStruct_t* CanSendData);
//void M1502AMotorCurrentCtrl(CAN_TypeDef *CANx,uint16_t ID_CAN,M1502ACANSendStruct_t* CanSendData);
//void M1502AMotorSpeedCtrl(CAN_TypeDef *CANx,uint16_t ID_CAN,M1502ACANSendStruct_t* CanSendData);
//void M1502AMotorPositionCtrl(CAN_TypeDef *CANx,uint16_t ID_CAN,M1502ACANSendStruct_t* CanSendData);
#endif
