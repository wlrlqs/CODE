#ifndef __CANDSEND_H
#define __CANDSEND_H
#include "BSP.h"
#include "Util.h"
#include "Driver_RMMotor.h"
#include "Driver_DMMotor.h"
#include "local_id.h"
#include "DRIVER_VL53L0X.h"
#include "pneumatic.h"
#include "stdbool.h"
#include "Driver_RMDMotor.h"
#include "supercapacitor.h"
#define ROBOT	robotConfigData.typeOfRobot
#pragma pack(1)
typedef struct {
	motorSerialNumber_t can1_0x2FF;
	motorSerialNumber_t can1_0x200;
	motorSerialNumber_t can1_0x1FF;
    motorSerialNumber_t can2_0x2FF;
	motorSerialNumber_t can2_0x200;
	motorSerialNumber_t can2_0x1FF;
	RMDLCANSendStruct_t can1_0x141;
	RMDLCANSendStruct_t can1_0x142;
	canUserData_t 		can1_0x401;
	vl53l0x_state_data_t   can1_0x501;	
	pneumatic_state_data_t can2_0x401;
	bool canForward;
	uint32_t loops;
} canSendStruct_t;
#pragma pack()
void canSendUpdate(void);
void canSendInit(void);

canSendStruct_t* getcanSendData(void);

#endif
