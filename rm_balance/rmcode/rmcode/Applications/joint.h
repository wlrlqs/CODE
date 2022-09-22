#ifndef __JOINT_H
#define __JOINT_H

#include "Util.h"
#include <stdbool.h>
#include "Driver_imuSC.h"

typedef union{
	uint8_t  u8_temp[72];
	uint32_t		u32_temp[9];
}crcStruct_t;

typedef struct _jointSendStruct_t{
	uint8_t    motorID;
	uint8_t    mode;
	uint8_t    modifyBit;
	uint8_t    readBit;
	uint8_t    reserved;
	uint32_t   modify;
	int16_t    torque;
	int16_t    speed;
	int32_t    position;
	uint16_t   position_Kp;
	uint16_t   speed_Kp;
	uint8_t    cmdIndex;
	uint8_t    cmdByte;
	uint32_t   res;
}jointSendStruct_t;

typedef struct _jointStateStruct_t{
	uint8_t    motorID;
	uint8_t    mode;
	uint8_t    temperature;
	uint8_t    error;
	int16_t    torque;
	int16_t    speed;
	int16_t    accSpeed;
	int32_t    position;
	f32_t      xGyro;
	f32_t      yGyro;
	f32_t      zGyro;
	f32_t      xAcc;
	f32_t      yAcc;
	f32_t      zAcc;
}jointStateStruct_t;

typedef struct {
	uint8_t  lastCount;
	uint8_t  nowCount;
}jointReceiveStruct_t;

typedef struct {
	uint8_t  initFlag;
	
	jointSendStruct_t   send[4];
	
	jointStateStruct_t  motor[4];
}jointDataStruct_t;

typedef struct _jointSerialStruct_t{
	void (*Send)		(USART_TypeDef *USARTx,struct _jointSendStruct_t *sendDatas);
	void (*update)		(uint8_t *bufferDecode,uint8_t usartX);	
	void (*Receive )  	(void);		
	uint8_t sendFlag;
	uint8_t sendLoop;
	uint8_t receiveFlag;
	uint8_t revSchedule;
	uint8_t buffer[255];
	formatTrans16Struct_t CNTR;
	errorScanStruct_t serialError;
}jointSerialStruct_t;

extern jointDataStruct_t jointData;
extern jointSerialStruct_t jointSerial;
extern jointReceiveStruct_t jointRevStatus;

void jointInit(void);
uint32_t crc32_core(uint32_t* ptr, uint32_t len);

#endif
