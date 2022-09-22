#ifndef __SLAVE_SENSOR_H
#define __SLAVE_SENSOR_H

#include "Util.h"
#include <stdbool.h>
#include "Driver_imuSC.h"

#define IMU_MODULAR_BEGIN				0xAD
#define IMU_MODULAR_ADDRESS				0x40


//#define CONTROL_ADDRESS					0x10
#define MASTERBOARD_BEGIN				0xAA
#define SLAVERBOARD_BEGIN				0xAB

#define VISION_BEGIN					0xD4
#define VISION_ADDRESS					0x20

#define CHASSIS_CONTROL_BEGIN			0x3F
#define CHASSIS_CONTROL_PACK_CLASS		2
//帧头长度
#define BUFFER_HEADER_LEN 4
//存放包长度成员下标
#define BUFFER_LEN_MENNUM 2

#define FRE_250 	  2
enum {
	TRANS_ONLY_INS = 0x00,
	TRANS_ADD_ANGLE = 0x01,
	TRANS_ADD_VISION = 0x02
};
enum {
	CHASSIS_DEVICE_JUDGE_1 			= 0x01,			    	//裁判系统数据
	CHASSIS_DEVICE_JUDGE_2 			= 0x02,			    	//裁判系统数据
	CHASSIS_DEVICE_SENSOR			= 0x04,        			//底盘传感器反馈
	CHASSIS_DEVICE_OTHER 			= 0x08,			   	 	//其他反馈
    CHASSIS_DEVICE_INIT             = 0x10,                 //初始化的标志发送
};
enum {
	CONTROL_DEVICE_DEFF_REF 		= 0x01,				    //底盘变形电机期望
	CONTROL_DEVICE_OTHER 			= 0x02,			   	 	//其他控制期望
	CONTROL_DEVICE_RC				= 0x04,	
	CONTROL_DEVICE_CHASE_REF		= 0x08,
	CONTROL_DEVICE_ENCODER			= 0x10,	
    CONTROL_DEVICE_INIT             = 0X20,                 //初始化的设备信息发送
};
enum {
	FRE_POSITION_250F = 1,
	FRE_POSITION_250B,
};


typedef struct{
	uint32_t slaveErrorCount;
	uint32_t slaveLastErrorCount;
	uint32_t intervalNum;	
	bool initFlag;
	uint8_t canForwardIndexPtr;
	
	formatTrans16Struct_t pitchCmd;
	formatTrans16Struct_t yawCmd;
	formatTrans16Struct_t pitchRecv[3];
	formatTrans16Struct_t yawRecv[3];
	
	formatTrans16Struct_t fricWheelCmd[2];
	formatTrans16Struct_t fricWheelRecv[2][3];
	
} slaveSensorStruct_t;

typedef struct{
	uint8_t seq;
	formatTrans32Struct_t pitchGyroAngle;
	formatTrans32Struct_t yawMotorAngle;
	formatTrans32Struct_t pitchAngleRef;
	formatTrans32Struct_t yawAngleRef;
	uint8_t otherRcValue[18];
	uint8_t otherRcReadly;
	uint8_t otherMode;
	bool otherCaptureFlag;
	bool otherEnemyType;
	bool otherfricMotorFlag;
	bool otherAutoMaticFlag;
	bool otherPatrolMode;
	bool othersenorFlag0;
	bool othersenorFlag1;
} controlTransStruct_t;

typedef struct _serialStruct_t{
	void (*Send)		(USART_TypeDef *USARTx,struct _serialStruct_t *serialBuffer);	//LCM: 发送函数
	void (*update)		(uint8_t *bufferDecode);	//LCM: 更新函数(处理数据)
	void (*Receive )  	(void);	//LCM: 接收函数（暂存数据）
	uint8_t header_length;		//LCM: 数据包头长度
	uint8_t length_position;	//LCM: 携带数据包长度的成员的下标
	uint8_t recepackClass;		
	uint8_t sendpackClass;
	formatTrans16Struct_t CNTR;
	buffefLoopStruct_t bufferLoop;
	errorScanStruct_t serialError;
}serialStruct_t;

typedef struct{
	void (*Send)		(uint8_t *array);
	void (*Upload)		(uint8_t *bufferDecode);
	
	uint8_t recepackClass;
	uint8_t sendpackClass;
	formatTrans16Struct_t CNTR;
	errorScanStruct_t serialError;
}CANFDStruct_t;

extern slaveSensorStruct_t slaveSensorData;
extern controlTransStruct_t controlTransData;
extern controlTransStruct_t slaveTransData;
//Vision Data
extern serialStruct_t visionSerial;
//smallGimbal Data
extern serialStruct_t slaverBoardSerial;
//sentry Data
extern serialStruct_t masterBoardSerial;
//Main Control Data
extern CANFDStruct_t controlSerial;
//Chassis Data
extern CANFDStruct_t chassisSerial;
extern CANFDStruct_t initSerial;

uint8_t controlPackFreDiv(uint8_t transferdevice,uint8_t *array,uint8_t index_ptr);
uint8_t controlPackReceive(uint8_t transferdevice,uint8_t packClass,uint8_t *buffdecode, uint8_t index_ptr);
uint8_t chassisPackFreDiv(uint8_t transferdevice,uint8_t *array,uint8_t index_ptr);
uint8_t chassisPackReceive(uint8_t transferdevice,uint8_t packClass,uint8_t *buffdecode, uint8_t index_ptr);

void chassisReceive(void);
void controlReceive(void);
void Driver_WT931Read(uint8_t *arrayWT931Receive , uint16_t *RxCount);
void Driver_WT931ReadDMA(uint8_t *arrayWT931Receive);
void slaveSensorRead(void);//黄锡标
void adis16470DataUpdate(void);
void moduleCommandUpload(USART_TypeDef *USARTx);
void otherControlRead(uint8_t *arrayOtherControl);
void slaveSensorConfig(void);
void mixControlCommand(uint8_t* array , controlTransStruct_t * dataStruct);
void otherControDataDecoding(uint8_t array, controlTransStruct_t * dataStruct);
#endif
