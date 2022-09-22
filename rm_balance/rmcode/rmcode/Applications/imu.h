#ifndef __IMU_H
#define __IMU_H

#include "imu.h"
#include "parameter.h"
#include "stdbool.h"
#include "NavAndData.h"
#include "Driver_WT931.h"
#include "Driver_imuSC.h"

enum{
	IMU_CAIL_NO_NEED = 0,
	IMU_CAIL_START ,
	IMU_CAIL_BEING ,
	IMU_CAIL_FINISH ,
};

typedef struct{
	f32_t x;
	f32_t y;
	f32_t z;
}coordinateFloat_t;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}coordinateInteger_t;

typedef struct{
	formatTrans16Struct_t x;
	formatTrans16Struct_t y;
	formatTrans16Struct_t z;
} coordinateUnion_t;

/* UNION Data 	   --------------------------------*/
typedef union{
	int16_t value;
	uint8_t bytes[2];
} int16AndUint8_t;

typedef union{
	int32_t value;
	uint8_t bytes[4];
} int32AndUint8_t;

typedef union{
	uint16_t value;
	uint8_t  bytes[2];
} uint16AndUint8_t;

typedef struct {
	int16AndUint8_t 	originalAccel[3];
	int16AndUint8_t 	originalGyro[3];
	int16AndUint8_t 	originalMag[3];
	int16AndUint8_t 	originalTemperature;
	f32_t 				rawAcc[3];
	f32_t 				rawGyo[3];
	f32_t 				rawMag[3];
	volatile f32_t 		acc[3];
	volatile f32_t 		temp;
	volatile f32_t 		gyo[3];
	volatile f32_t 		mag[3];
	f32_t 				accBIAS[3];
	f32_t 				gyoBIAS[3];
	f32_t 				magBIAS[3];
	f32_t 				expTemp;
	volatile uint8_t	accTare;
	uint32_t 			imuTareLoop;
	uint8_t 			state;
	double 				time[2];
	f32_t 				intervalTime;
	bool 				initFlag;
	uint32_t 			loops;
}imusensorStruct_t;

typedef struct {
	formatTrans32Struct_t 	pitch,yaw,roll;
	formatTrans16Struct_t 	gyo[3];
	formatTrans16Struct_t 	CNTR;
	formatTrans16Struct_t		pitch_an,roll_an;
	f32_t 					lastCNTR;
	f32_t 					sinRot, cosRot;
  f32_t           gyroF32[3];//»ÆÎý±ê
	uint32_t 				fullUpdates;
	uint32_t 				halfUpdates;
} imuStruct_t;
#define IMU_HERO 0
/*»ÆÎý±ê  ADD  ADIS16470*/
#if IMU_HERO	
#define IMU_RATEX		    imuData.gyroF32[1]
#define IMU_RATEY		    imuData.gyroF32[0]
#define IMU_RATEZ		    imuData.gyroF32[2]

#define SC_PITCH				imuData.pitch_an.s16_temp*0.01
#define SC_ROLL		      imuData.roll_an.s16_temp*0.01
#define SC_YAW		      imuData.yaw.s32_temp*0.01

#else
/*SCÍÓÂÝÒÇ*/
#define IMU_RATEX		(f32_t)getScimuData()->gyro[0]
#define IMU_RATEY		(f32_t)getScimuData()->gyro[1]
#define IMU_RATEZ		(f32_t)getScimuData()->gyro[2]

#define SC_PITCH		(f32_t)getScimuData()->euler[0]
#define SC_ROLL			(f32_t)getScimuData()->euler[1]
#define SC_YAW			(f32_t)getScimuData()->euler[2]
#endif
extern imuStruct_t imuData;
extern imusensorStruct_t imuSensorOfChassisData;

void imuChassisUpdate(void);
void imuSensorReady(void);
void imuInit(void);

#endif




