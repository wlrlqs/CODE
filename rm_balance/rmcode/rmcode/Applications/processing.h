#ifndef __PROCESSING_H
#define __PROCESSING_H

#include "FreeRTOS_board.h"
#include "imu.h"
#include "parameter.h"
#include "NavAndData.h"
 
#define	GYO_SCALE  1.0f/((1<<16)/(2000*2.0f))*DEG_TO_RAD
#define	ACC_SCALE  1.0f/((1<<16)/(16*2.0f))*GRAVITY
#define	MAG_SCALE  1.0f / 187.88f

enum {
	ACCEL_X_500HZ_LOWPASS = 0,
	ACCEL_Y_500HZ_LOWPASS,
	ACCEL_Z_500HZ_LOWPASS,
};

typedef struct LowPassFilterData {
	f32_t   gx1;
	f32_t   gx2;
	f32_t   gx3;
	f32_t   previousInput;
	f32_t   previousOutput;
} lowPassFilterStruct_t;

typedef struct {
	int16_t 				accRotate[3];
	int16_t 				gyoRotate[3];
	int16_t 				magRotate[3];
	int16_t 				accOrient[3];
	int16_t 				gyoOrient[3];
	int16_t 				magOrient[3];
	int16_t 				tempOrient[1];
	lowPassFilterStruct_t 	lowPassFilterData[6];
} sensorStruct_t;

extern sensorStruct_t sensorRotate;
typedef void SENSORUpdate_t(f32_t *in,f32_t *out,uint8_t t,f32_t translate);
void sensorProcessUpdate(imusensorStruct_t *imuSensor,coordinateFloat_t *accdata,\
						 coordinateFloat_t *gyodata,\
						 coordinateFloat_t *magdata,\
						 int16_t tempreature);
void sensorProcessInit(void);
f32_t meanRecursiveFilter(int16_t newVuale,uint8_t axis);
f32_t LowPass_Filter(f32_t input, struct LowPassFilterData *filterParameters);
void initLowPassFilter(void);
void imuQuasiStatic(int n);

#endif 



