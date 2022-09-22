#ifndef __MOTORHANDLE_H
#define __MOTORHANDLE_H
#include "board.h"
#define ENCODER_ANGLE_RATIO14    (360.0f/16384.0f)  //14Î»±àÂëÆ÷
#define ENCODER_ANGLE_RATIO13	 (360.0f/8192.0f)	//13Î»±àÂëÆ÷
typedef struct{
	f32_t 		(*Position)		(motorConfigStruct_t *motorConfig,int16_t centerOffSet);
	int8_t		(*Temperature)	(motorConfigStruct_t *motorConfig);
	int16_t 	(*Current)		(motorConfigStruct_t *motorConfig);
	int16_t		(*Speed)		(motorConfigStruct_t *motorConfig);
	uint16_t	(*Encoder)		(motorConfigStruct_t *motorConfig);
	void		(*Servo)		(uint32_t *inputPWM);
}motorHandleStruct_t;
extern motorHandleStruct_t motorHandleClass;



#endif
