#ifndef __DIFFERENTIAL_CALCULATION_H
#define __DIFFERENTIAL_CALCULATION_H

#include "util.h"

typedef struct{
	int16_t inputBuffer;
	int16_t	outputBuffer;
}dataAcquisitionBufStruct_t;

typedef struct{
	f32_t *yCoefficient;
	f32_t *xCoefficient;
}coefficientStruct_t;

typedef struct{
	uint8_t differentialLength;      //²î·Ö³¤¶È
	f32_t 	*inputData;
	f32_t 	*outputData;
	uint8_t	initFlag;
	coefficientStruct_t *coefficient;
}differentialDataStruct_t;


f32_t differentialCal(differentialDataStruct_t *data,f32_t currentData);
differentialDataStruct_t *differentialInit(f32_t *coefficientData,uint8_t length);
void dataAcquisition(f32_t *inputData,f32_t outputData,f32_t amplitude);
void dataAcquisitionInit(void);

extern f32_t transferData[8];

#endif
