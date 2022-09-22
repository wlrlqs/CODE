#ifndef __IIR_FILTER_H
#define __IIR_FILTER_H

#include "stm32f4xx.h"
#include "std_lib.h"

typedef struct {
    const float *a2;
    const float *b2;
    float xBuf[3];
    float yBuf[3];
} butterworth_2thOrderStruct_t;

typedef struct {
    const float *a2;
    const float *b2;
    float xBuf[5];
    float yBuf[5];
} butterworth_4thOrderStruct_t;

float iirButterworth_2thFilter(butterworth_2thOrderStruct_t *fliter, float rawVaule);
float iirButterworth_4thFilter(butterworth_4thOrderStruct_t *fliter, float rawVaule);
void iirButterworth_2thInit(butterworth_2thOrderStruct_t *fliter, const float *a2, const float *b2);
void iirButterworth_4thInit(butterworth_4thOrderStruct_t *fliter, const float *a2, const float *b2);
butterworth_4thOrderStruct_t *getfilter_motorEncoder(void);
butterworth_4thOrderStruct_t *getfilter_gyro(void);
void iir_filter_init(void);

extern const float butterworth_100Hz_b2[5];
extern const float butterworth_100Hz_a2[5];
extern const float butterworth_80Hz_b2[5];
extern const float butterworth_80Hz_a2[5];
extern const float butterworth_60Hz_b2[5];
extern const float butterworth_60Hz_a2[5];
extern const float butterworth_40Hz_b2[5];
extern const float butterworth_40Hz_a2[5];


#endif
