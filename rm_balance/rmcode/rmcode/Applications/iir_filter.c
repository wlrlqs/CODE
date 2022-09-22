#include "iir_filter.h"

static butterworth_4thOrderStruct_t motorEncoder_filter;
static butterworth_4thOrderStruct_t gyro_filter;

butterworth_4thOrderStruct_t *getfilter_motorEncoder(void){
	return &motorEncoder_filter;
}

butterworth_4thOrderStruct_t *getfilter_gyro(void){
	return &gyro_filter;
}

const float butterworth_100Hz_b2[5] = {
   0.004824343357716f,  0.01929737343086f,   0.0289460601463f,  0.01929737343086f,
   0.004824343357716f
};

const float butterworth_100Hz_a2[5] = {
                   1,   -2.369513007182f,    2.313988414416f,   -1.054665405879f,
     0.1873794923682f
};

const float butterworth_80Hz_b2[5] = {
    0.002234891698082f, 0.008939566792329f, 0.01340935018849f,  0.008939566792329f,
    0.002234891698082f
};

const float butterworth_80Hz_a2[5] = {
                    1,  -2.692610987017f,   2.867399109111f,    -1.403484671368f,
    0.2644548164435f
};

const float butterworth_60Hz_b2[5] = {
  0.0008063598650371f, 0.003225439460148f, 0.004838159190223f, 0.003225439460148f,
  0.0008063598650371f
};

const float butterworth_60Hz_a2[5] = {
                   1,   -3.017555238686f,    3.507193724716f,   -1.847550944119f,
     0.3708142159295f
};

const float butterworth_40Hz_b2[5] = {
  0.0001832160233696f,0.0007328640934784f, 0.001099296140218f,0.0007328640934784f,
  0.0001832160233696f
};

const float butterworth_40Hz_a2[5] = {
                   1,   -3.344067837712f,    4.238863950884f,   -2.409342856586f,
      0.517478199788f
};

float iirButterworth_2thFilter(butterworth_2thOrderStruct_t *fliter, float rawVaule) {
    int i;
    for(i = 2; i > 0; i--) {
        fliter->yBuf[i] = fliter->yBuf[i - 1];
        fliter->xBuf[i] = fliter->xBuf[i - 1];
    }
    fliter->xBuf[0] = rawVaule;
    fliter->yBuf[0] = 0;
    for(i = 1; i < 3; i++) {
        fliter->yBuf[0] = fliter->yBuf[0] + fliter->b2[i] * fliter->xBuf[i];
        fliter->yBuf[0] = fliter->yBuf[0] - fliter->a2[i] * fliter->yBuf[i];
    }
    fliter->yBuf[0] = fliter->yBuf[0] + fliter->b2[0] * fliter->xBuf[0];
    return fliter->yBuf[0];
}

float iirButterworth_4thFilter(butterworth_4thOrderStruct_t *fliter, float rawVaule) {
    int i;
    for(i = 4; i > 0; i--) {
        fliter->yBuf[i] = fliter->yBuf[i - 1];
        fliter->xBuf[i] = fliter->xBuf[i - 1];
    }
    fliter->xBuf[0] = rawVaule;
    fliter->yBuf[0] = 0;
    for(i = 1; i < 5; i++) {
        fliter->yBuf[0] = fliter->yBuf[0] + fliter->b2[i] * fliter->xBuf[i];
        fliter->yBuf[0] = fliter->yBuf[0] - fliter->a2[i] * fliter->yBuf[i];
    }
    fliter->yBuf[0] = fliter->yBuf[0] + fliter->b2[0] * fliter->xBuf[0];
    return fliter->yBuf[0];
}

void iirButterworth_2thInit(butterworth_2thOrderStruct_t *fliter, const float *a2, const float *b2) {
    memset(fliter, 0, sizeof(butterworth_2thOrderStruct_t));
    fliter->a2 = a2;
    fliter->b2 = b2;
}

void iirButterworth_4thInit(butterworth_4thOrderStruct_t *fliter, const float *a2, const float *b2) {
    memset(fliter, 0, sizeof(butterworth_4thOrderStruct_t));
    fliter->a2 = a2;
    fliter->b2 = b2;
}

void iir_filter_init(void){
	//ÔÆÌ¨yawÖáÂëÅÌÖµµÍÍ¨ÂË²¨
	iirButterworth_4thInit(&motorEncoder_filter,butterworth_80Hz_a2,butterworth_80Hz_b2);
	//imuÍÓÂİÒÇµÍÍ¨ÂË²¨
	iirButterworth_4thInit(&gyro_filter,butterworth_100Hz_a2,butterworth_100Hz_b2);
}
