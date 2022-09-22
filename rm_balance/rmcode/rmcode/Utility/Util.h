
#ifndef __UTIL_H
#define __UTIL_H

#include "stm32f4xx.h"
#include "FreeRTOS_board.h"
#include <stdlib.h>

#define digitalHi(p)        *p = 1
#define digitalLo(p)        *p = 0
#define digitalSet(p, n)    *p = n
#define digitalGet(p)       (*p)
#define digitalTogg(p)      *p = !(*p)
#define digitalIncreasing(p) *p += 1
#define digitalDecline(p)   *p -= 1
#define digitalClan(p)		*p = 0										//指针所指向的内容清零
#define High	1
#define Low   0
#define T_OR_F0(x)	((x)<0?(true):(false))
#define SINF_RUDD(x) ((x)<1?(1):(x))	
#define constrainInt(v, lo, hi)	    (((int)(v) < (int)(lo)) ? (int)(lo) : (((int)(v) > (int)(hi)) ? (int)(hi) : (int)(v)))
#define constrainFloat(v, lo, hi)   (((float)(v) < (float)(lo)) ? (float)(lo) : (((float)(v) > (float)(hi)) ? (float)(hi) : (float)(v)))	//LCM: 将v值限制在low与high之间

#define ABS(x) ( (x)>0?(x):-(x) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define	UTIL_STACK_CHECK	    12

#define UTIL_CCM_HEAP_SIZE	    (0x2800)	// 40KB

#define UTIL_ISR_DISABLE	    __asm volatile ( "CPSID   F\n")
#define UTIL_ISR_ENABLE		    __asm volatile ( "CPSIE   F\n")

#define yield(n)		    vTaskDelay(n)

#define PERIPH2BB(addr, bit)        ((uint32_t *)(PERIPH_BB_BASE + ((addr) - PERIPH_BASE) * 32 + ((bit) * 4)))

// first order filter		一阶滤波器
typedef struct {
    float tc;
    float z1;
} utilFilter_t;

typedef struct {
    const float *window;
    float *data;
    uint8_t n;
    uint8_t i;
} utilFirFilter_t;

//联合体用于转换数据
typedef union{
	uint8_t 		u8_temp[4];
	float float_temp;
	s32 	s32_temp;
	uint32_t		u32_temp;
} FormatTrans;

//联合体用于转换数据
typedef union{
	uint8_t  u8_temp[4];
	uint16_t u16_temp[2];
	int16_t s16_temp[2];
	float   float_temp;
	s32 	s32_temp;
	uint32_t		u32_temp;
} formatTrans32Struct_t;
typedef union{
	uint8_t 		u8_temp[8];
	uint64_t		u64_temp;
} formatTrans64Struct_t;

typedef union {
    uint8_t u8Union[8];
    uint16_t u16Union[4];
    int16_t s16Union[4];
    float fp32Union[2];
    uint32_t u32Union[2];
    int32_t s32Union[2];
    double dp64Union;
    uint64_t u64Union;
    int64_t s64Union;
} format64BitStruct_t;

typedef union {
	uint8_t u8Union[4];
    uint16_t u16Union[2];
    int16_t s16Union[2];
	float fp32Union;
	uint32_t u32Union;
    int32_t s32Union;
} format32BitStruct_t;

typedef union {
	uint8_t u8Union[2];
	int16_t s16Union;
	uint16_t u16Union;
} format16BitStruct_t;



typedef union{
	uint8_t 	u8_temp;
	struct {
		uint8_t a_temp : 1;
		uint8_t b_temp : 1;
		uint8_t c_temp : 1;
		uint8_t d_temp : 1;
		uint8_t e_temp : 1;
		uint8_t f_temp : 1;
		uint8_t g_temp : 1;
		uint8_t h_temp : 1;
	}byte;
} formatTrans8Struct_t;

typedef union{
	uint8_t 		u8_temp[2];
	int16_t 		s16_temp;
	uint16_t		u16_temp;
} formatTrans16Struct_t;

typedef struct {
	formatTrans32Struct_t errorCount;
	uint32_t lastErrorCount;
	uint32_t intervalNum;
	uint16_t waitingConnect;
} errorScanStruct_t;

extern void delay(unsigned long t);
extern void delayMicros(unsigned long t);
extern void dumpFloat(unsigned char n, float *floats);
extern void dumpInt(unsigned char n, int *ints);
extern uint16_t *aqStackInit(uint16_t size, char *name);
extern void *aqCalloc(size_t count, size_t size);
extern void aqFree(void *ptr, size_t count, size_t size);
extern void *aqDataCalloc(uint16_t count, uint16_t size);
extern float removeDeadBand(float rcValue,float deadBand);
extern void utilFilterInit(utilFilter_t *f, float dt, float tau, float setpoint);
extern void utilFilterInit2(utilFilter_t *f, float dt, float tau, float setpoint);
extern void utilFilterInit3(utilFilter_t *f, float dt, float tau, float setpoint);
extern float utilFilter(utilFilter_t *f, float signal);
extern float utilFilter2(utilFilter_t *f, float signal);
extern float utilFilter3(utilFilter_t *f, float signal);
extern void utilFilterReset(utilFilter_t *f, float setpoint);
extern void utilFilterReset3(utilFilter_t *f, float setpoint);
extern int ftoa(char *buf, float f, unsigned int digits);
extern float utilFirFilter(utilFirFilter_t *f, float newValue);
extern void utilFirFilterInit(utilFirFilter_t *f, const float *window, float *buffer, uint8_t n);
#ifdef UTIL_STACK_CHECK
extern uint16_t stackFrees[UTIL_STACK_CHECK];
extern uint16_t utilGetStackFree(const char *stackName);
#endif
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);

#endif


