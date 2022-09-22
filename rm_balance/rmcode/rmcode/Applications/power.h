#ifndef __POWER_H
#define __POWER_H

#include "bsp.h"
#include "Driver_powerContol.h"
#include "stdbool.h"

#define openCap   (remoteControlData.dt7Value.keyBoard.bit.SHIFT)
#define chargeVol (remoteControlData.dt7Value.keyBoard.bit.X)

#define LINK_CAP	    PDout(14)
#define LINK_CHASSIS	PDout(15)

#define autocharge

#define average_times_cap           5     //≤…µÁ»›µÁ—π
#define capChannel 					1

enum linkChassisState{
	CHASSIS_ON = 0,
	CHASSIS_OFF = 1
};

enum linkCapState{
	CAP_OFF = 0,
	CAP_ON = 1
};

typedef enum{
	linkCap = 0,
	linkCha 
}linkWay_e;

typedef struct{
	linkWay_e 	SwitchPolicy;
	f32_t 		capVolMax;
	f32_t 		capVol;
	f32_t 		standardVol;
	f32_t   	powerLimit;         //
	f32_t   	warningPower;
	f32_t   	WarningPowerBuff;
	f32_t   	noJudgeTotalCurrentLimit;
	f32_t   	judgeTotalCurrentLimit;
	f32_t   	addPowerCurrent;
	f32_t   	volPIn;
	f32_t   	volCap;
	uint8_t		initFlag;
	uint32_t 	loops;
	uint8_t 	step;
	bool    	linkCapFlag;
	bool    	chargeFlag;
	bool		CapError;
	bool    	rotateFast;
}powerstruct_t;	

powerstruct_t* getpowerData(void);
	
void sendPowerDataInit(void);
void powerDataUpdata(void);
void linkAimUpdata(void);
void powerLinkInit(void);
void adcInit_cap(void);
void powerDataReceive(uint8_t *array);
void powerDataTransportTask(void *Parameters);

#endif
