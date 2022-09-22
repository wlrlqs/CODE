#ifndef __SUPERCAPACITOR_H
#define __SUPERCAPACITOR_H
#include "BSP.h"
#include "Util.h"
#include "stdint.h"
#include <stdbool.h>

#define ID_CAPACITOR    0x702	//超级电容ID
#define CHARGER         0x56   //充电
#define DISCHARGER      0x57   //放电
#define NOTHING         0x58   //保护状态什么都不干

typedef struct 
{
	uint8_t capacitorFlag;
	formatTrans8Struct_t sendData[2];
	uint8_t send1[6];
	uint8_t send2[6];
	uint8_t send3[6];
	uint8_t send4[6];
	uint8_t read1[6];

	formatTrans8Struct_t readData[2];
}capacitor_state_data_t;

typedef struct 
{
    u8 state_ing;                       //放电
    u8 charging_stop_discharging;       //充电
    u8 percentage;                      //电容电量
    u8 capacitor_status;                //电容充放电状态 
	capacitor_state_data_t capacitor_1;
	capacitor_state_data_t capacitor_2;
	capacitor_state_data_t capacitor_3;
	
	f32_t CapacitorErrorCount;  //超级电容通信计数位
	f32_t lastErrorCount;    //上一次计数值
	f32_t intervalNum;   //计数间隔
}capacitorData_t;

typedef struct{

	uint8_t Stop_discharge; //停止放电
	uint8_t charging;       //充电
	uint8_t discharge;      //放电
  uint8_t state;
	bool PressFlag;     //按键标志
}capacitance_Struct_t;

//超级电容电量
enum{
  twenty_percent  = 0x01,   //百分之二十
	forty_percent   = 0x02,   //百分之四十
	sixty_percent   = 0x03,   //百分之六十
	eighty_percent  = 0x04,   //百分之八十
	hundred_percent = 0x05,   //百分之百
};

//超级电容状态
enum{
  Charging    = 0x55,  //充电
	Discharging = 0x44,  //放电
	Protection  = 0x66,  //保护
};

capacitance_Struct_t *get_capacitance(void);//HXB

void capacitor_can1_sentData(uint32_t ID_CAN);
void capacitor_can2_sentData(uint32_t ID_CAN);
void capacitor_readData(CanRxMsg *can_rx_msg);
void super_capacitorTask(void);
void super_capacitorTask_dowm(void);
void super_capacitorTask_updowm(void);
void capacitance_cansend(uint8_t __state);//HXB


extern capacitorData_t	capacitorData;
capacitorData_t *getcapacitorData(void);
#endif

