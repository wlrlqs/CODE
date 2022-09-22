#ifndef __WIRELESS_H
#define __WIRELESS_H

#include "Driver_DTU.h"
#include "FreeRTOS_board.h"
#include "BSP.h"
typedef struct{
	uint8_t sendCheck;
	uint8_t sendVersion;
	uint8_t sendStatus;
	uint8_t sendSenser;
	uint8_t sendSenser2;
	uint8_t sendPid1;
	uint8_t sendPid2;
	uint8_t sendPid3;
	uint8_t sendPid4;
	uint8_t sendPid5;
	uint8_t sendPid6;
	uint8_t sendRcData;
	uint8_t sendOffSet;
	uint8_t sendMotoPwm;
	uint8_t sendPower;
	uint8_t sendUser;
	uint8_t sendSpeed;
	uint8_t sendLocation;
}dt_flag_t;

typedef struct{
	TaskHandle_t xHandleTask;
	uint8_t imuCalibrate;
	uint8_t magCalibrate;
	uint32_t loops;
	uint8_t dataNeedSend[50];
	uint8_t checkDataNeedSend;
	uint8_t checkSumNeedSend;
}wirelessStruct_t;

extern dt_flag_t f;
extern wirelessStruct_t wirelessData;

#define WIRELESS_PRIORITY	    6
#define WIRELESS_STACK_SIZE	  256
#define WIRELESS_PERIOD				5

extern TaskHandle_t xHandleTaskWireless;

void ANO_DT_Data_Exchange(void);
void ANO_DT_Send_Status(f32_t angle_rol, f32_t angle_pit, f32_t angle_yaw, s32 alt, uint8_t fly_model, uint8_t armed);
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z);
void ANO_DT_Send_Senser2(s32 bar_alt,uint16_t csb_alt);
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6);
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8);
void ANO_DT_Send_PID(uint8_t group,f32_t p1_p,f32_t p1_i,f32_t p1_d,f32_t p2_p,f32_t p2_i,f32_t p2_d,f32_t p3_p,f32_t p3_i,f32_t p3_d);
void ANO_DT_Send_User(void);
void ANO_DT_Send_Speed(f32_t x_s,f32_t y_s,f32_t z_s);
void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver);
void ANO_DT_Data_Receive_Prepare(uint8_t data);
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
void dataSendToGroundStation(void);
void dataTransportTask(void *Parameters);
void wirelessInit(void);

#endif


