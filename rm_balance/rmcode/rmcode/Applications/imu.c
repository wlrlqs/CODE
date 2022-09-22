#include "imu.h"
#include "supervisor.h"
#include "app_Init.h"
#include "Driver_MPU6500.h"
#include "processing.h"
#include "Driver_USBVCP.h"
#include "driver_icm42605.h"
/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

imuStruct_t imuData;
imusensorStruct_t imuSensorOfChassisData;
uint8_t gyroFsr = ICM42605_FSR_2000DPS;
uint8_t accFsr = ICM42605_FSR_16G;

imuStruct_t *getimuData(){
	return &imuData;
}

bool moduleUpdateCheck(void){
	bool res;
	if(imuData.CNTR.u16_temp != imuData.lastCNTR)
		res = true;
	else
		res = false;
	imuData.lastCNTR = imuData.CNTR.u16_temp;
	return res;
}

//����IMU����
void imuChassisUpdate(void){
//	imu_data_read();
	digitalIncreasing(&imuSensorOfChassisData.loops); 
}
void imuInit(void){
	memset((void *)&imuData, 0, sizeof(imuData));
	//����ʹ��IMU SC
	if(BOARD_TYPE == BOARD_CONTROL){
		//imuBroadcastInit();
	}
	//����ʹ��BMI088
	else if(BOARD_TYPE == BOARD_CHASSIS){
		driver_Icm42605_Init(&gyroFsr,&accFsr);
	}
		
	imuSensorOfChassisData.initFlag = true;
}


