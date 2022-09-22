#include "imu.h"
#include "supervisor.h"
#include "app_Init.h"
#include "Driver_MPU6500.h"
#include "processing.h"
#include "Driver_USBVCP.h"
#include "driver_icm42605.h"
/*--------------------------------------------- 
-----------------------------------------------
//		警告：公共代码区域，故障责任重大		 //
//			除特定区域可以自由可以更改外，		 //
//	其余部分更改，请与公共区域负责人取得更改权限  // 
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

//底盘IMU接收
void imuChassisUpdate(void){
//	imu_data_read();
	digitalIncreasing(&imuSensorOfChassisData.loops); 
}
void imuInit(void){
	memset((void *)&imuData, 0, sizeof(imuData));
	//主控使用IMU SC
	if(BOARD_TYPE == BOARD_CONTROL){
		//imuBroadcastInit();
	}
	//底盘使用BMI088
	else if(BOARD_TYPE == BOARD_CHASSIS){
		driver_Icm42605_Init(&gyroFsr,&accFsr);
	}
		
	imuSensorOfChassisData.initFlag = true;
}


