#ifndef __DRIVER_H
#define __DRIVER_H

#include "Driver_ADC.h"
#include "Driver_RMDT7.h"
#include "Driver_DTU.h"
#include "Driver_MPU6500.h"
#include "Driver_Motor_Dshot.h"
#include "Driver_SK6812.h"
#include "Driver_beep.h"
#include "Driver_Flow.h"
#include "Driver_RMMotor.h"
#include "DRIVER_VL53L0X.h"
#include "Driver_ADC.h"
#include "Driver_USBVCP.h"
#include "Driver_Power.h"
#include "Driver_Slave_Sensor.h"
#include "Driver_DMMotor.h"
#include "Driver_MotorSever.h"
#include "Driver_Servo.h"
#include "Driver_imuSC.h"
#include "Driver_icm42605.h"
#include "Driver_unitreeMotor.h"
/*初始化类结构体*/
typedef struct{
	void (*Init) 	(void);
}deviceInitClass;
/*时钟类*/
extern deviceInitClass clockClass;
/*ADC类*/
extern deviceInitClass adcClass;
/*外饰类*/
extern deviceInitClass sightClass;
/*数传类*/
extern deviceInitClass DTUClass;
/*陀螺仪类*/
extern deviceInitClass IMUIintClass;
/*遥控器类*/
extern deviceInitClass DT7IintClass;
/*CAN电机类*/
extern deviceInitClass motorSeverInitClass;
/*舵机类*/
extern deviceInitClass servoInitClass;
#endif
