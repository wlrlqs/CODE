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
/*��ʼ����ṹ��*/
typedef struct{
	void (*Init) 	(void);
}deviceInitClass;
/*ʱ����*/
extern deviceInitClass clockClass;
/*ADC��*/
extern deviceInitClass adcClass;
/*������*/
extern deviceInitClass sightClass;
/*������*/
extern deviceInitClass DTUClass;
/*��������*/
extern deviceInitClass IMUIintClass;
/*ң������*/
extern deviceInitClass DT7IintClass;
/*CAN�����*/
extern deviceInitClass motorSeverInitClass;
/*�����*/
extern deviceInitClass servoInitClass;
#endif
