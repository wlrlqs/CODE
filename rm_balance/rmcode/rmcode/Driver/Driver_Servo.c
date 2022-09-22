#include "Driver_Servo.h"

static void Init(void);
static void device_servo_control(int16_t parameter);
deviceInitClass servoInitClass = {
	Init,
};

servo_control_Class  servocontrolClass ={
  device_servo_control,
};

static void Init(void){

	BSP_TIM_PWM_Init(SERVO_TIM,SERVO_PERIOD,SERVO_PESCALER,NULL,NULL,SERVO_GPIO,NULL);// 舵机初始化

}

static void device_servo_control(int16_t parameter){

 	TIM_SetCompare3(TIM5 , parameter);// 舵机控制

}
