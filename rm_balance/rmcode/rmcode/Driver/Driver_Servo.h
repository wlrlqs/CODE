#ifndef __DRIVER_SERVO_H
#define __DRIVER_SERVO_H

#include "stm32f4xx.h"
#include "driver.h"
#include "BSP_GPIO.h"
#include "BSP_PWM.h"

#define SERVO_TIM 		TIM5
#define SERVO_GPIO		BSP_GPIOA2
#define SERVO_PERIOD 	(3030-1)
#define SERVO_PESCALER 	(84-1) 

typedef struct{
	void (*control) 	(int16_t parameter);
}servo_control_Class;

extern servo_control_Class  servocontrolClass;

#endif
