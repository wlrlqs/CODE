#include "motorHandle.h"
/****************************************************************************/
f32_t gimbalPosHandle(motorConfigStruct_t *motorConfig,int16_t centerOffSet);
int8_t getMotorTemperature(motorConfigStruct_t *motorConfig);
int16_t getMotorCurrent(motorConfigStruct_t *motorConfig);
int16_t getMotorSpeed(motorConfigStruct_t *motorConfig);
uint16_t getMotorEncoder(motorConfigStruct_t *motorConfig);
void setServoPWM(uint32_t *inputPWM);
/****************************************************************************/

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

motorHandleStruct_t motorHandleClass = {
	gimbalPosHandle,
	getMotorTemperature,
	getMotorCurrent,
	getMotorSpeed,
	getMotorEncoder,
	setServoPWM,
};

f32_t gimbalPosHandle(motorConfigStruct_t *motorConfig,int16_t centerOffSet){
	f32_t tmp = 0;
	if(centerOffSet >= (motorConfig->motor_lib.motor_encoder / 2)){
		if(motorHandleClass.Encoder(motorConfig) > centerOffSet - (motorConfig->motor_lib.motor_encoder / 2))
			tmp = motorHandleClass.Encoder(motorConfig) - centerOffSet;
		else
			tmp = motorHandleClass.Encoder(motorConfig) + motorConfig->motor_lib.motor_encoder - centerOffSet;
	}
	else{
		if(motorHandleClass.Encoder(motorConfig) > centerOffSet + (motorConfig->motor_lib.motor_encoder / 2))
			tmp = motorHandleClass.Encoder(motorConfig) - motorConfig->motor_lib.motor_encoder - centerOffSet;
		else
			tmp = motorHandleClass.Encoder(motorConfig) - centerOffSet;
	}
	tmp *= (360.0f/motorConfig->motor_lib.motor_encoder);
	return tmp;
}

int8_t getMotorTemperature(motorConfigStruct_t *motorConfig){
	return motorConfig->motor_staus->temperature;
}

int16_t getMotorCurrent(motorConfigStruct_t *motorConfig){
	return motorConfig->motor_staus->currunt;
}

int16_t getMotorSpeed(motorConfigStruct_t *motorConfig){
	return motorConfig->motor_staus->motorSpeed;
}

uint16_t getMotorEncoder(motorConfigStruct_t *motorConfig){
	return motorConfig->motor_staus->motorEncoder;
}

void setServoPWM(uint32_t *inputPWM){
	SERVO_TIM->CCR1 = *inputPWM;
}
	
