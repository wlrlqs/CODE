#ifndef __DRIVER_MOTORSEVERINIT_H
#define __DRIVER_MOTORSEVERINIT_H
#include "Driver_MotorSever.h"
/*
CAN发送初始化
*/
f32_t RMDcmd = 0xA1;
//CAN网络缓冲
CAN_SendForm motor_pack[NET_LIST][CAN_NET_LIST];
/*
CAN接收地址初始化
*/
CAN_RevForm RevPitch = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
CAN_RevForm RevYaw = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
CAN_RevForm RevRoll = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
CAN_RevForm RevPoke = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
CAN_RevForm RevSupply = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
CAN_RevForm RevFric[DIR_OF_FRIC] = {
    {STANDBY,STANDBY,STANDBY,STANDBY},
	{STANDBY,STANDBY,STANDBY,STANDBY},	
};
CAN_RevForm RevsmellPoke = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
CAN_RevForm RevChassis[NUM_OF_WHEEL] = {
    {STANDBY,STANDBY,STANDBY,STANDBY},
    {STANDBY,STANDBY,STANDBY,STANDBY},
    {STANDBY,STANDBY,STANDBY,STANDBY},
    {STANDBY,STANDBY,STANDBY,STANDBY},
};
/*************INFANTRY OR F_TANK DEFORMING***************/
CAN_RevForm RevArm = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
CAN_RevForm RevSynchronous = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
/**********************************************/
/*************P_TANK DEFORMING***************/
CAN_RevForm RevSlavePoke = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
CAN_RevForm RevSlaveSupply = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
/**********************************************/
/***********AUXILIARY DEFORMING****************/
CAN_RevForm RevEle[DIR_OF_ELE] = {
	{STANDBY,STANDBY,STANDBY,STANDBY},
	{STANDBY,STANDBY,STANDBY,STANDBY},
};
CAN_RevForm RevSwy[DIR_OF_SWY] = {
	{STANDBY,STANDBY,STANDBY,STANDBY}, //X
	{STANDBY,STANDBY,STANDBY,STANDBY},//Y
};
CAN_RevForm RevClaw = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
CAN_RevForm RevRev[DIR_OF_REV] = {
	{STANDBY,STANDBY,STANDBY,STANDBY}, //L
	{STANDBY,STANDBY,STANDBY,STANDBY},//R
};
/**********************************************/
/***********SENTRY DEFORMING****************/
CAN_RevForm RevDirect = {
	STANDBY,STANDBY,STANDBY,STANDBY,
};
/**********************************************/
#endif
