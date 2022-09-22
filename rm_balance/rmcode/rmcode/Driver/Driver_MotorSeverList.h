#ifndef __DRIVER_MOTORSEVRTLIST_H
#define __DRIVER_MOTORSEVRTLIST_H

#include "cansend.h"
/*底盘对应电机接收表*/
#define CHASSIS_RF          0x201
#define CHASSIS_LF          0x202
#define CHASSIS_LB          0x203     //左后
#define CHASSIS_RB          0x204     //右后

/*步兵对应电机ID接收表*/
#define INFANTRY_YAW 		  	0x20B
#define INFANTRY_PITCH 		    0x20A
#define INFANTRY_POKE 		    0x207
#define INFANTRY_SUPPLY 	    0x208
#define INFANTRY_FRIC_L 	    0x206
#define INFANTRY_FRIC_R 	    0x205
#define INFANTRY_ARM		    0x206
#define INFANTRY_SYNCHRONOUS    0x205
//tof
#define INFANTRY_SLIDE_LEFT	    0x201
#define INFANTRY_SLIDE_RIGHT    0x202

/*新英雄对应电机ID接收表*/
#define P_TANK_YAW 		        0x141
#define P_TANK_PITCH 		    0x142
#define P_TANK_BIG_POKE 		0x207
#define P_TANK_BIG_SUPPLY 	    0x208
#define P_TANK_FRIC_L 	        0x206
#define P_TANK_FRIC_R 	        0x205
#define P_TANK_SMALL_POKE       0x207
#define P_TANK_SMALL_SUPPLY		0x208

/*老英雄对应电机ID接收表*/
#define F_TANK_YAW 		        0x141
#define F_TANK_PITCH 		    0x142
#define F_TANK_POKE 			0x207
#define F_TANK_SUPPLY 	    	0x208
#define F_TANK_FRIC_L 	        0x206
#define F_TANK_FRIC_R 	        0x205
#define F_TANK_ARM       		0x205
#define F_TANK_SYNCHRONOUS		0x206

/*工程对应电机ID接收表*/
#define AUXILIARY_YAW 		    0x141
#define AUXILIARY_PITCH         0x202
//CAN2
#define AUXILIARY_FRONT_CLAW    0x201
#define AUXILIARY_LIFT_ELEVATOR 0x202
#define AUXILIARY_RIGHT_ELEVATOR 0x203
#define AUXILIARY_X_SLIDEWAY    0x204
#define AUXILIARY_Y_SLIDEWAY    0x205
#define AUXILIARY_LIFT_REVERSING 0x206
#define AUXILIARY_RIGHT_REVERSING 0x207

/*哨兵对应电机ID接收表*/
#define SENTRY_YAW 		        0x209
#define SENTRY_PITCH 	        0x142
#define SENTRY_POKE 	        0x207
#define SENTRY_FRIC_L 	        0x206
#define SENTRY_FRIC_R 	        0x205
#define SENTRY_DIRECT 	        0x203

/*哨兵上云台对应电机ID接收表*/
#define SMALLGIMBAL_YAW 		0x209
#define SMALLGIMBAL_PITCH 	    0x20A
#define SMALLGIMBAL_POKE 		0x207
#define SMALLGIMBAL_FRIC_L 		0x206
#define SMALLGIMBAL_FRIC_R 		0x205

//LCM: /*无人机对应电机ID接受表*/
/*无人机对应电机ID接受表*/
#define UAV_YAW 		0x209
#define UAV_PITCH       0x20A
#define UAV_ROLL        0x20B        
#define UAV_POKE 		0x207
#define UAV_FRIC_L 	    0x206
#define UAV_FRIC_R 	    0x205 

#endif
