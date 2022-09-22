#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f4xx.h"
#include "parameter.h"
#include "Util.h"

#define CONFIG_FILE_BUF_SIZE	            512
#define CONFIG_LINE_BUF_SIZE	            128

#define DEFAULT_CONFIG_VERSION	  	        130
#define DEFAULT_ROBOT_TYPE			  	    1
#define DEFAULT_DEAD_BAND     	 		    30.0f
#define DEFAULT_MOUSE_FEEL			  	    0.0025f
#define DEFAULT_GIMBAL_CTR_SCALE  	        0.0012f
#define DEFAULT_GIMBAL_KB_SCALE			    0.013f
#define DEFAULT_CHASSIS_KB_SPEED  	        8000.0f
#define DEFAULT_CHASSIS_RC_SPEED  	        8000.0f
#define DEFAULT_CHASSIS_KB_ACC			    20.0f

#define DEFAULT_YAW_ANGLE_P					0.300000012
#define DEFAULT_YAW_ANGLE_I					0
#define DEFAULT_YAW_ANGLE_D					0
#define DEFAULT_YAW_ANGLE_F					0.25
#define DEFAULT_YAW_ANGLE_PM				8
#define DEFAULT_YAW_ANGLE_IM				0.30f
#define DEFAULT_YAW_ANGLE_DM				25000
#define DEFAULT_YAW_ANGLE_OM				25000

#define DEFAULT_YAW_RATE_P					300.00f
#define DEFAULT_YAW_RATE_I					0.00f
#define DEFAULT_YAW_RATE_D					0.00f
#define DEFAULT_YAW_RATE_F					0.25f
#define DEFAULT_YAW_RATE_PM					2000.0f
#define DEFAULT_YAW_RATE_IM					1200.0f
#define DEFAULT_YAW_RATE_DM					2000.0f
#define DEFAULT_YAW_RATE_OM					25000.0f

#define DEFAULT_PITCH_ANGLE_P				0.38f
#define DEFAULT_PITCH_ANGLE_I				0.00f
#define DEFAULT_PITCH_ANGLE_D				0.00f
#define DEFAULT_PITCH_ANGLE_F  			    0.25f
#define DEFAULT_PITCH_ANGLE_PM			    8.00f	
#define DEFAULT_PITCH_ANGLE_IM			    0.30f
#define DEFAULT_PITCH_ANGLE_DM			    8.00f
#define DEFAULT_PITCH_ANGLE_OM			    25000

#define DEFAULT_PITCH_RATE_P				9000.00f
#define DEFAULT_PITCH_RATE_I				0.00f
#define DEFAULT_PITCH_RATE_D				0.00f
#define DEFAULT_PITCH_RATE_F				0.25f
#define DEFAULT_PITCH_RATE_PM				25000.00f
#define DEFAULT_PITCH_RATE_IM				1200.00f
#define DEFAULT_PITCH_RATE_DM				2000.00f
#define DEFAULT_PITCH_RATE_OM				25000.00f

#define DEFAULT_ROLL_ANGLE_P				0.22f
#define DEFAULT_ROLL_ANGLE_I				0.01f
#define DEFAULT_ROLL_ANGLE_D				0.30f
#define DEFAULT_ROLL_ANGLE_F				0.25f
#define DEFAULT_ROLL_ANGLE_PM				8.00f
#define DEFAULT_ROLL_ANGLE_IM				0.30f
#define DEFAULT_ROLL_ANGLE_DM				8.00f
#define DEFAULT_ROLL_ANGLE_OM				8.30f    

#define DEFAULT_ROLL_RATE_P					610.00f
#define DEFAULT_ROLL_RATE_I					500.00f
#define DEFAULT_ROLL_RATE_D					125.00f
#define DEFAULT_ROLL_RATE_F					0.25f
#define DEFAULT_ROLL_RATE_PM				850.0f
#define DEFAULT_ROLL_RATE_IM				500.0f
#define DEFAULT_ROLL_RATE_DM				500.0f
#define DEFAULT_ROLL_RATE_OM				850.0f

#define DEFAULT_CHASSIS_ALL_SPEED_P			    3.5f
#define DEFAULT_CHASSIS_ALL_SPEED_I			    0.0f
#define DEFAULT_CHASSIS_ALL_SPEED_D			    0.0f
#define DEFAULT_CHASSIS_ALL_SPEED_F			    0.2f
#define DEFAULT_CHASSIS_ALL_SPEED_PM		    10000.0f
#define DEFAULT_CHASSIS_ALL_SPEED_IM		    5000.0f
#define DEFAULT_CHASSIS_ALL_SPEED_DM		    0.0f
#define DEFAULT_CHASSIS_ALL_SPEED_OM		    10000.0f

#define DEFAULT_CHASSIS_SPEED_P			    400.0f
#define DEFAULT_CHASSIS_SPEED_I			    20.4f
#define DEFAULT_CHASSIS_SPEED_D			    0.0f
#define DEFAULT_CHASSIS_SPEED_F			    0.2f
#define DEFAULT_CHASSIS_SPEED_PM		    18000.0f
#define DEFAULT_CHASSIS_SPEED_IM		    5000.0f
#define DEFAULT_CHASSIS_SPEED_DM		    0.0f
#define DEFAULT_CHASSIS_SPEED_OM		    18000.0f

#define DEFAULT_CHASSIS_POSITION_P			    5.0f
#define DEFAULT_CHASSIS_POSITION_I			    0.0f
#define DEFAULT_CHASSIS_POSITION_D			    0.0f
#define DEFAULT_CHASSIS_POSITION_F			    0.2f
#define DEFAULT_CHASSIS_POSITION_PM		    5000.0f
#define DEFAULT_CHASSIS_POSITION_IM		    0.0f
#define DEFAULT_CHASSIS_POSITION_DM		    1000.0f
#define DEFAULT_CHASSIS_POSITION_OM		    5000.0f

#define DEFAULT_CHASSIS_PITCH_P			    1500.0f
#define DEFAULT_CHASSIS_PITCH_I			    0.0f
#define DEFAULT_CHASSIS_PITCH_D			    -1700.0f
#define DEFAULT_CHASSIS_PITCH_F			    0.4f
#define DEFAULT_CHASSIS_PITCH_PM		    18000.0f
#define DEFAULT_CHASSIS_PITCH_IM		    0.0f
#define DEFAULT_CHASSIS_PITCH_DM		    12000.0f
#define DEFAULT_CHASSIS_PITCH_OM		    18000.0f

#define DEFAULT_CHASSIS_ROLL_P			    0.69f
#define DEFAULT_CHASSIS_ROLL_I			    0.15f
#define DEFAULT_CHASSIS_ROLL_D			    0.10f
#define DEFAULT_CHASSIS_ROLL_F			    0.2f
#define DEFAULT_CHASSIS_ROLL_PM		      5000.0f
#define DEFAULT_CHASSIS_ROLL_IM		      2000.0f
#define DEFAULT_CHASSIS_ROLL_DM		      4500.0f
#define DEFAULT_CHASSIS_ROLL_OM		      5000.0f

#define DEFAULT_JOINT_ANGLE_P			      0.25f
#define DEFAULT_JOINT_ANGLE_I			      0.2f
#define DEFAULT_JOINT_ANGLE_D			      15.0f
#define DEFAULT_JOINT_ANGLE_F			      0.2f
#define DEFAULT_JOINT_ANGLE_PM		      5000.0f
#define DEFAULT_JOINT_ANGLE_IM	        3000.0f
#define DEFAULT_JOINT_ANGLE_DM		      5000.0f
#define DEFAULT_JOINT_ANGLE_OM		      5000.0f

#define DEFAULT_JOINT_PITCH_P			      0.0025f
#define DEFAULT_JOINT_PITCH_I			      0.0f
#define DEFAULT_JOINT_PITCH_D			      1.50f
#define DEFAULT_JOINT_PITCH_F			      0.2f
#define DEFAULT_JOINT_PITCH_PM		      1000.0f
#define DEFAULT_JOINT_PITCH_IM	        0.0f
#define DEFAULT_JOINT_PITCH_DM		      1000.0f
#define DEFAULT_JOINT_PITCH_OM		      1000.0f

#define DEFAULT_CHASSIS_POS_P  			    5.0f
#define DEFAULT_CHASSIS_POS_I			    	0.0f
#define DEFAULT_CHASSIS_POS_D 			    0.0f
#define DEFAULT_CHASSIS_POS_F 			    0.20f
#define DEFAULT_CHASSIS_POS_PM			    5000.0f
#define DEFAULT_CHASSIS_POS_IM			    0.0f
#define DEFAULT_CHASSIS_POS_DM			    5000.0f
#define DEFAULT_CHASSIS_POS_OM			    5000.0f

#define DEFAULT_CHASSIS_CHASE_P         380.0f
#define DEFAULT_CHASSIS_CHASE_I         0.0f
#define	DEFAULT_CHASSIS_CHASE_D         25000.0f
#define	DEFAULT_CHASSIS_CHASE_F         0.2f
#define	DEFAULT_CHASSIS_CHASE_PM        9000.0f
#define	DEFAULT_CHASSIS_CHASE_IM        0.0f
#define	DEFAULT_CHASSIS_CHASE_DM        9000.0f
#define	DEFAULT_CHASSIS_CHASE_OM        9000.0f

#define DEFAULT_CHASSIS_RATE_P     	    600.0f
#define DEFAULT_CHASSIS_RATE_I     	    0.0f
#define	DEFAULT_CHASSIS_RATE_D     	    0.0f
#define	DEFAULT_CHASSIS_RATE_F     	    0.2f
#define	DEFAULT_CHASSIS_RATE_PM    	    8000.0f
#define	DEFAULT_CHASSIS_RATE_IM    	    2000.0f
#define	DEFAULT_CHASSIS_RATE_DM   	    5000.0f
#define	DEFAULT_CHASSIS_RATE_OM   	    8000.0f

#define DEFAULT_POWER_LIMIT_P		    1.2f	
#define DEFAULT_POWER_LIMIT_I			1.0f
#define DEFAULT_POWER_LIMIT_D			0.0f
#define DEFAULT_POWER_LIMIT_F			0.2f
#define DEFAULT_POWER_LIMIT_PM			16000.0f
#define DEFAULT_POWER_LIMIT_IM			2000.0f
#define DEFAULT_POWER_LIMIT_DM			8000.0f
#define DEFAULT_POWER_LIMIT_OM			16000.0f

#define DEFAULT_SHOOT_SPEED_P	 		10.0f
#define DEFAULT_SHOOT_SPEED_I			0.0f
#define DEFAULT_SHOOT_SPEED_D	 		0.0f
#define DEFAULT_SHOOT_SPEED_F			0.2f
#define DEFAULT_SHOOT_SPEED_PM 			16000.0f
#define DEFAULT_SHOOT_SPEED_IM 			8000.0f
#define DEFAULT_SHOOT_SPEED_DM 			8000.0f
#define DEFAULT_SHOOT_SPEED_OM 			16000.0f

#define DEFAULT_ROLLBULL_SPEED_P  		8.0f	
#define DEFAULT_ROLLBULL_SPEED_I  		0.0f
#define DEFAULT_ROLLBULL_SPEED_D  		0.0f
#define DEFAULT_ROLLBULL_SPEED_F  		0.2f
#define DEFAULT_ROLLBULL_SPEED_PM 		10000.0f
#define DEFAULT_ROLLBULL_SPEED_IM 		2000.0f
#define DEFAULT_ROLLBULL_SPEED_DM 		10000.0f
#define DEFAULT_ROLLBULL_SPEED_OM 		10000.0f

#define DEFAULT_LOADED_SPEED_P  		8.0f	
#define DEFAULT_LOADED_SPEED_I  		0.0f
#define DEFAULT_LOADED_SPEED_D  		0.0f
#define DEFAULT_LOADED_SPEED_F  		0.2f
#define DEFAULT_LOADED_SPEED_PM 		10000.0f
#define DEFAULT_LOADED_SPEED_IM 		2000.0f
#define DEFAULT_LOADED_SPEED_DM 		10000.0f
#define DEFAULT_LOADED_SPEED_OM 		10000.0f


/*----------------	电机参数		---------------*/
#define DEFAULT_LOCAL_ID				0x0101
#define DEFAULT_WEAPON_TYPE				0
#define DEFAULT_BACK_CENTER_TIME        500.0f
#define DEFAULT_CHASSIS_CURRENT			3000.0f
#define DEFAULT_RC_RESOLUTION			500.0f
#define DEFAULT_YAW_CENTER 				0.0f
#define	DEFAULT_PITCH_CENTER			0.0f
#define	DEFAULT_ROLL_CENTER				0.0f
#define DEFAULT_PITCH_MIN_RANGE			-22.f
#define DEFAULT_PITCH_MAX_RANGE			43.0f
#define	DEFAULT_YAW_FIX                 0
#define	DEFAULT_YAW_TURN                1
#define	DEFAULT_YAW_TYPE                6
#define DEFAULT_YAW_INSTALL							1
#define	DEFAULT_PITCH_FIX               2 
#define	DEFAULT_PITCH_TURN              1
#define	DEFAULT_PITCH_TYPE              5
#define DEFAULT_PITCH_INSTALL			1
#define	DEFAULT_ROLL_FIX          	    2 
#define	DEFAULT_ROLL_TURN          	    1
#define	DEFAULT_ROLL_TYPE          	    1  
#define DEFAULT_ROLL_INSTALL 			0
#define DEFAULT_JOINT_POSITION_LF			    3021
#define DEFAULT_JOINT_POSITION_LB			    5554
#define DEFAULT_JOINT_POSITION_RF			    26123
#define DEFAULT_JOINT_POSITION_RB			    15808


//使用宏定义调用三个轴的电机配置

#define YAW_FIX_CONFIG             getConfigData()->yawSetting.u8_temp[0]
#define YAW_TURN_CONFIG            getConfigData()->yawSetting.u8_temp[1]
#define YAW_TYPE_CONFIG            getConfigData()->yawSetting.u8_temp[2]
#define YAW_INSTALL_CONFIG         getConfigData()->yawSetting.u8_temp[3]


#define PITCH_FIX_CONFIG           getConfigData()->pitchSetting.u8_temp[0]   
#define PITCH_TURN_CONFIG          getConfigData()->pitchSetting.u8_temp[1]
#define PITCH_TYPE_CONFIG          getConfigData()->pitchSetting.u8_temp[2]
#define PITCH_INSTALL_CONFIG       getConfigData()->pitchSetting.u8_temp[3]


#define ROLL_FIX_CONFIG            getConfigData()->rollSetting.u8_temp[0]
#define ROLL_TURN_CONFIG           getConfigData()->rollSetting.u8_temp[1]
#define ROLL_TYPE_CONFIG           getConfigData()->rollSetting.u8_temp[2]
#define ROLL_INSTALL_CONFIG        getConfigData()->rollSetting.u8_temp[3]


typedef struct{
    float PID_P;
    float PID_I;
    float PID_D;
    float PID_F;
    float PID_PM;
    float PID_IM;
    float PID_DM;
    float PID_OM;
}systemConfigPID_t;

typedef struct {
	float configVersion;
	float robotType;
	float boardType;
	float weaponType;
	float localID;
	
	float pitchMaxRange;
	float pitchMinRange;
	float pitchCenter;
	float yawCenter;
	float rollCenter;
	float rcResolution;
	float chassisCurrent;
	float backCenterTime;
	float gimbalGTRScale;
	float gimbalKBScale;
	float chassisKBSpeed;
	float chassisRCSpeed;
	float chassisKBAcc;
	int32_t jointPosition[4];
	//三轴角度环PID
	systemConfigPID_t yawAnglePID;
	systemConfigPID_t pitchAnglePID;
	systemConfigPID_t rollAnglePID;
	//三轴速度环PID
	systemConfigPID_t yawRatePID;
	systemConfigPID_t pitchRatePID;
	systemConfigPID_t rollRatePID;
	//底盘PID
	systemConfigPID_t chassisAllSpeedPID;
	systemConfigPID_t chassisSpeedPID;
	systemConfigPID_t chassisPositionPID;
	systemConfigPID_t chassisPitchPID;
	systemConfigPID_t chassisYawPID;
	systemConfigPID_t chassisRollPID;
	systemConfigPID_t jointAnglePID;
	systemConfigPID_t jointPitchPID;
	systemConfigPID_t chassisPosPID;
	systemConfigPID_t chassisChasePID;
	systemConfigPID_t chassisRatePID;
	//其他PID
	systemConfigPID_t powerLimitPID;
	systemConfigPID_t shootFricPID;
	systemConfigPID_t shootSmallDialPID;
	systemConfigPID_t shootLargeDialPID;
	//工程相关电机机构PID
	systemConfigPID_t elevatorSpeedPID;
	systemConfigPID_t elevatorAnglePID;
	systemConfigPID_t xSlidewaySpeedPID;
	systemConfigPID_t ySlidewaySpeedPID;
	systemConfigPID_t frontClawSpeedPID;	
	//三轴的配置参数
	formatTrans32Struct_t yawSetting;
	formatTrans32Struct_t pitchSetting;
	formatTrans32Struct_t rollSetting;
	
	//
    
} systemConfig_t;


extern const char *configTFStrings[] ;
systemConfig_t *getConfigData(void);
void changeConfigGroup(uint32_t group);
void configFlashRead(void);
uint8_t configFlashWrite(void);
void configInit(void);


#endif






