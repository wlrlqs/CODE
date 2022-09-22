#include "Driver_USBVCP.h"
#include "Driver_Flash.h"
#include "config.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "tf_card_parameter.h"
#include "type_robot.h"
#include "supervisor.h"
systemConfig_t  systemConfigData __attribute__((at(0x10006000)));
systemConfig_t *getConfigData(void)
	{
    return &systemConfigData;
}


void configLoadDefault(void){
	systemConfigData.configVersion = DEFAULT_CONFIG_VERSION;
	systemConfigData.gimbalGTRScale = DEFAULT_GIMBAL_CTR_SCALE;
	systemConfigData.gimbalKBScale = DEFAULT_GIMBAL_KB_SCALE;
	systemConfigData.chassisKBSpeed = DEFAULT_CHASSIS_KB_SPEED;
	systemConfigData.chassisRCSpeed = DEFAULT_CHASSIS_RC_SPEED;
	systemConfigData.chassisKBAcc = DEFAULT_CHASSIS_KB_ACC;
	systemConfigData.jointPosition[0] = DEFAULT_JOINT_POSITION_LF;
	systemConfigData.jointPosition[1] = DEFAULT_JOINT_POSITION_LB;
	systemConfigData.jointPosition[2] = DEFAULT_JOINT_POSITION_RF;
	systemConfigData.jointPosition[3] = DEFAULT_JOINT_POSITION_RB;
	
	systemConfigData.yawAnglePID.PID_P = DEFAULT_YAW_ANGLE_P;
	systemConfigData.yawAnglePID.PID_I = DEFAULT_YAW_ANGLE_I;
	systemConfigData.yawAnglePID.PID_D = DEFAULT_YAW_ANGLE_D;
	systemConfigData.yawAnglePID.PID_F = DEFAULT_YAW_ANGLE_F;
	systemConfigData.yawAnglePID.PID_PM = DEFAULT_YAW_ANGLE_PM;
	systemConfigData.yawAnglePID.PID_IM = DEFAULT_YAW_ANGLE_IM;
	systemConfigData.yawAnglePID.PID_DM = DEFAULT_YAW_ANGLE_DM;
	systemConfigData.yawAnglePID.PID_OM = DEFAULT_YAW_ANGLE_OM;
	
	systemConfigData.yawRatePID.PID_P = DEFAULT_YAW_RATE_P;
	systemConfigData.yawRatePID.PID_I = DEFAULT_YAW_RATE_I;
	systemConfigData.yawRatePID.PID_D = DEFAULT_YAW_RATE_D;
	systemConfigData.yawRatePID.PID_F = DEFAULT_YAW_RATE_F;
	systemConfigData.yawRatePID.PID_PM = DEFAULT_YAW_RATE_PM;
	systemConfigData.yawRatePID.PID_IM = DEFAULT_YAW_RATE_IM;
	systemConfigData.yawRatePID.PID_DM = DEFAULT_YAW_RATE_DM;
	systemConfigData.yawRatePID.PID_OM = DEFAULT_YAW_RATE_OM;
	
	systemConfigData.pitchAnglePID.PID_P = DEFAULT_PITCH_ANGLE_P;
	systemConfigData.pitchAnglePID.PID_I = DEFAULT_PITCH_ANGLE_I;
	systemConfigData.pitchAnglePID.PID_D = DEFAULT_PITCH_ANGLE_D;
	systemConfigData.pitchAnglePID.PID_F = DEFAULT_PITCH_ANGLE_F;
	systemConfigData.pitchAnglePID.PID_PM = DEFAULT_PITCH_ANGLE_PM;
	systemConfigData.pitchAnglePID.PID_IM = DEFAULT_PITCH_ANGLE_IM;
	systemConfigData.pitchAnglePID.PID_DM = DEFAULT_PITCH_ANGLE_DM;
	systemConfigData.pitchAnglePID.PID_OM = DEFAULT_PITCH_ANGLE_OM;
	
	systemConfigData.pitchRatePID.PID_P = DEFAULT_PITCH_RATE_P;
	systemConfigData.pitchRatePID.PID_I = DEFAULT_PITCH_RATE_I;
	systemConfigData.pitchRatePID.PID_D = DEFAULT_PITCH_RATE_D;
	systemConfigData.pitchRatePID.PID_F = DEFAULT_PITCH_RATE_F;
	systemConfigData.pitchRatePID.PID_PM = DEFAULT_PITCH_RATE_PM;
	systemConfigData.pitchRatePID.PID_IM = DEFAULT_PITCH_RATE_IM;
	systemConfigData.pitchRatePID.PID_DM = DEFAULT_PITCH_RATE_DM;
	systemConfigData.pitchRatePID.PID_OM = DEFAULT_PITCH_RATE_OM;
	
	systemConfigData.rollAnglePID.PID_P = DEFAULT_ROLL_ANGLE_P;
	systemConfigData.rollAnglePID.PID_I = DEFAULT_ROLL_ANGLE_I;
	systemConfigData.rollAnglePID.PID_D = DEFAULT_ROLL_ANGLE_D;
	systemConfigData.rollAnglePID.PID_F = DEFAULT_ROLL_ANGLE_F;
	systemConfigData.rollAnglePID.PID_PM = DEFAULT_ROLL_ANGLE_PM;
	systemConfigData.rollAnglePID.PID_IM = DEFAULT_ROLL_ANGLE_IM;
	systemConfigData.rollAnglePID.PID_DM = DEFAULT_ROLL_ANGLE_DM;
	systemConfigData.rollAnglePID.PID_OM = DEFAULT_ROLL_ANGLE_OM;
	
	systemConfigData.rollRatePID.PID_P = DEFAULT_ROLL_RATE_P;
	systemConfigData.rollRatePID.PID_I = DEFAULT_ROLL_RATE_I;
	systemConfigData.rollRatePID.PID_D = DEFAULT_ROLL_RATE_D;
	systemConfigData.rollRatePID.PID_F = DEFAULT_ROLL_RATE_F;
	systemConfigData.rollRatePID.PID_PM = DEFAULT_ROLL_RATE_PM;
	systemConfigData.rollRatePID.PID_IM = DEFAULT_ROLL_RATE_IM;
	systemConfigData.rollRatePID.PID_DM = DEFAULT_ROLL_RATE_DM;
	systemConfigData.rollRatePID.PID_OM = DEFAULT_ROLL_RATE_OM;
	
	systemConfigData.chassisAllSpeedPID.PID_P = DEFAULT_CHASSIS_ALL_SPEED_P;
	systemConfigData.chassisAllSpeedPID.PID_I = DEFAULT_CHASSIS_ALL_SPEED_I;
	systemConfigData.chassisAllSpeedPID.PID_D = DEFAULT_CHASSIS_ALL_SPEED_D;
	systemConfigData.chassisAllSpeedPID.PID_F = DEFAULT_CHASSIS_ALL_SPEED_F;
	systemConfigData.chassisAllSpeedPID.PID_PM = DEFAULT_CHASSIS_ALL_SPEED_PM;
	systemConfigData.chassisAllSpeedPID.PID_IM = DEFAULT_CHASSIS_ALL_SPEED_IM;
	systemConfigData.chassisAllSpeedPID.PID_DM = DEFAULT_CHASSIS_ALL_SPEED_DM;
	systemConfigData.chassisAllSpeedPID.PID_OM = DEFAULT_CHASSIS_ALL_SPEED_OM;
	
	systemConfigData.chassisSpeedPID.PID_P = DEFAULT_CHASSIS_SPEED_P;
	systemConfigData.chassisSpeedPID.PID_I = DEFAULT_CHASSIS_SPEED_I;
	systemConfigData.chassisSpeedPID.PID_D = DEFAULT_CHASSIS_SPEED_D;
	systemConfigData.chassisSpeedPID.PID_F = DEFAULT_CHASSIS_SPEED_F;
	systemConfigData.chassisSpeedPID.PID_PM = DEFAULT_CHASSIS_SPEED_PM;
	systemConfigData.chassisSpeedPID.PID_IM = DEFAULT_CHASSIS_SPEED_IM;
	systemConfigData.chassisSpeedPID.PID_DM = DEFAULT_CHASSIS_SPEED_DM;
	systemConfigData.chassisSpeedPID.PID_OM = DEFAULT_CHASSIS_SPEED_OM;
	
	systemConfigData.chassisSpeedPID.PID_P = DEFAULT_CHASSIS_SPEED_P;
	systemConfigData.chassisSpeedPID.PID_I = DEFAULT_CHASSIS_SPEED_I;
	systemConfigData.chassisSpeedPID.PID_D = DEFAULT_CHASSIS_SPEED_D;
	systemConfigData.chassisSpeedPID.PID_F = DEFAULT_CHASSIS_SPEED_F;
	systemConfigData.chassisSpeedPID.PID_PM = DEFAULT_CHASSIS_SPEED_PM;
	systemConfigData.chassisSpeedPID.PID_IM = DEFAULT_CHASSIS_SPEED_IM;
	systemConfigData.chassisSpeedPID.PID_DM = DEFAULT_CHASSIS_SPEED_DM;
	systemConfigData.chassisSpeedPID.PID_OM = DEFAULT_CHASSIS_SPEED_OM;
	
	systemConfigData.jointAnglePID.PID_P = DEFAULT_JOINT_ANGLE_P;
	systemConfigData.jointAnglePID.PID_I = DEFAULT_JOINT_ANGLE_I;
	systemConfigData.jointAnglePID.PID_D = DEFAULT_JOINT_ANGLE_D;
	systemConfigData.jointAnglePID.PID_F = DEFAULT_JOINT_ANGLE_F;
	systemConfigData.jointAnglePID.PID_PM = DEFAULT_JOINT_ANGLE_PM;
	systemConfigData.jointAnglePID.PID_IM = DEFAULT_JOINT_ANGLE_IM;
	systemConfigData.jointAnglePID.PID_DM = DEFAULT_JOINT_ANGLE_DM;
	systemConfigData.jointAnglePID.PID_OM = DEFAULT_JOINT_ANGLE_OM;
	
	systemConfigData.jointPitchPID.PID_P = DEFAULT_JOINT_PITCH_P;
	systemConfigData.jointPitchPID.PID_I = DEFAULT_JOINT_PITCH_I;
	systemConfigData.jointPitchPID.PID_D = DEFAULT_JOINT_PITCH_D;
	systemConfigData.jointPitchPID.PID_F = DEFAULT_JOINT_PITCH_F;
	systemConfigData.jointPitchPID.PID_PM = DEFAULT_JOINT_PITCH_PM;
	systemConfigData.jointPitchPID.PID_IM = DEFAULT_JOINT_PITCH_IM;
	systemConfigData.jointPitchPID.PID_DM = DEFAULT_JOINT_PITCH_DM;
	systemConfigData.jointPitchPID.PID_OM = DEFAULT_JOINT_PITCH_OM;
	
	systemConfigData.chassisPositionPID.PID_P = DEFAULT_CHASSIS_POSITION_P;
	systemConfigData.chassisPositionPID.PID_I = DEFAULT_CHASSIS_POSITION_I;
	systemConfigData.chassisPositionPID.PID_D = DEFAULT_CHASSIS_POSITION_D;
	systemConfigData.chassisPositionPID.PID_F = DEFAULT_CHASSIS_POSITION_F;
	systemConfigData.chassisPositionPID.PID_PM = DEFAULT_CHASSIS_POSITION_PM;
	systemConfigData.chassisPositionPID.PID_IM = DEFAULT_CHASSIS_POSITION_IM;
	systemConfigData.chassisPositionPID.PID_DM = DEFAULT_CHASSIS_POSITION_DM;
	systemConfigData.chassisPositionPID.PID_OM = DEFAULT_CHASSIS_POSITION_OM;
	
	systemConfigData.chassisPitchPID.PID_P = DEFAULT_CHASSIS_PITCH_P;
	systemConfigData.chassisPitchPID.PID_I = DEFAULT_CHASSIS_PITCH_I;
	systemConfigData.chassisPitchPID.PID_D = DEFAULT_CHASSIS_PITCH_D;
	systemConfigData.chassisPitchPID.PID_F = DEFAULT_CHASSIS_PITCH_F;
	systemConfigData.chassisPitchPID.PID_PM = DEFAULT_CHASSIS_PITCH_PM;
	systemConfigData.chassisPitchPID.PID_IM = DEFAULT_CHASSIS_PITCH_IM;
	systemConfigData.chassisPitchPID.PID_DM = DEFAULT_CHASSIS_PITCH_DM;
	systemConfigData.chassisPitchPID.PID_OM = DEFAULT_CHASSIS_PITCH_OM;
	
	systemConfigData.chassisRollPID.PID_P = DEFAULT_CHASSIS_ROLL_P;
	systemConfigData.chassisRollPID.PID_I = DEFAULT_CHASSIS_ROLL_I;
	systemConfigData.chassisRollPID.PID_D = DEFAULT_CHASSIS_ROLL_D;
	systemConfigData.chassisRollPID.PID_F = DEFAULT_CHASSIS_ROLL_F;
	systemConfigData.chassisRollPID.PID_PM = DEFAULT_CHASSIS_ROLL_PM;
	systemConfigData.chassisRollPID.PID_IM = DEFAULT_CHASSIS_ROLL_IM;
	systemConfigData.chassisRollPID.PID_DM = DEFAULT_CHASSIS_ROLL_DM;
	systemConfigData.chassisRollPID.PID_OM = DEFAULT_CHASSIS_ROLL_OM;
	
	systemConfigData.chassisPosPID.PID_P = DEFAULT_CHASSIS_POS_P;
	systemConfigData.chassisPosPID.PID_I = DEFAULT_CHASSIS_POS_I;
	systemConfigData.chassisPosPID.PID_D = DEFAULT_CHASSIS_POS_D;
	systemConfigData.chassisPosPID.PID_F = DEFAULT_CHASSIS_POS_F;
	systemConfigData.chassisPosPID.PID_PM = DEFAULT_CHASSIS_POS_PM;
	systemConfigData.chassisPosPID.PID_IM = DEFAULT_CHASSIS_POS_IM;
	systemConfigData.chassisPosPID.PID_DM = DEFAULT_CHASSIS_POS_DM;
	systemConfigData.chassisPosPID.PID_OM = DEFAULT_CHASSIS_POS_OM;
	
	systemConfigData.chassisChasePID.PID_P = DEFAULT_CHASSIS_CHASE_P;
	systemConfigData.chassisChasePID.PID_I = DEFAULT_CHASSIS_CHASE_I;
	systemConfigData.chassisChasePID.PID_D = DEFAULT_CHASSIS_CHASE_D;
	systemConfigData.chassisChasePID.PID_F = DEFAULT_CHASSIS_CHASE_F;
	systemConfigData.chassisChasePID.PID_PM = DEFAULT_CHASSIS_CHASE_PM;
	systemConfigData.chassisChasePID.PID_IM = DEFAULT_CHASSIS_CHASE_IM;
	systemConfigData.chassisChasePID.PID_DM = DEFAULT_CHASSIS_CHASE_DM;
	systemConfigData.chassisChasePID.PID_OM = DEFAULT_CHASSIS_CHASE_OM;	
	
	systemConfigData.chassisRatePID.PID_P = DEFAULT_CHASSIS_RATE_P;						
	systemConfigData.chassisRatePID.PID_I = DEFAULT_CHASSIS_RATE_I;
	systemConfigData.chassisRatePID.PID_D = DEFAULT_CHASSIS_RATE_D;
	systemConfigData.chassisRatePID.PID_F = DEFAULT_CHASSIS_RATE_F;
	systemConfigData.chassisRatePID.PID_PM = DEFAULT_CHASSIS_RATE_PM;
	systemConfigData.chassisRatePID.PID_IM = DEFAULT_CHASSIS_RATE_IM;
	systemConfigData.chassisRatePID.PID_DM = DEFAULT_CHASSIS_RATE_DM;
	systemConfigData.chassisRatePID.PID_OM = DEFAULT_CHASSIS_RATE_OM;
	
	systemConfigData.powerLimitPID.PID_P = DEFAULT_POWER_LIMIT_P;
    systemConfigData.powerLimitPID.PID_I = DEFAULT_POWER_LIMIT_I;
	systemConfigData.powerLimitPID.PID_D = DEFAULT_POWER_LIMIT_D;
	systemConfigData.powerLimitPID.PID_F = DEFAULT_POWER_LIMIT_F;
	systemConfigData.powerLimitPID.PID_PM = DEFAULT_POWER_LIMIT_PM;
	systemConfigData.powerLimitPID.PID_IM = DEFAULT_POWER_LIMIT_IM;
	systemConfigData.powerLimitPID.PID_DM = DEFAULT_POWER_LIMIT_DM;
	systemConfigData.powerLimitPID.PID_OM = DEFAULT_POWER_LIMIT_OM;	
	
	systemConfigData.shootFricPID.PID_P = DEFAULT_SHOOT_SPEED_P;
	systemConfigData.shootFricPID.PID_I = DEFAULT_SHOOT_SPEED_I;
	systemConfigData.shootFricPID.PID_D = DEFAULT_SHOOT_SPEED_D;
	systemConfigData.shootFricPID.PID_F = DEFAULT_SHOOT_SPEED_F;
	systemConfigData.shootFricPID.PID_PM = DEFAULT_SHOOT_SPEED_PM;
	systemConfigData.shootFricPID.PID_IM = DEFAULT_SHOOT_SPEED_IM;
	systemConfigData.shootFricPID.PID_DM = DEFAULT_SHOOT_SPEED_DM;
	systemConfigData.shootFricPID.PID_OM = DEFAULT_SHOOT_SPEED_OM;	
	
	systemConfigData.shootLargeDialPID.PID_P = DEFAULT_ROLLBULL_SPEED_P;
	systemConfigData.shootLargeDialPID.PID_I = DEFAULT_ROLLBULL_SPEED_I;
	systemConfigData.shootLargeDialPID.PID_D = DEFAULT_ROLLBULL_SPEED_D;
	systemConfigData.shootLargeDialPID.PID_F = DEFAULT_ROLLBULL_SPEED_F;
	systemConfigData.shootLargeDialPID.PID_PM = DEFAULT_ROLLBULL_SPEED_PM;
	systemConfigData.shootLargeDialPID.PID_IM = DEFAULT_ROLLBULL_SPEED_IM;
	systemConfigData.shootLargeDialPID.PID_DM = DEFAULT_ROLLBULL_SPEED_DM;
	systemConfigData.shootLargeDialPID.PID_OM = DEFAULT_ROLLBULL_SPEED_OM;
	
	systemConfigData.shootSmallDialPID.PID_P = DEFAULT_LOADED_SPEED_P;
	systemConfigData.shootSmallDialPID.PID_I = DEFAULT_LOADED_SPEED_I;
	systemConfigData.shootSmallDialPID.PID_D = DEFAULT_LOADED_SPEED_D;
	systemConfigData.shootSmallDialPID.PID_F = DEFAULT_LOADED_SPEED_F;
    systemConfigData.shootSmallDialPID.PID_PM = DEFAULT_LOADED_SPEED_PM;
	systemConfigData.shootSmallDialPID.PID_IM = DEFAULT_LOADED_SPEED_IM;
	systemConfigData.shootSmallDialPID.PID_DM = DEFAULT_LOADED_SPEED_DM;
	systemConfigData.shootSmallDialPID.PID_OM = DEFAULT_LOADED_SPEED_OM;			
	
	
	systemConfigData.localID = DEFAULT_LOCAL_ID;
	systemConfigData.robotType = 0;
    systemConfigData.boardType = 1;
	systemConfigData.weaponType = DEFAULT_WEAPON_TYPE;			
	systemConfigData.backCenterTime = DEFAULT_BACK_CENTER_TIME;			
	systemConfigData.chassisCurrent = DEFAULT_CHASSIS_CURRENT;				
	systemConfigData.rcResolution = DEFAULT_RC_RESOLUTION;						
	systemConfigData.yawCenter= DEFAULT_YAW_CENTER;								
	systemConfigData.pitchCenter = DEFAULT_PITCH_CENTER;	
	systemConfigData.rollCenter = DEFAULT_ROLL_CENTER;			
	systemConfigData.pitchMinRange = DEFAULT_PITCH_MIN_RANGE;				
	systemConfigData.pitchMaxRange = DEFAULT_PITCH_MAX_RANGE;
    systemConfigData.yawSetting.u8_temp[0] = DEFAULT_YAW_FIX;
	systemConfigData.yawSetting.u8_temp[1] = DEFAULT_YAW_TURN;
	systemConfigData.yawSetting.u8_temp[2] = DEFAULT_YAW_TYPE;
    systemConfigData.yawSetting.u8_temp[3] = DEFAULT_YAW_INSTALL;	
	
    systemConfigData.pitchSetting.u8_temp[0] = DEFAULT_PITCH_FIX; 
	systemConfigData.pitchSetting.u8_temp[1] = DEFAULT_PITCH_TURN;
	systemConfigData.pitchSetting.u8_temp[2] = DEFAULT_PITCH_TYPE; 
    systemConfigData.pitchSetting.u8_temp[3] = DEFAULT_PITCH_INSTALL;	
	
	systemConfigData.rollSetting.u8_temp[0] = DEFAULT_ROLL_FIX;
	systemConfigData.rollSetting.u8_temp[1] = DEFAULT_ROLL_TURN;
    systemConfigData.rollSetting.u8_temp[2] = DEFAULT_ROLL_TYPE; 
    systemConfigData.rollSetting.u8_temp[3] = DEFAULT_ROLL_INSTALL;
	
	
}




/*------------------- 配置FLASH读取 ---------------------*/
void configFlashRead(void) {
    float *recs;
    int i;
    //从扇区首地址开始读取
    recs = (void *)flashStartAddr();
    for (i = 0; i < sizeof(systemConfig_t); i++) {
        *((&systemConfigData.configVersion) + i) = recs[i];
    }
}


/*------------------- 配置FLASH写入 ---------------------*/
uint8_t configFlashWrite(void) {
    float *recs;
	uint8_t ret = 0;
	int i;
    
    //申请一段动态内存用于储存临时信息
	recs = (void *)aqCalloc(sizeof(systemConfig_t), sizeof(float));
    //如果成功申请到内存地址就执行写入操作
	if (recs) {
        //从扇区初始地址开始擦除flash
		ret = flashErase(flashStartAddr(), sizeof(systemConfig_t) * sizeof(float) / sizeof(uint32_t));
        //使闪存数据缓存无效	        
		FLASH_DataCacheCmd(DISABLE);
        //重置数据缓存
		FLASH_DataCacheReset();
        //启用数据缓存功能
		FLASH_DataCacheCmd(ENABLE);
        //如果擦除成功，执行写入操作
		if (ret) {
            //在内存中创建参数列表
			for (i = 0; i < sizeof(systemConfig_t); i++) {
                recs[i] = *((&systemConfigData.configVersion) + i);
			}
            //存flash数据，从扇区起始地址往前存
			ret = flashAddress(flashStartAddr(), (uint32_t *)recs, \
                                sizeof(systemConfig_t) * sizeof(float) / sizeof(uint32_t));
		}
        else {
            usbVCP_Printf("Flash erase failed! \r\n");
        }
        //释放内存
		aqFree(recs, sizeof(systemConfig_t), sizeof(float));
	}
	else {
        usbVCP_Printf("Flash failed to apply for memory! \r\n");
    }
	return ret;
}

float _ver = 99;
	
void configInit(void) {
    uint8_t robotType;
	float ver;	
	uint8_t tfRec;	
    //从Flash中开始
	configFlashRead();																							
	//加载tf卡的配置
	tfRec = tFCardConfig();						
    //读取机器人类型
    robotType = getRobotType();
    //从TF卡里读PID参数
	if(!tfRec && robotType > NO_ID){
		parameterReadDataFromTFCard(robotType);                                                    
	}
    //读取当前flash版本
	ver = *(float *)(flashStartAddr());																
	if (isnan(ver))
		ver = 0.0f;
	
	_ver = ver;
    //如果编译的默认值大于flash版本和加载版本																											
	if (DEFAULT_CONFIG_VERSION > ver || DEFAULT_CONFIG_VERSION > systemConfigData.configVersion){
        //加载默认值
		configLoadDefault();																					
		digitalHi(&supervisorData.flashSave);	
	}
    //如果flash版本大于当前或等于当前版本
	else if (ver >= systemConfigData.configVersion){                                                              
        //读取flash   
        configFlashRead();																								
	}																			
    //如果加载的版本大于flash版本
	else if (systemConfigData.configVersion > ver){                                                                          
        //写入flash,这个情况只存在于有SD卡时，且SD卡中的版本高于flash中的版本才会发生        
        configFlashWrite();	
	}
    usbVCP_Printf("ConfigInit Successfully \r\n");
}

const char *configTFStrings[] = {
	"CONFIG_VERSION",				
    "ROBOT_TYPE"
    "BOARD_TYPE"
	"WEAPON_TYPE",
    "LOCAL_ID",
    
    "PITCH_MIN_RANGE",
	"PITCH_MAX_RANGE",
    "PITCH_CENTER",
	"YAW_CENTER",	
	"ROLL_CENTER",
    "RC_RESOLUTION",
    "CHASSIS_CURRENT",
    "BACK_CENTER_TIME",
	"GIMBAL_CTR_SCALE",
	"GIMBAL_KB_SCALE", 
	"CHASSIS_KB_SPEED",
	"CHASSIS_RC_SPEED",
	"CHASSIS_KB_ACC",
    
	"YAW_ANGLE_P",
	"YAW_ANGLE_I",
	"YAW_ANGLE_D",
	"YAW_ANGLE_F",
	"YAW_ANGLE_PM",
	"YAW_ANGLE_IM",
	"YAW_ANGLE_DM",
	"YAW_ANGLE_OM",
    
    "PITCH_ANGLE_P",
	"PITCH_ANGLE_I",
	"PITCH_ANGLE_D",
	"PITCH_ANGLE_F",
	"PITCH_ANGLE_PM",
	"PITCH_ANGLE_IM",
	"PITCH_ANGLE_DM",
	"PITCH_ANGLE_OM",
    
    "ROLL_ANGLE_P",
	"ROLL_ANGLE_I",
	"ROLL_ANGLE_D",
	"ROLL_ANGLE_F",
	"ROLL_ANGLE_PM",
	"ROLL_ANGLE_IM",
	"ROLL_ANGLE_DM",
	"ROLL_ANGLE_OM",
		
	"YAW_RATE_P",
	"YAW_RATE_I",
	"YAW_RATE_D",
	"YAW_RATE_F",
	"YAW_RATE_PM",
	"YAW_RATE_IM",
	"YAW_RATE_DM",
	"YAW_RATE_OM",
	
	"PITCH_RATE_P",
	"PITCH_RATE_I",
	"PITCH_RATE_D",
	"PITCH_RATE_F",
	"PITCH_RATE_PM",
	"PITCH_RATE_IM",
	"PITCH_RATE_DM",
	"PITCH_RATE_OM",	
	
	"ROLL_RATE_P",
	"ROLL_RATE_I",
	"ROLL_RATE_D",
	"ROLL_RATE_F",
	"ROLL_RATE_PM",
	"ROLL_RATE_IM",
	"ROLL_RATE_DM",
	"ROLL_RATE_OM",
	
	"CHASSIS_SPEED_P",
	"CHASSIS_SPEED_I",
	"CHASSIS_SPEED_D",
	"CHASSIS_SPEED_F",
	"CHASSIS_SPEED_PM",
	"CHASSIS_SPEED_IM",
	"CHASSIS_SPEED_DM",
	"CHASSIS_SPEED_OM",
	
	"CHASSIS_POS_P",
	"CHASSIS_POS_I",
	"CHASSIS_POS_D",
	"CHASSIS_POS_F",
	"CHASSIS_POS_PM",
	"CHASSIS_POS_IM",
	"CHASSIS_POS_DM",
	"CHASSIS_POS_OM",
	
	"CHASSIS_CHASE_P",
	"CHASSIS_CHASE_I",
	"CHASSIS_CHASE_D",
	"CHASSIS_CHASE_F",
	"CHASSIS_CHASE_PM",
	"CHASSIS_CHASE_IM",
	"CHASSIS_CHASE_DM",
	"CHASSIS_CHASE_OM",
	
	"CHASSIS_RATE_P",							
	"CHASSIS_RATE_I",
	"CHASSIS_RATE_D",
	"CHASSIS_RATE_F",
	"CHASSIS_RATE_PM",
	"CHASSIS_RATE_IM",
	"CHASSIS_RATE_DM",
	"CHASSIS_RATE_OM",
	
	"POWER_LIMIT_P",
	"POWER_LIMIT_I",
	"POWER_LIMIT_D",
	"POWER_LIMIT_F",
	"POWER_LIMIT_PM",
	"POWER_LIMIT_IM",
	"POWER_LIMIT_DM",
	"POWER_LIMIT_OM",

	"SHOOT_FRIC_P",
	"SHOOT_FRIC_I",
	"SHOOT_FRIC_D",
	"SHOOT_FRIC_F",
	"SHOOT_FRIC_PM",
	"SHOOT_FRIC_IM",
	"SHOOT_FRIC_DM",
	"SHOOT_FRIC_OM",
	
	"DIAL_SMALL_P",
	"DIAL_SMALL_I",
	"DIAL_SMALL_D",
	"DIAL_SMALL_F",
	"DIAL_SMALL_PM",
	"DIAL_SMALL_IM",
	"DIAL_SMALL_DM",
	"DIAL_SMALL_OM",
    
    "DIAL_LARGE_P",
	"DIAL_LARGE_I",
	"DIAL_LARGE_D",
	"DIAL_LARGE_F",
	"DIAL_LARGE_PM",
	"DIAL_LARGE_IM",
	"DIAL_LARGE_DM",
	"DIAL_LARGE_OM",
    
	"YAW_CONFIG",
	   
    "PITCH_CONFIG",
  
    "ROLL_CONFIG",
	
};


