#include "shoot.h"
#include "rc.h"
#include "config.h"
#include "Driver_RMMotor.h"
#include "judge.h"
#include "cansend.h"
#include "keyboard.h"
#include "control.h"
#include "pneumatic.h"
#include "motorHandle.h"
#include "auto_auxiliary.h"
#include "auto_sentry.h"
#include "auto_uav.h"
#include "Driver_USBVCP.h"
           
#define POKE_MOTER_SINGLE						 		(shootData.pokeSpeedRef = shootData.pokeSpeedSet) //17mm拨弹盘单发转速
#define POKE_MOTER_TRIPLE							    (shootData.pokeSpeedRef = shootData.pokeSpeedSet)	//17mm拨弹盘三连发转速
#define POKE_MOTOR_BURST						  	    (shootData.pokeSpeedRef = shootData.pokeSpeedSet)	//17mm拨弹盘连发转速
#define POKE_MOTOR_CLEAR								(shootData.pokeSpeedRef = 0.4f * shootData.pokeSpeedSet)	//17mm拨弹盘退弹转速
#define BIG_POKE_ON										(shootData.pokeSpeedRef = 8500)	//42mm拨弹盘单发转速
#define POKE_MOTER_OFF							        (shootData.pokeSpeedRef = 0)
#define FIRC_42MM_WHEEL_ON							    (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_42mm)		//42mm摩擦轮转速
#define FIRC_42MM_WHEEL_OFF							    (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0)
/**/
#define FIRC_17MM_WHEEL_SINGLE					        (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_17mm)  //17mm摩擦轮转速，单发
#define FIRC_17MM_WHEEL_TRIPLE					        (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_17mm)  //三连发
#define FIRC_17MM_WHEEL_BURST						    (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_17mm) //连发
#define FIRC_17MM_CLEAR_BULLET                          (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0.1f * shootData.fricSpeedSet_17mm)  //退弹
#define FIRC_17MM_WHEEL_OFF                             (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0)
/**/
#define SHOOT_PWM_VARIABLE_QUANTITY			            2

#define BULLET_READY										shootData.bulletMonitorFlag
#define GUN_READY												!pneumaticData.pneumatic_1.read1[0]    // 等待修改打弹逻辑后删除
#define GIMBAL_READY										gimbalData.initFinishFlag

extern shootStruct_t shootData;
shootStruct_t* getshootData(){
    return &shootData;
}
/* 射击参数重置 */
void shootDataReset(void){
	digitalLo(&shootData.fireFlag_17mm);
	digitalLo(&shootData.fireFlag_42mm);
	digitalLo(&shootData.shootManualNumber);
	digitalLo(&shootData.fireDataInit);
	digitalLo(&shootData.shootTrigger_17mm);
	digitalLo(&shootData.shootTrigger_42mm);
}

#if SNAIL_MOTOR 
	/* 摩擦轮更新(10ms) */
	static void shootFrictiongearUpdate( uint8_t FricONflag ){
		static uint16_t shootPwm = 1000;
		static uint16_t maxShootPwm;

		if(shootData.shootMode_17mm == MANUAL_SINGLE){
			maxShootPwm = parameter[SHOOT_HIGH_PWM];
		}
		else{
			maxShootPwm = parameter[SHOOT_LOW_PWM];
		}
		if(FricONflag){
			if(shootPwm > maxShootPwm){
				shootPwm -= SHOOT_PWM_VARIABLE_QUANTITY;
				if(shootPwm < maxShootPwm)
					shootPwm = maxShootPwm;
			}
			if(shootPwm < maxShootPwm){
				shootPwm += SHOOT_PWM_VARIABLE_QUANTITY; 
				//单发连发模式大概26m/s射速
				if( shootPwm > maxShootPwm )
					shootPwm = maxShootPwm;																	
			}
			FricMotor_L = FricMotor_R = shootPwm;
		}
		else{
			FricMotor_L = FricMotor_R = 1000;
		}
	}
#endif
static void fricWheel_update(uint8_t FricONflag){
	if(FricONflag){
		//单发
		if(shootData.shootMode_17mm == MANUAL_SINGLE)
			FIRC_17MM_WHEEL_SINGLE;	
		//三连发
		else if(shootData.shootMode_17mm == MANUAL_CONTINUOUS)
			FIRC_17MM_WHEEL_TRIPLE;
		//连发
		else if(shootData.shootMode_17mm == AUTO_CONTINUOUS){
			if(ROBOT == SENTRY_ID || ROBOT == SMALLGIMBAL_ID){
				FIRC_17MM_WHEEL_SINGLE;//17mm摩擦轮转速默认值
			}
			else{
				FIRC_17MM_WHEEL_BURST;
			}
		}
		//退弹
		else if(shootData.clearBulletFlag)
			FIRC_17MM_CLEAR_BULLET;
		if(ROBOT == F_TANK_ID)
			//英雄42mm摩擦轮
			FIRC_42MM_WHEEL_ON;
	}
	else{
		FIRC_17MM_WHEEL_OFF;
		FIRC_42MM_WHEEL_OFF;
	}
}



static void p_heroAutoLoad(void){
	//滚筒停止标志位
    static bool turnStallstoplflag;
	//装填时间监测
	static uint16_t cleanLoadWakeTime;									
	static uint16_t turnStallCount = 0;
	static TickType_t turnWakeTime = 0;
	static uint8_t lastswitch = 0 ;
    static uint8_t lastswitch_Mode = 0 ;
	if(shootData.fricMotorFlag)
		//单发
		FIRC_17MM_WHEEL_SINGLE;
	else if(shootData.clearBulletFlag)
		//退弹
		FIRC_17MM_CLEAR_BULLET;			
	else
		FIRC_17MM_WHEEL_OFF;
   switch(shootData.speed_limit) {
       case 10 : UPPER_DOWN;  break;
       case 16 : UPPER_UP;    break;
   }
    
    
         if(lastswitch_Mode == RCSW_MID && RC_MODE == RCSW_BOTTOM)
            digitalHi(&shootData.bulletExistStop);
         if(RC_GEAR==RCSW_BOTTOM || RC_MODE==RCSW_BOTTOM)
            digitalClan(&shootData.loadStep);

	switch(shootData.loadStep){
		case 0:{
			if(RC_GEAR==RCSW_BOTTOM || RC_MODE==RCSW_BOTTOM)
            {
                if(RC_ROTATE<-250)
                    {
                    digitalHi(&shootData.bulletfreeFlag);
                    BIG_GUN_WEAPONS_ON;//往前推  
                    BIG_GUN_ON;      //蓄力 
                    }
           else if(RC_ROTATE>250 && shootData.bulletfreeFlag)
               {
                    digitalIncreasing(&cleanLoadWakeTime);
                    BIG_GUN_OFF;
                    BIG_GUN_WEAPONS_OFF;
                    if(cleanLoadWakeTime > 500)
                    {
                    digitalLo(&shootData.bulletfreeFlag);
                    digitalClan(&cleanLoadWakeTime);   
                    }
               }
               
		  else if(!shootData.bulletfreeFlag)
              {
                if(ELEC_MAG_SENSOR_FRONT)
                    {
                    BIG_GUN_WEAPONS_ON;
                    shootData.loadStep = 2;
                    }
		    else { 
                BIG_GUN_WEAPONS_OFF;	//气锤后退		
                 BIG_GUN_ON;                
				   digitalClan(&turnStallCount);
				   digitalClan(&shootData.turntableStalltime);
                 }
               }
            }
          
            shootData.slave_pokeSpeedRef = 0;
			if((RC_GEAR!= RCSW_BOTTOM)&&(RC_MODE!= RCSW_BOTTOM))
				digitalIncreasing(&shootData.loadStep);
			break;
		}
		case 1:{
             BIG_GUN_ON;		           //充气	
            if(GUN_AWAIT2 || GUN_AWAIT1)
            {
                digitalIncreasing(&cleanLoadWakeTime);
                shootData.slave_pokeSpeedRef = 1800;
                if(cleanLoadWakeTime > 0x35)
                {     
                    BIG_GUN_WEAPONS_ON;		   //上镗		
                    
                    shootData.slave_pokeSpeedRef = 0;
                    digitalIncreasing(&shootData.loadStep);
                    digitalHi(&shootData.bulletBuzyFlag);
                    digitalClan(&cleanLoadWakeTime);
                }
            }
			else
                {
                    BIG_GUN_WEAPONS_OFF;           //退镗
//                    if(ELEC_MAG_SENSOR_BEHIND)     //如果后面的磁开关检测到气锤再让拨弹盘转             
                    shootData.slave_pokeSpeedRef = 1600;
			    }
			break;
		}
		case 2:{
            digitalIncreasing(&cleanLoadWakeTime);
            shootData.slave_pokeSpeedRef = 0;
            if(cleanLoadWakeTime > 0xFF)
            {
			  if((lastswitch==RCSW_MID && RC_GEAR==RCSW_TOP)|| KB_42MM_SHOOT_CONTINUOUS)
                {				
                    if(shootData.shooterHeatRemain_42mm >=__42MM_HEAT && ELEC_MAG_SENSOR_FRONT && (judgeData.extGameRobotState.remain_HP!=0))
                    {
                        digitalIncreasing(&shootData.loadStep);
					    BIG_GUN_OFF;                       //开火
                        digitalClan(&cleanLoadWakeTime);
                        
                    }                        
					digitalLo(&shootData.bulletBuzyFlag);					
					//拉低堵转标志位
					digitalLo(&turnStallstoplflag);                     

			     }
             }
			break;	
		}        
		case 3:{
             shootData.slave_pokeSpeedRef = 0;
			digitalIncreasing(&cleanLoadWakeTime);
			if (cleanLoadWakeTime > 0xF0){
				digitalLo(&P_HERO_LOAD);
				//退镗
				BIG_GUN_WEAPONS_OFF;		
                BIG_GUN_ON;                
				digitalLo(&shootData.bulletBuzyFlag);
				digitalClan(&cleanLoadWakeTime);
				digitalIncreasing(&shootData.loadStep);
               
			}
			break;	
		}
//		case 4:{
//			digitalIncreasing(&cleanLoadWakeTime);	  
//			if(cleanLoadWakeTime > 200){
//				digitalClan(&cleanLoadWakeTime);			
//				digitalClan(&shootData.loadStep);
//				digitalLo(&shootData.bulletBuzyFlag);

//			}
//			break;	
//		}

		default:{
            BIG_GUN_ON;
            shootData.slave_pokeSpeedRef = 0;
			digitalIncreasing(&cleanLoadWakeTime);
             digitalLo(&shootData.bulletfreeFlag);
			if(cleanLoadWakeTime > 0xff){                   
				BIG_GUN_WEAPONS_OFF;
				digitalLo(&shootData.bulletBuzyFlag);
				digitalClan(&cleanLoadWakeTime);			
				digitalClan(&shootData.loadStep);
                
			}    
			break;	
		}      
	}
    lastswitch = RC_GEAR;
	//滚筒堵转保护
	turntableData.currunt=motorHandleClass.Current(&commandoMotorConfig[SLAVE_POKEMOTOR]);
	if(shootData.slave_pokeStall == MOTOR_FOWARD){
		//如果堵转转矩电流大于8500持续500毫秒，说明堵转
		if(turntableData.currunt > 9200){												
			digitalIncreasing(&turnStallCount);
			if(turnStallCount > 0x100 ){
				if(turntableData.currunt > 9200){
					turnWakeTime = xTaskGetTickCount();
					digitalClan(&turnStallCount);
					shootData.slave_pokeStall = MOTOR_REVERSAL;
					//若正在清理弹道则停转
					if(shootData.loadStep == 5)																
						digitalClan(&shootData.slave_pokeSpeedRef);
				}
				else
				  digitalClan(&turnStallCount);				
			}
		}
		else
		  digitalClan(&turnStallCount);
	}
	else if(shootData.slave_pokeStall == MOTOR_REVERSAL){
		//反转200毫秒
		if(xTaskGetTickCount() - turnWakeTime > 0x20){								
			shootData.slave_pokeStall = MOTOR_FOWARD;
			//然后正转
            digitalIncreasing(&shootData.turntableStalltime);
		}
	}
}

/* 拨弹盘更新(2ms) */
static void shootpokeMoterUpdate(uint8_t FricONflag){
	if(!shootData.shootTrigger_17mm || !FricONflag  ||(!shootData.shootTrigger_42mm && (ROBOT == P_TANK_ID || ROBOT == F_TANK_ID))){ 
	    POKE_MOTER_OFF;
	}	
	if(shootData.shootTrigger_17mm){
		shootData.timeing = shootData.loops;
		switch(shootData.shootMode_17mm){
			//单发
			case MANUAL_SINGLE: 
				SLAVE_POKE_SET = POKE_MOTER_SINGLE;
				break;
			//三连发
			case MANUAL_CONTINUOUS: 
				SLAVE_POKE_SET = POKE_MOTER_TRIPLE;
				break;
			//连发
			case AUTO_CONTINUOUS:
				if(ROBOT == SENTRY_ID || ROBOT == SMALLGIMBAL_ID){
					POKE_MOTER_SINGLE;
				}
				else{ 
					if(shootData.clearBulletFlag)
						//场间退弹模式
						SLAVE_POKE_SET = POKE_MOTOR_CLEAR;
					else
						//机枪模式
						SLAVE_POKE_SET = POKE_MOTOR_BURST;					
				}				
				break;
		}
		controlDeviceConfirm(DEVICE_BuiltInPokeMotor,smallPokeSpeedRefChange);     //步兵有内置拨弹盘设备
			
	}
	if(shootData.shootTrigger_42mm && (ROBOT == P_TANK_ID || ROBOT == F_TANK_ID)&&(!(getTankAutoData()->get_pill_Flag))){		
		BIG_POKE_ON;
	}
	if(ROBOT == P_TANK_ID){
		//P_TANK bigpill 
		shootData.slave_pokeSpeedOut = pidUpdate(shootData.slave_pokeSpeedPID,\
											shootData.slave_pokeSpeedRef * shootData.slave_pokeStall,\
											(f32_t)motorHandleClass.Speed(&commandoMotorConfig[SLAVE_POKEMOTOR]),\
											0.002f );
		//P_TANK smallpill 
		shootData.pokeSpeedOut = pidUpdate(shootData.speedPID,\
											shootData.pokeSpeedRef * -shootData.pokeStall * shootData.p_tankshootmode,\
											(f32_t)motorHandleClass.Speed(&commandoMotorConfig[POKEMOTOR]),\
											0.002f);
	}
	else 
		if(ROBOT == F_TANK_ID){
			shootData.pokeSpeedOut = pidUpdate(shootData.speedPID,\
											shootData.pokeSpeedRef * -shootData.pokeStall,\
											-(f32_t)motorHandleClass.Speed(&commandoMotorConfig[POKEMOTOR]),\
											0.002f );
			shootData.pokeSpeedOut = - (shootData.pokeSpeedOut);
		}
	else {
		shootData.pokeSpeedOut = pidUpdate(shootData.speedPID,\
											shootData.pokeSpeedRef * shootData.pokeStall,\
											(f32_t)motorHandleClass.Speed(&commandoMotorConfig[POKEMOTOR]),\
											0.002f );
	}
		
		
}

void  smallPokeSpeedRefChange(void){
	shootData.pokeSpeedRef = -shootData.pokeSpeedRef;
}

/* 17mm手动单发模式 */
static shootStatus_e shootModeManualSingle_17mm(void){
	if(shootData.shooterHeatRemain_17mm >= shootData.safe_heat_17mm){
		return SAFE;//返回射击状态  安全
	}
	else return DANGEROUS;
}
/* 42mm手动单发模式 */
static shootStatus_e shootModeManualSingle_42mm(void){
	if(shootData.shooterHeatRemain_42mm >= shootData.safe_heat_42mm){
		return SAFE;//返回射击状态  安全
	}
	else return DANGEROUS;
}
int Switch;
/* 子弹剩余数计算 */
static void bulletRemainCount(void){
	Switch = SHOOTER_SWITCH;
	if(SHOOTER_SWITCH || (TANK_SHOOTER_SWITCH && (ROBOT == P_TANK_ID))){				//轻触开关没有检测到子弹														
		if(shootData.shootTrigger_17mm && shootData.shootTriggerReserve){
			digitalHi(&shootData.monitorFlag);
			shootData.shootWaitTime = xTaskGetTickCount();
		}
		else if((SHOOTER_SWITCH || (TANK_SHOOTER_SWITCH && (ROBOT == P_TANK_ID))) && shootData.monitorFlag && !shootData.shootTriggerReserve){
			digitalLo(&shootData.shootTrigger_17mm);
			digitalLo(&shootData.monitorFlag);
			digitalHi(&shootData.shootTriggerReserve);
			shootData.pokeSpeedSet = POKE_STATIC_SPEED;
		}
	}
	else{
		if(shootData.shootTrigger_17mm && !shootData.shootTriggerReserve){
			shootData.pokeSpeedSet = POKE_STATIC_SPEED * POKE_SPEED_RATA;
			digitalHi(&shootData.monitorFlag);
			digitalLo(&shootData.shootTriggerReserve);
			shootData.shootWaitTime = xTaskGetTickCount();
		}
		else if((!SHOOTER_SWITCH || (!TANK_SHOOTER_SWITCH && (ROBOT == P_TANK_ID))) && shootData.monitorFlag && shootData.shootTriggerReserve){
			digitalLo(&shootData.shootTrigger_17mm);
			digitalLo(&shootData.monitorFlag);
			digitalLo(&shootData.shootTriggerReserve);
		}
	}
	if(shootData.clearBulletFlag && (shootData.shooterHeatRemain_17mm > 40.0f))
		digitalHi(&shootData.shootTrigger_17mm);		//LCM: 清弹模式下拨弹盘转动
}

/*
摩擦轮转速分配
17mm摩擦轮机构
42mm摩擦轮机构
42mm气动机构+机动17mm摩擦轮机构
*/
f32_t speed__ = 0;
static void  fric_wheel_calc(void){
	if(get_judgeData()->extGameRobotState.shooter_id1_42mm_speed_limit )
 		shootData.speed_limit = get_judgeData()->extGameRobotState.shooter_id1_42mm_speed_limit;
	else
		shootData.speed_limit = get_judgeData()->extGameRobotState.shooter_id1_17mm_speed_limit;
	if(get_judgeData()->extGameRobotState.max_HP){
		//17mm热量计算:热量上限-目前枪口所用的热量
		shootData.shooterHeatRemain_17mm = get_judgeData()->extGameRobotState.shooter_id1_17mm_cooling_limit - shootData.shooterHeat_17mm;    
		//42mm热量计算
		shootData.shooterHeatRemain_42mm = get_judgeData()->extGameRobotState.shooter_id1_42mm_cooling_limit - shootData.shooterHeat_42mm;
	}
	else
		shootData.shooterHeatRemain_17mm = shootData.shooterHeatRemain_42mm = 65535;
	// 每个检测周期热量冷却值 = 每秒冷却值/ 10。
	shootData.shooterHeatCoolingPs_17mm = (f32_t)get_judgeData()->extGameRobotState.shooter_id1_17mm_cooling_rate / 10;  
	shootData.shooterHeatCoolingPs_42mm = (f32_t)get_judgeData()->extGameRobotState.shooter_id1_42mm_cooling_rate / 10;
	switch(shootData.speed_limit){
		//17mm 15m/s
		case SPEED_17MM_LOW:
			shootData.fricSpeedSet_17mm = 4500+ shootData.infantry_add_fire_speed_low;   //四号：4470    三号：4650

			getvisionData()->shootSpeed = 13;
			break;
		//17mm 18m/s
		
		case SPEED_17MM_MID:

			shootData.fricSpeedSet_17mm = 4920+ shootData.infantry_add_fire_speed_mid;   //四号：5100    三号：5140

			getvisionData()->shootSpeed = 16;
			break;
		//17mm 22m/s
		case SPEED_17MM_HIGH:
			shootData.fricSpeedSet_17mm = 5600 + shootData.infantry_add_fire_speed_high;  //四号：5600    三号：5780
			getvisionData()->shootSpeed = 20;
			break;
		//17mm 30m/s
		case SPEED_17MM_EXTREME:

			shootData.fricSpeedSet_17mm = 6788 + shootData.infantry_add_fire_speed_extreme;  //四号：6890    三号：7050

			getvisionData()->shootSpeed = 26;
			break;
		//42mm 10m/s
		case SPEED_42MM_LOW:
			shootData.fricSpeedSet_42mm = 3950;
			getvisionData()->shootSpeed = 9;
			break;
		//42mm 12m/s
		case SPEED_42MM_MID:
			shootData.fricSpeedSet_42mm = 4350;
			getvisionData()->shootSpeed = 11;
			break;
		//42mm 14m/s
		case SPEED_42MM_HIGH:
			shootData.fricSpeedSet_42mm = 4850;
			getvisionData()->shootSpeed = 13;
			break;
		//42mm 16m/s
		case SPEED_42MM_EXTREME:
			shootData.fricSpeedSet_42mm = 5660;
			getvisionData()->shootSpeed = 15;
			break;
		default:
				shootData.fricSpeedSet_17mm = 6600;
				shootData.fricSpeedSet_42mm = 5450;
				getvisionData()->shootSpeed = 26;
			break;
	}
}

/* 计算应打出的射击次数和射击状态 */
static void shootTimesCount(void){
	shootData.shootStatus_17mm = shootModeManualSingle_17mm();
	if(ROBOT == P_TANK_ID || ROBOT == F_TANK_ID)
		shootData.shootStatus_42mm = shootModeManualSingle_42mm();
	shootData.xLastWakeTime = xTaskGetTickCount();
}

/* 超时检测函数 */
static void shootOverTimeCheck_17mm(void){
	switch (shootData.shootMode_17mm){
		case MANUAL_SINGLE :{
            //如果0.2s还没打完单发，就停止射击
			if((xTaskGetTickCount() - shootData.xLastWakeTime) > 100)  
				digitalLo(&shootData.shootTrigger_17mm);																 
		}break;
		case MANUAL_CONTINUOUS :{
            //如果0.6s还没打完三连发，就停止射击
			if((xTaskGetTickCount() - shootData.xLastWakeTime) > 300)  
				digitalLo(&shootData.shootTrigger_17mm);																 
		}break;		
		case AUTO_CONTINUOUS :		break;
	}   		 
}

static void shootOverTimeCheck_42mm(void){
	if((xTaskGetTickCount() - shootData.xLastWakeTime) > 45)
		digitalLo(&shootData.shootTrigger_42mm);
}


/* 射击保护函数 */
static void shootProtect(void){
    //17mm机枪模式
	if(shootData.shootMode_17mm == AUTO_CONTINUOUS){		
        //除危险模式之外
		if(shootData.shootStatusMode_17mm != DANGEROUS){	
            //预留一发子弹的热量
			if(shootData.shooterHeatRemain_17mm < shootData.safe_heat_17mm){
                //机枪模式下为了预防卡弹现象导致的超热量掉血，预留一发子弹的热量(安全模式)
				digitalLo(&shootData.shootTrigger_17mm); 		
			}		
		}		
	}
	if(shootData.shootStatus_17mm > shootData.shootStatusMode_17mm)
        //安全性不满足射击条件就停止射击
		digitalLo(&shootData.shootTrigger_17mm);
	if(shootData.shootStatus_42mm > SAFE)
		digitalLo(&shootData.shootTrigger_42mm);
	//存在裁判系统
	if(get_judgeData()->extGameRobotState.max_HP){
		if(shootData.shooterHeatRemain_17mm < shootData.safe_heat_17mm && shootData.shootStatusMode_17mm == SAFE)
			digitalLo(&shootData.shootTrigger_17mm);
		if(shootData.shooterHeatRemain_42mm < shootData.safe_heat_42mm && shootData.shootStatus_42mm == SAFE)
			digitalLo(&shootData.shootTrigger_42mm);
	}	
}

/* 枪口热量控制函数 */
static void shooterHeatControl(void){
	shootData.shooterHeat_17mm = shootData.shooterHeat_17mm < 0 ? 0 : shootData.shooterHeat_17mm;
	shootData.shooterHeat_42mm = shootData.shooterHeat_42mm < 0 ? 0 : shootData.shooterHeat_42mm;
	shootData.shooterHeat_17mm = shootData.shooterHeat_17mm > 2.0f * get_judgeData()->extGameRobotState.shooter_id1_17mm_cooling_limit ? 2.0f * get_judgeData()->extGameRobotState.shooter_id1_17mm_cooling_limit : shootData.shooterHeat_17mm; 
	shootData.shooterHeat_42mm = shootData.shooterHeat_42mm > 2.0f * get_judgeData()->extGameRobotState.shooter_id1_42mm_cooling_limit ? 2.0f * get_judgeData()->extGameRobotState.shooter_id1_42mm_cooling_limit : shootData.shooterHeat_42mm;
}

/* 枪口热量冷却函数 */
void shooterHeatCooling(void){
	if(!(getcontrolData()->loops % 50)){								//100ms结算一次  10HZ
		shootData.shooterHeat_42mm -= shootData.shooterHeatCoolingPs_42mm;
		shootData.shooterHeat_17mm -= shootData.shooterHeatCoolingPs_17mm;	
	}
	shooterHeatControl();
}

/* 枪口热量增加函数 */
void shooterHeatIncreasing(void){
    //如果射速不相同，则证明打出了一发子弹
	if(get_judgeData()->extShootData.bullet_speed != shootData.shootSpeedLast){		
        //根据不同的子弹类型增加热量
		if(get_judgeData()->extShootData.bullet_type == TYPE_17MM)
			shootData.shooterHeat_17mm += __17MM_HEAT;
		if(get_judgeData()->extShootData.bullet_type == TYPE_42MM)
			shootData.shooterHeat_42mm += __42MM_HEAT;
		shooterHeatControl();
	}
	shootData.shootSpeedLast = get_judgeData()->extShootData.bullet_speed;
}

/* 枪口热量矫正函数 */
void shooterHeatAbjust(void){
	/*裁判系统返回枪口热量数据后就矫正计算的枪口热量 */
	shootData.shooterJudgeHeat_17mm = get_judgeData()->extPowerHeatData.shooter_id1_17mm_cooling_heat;
	shootData.shooterJudgeHeat_42mm = get_judgeData()->extPowerHeatData.shooter_id1_42mm_cooling_heat;
	
	if(shootData.shooterJudgeHeat_17mm != shootData.shooterJudgeLastHeat_17mm){					
		shootData.shooterJudgeLastHeat_17mm = shootData.shooterJudgeHeat_17mm;
	}
	if(shootData.shooterJudgeHeat_42mm != shootData.shooterJudgeLastHeat_42mm){					
		shootData.shooterJudgeLastHeat_42mm = shootData.shooterJudgeHeat_42mm;
	}
	
	if(shootData.shooterJudgeHeat_42mm >= shootData.shooterHeat_42mm)
		shootData.shooterHeat_42mm = shootData.shooterJudgeHeat_42mm;
	if(shootData.shooterJudgeHeat_17mm >= shootData.shooterHeat_17mm){
		shootData.shooterHeat_17mm = shootData.shooterJudgeHeat_17mm;
	}
}

static void shooter_convinced_17mm(void){
	if(!(shootData.loops % shootData.waitTime)){
		//拉高单次初始化标志，防止重复击发
		digitalHi(&shootData.fireDataInit);
		//如果剩余发射数量大于0
		if(shootData.shootManualNumber > 0) {				
            //允许发射
			digitalHi(&shootData.shootTrigger_17mm);	//LCM: 剩余发射数量足够，允许拨弹盘转动
			//17mm机枪模式
			if(shootData.shootMode_17mm != AUTO_CONTINUOUS)			
                //如果不是连发，则等待发射次数减一
				digitalDecline(&shootData.shootManualNumber);
			else if((!KB_17MM_SHOOT_CONTINUOUS)&&(ROBOT != SENTRY_ID)&&(ROBOT != SMALLGIMBAL_ID)){
				digitalLo(&shootData.fireDataInit);
                //清开火标志	
				digitalClan(&FIRE_FLAG_17MM);
				//立马停住拨弹盘
				digitalLo(&shootData.shootTrigger_17mm);
			}
		}
		else{
			digitalLo(&shootData.fireDataInit);
            //清开火标志		
			digitalClan(&FIRE_FLAG_17MM);
		}
	}
	
}

static void shooter_convinced_42mm(void){
	if(!(shootData.loops % shootData.waitTime)){
		//拉高单次初始化标志，防止重复击发
		digitalHi(&shootData.fireDataInit);
		//如果剩余发射数量大于0
		if((shootData.shootManualNumber > 0) && (shootData.shooterHeatRemain_42mm >=__42MM_HEAT)){				
            //允许发射			
			digitalHi(&shootData.shootTrigger_42mm);
			digitalDecline(&shootData.shootManualNumber);
		}
		else{
			digitalLo(&shootData.fireDataInit);
            //清开火标志		
			digitalClan(&FIRE_FLAG_42MM);
		}
	}
}

/*更新射击次数和射击等待时间*/
void shootManualUpdate(void){
	if(!shootData.fireDataInit){					
        //如果单次发射初始化没有拉高射击参数初始化拉低
		//42MM无射击模式
		if(ROBOT == F_TANK_ID){
			shootData.shootManualNumber = 1;
			shootData.waitTime = 40;
		}
		else{			
		    switch(shootData.shootMode_17mm){
				//单发
	            case MANUAL_SINGLE :{
				     //射击次数1次
				     shootData.shootManualNumber = 1;
					 //下次射击等待时间50ms
				     shootData.waitTime = 25;
				     break;
			    }
				//3连发
			    case MANUAL_CONTINUOUS :{
					 //射击次数3次
				     shootData.shootManualNumber = 3;
				     //下次射击等待时间100ms
				     shootData.waitTime = 50;
				     break;
			    }
			    //连发(机枪模式)
			    case AUTO_CONTINUOUS :{
				     //射击次数1次
				     shootData.shootManualNumber = 1;
				     //下次射击等待时间2ms
				     shootData.waitTime = 1;
				     break;
			    }
		}
	}
        //清除循环数，可以立即发射
		digitalClan(&shootData.loops);			
	}
	
	if(ROBOT == P_TANK_ID ||ROBOT == F_TANK_ID)
		shooter_convinced_42mm();
	else 
		shooter_convinced_17mm();
}

bool pokeMotor_stall_search(int16_t motorCurrent,uint16_t stallCurrent){
	if((motorCurrent > stallCurrent) || (motorCurrent < -stallCurrent)) return true;
	return false;
}

//拨叉堵转检测
static void pokeStallCheck(void){
	static uint16_t stallCount = 0;
	static TickType_t xLastWakeTime = 0;
	
	if(shootData.pokeStall == MOTOR_FOWARD){
        //如果堵转转矩电流大于9000持续500毫秒，说明堵转
		//拨弹盘和英雄从拨弹盘，42mm无连发
		if(pokeMotor_stall_search(motorHandleClass.Current(&MAIN_POKE),SHOOT_STALL_CURRENT) || 
			(pokeMotor_stall_search(motorHandleClass.Current(&SLAVE_POKE),SHOOT_STALL_CURRENT) && (ROBOT == P_TANK_ID))){   
			digitalIncreasing(&stallCount);
			if((stallCount > 15 && ROBOT == F_TANK_ID) || (stallCount > 40 && ROBOT != F_TANK_ID)){
				if(pokeMotor_stall_search(motorHandleClass.Current(&MAIN_POKE),SHOOT_STALL_CURRENT) || 
					(pokeMotor_stall_search(motorHandleClass.Current(&SLAVE_POKE),SHOOT_STALL_CURRENT) && (ROBOT == P_TANK_ID))){
					xLastWakeTime = xTaskGetTickCount();
					digitalClan(&stallCount);
					shootData.pokeStall = MOTOR_REVERSAL;
					
				}
				else{
				  digitalClan(&stallCount);
				}					
			}
		}
		else
			digitalClan(&stallCount);
	}
	else if(shootData.pokeStall == MOTOR_REVERSAL){
        //反转100毫秒
		if(xTaskGetTickCount() - xLastWakeTime > 100){
            //然后正转
			shootData.pokeStall = MOTOR_FOWARD;                            
		}
	}
}
//红点控制
static void laser_working(uint8_t __switch){
	if(__switch)
		LASER_OFF;
	else
		LASER_ON;
}

static void fricWheelSwitch(AutoTaskStruct_t *robotTask,robotModeStruct_t robotmode, uint8_t switch_now){
	static uint8_t lastmode = 0;
	static uint8_t lastswitch = 0;
	
    //如果机器人有模式切换
    if(lastmode != robotmode){                        		
        digitalHi(&shootData.laser_close);
        //摩擦轮关闭，气动发射准备关闭
        digitalLo(&shootData.fricMotorFlag);							
        //关闭大弹仓
        digitalLo(&P_HERO_42_LID);									
        //关闭小弹仓
        digitalLo(&P_HERO_17_LID);									
	}
	else{                             
        //在摇杆或键鼠模式下
        if(robotmode == MODE_RC){												
            //SW1打到最低档时摩擦轮关闭
			if(switch_now == RCSW_BOTTOM){               											
                //激光关
				digitalHi(&shootData.laser_close);																			
                //摩擦轮关闭
				digitalLo(&shootData.fricMotorFlag);												
			}
			else if(robotmode == MODE_RC && lastswitch == RCSW_BOTTOM && switch_now == RCSW_MID && ROBOT != SENTRY_ID){												
                //只有RC模式SW1从最低档打到中间档时摩擦轮开启同時開啓激光							
				digitalHi(&shootData.fricMotorFlag); 
        digitalLo(&shootData.laser_close);				
			}
			else if(lastswitch == RCSW_BOTTOM && switch_now == RCSW_MID)//哨兵不開啓激光
				digitalHi(&shootData.fricMotorFlag); 
		}
			else if(robotmode == MODE_KM){
								if(ROBOT==INFANTRY_ID){             //PY: KM模式下的摩擦轮开关逻辑
			if((switch_now == RCSW_MID && ROBOT != SENTRY_ID && getinfantryFricState()->infantryFricStateFlag == 1)||(lastswitch == RCSW_BOTTOM && switch_now == RCSW_MID)){												
                //KM模式SW1打到中间档时，且摩擦轮标志位为1时摩擦轮开启同時開啓激光			
				getinfantryFricState()->infantryFricStateFlag = 1;
				digitalHi(&shootData.fricMotorFlag); 
        digitalLo(&shootData.laser_close);				
			}
			else if(switch_now == RCSW_BOTTOM || getinfantryFricState()->infantryFricStateFlag == 0){               											
                //激光关
				digitalHi(&shootData.laser_close);																			
                //摩擦轮关闭
				digitalLo(&shootData.fricMotorFlag);												
			} 
                }
            if(ROBOT==F_TANK_ID){
			if(switch_now == RCSW_BOTTOM || getftankFricState()->ftankFricStateFlag == 0){               											
                //激光关
				digitalHi(&shootData.laser_close);																			
                //摩擦轮关闭
				digitalLo(&shootData.fricMotorFlag);												
			}
			else if(switch_now == RCSW_MID && ROBOT != SENTRY_ID && getftankFricState()->ftankFricStateFlag == 1){												
                //KM模式SW1打到中间档时，且摩擦轮标志位为1时摩擦轮开启同時開啓激光									
				digitalHi(&shootData.fricMotorFlag); 
        digitalLo(&shootData.laser_close);				
			}
                }
		}			
	
		
		else{
			digitalHi(&shootData.laser_close);
            //除摇杆或者键鼠模式外摩擦轮都关闭
            digitalLo(&shootData.fricMotorFlag);              
		}
	}

	//红点控制
	laser_working(shootData.laser_close);
    //已执行的机器人模式看作上次的模式
	lastmode = robotmode;																	
    //已执行的SW1看作上次的SW1		RC_GEAR	
	lastswitch = switch_now;															
}

static void shoot_in_modeRC(uint8_t lastSwitch){
	//RC模式且拨轮拨动步兵进入清弹模式
	//步兵场间快速清弹
	if(RC_ROTATE < -100 && ROBOT == INFANTRY_ID){					
		shootData.shootMode_17mm = AUTO_CONTINUOUS;
		shootData.clearBulletFlag = true;
	}
	//RC模式下只有单发
	else{
		shootData.shootMode_17mm = MANUAL_SINGLE;
		shootData.clearBulletFlag = false;
	}
	//左拨杆从中间拨到顶部
	if(lastSwitch == RCSW_MID && RC_GEAR == RCSW_TOP){
		if (ROBOT == F_TANK_ID)
			digitalHi(&shootData.fireFlag_42mm);
		else
			if(ROBOT == P_TANK_ID){
		        digitalHi(&shootData.fireFlag_42mm);	//LCM: RC模式拨杆击发
		        digitalHi(&shootData.fireFlag_17mm);	//LCM: RC模式拨杆击发
			}
		else
			digitalHi(&shootData.fireFlag_17mm);		//LCM: RC模式拨杆击发
	}
	// 无人机RC模式下连发 
	else if(RC_GEAR == RCSW_TOP&&lastSwitch == RCSW_TOP){
		if(ROBOT == UAV_ID){
			digitalHi(&shootData.shootTrigger_17mm);	//LCM: RC模式拨杆击发
			digitalHi(&shootData.fireFlag_17mm);		//LCM: RC模式拨杆击发
		}
	}
	//射击参数重置
	else if(RC_GEAR != RCSW_TOP) shootDataReset();
	else if(RC_GEAR == RCSW_BOTTOM) shootData.gsTime = 0;
}

double time1;
static void  shoot_in_modeKM(uint8_t lastPress,uint8_t lastSwitch){
	//键鼠没有清弹功能
	shootData.clearBulletFlag = false;
		//左拨杆从中间拨到顶部
	if(lastSwitch == RCSW_MID && RC_GEAR == RCSW_TOP){
		if (ROBOT == F_TANK_ID)
		digitalHi(&shootData.fireFlag_42mm);
		else
			if(ROBOT == P_TANK_ID){
		        digitalHi(&shootData.fireFlag_42mm);	//LCM: KM模式拨杆击发
		        digitalHi(&shootData.fireFlag_17mm);	//LCM: KM模式拨杆击发
			}
		else
			digitalHi(&shootData.fireFlag_17mm);		//LCM: KM模式拨杆击发
	}
	if(KB_TYPY_SHOOT && !lastPress){         //键鼠模式按下B键改变射击状态
		digitalIncreasing(&shootData.shootMode_17mm);
		if(shootData.shootMode_17mm > AUTO_CONTINUOUS)
			shootData.shootMode_17mm = MANUAL_SINGLE;
	}
	if(ROBOT == UAV_ID || ROBOT == SENTRY_ID || ROBOT == SMALLGIMBAL_ID)
		shootData.shootMode_17mm = AUTO_CONTINUOUS;
	//安全模式和危险模式之间切换
	else if(KB_TYPY_HEAT_PROTECT)
		//无警告模式						 
		shootData.shootStatusMode_17mm = DANGEROUS;
	else
		shootData.shootStatusMode_17mm = SAFE;//无热量控制模式
	
	if(robotConfigData.typeOfRobot == P_TANK_ID || robotConfigData.typeOfRobot == F_TANK_ID) {
		if(KB_42MM_SHOOT){
			digitalHi(&shootData.fireFlag_42mm);	
		}	
	}
	else {
		if(shootData.shootMode_17mm == MANUAL_SINGLE || shootData.shootMode_17mm == MANUAL_CONTINUOUS){
			if(KB_17MM_SHOOT){
				digitalHi(&shootData.fireFlag_17mm);	
				time1 = getClockCount();
			}
		}
		else if(shootData.shootMode_17mm == AUTO_CONTINUOUS){
			if(KB_17MM_SHOOT_CONTINUOUS)
				digitalHi(&shootData.fireFlag_17mm);	
		}
	}
}

void getShootMode(void){
	static uint8_t lastswitch = 0;
	static uint8_t KB_TYPY_SHOOT_LAST = 0;
	switch(ROBOT){
		case INFANTRY_ID:
			fricWheelSwitch(getinfantryAutoData(),getrobotMode(),RC_GEAR);
		break;
		case P_TANK_ID:
			fricWheelSwitch(getTankAutoData(),getrobotMode(),RC_GEAR);
		break;
		case F_TANK_ID:
			fricWheelSwitch(getTankAutoData(),getrobotMode(),RC_GEAR);
		break;
		case AUXILIARY_ID:
			fricWheelSwitch(getauxiliaryAutoData(),getrobotMode(),RC_GEAR);
		break;
		case SENTRY_ID:
			fricWheelSwitch(getsentryAutoData(),getrobotMode(),RC_GEAR);
		break;
		case SMALLGIMBAL_ID:
			fricWheelSwitch(getsentryAutoData(),getrobotMode(),RC_GEAR);
		break;
		case UAV_ID:
			fricWheelSwitch(getuavAutoData(),getrobotMode(),RC_GEAR);
		break;
	}
	if(getrobotMode() == MODE_RELAX){
        //释放控制权后回到单发模式
		shootData.shootMode_17mm = MANUAL_SINGLE;								
        //开始弹道填充
		digitalHi(&shootData.ballisticFill);									
	}	
    //摩擦轮开启状态才可以发射子弹
	if(shootData.fricMotorFlag){											
        //遥控器控制模式
		if(getrobotMode() == MODE_RC)
			shoot_in_modeRC(lastswitch);
        //键盘控制模式
		else if(getrobotMode() == MODE_KM)
			shoot_in_modeKM(KB_TYPY_SHOOT_LAST,lastswitch);
	}
	else{
		digitalLo(&shootData.fireFlag_17mm);
		digitalLo(&shootData.fireFlag_42mm);
	}	
	KB_TYPY_SHOOT_LAST = KB_TYPY_SHOOT;
	lastswitch = RC_GEAR;																									
}
double time2,time3;
/* 射击函数更新 */
void shootUpdate(void){
    //计算剩余枪口热量和当前摩擦轮转速
	 fric_wheel_calc();			
    //如果接收到开火信号，根据当前的射击模式判断应发射的子弹数，并计算安全性
	if(FIRE_FLAG_17MM || FIRE_FLAG_42MM){						
		shootManualUpdate();
		shootTimesCount();	
	}
	if(ROBOT == P_TANK_ID){
		p_heroAutoLoad();
	}
	shootProtect();
    //射击超时检测,根据收到开火信号的时间确定射击命令是否超时，如果超时则清零射击次数
	shootOverTimeCheck_17mm();
	shootOverTimeCheck_42mm();
    //计算剩余子弹,并判断是否停止拨弹盘
	bulletRemainCount();	
  if(!SHOOTER_SWITCH)	{
		time2 = getClockCount();
		time3 = time2 - time1;
		time1 ++;
	}
	
#if SNAIL_MOTOR 
	if(!(getcontrolData()->loops % 5)){
        //摩擦轮电机输出10ms一次
		shootFrictiongearUpdate(shootData.fricMotorFlag);	
	}
#else
	fricWheel_update(shootData.fricMotorFlag);
#endif
    //拨叉电机堵转检测
	pokeStallCheck();         
    //拨弹盘电机输出,最终开火指令
	shootpokeMoterUpdate(shootData.fricMotorFlag);
    //热量冷却
	shooterHeatCooling();			
	digitalIncreasing(&shootData.loops);
	shooterHeatAbjust();
	shooterHeatIncreasing(); 	
//    //打弹函数
//	heroFire();
	switch(shootData.shootMode_17mm){
		case MANUAL_SINGLE:
			shootData.shootmode_ui = 1;
		break;
		case  MANUAL_CONTINUOUS:
			shootData.shootmode_ui = 2;
		break;
		case AUTO_CONTINUOUS:
			shootData.shootmode_ui = 3;
		break;
	}
	shootData.time[0] = getClockCount();
	shootData.intervalTime = (f32_t)(shootData.time[0] - shootData.time[1]);
	shootData.time[1] = shootData.time[0];
//摩擦轮PID计算	
	if(ROBOT == P_TANK_ID){
		shootData.fricWheelSpeedOut[0] = pidUpdate(shootData.fricWheelSpeedPID[0],\
												   shootData.fricWheelSpeedRef[0],\
												   (f32_t)motorHandleClass.Speed(&commandoMotorConfig[HERO_FRICLMOTOR]),\
												   shootData.intervalTime);		
		shootData.fricWheelSpeedOut[1] = -pidUpdate(shootData.fricWheelSpeedPID[1],\
													shootData.fricWheelSpeedRef[1],\
													-(f32_t)motorHandleClass.Speed(&commandoMotorConfig[HERO_FRICRMOTOR]),\
													shootData.intervalTime);											
	}
	else{
		shootData.fricWheelSpeedOut[0] = pidUpdate(shootData.fricWheelSpeedPID[0],\
												   shootData.fricWheelSpeedRef[0],\
												   (f32_t)motorHandleClass.Speed(&commandoMotorConfig[FRICRMOTOR]),\
												   shootData.intervalTime);		
		shootData.fricWheelSpeedOut[1] = -pidUpdate(shootData.fricWheelSpeedPID[1],\
													shootData.fricWheelSpeedRef[1],\
													-(f32_t)motorHandleClass.Speed(&commandoMotorConfig[FRICLMOTOR]),\
													shootData.intervalTime);
		
		usbVCP_Printf("l = %f\r\n",-(f32_t)motorHandleClass.Speed(&commandoMotorConfig[FRICLMOTOR]));
		usbVCP_Printf("o = %f\r\n",shootData.fricWheelSpeedRef[1]);
		usbVCP_Printf("r = %f\r\n",(f32_t)motorHandleClass.Speed(&commandoMotorConfig[FRICRMOTOR]));
	}																				
	if(shootData.speeding != judgeData.extShootData.bullet_speed){
		shootData.gsTime = shootData.loops - shootData.timeing;
	}
	shootData.speeding = judgeData.extShootData.bullet_speed;
	//拨弹盘PID计算
    //如果是释放控制权模式或停止模式
	if(getrobotMode() == MODE_RELAX||getrobotMode() == MODE_STOP){																
        //摩擦轮不开，拨弹电机也不开				
        shootData.fricWheelSpeedOut[0] = shootData.fricWheelSpeedOut[1] = 0;											
		shootData.pokeSpeedOut = shootData.bigPokeSpeedOut = 0;
		shootData.supplySpeedOut = 0.0f;
		
	}
}
  

f32_t supsped = 0;
f32_t spud = 0;
void supplyUpdate(void){
    shootData.supplyTime[0] = getClockCount();
	shootData.supplyIntervalTime = (f32_t)(shootData.supplyTime[0] - shootData.supplyTime[1]);
	shootData.supplyTime[1] = shootData.supplyTime[0];
    //小弹仓盖PID更新
    shootData.supplySpeedOut = pidUpdate(shootData.shootsupplyPID,\
										  shootData.supplyRef,\
										 (f32_t)motorHandleClass.Speed(&commandoMotorConfig[SUPPLYMOTOR]),\
										 shootData.supplyIntervalTime);
}

static void safe_heat_set(void){
	//存在裁判系统时有安全热量保护
	if(get_judgeData()->extGameRobotState.max_HP){
		getshootData()->safe_heat_17mm = __17MM_HEAT;
		getshootData()->safe_heat_42mm = __42MM_HEAT;
	}
	else{
		getshootData()->safe_heat_17mm = __NO_HEAT;
		getshootData()->safe_heat_42mm = __NO_HEAT;
	}
	
}

/* 发射机构初始化 */
void shootInit(void){		
#if SNAIL_MOTOR 	
	//两个无刷电机,频率400Hz																								
	BSP_TIM_PWM_Init( TIM9, 2500-1, 168-1, BSP_GPIOE5, BSP_GPIOE6, NULL,NULL );	
	FricMotor_L = FricMotor_R = 1000;
#endif	
	//激光初始化	
	BSP_GPIO_Init(LASER_GPIO,GPIO_Mode_Out_PP);
	//轻触开关初始化
	BSP_GPIO_Init(SHOOTER_SWITCH_GPIO,GPIO_Mode_IPU);
	//分配安全热量
	safe_heat_set();
	shootData.speedPID = pidInit(&getConfigData()->shootSmallDialPID);
    shootData.slave_pokeSpeedPID = pidInit(&getConfigData()->shootLargeDialPID);	
	/**/
	shootData.fricWheelSpeedPID[0]  = pidInit(&getConfigData()->shootFricPID); 
	shootData.fricWheelSpeedPID[1]  = pidInit(&getConfigData()->shootFricPID); 
	/**/
    shootData.shootsupplyPID = pidInit(&getConfigData()->shootSmallDialPID); 
	if (ROBOT == F_TANK_ID )
		shootData.supplyRef = 150;
	else
	    shootData.supplyRef = 80;
	shootDataReset();
    shootData.pokeStall = MOTOR_FOWARD;
    //默认单发
	shootData.shootMode_17mm = MANUAL_SINGLE;
    //默认不开火 不开摩擦轮 没有收到补给标志 单发安全模式
	shootData.shootStatusMode_17mm = SAFE;
	if(ROBOT == SENTRY_ID){
		shootData.shootMode_17mm = AUTO_CONTINUOUS;	//连发
        //17mm摩擦轮转速默认值
		shootData.fricSpeedSet_17mm = 6600;
	}
	/**/
	else{
        //17mm摩擦轮转速默认值
		shootData.fricSpeedSet_17mm = 6560;
	}
	/**/
    //42mm摩擦轮转速默认值
	shootData.fricSpeedSet_42mm = 5650;
	shootData.pokeSpeedSet = POKE_STATIC_SPEED;
	shootData.slave_pokeStall = MOTOR_FOWARD;											//然后正转
	shootData.timeing = 0;
	shootData.gsTime = 0;
	shootData.lasttimez = 0;
	shootData.speeding = 0;
}
