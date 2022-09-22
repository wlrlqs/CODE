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
           
#define POKE_MOTER_SINGLE						 		(shootData.pokeSpeedRef = shootData.pokeSpeedSet) //17mm�����̵���ת��
#define POKE_MOTER_TRIPLE							    (shootData.pokeSpeedRef = shootData.pokeSpeedSet)	//17mm������������ת��
#define POKE_MOTOR_BURST						  	    (shootData.pokeSpeedRef = shootData.pokeSpeedSet)	//17mm����������ת��
#define POKE_MOTOR_CLEAR								(shootData.pokeSpeedRef = 0.4f * shootData.pokeSpeedSet)	//17mm�������˵�ת��
#define BIG_POKE_ON										(shootData.pokeSpeedRef = 8500)	//42mm�����̵���ת��
#define POKE_MOTER_OFF							        (shootData.pokeSpeedRef = 0)
#define FIRC_42MM_WHEEL_ON							    (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_42mm)		//42mmĦ����ת��
#define FIRC_42MM_WHEEL_OFF							    (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0)
/**/
#define FIRC_17MM_WHEEL_SINGLE					        (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_17mm)  //17mmĦ����ת�٣�����
#define FIRC_17MM_WHEEL_TRIPLE					        (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_17mm)  //������
#define FIRC_17MM_WHEEL_BURST						    (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_17mm) //����
#define FIRC_17MM_CLEAR_BULLET                          (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0.1f * shootData.fricSpeedSet_17mm)  //�˵�
#define FIRC_17MM_WHEEL_OFF                             (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0)
/**/
#define SHOOT_PWM_VARIABLE_QUANTITY			            2

#define BULLET_READY										shootData.bulletMonitorFlag
#define GUN_READY												!pneumaticData.pneumatic_1.read1[0]    // �ȴ��޸Ĵ��߼���ɾ��
#define GIMBAL_READY										gimbalData.initFinishFlag

extern shootStruct_t shootData;
shootStruct_t* getshootData(){
    return &shootData;
}
/* ����������� */
void shootDataReset(void){
	digitalLo(&shootData.fireFlag_17mm);
	digitalLo(&shootData.fireFlag_42mm);
	digitalLo(&shootData.shootManualNumber);
	digitalLo(&shootData.fireDataInit);
	digitalLo(&shootData.shootTrigger_17mm);
	digitalLo(&shootData.shootTrigger_42mm);
}

#if SNAIL_MOTOR 
	/* Ħ���ָ���(10ms) */
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
				//��������ģʽ���26m/s����
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
		//����
		if(shootData.shootMode_17mm == MANUAL_SINGLE)
			FIRC_17MM_WHEEL_SINGLE;	
		//������
		else if(shootData.shootMode_17mm == MANUAL_CONTINUOUS)
			FIRC_17MM_WHEEL_TRIPLE;
		//����
		else if(shootData.shootMode_17mm == AUTO_CONTINUOUS){
			if(ROBOT == SENTRY_ID || ROBOT == SMALLGIMBAL_ID){
				FIRC_17MM_WHEEL_SINGLE;//17mmĦ����ת��Ĭ��ֵ
			}
			else{
				FIRC_17MM_WHEEL_BURST;
			}
		}
		//�˵�
		else if(shootData.clearBulletFlag)
			FIRC_17MM_CLEAR_BULLET;
		if(ROBOT == F_TANK_ID)
			//Ӣ��42mmĦ����
			FIRC_42MM_WHEEL_ON;
	}
	else{
		FIRC_17MM_WHEEL_OFF;
		FIRC_42MM_WHEEL_OFF;
	}
}



static void p_heroAutoLoad(void){
	//��Ͳֹͣ��־λ
    static bool turnStallstoplflag;
	//װ��ʱ����
	static uint16_t cleanLoadWakeTime;									
	static uint16_t turnStallCount = 0;
	static TickType_t turnWakeTime = 0;
	static uint8_t lastswitch = 0 ;
    static uint8_t lastswitch_Mode = 0 ;
	if(shootData.fricMotorFlag)
		//����
		FIRC_17MM_WHEEL_SINGLE;
	else if(shootData.clearBulletFlag)
		//�˵�
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
                    BIG_GUN_WEAPONS_ON;//��ǰ��  
                    BIG_GUN_ON;      //���� 
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
                BIG_GUN_WEAPONS_OFF;	//��������		
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
             BIG_GUN_ON;		           //����	
            if(GUN_AWAIT2 || GUN_AWAIT1)
            {
                digitalIncreasing(&cleanLoadWakeTime);
                shootData.slave_pokeSpeedRef = 1800;
                if(cleanLoadWakeTime > 0x35)
                {     
                    BIG_GUN_WEAPONS_ON;		   //����		
                    
                    shootData.slave_pokeSpeedRef = 0;
                    digitalIncreasing(&shootData.loadStep);
                    digitalHi(&shootData.bulletBuzyFlag);
                    digitalClan(&cleanLoadWakeTime);
                }
            }
			else
                {
                    BIG_GUN_WEAPONS_OFF;           //����
//                    if(ELEC_MAG_SENSOR_BEHIND)     //�������Ĵſ��ؼ�⵽�������ò�����ת             
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
					    BIG_GUN_OFF;                       //����
                        digitalClan(&cleanLoadWakeTime);
                        
                    }                        
					digitalLo(&shootData.bulletBuzyFlag);					
					//���Ͷ�ת��־λ
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
				//����
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
	//��Ͳ��ת����
	turntableData.currunt=motorHandleClass.Current(&commandoMotorConfig[SLAVE_POKEMOTOR]);
	if(shootData.slave_pokeStall == MOTOR_FOWARD){
		//�����תת�ص�������8500����500���룬˵����ת
		if(turntableData.currunt > 9200){												
			digitalIncreasing(&turnStallCount);
			if(turnStallCount > 0x100 ){
				if(turntableData.currunt > 9200){
					turnWakeTime = xTaskGetTickCount();
					digitalClan(&turnStallCount);
					shootData.slave_pokeStall = MOTOR_REVERSAL;
					//��������������ͣת
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
		//��ת200����
		if(xTaskGetTickCount() - turnWakeTime > 0x20){								
			shootData.slave_pokeStall = MOTOR_FOWARD;
			//Ȼ����ת
            digitalIncreasing(&shootData.turntableStalltime);
		}
	}
}

/* �����̸���(2ms) */
static void shootpokeMoterUpdate(uint8_t FricONflag){
	if(!shootData.shootTrigger_17mm || !FricONflag  ||(!shootData.shootTrigger_42mm && (ROBOT == P_TANK_ID || ROBOT == F_TANK_ID))){ 
	    POKE_MOTER_OFF;
	}	
	if(shootData.shootTrigger_17mm){
		shootData.timeing = shootData.loops;
		switch(shootData.shootMode_17mm){
			//����
			case MANUAL_SINGLE: 
				SLAVE_POKE_SET = POKE_MOTER_SINGLE;
				break;
			//������
			case MANUAL_CONTINUOUS: 
				SLAVE_POKE_SET = POKE_MOTER_TRIPLE;
				break;
			//����
			case AUTO_CONTINUOUS:
				if(ROBOT == SENTRY_ID || ROBOT == SMALLGIMBAL_ID){
					POKE_MOTER_SINGLE;
				}
				else{ 
					if(shootData.clearBulletFlag)
						//�����˵�ģʽ
						SLAVE_POKE_SET = POKE_MOTOR_CLEAR;
					else
						//��ǹģʽ
						SLAVE_POKE_SET = POKE_MOTOR_BURST;					
				}				
				break;
		}
		controlDeviceConfirm(DEVICE_BuiltInPokeMotor,smallPokeSpeedRefChange);     //���������ò������豸
			
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

/* 17mm�ֶ�����ģʽ */
static shootStatus_e shootModeManualSingle_17mm(void){
	if(shootData.shooterHeatRemain_17mm >= shootData.safe_heat_17mm){
		return SAFE;//�������״̬  ��ȫ
	}
	else return DANGEROUS;
}
/* 42mm�ֶ�����ģʽ */
static shootStatus_e shootModeManualSingle_42mm(void){
	if(shootData.shooterHeatRemain_42mm >= shootData.safe_heat_42mm){
		return SAFE;//�������״̬  ��ȫ
	}
	else return DANGEROUS;
}
int Switch;
/* �ӵ�ʣ�������� */
static void bulletRemainCount(void){
	Switch = SHOOTER_SWITCH;
	if(SHOOTER_SWITCH || (TANK_SHOOTER_SWITCH && (ROBOT == P_TANK_ID))){				//�ᴥ����û�м�⵽�ӵ�														
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
		digitalHi(&shootData.shootTrigger_17mm);		//LCM: �嵯ģʽ�²�����ת��
}

/*
Ħ����ת�ٷ���
17mmĦ���ֻ���
42mmĦ���ֻ���
42mm��������+����17mmĦ���ֻ���
*/
f32_t speed__ = 0;
static void  fric_wheel_calc(void){
	if(get_judgeData()->extGameRobotState.shooter_id1_42mm_speed_limit )
 		shootData.speed_limit = get_judgeData()->extGameRobotState.shooter_id1_42mm_speed_limit;
	else
		shootData.speed_limit = get_judgeData()->extGameRobotState.shooter_id1_17mm_speed_limit;
	if(get_judgeData()->extGameRobotState.max_HP){
		//17mm��������:��������-Ŀǰǹ�����õ�����
		shootData.shooterHeatRemain_17mm = get_judgeData()->extGameRobotState.shooter_id1_17mm_cooling_limit - shootData.shooterHeat_17mm;    
		//42mm��������
		shootData.shooterHeatRemain_42mm = get_judgeData()->extGameRobotState.shooter_id1_42mm_cooling_limit - shootData.shooterHeat_42mm;
	}
	else
		shootData.shooterHeatRemain_17mm = shootData.shooterHeatRemain_42mm = 65535;
	// ÿ���������������ȴֵ = ÿ����ȴֵ/ 10��
	shootData.shooterHeatCoolingPs_17mm = (f32_t)get_judgeData()->extGameRobotState.shooter_id1_17mm_cooling_rate / 10;  
	shootData.shooterHeatCoolingPs_42mm = (f32_t)get_judgeData()->extGameRobotState.shooter_id1_42mm_cooling_rate / 10;
	switch(shootData.speed_limit){
		//17mm 15m/s
		case SPEED_17MM_LOW:
			shootData.fricSpeedSet_17mm = 4500+ shootData.infantry_add_fire_speed_low;   //�ĺţ�4470    ���ţ�4650

			getvisionData()->shootSpeed = 13;
			break;
		//17mm 18m/s
		
		case SPEED_17MM_MID:

			shootData.fricSpeedSet_17mm = 4920+ shootData.infantry_add_fire_speed_mid;   //�ĺţ�5100    ���ţ�5140

			getvisionData()->shootSpeed = 16;
			break;
		//17mm 22m/s
		case SPEED_17MM_HIGH:
			shootData.fricSpeedSet_17mm = 5600 + shootData.infantry_add_fire_speed_high;  //�ĺţ�5600    ���ţ�5780
			getvisionData()->shootSpeed = 20;
			break;
		//17mm 30m/s
		case SPEED_17MM_EXTREME:

			shootData.fricSpeedSet_17mm = 6788 + shootData.infantry_add_fire_speed_extreme;  //�ĺţ�6890    ���ţ�7050

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

/* ����Ӧ�����������������״̬ */
static void shootTimesCount(void){
	shootData.shootStatus_17mm = shootModeManualSingle_17mm();
	if(ROBOT == P_TANK_ID || ROBOT == F_TANK_ID)
		shootData.shootStatus_42mm = shootModeManualSingle_42mm();
	shootData.xLastWakeTime = xTaskGetTickCount();
}

/* ��ʱ��⺯�� */
static void shootOverTimeCheck_17mm(void){
	switch (shootData.shootMode_17mm){
		case MANUAL_SINGLE :{
            //���0.2s��û���굥������ֹͣ���
			if((xTaskGetTickCount() - shootData.xLastWakeTime) > 100)  
				digitalLo(&shootData.shootTrigger_17mm);																 
		}break;
		case MANUAL_CONTINUOUS :{
            //���0.6s��û��������������ֹͣ���
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


/* ����������� */
static void shootProtect(void){
    //17mm��ǹģʽ
	if(shootData.shootMode_17mm == AUTO_CONTINUOUS){		
        //��Σ��ģʽ֮��
		if(shootData.shootStatusMode_17mm != DANGEROUS){	
            //Ԥ��һ���ӵ�������
			if(shootData.shooterHeatRemain_17mm < shootData.safe_heat_17mm){
                //��ǹģʽ��Ϊ��Ԥ�����������µĳ�������Ѫ��Ԥ��һ���ӵ�������(��ȫģʽ)
				digitalLo(&shootData.shootTrigger_17mm); 		
			}		
		}		
	}
	if(shootData.shootStatus_17mm > shootData.shootStatusMode_17mm)
        //��ȫ�Բ��������������ֹͣ���
		digitalLo(&shootData.shootTrigger_17mm);
	if(shootData.shootStatus_42mm > SAFE)
		digitalLo(&shootData.shootTrigger_42mm);
	//���ڲ���ϵͳ
	if(get_judgeData()->extGameRobotState.max_HP){
		if(shootData.shooterHeatRemain_17mm < shootData.safe_heat_17mm && shootData.shootStatusMode_17mm == SAFE)
			digitalLo(&shootData.shootTrigger_17mm);
		if(shootData.shooterHeatRemain_42mm < shootData.safe_heat_42mm && shootData.shootStatus_42mm == SAFE)
			digitalLo(&shootData.shootTrigger_42mm);
	}	
}

/* ǹ���������ƺ��� */
static void shooterHeatControl(void){
	shootData.shooterHeat_17mm = shootData.shooterHeat_17mm < 0 ? 0 : shootData.shooterHeat_17mm;
	shootData.shooterHeat_42mm = shootData.shooterHeat_42mm < 0 ? 0 : shootData.shooterHeat_42mm;
	shootData.shooterHeat_17mm = shootData.shooterHeat_17mm > 2.0f * get_judgeData()->extGameRobotState.shooter_id1_17mm_cooling_limit ? 2.0f * get_judgeData()->extGameRobotState.shooter_id1_17mm_cooling_limit : shootData.shooterHeat_17mm; 
	shootData.shooterHeat_42mm = shootData.shooterHeat_42mm > 2.0f * get_judgeData()->extGameRobotState.shooter_id1_42mm_cooling_limit ? 2.0f * get_judgeData()->extGameRobotState.shooter_id1_42mm_cooling_limit : shootData.shooterHeat_42mm;
}

/* ǹ��������ȴ���� */
void shooterHeatCooling(void){
	if(!(getcontrolData()->loops % 50)){								//100ms����һ��  10HZ
		shootData.shooterHeat_42mm -= shootData.shooterHeatCoolingPs_42mm;
		shootData.shooterHeat_17mm -= shootData.shooterHeatCoolingPs_17mm;	
	}
	shooterHeatControl();
}

/* ǹ���������Ӻ��� */
void shooterHeatIncreasing(void){
    //������ٲ���ͬ����֤�������һ���ӵ�
	if(get_judgeData()->extShootData.bullet_speed != shootData.shootSpeedLast){		
        //���ݲ�ͬ���ӵ�������������
		if(get_judgeData()->extShootData.bullet_type == TYPE_17MM)
			shootData.shooterHeat_17mm += __17MM_HEAT;
		if(get_judgeData()->extShootData.bullet_type == TYPE_42MM)
			shootData.shooterHeat_42mm += __42MM_HEAT;
		shooterHeatControl();
	}
	shootData.shootSpeedLast = get_judgeData()->extShootData.bullet_speed;
}

/* ǹ�������������� */
void shooterHeatAbjust(void){
	/*����ϵͳ����ǹ���������ݺ�ͽ��������ǹ������ */
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
		//���ߵ��γ�ʼ����־����ֹ�ظ�����
		digitalHi(&shootData.fireDataInit);
		//���ʣ�෢����������0
		if(shootData.shootManualNumber > 0) {				
            //������
			digitalHi(&shootData.shootTrigger_17mm);	//LCM: ʣ�෢�������㹻����������ת��
			//17mm��ǹģʽ
			if(shootData.shootMode_17mm != AUTO_CONTINUOUS)			
                //���������������ȴ����������һ
				digitalDecline(&shootData.shootManualNumber);
			else if((!KB_17MM_SHOOT_CONTINUOUS)&&(ROBOT != SENTRY_ID)&&(ROBOT != SMALLGIMBAL_ID)){
				digitalLo(&shootData.fireDataInit);
                //�忪���־	
				digitalClan(&FIRE_FLAG_17MM);
				//����ͣס������
				digitalLo(&shootData.shootTrigger_17mm);
			}
		}
		else{
			digitalLo(&shootData.fireDataInit);
            //�忪���־		
			digitalClan(&FIRE_FLAG_17MM);
		}
	}
	
}

static void shooter_convinced_42mm(void){
	if(!(shootData.loops % shootData.waitTime)){
		//���ߵ��γ�ʼ����־����ֹ�ظ�����
		digitalHi(&shootData.fireDataInit);
		//���ʣ�෢����������0
		if((shootData.shootManualNumber > 0) && (shootData.shooterHeatRemain_42mm >=__42MM_HEAT)){				
            //������			
			digitalHi(&shootData.shootTrigger_42mm);
			digitalDecline(&shootData.shootManualNumber);
		}
		else{
			digitalLo(&shootData.fireDataInit);
            //�忪���־		
			digitalClan(&FIRE_FLAG_42MM);
		}
	}
}

/*�����������������ȴ�ʱ��*/
void shootManualUpdate(void){
	if(!shootData.fireDataInit){					
        //������η����ʼ��û���������������ʼ������
		//42MM�����ģʽ
		if(ROBOT == F_TANK_ID){
			shootData.shootManualNumber = 1;
			shootData.waitTime = 40;
		}
		else{			
		    switch(shootData.shootMode_17mm){
				//����
	            case MANUAL_SINGLE :{
				     //�������1��
				     shootData.shootManualNumber = 1;
					 //�´�����ȴ�ʱ��50ms
				     shootData.waitTime = 25;
				     break;
			    }
				//3����
			    case MANUAL_CONTINUOUS :{
					 //�������3��
				     shootData.shootManualNumber = 3;
				     //�´�����ȴ�ʱ��100ms
				     shootData.waitTime = 50;
				     break;
			    }
			    //����(��ǹģʽ)
			    case AUTO_CONTINUOUS :{
				     //�������1��
				     shootData.shootManualNumber = 1;
				     //�´�����ȴ�ʱ��2ms
				     shootData.waitTime = 1;
				     break;
			    }
		}
	}
        //���ѭ������������������
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

//�����ת���
static void pokeStallCheck(void){
	static uint16_t stallCount = 0;
	static TickType_t xLastWakeTime = 0;
	
	if(shootData.pokeStall == MOTOR_FOWARD){
        //�����תת�ص�������9000����500���룬˵����ת
		//�����̺�Ӣ�۴Ӳ����̣�42mm������
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
        //��ת100����
		if(xTaskGetTickCount() - xLastWakeTime > 100){
            //Ȼ����ת
			shootData.pokeStall = MOTOR_FOWARD;                            
		}
	}
}
//������
static void laser_working(uint8_t __switch){
	if(__switch)
		LASER_OFF;
	else
		LASER_ON;
}

static void fricWheelSwitch(AutoTaskStruct_t *robotTask,robotModeStruct_t robotmode, uint8_t switch_now){
	static uint8_t lastmode = 0;
	static uint8_t lastswitch = 0;
	
    //�����������ģʽ�л�
    if(lastmode != robotmode){                        		
        digitalHi(&shootData.laser_close);
        //Ħ���ֹرգ���������׼���ر�
        digitalLo(&shootData.fricMotorFlag);							
        //�رմ󵯲�
        digitalLo(&P_HERO_42_LID);									
        //�ر�С����
        digitalLo(&P_HERO_17_LID);									
	}
	else{                             
        //��ҡ�˻����ģʽ��
        if(robotmode == MODE_RC){												
            //SW1����͵�ʱĦ���ֹر�
			if(switch_now == RCSW_BOTTOM){               											
                //�����
				digitalHi(&shootData.laser_close);																			
                //Ħ���ֹر�
				digitalLo(&shootData.fricMotorFlag);												
			}
			else if(robotmode == MODE_RC && lastswitch == RCSW_BOTTOM && switch_now == RCSW_MID && ROBOT != SENTRY_ID){												
                //ֻ��RCģʽSW1����͵����м䵵ʱĦ���ֿ���ͬ�r�_������							
				digitalHi(&shootData.fricMotorFlag); 
        digitalLo(&shootData.laser_close);				
			}
			else if(lastswitch == RCSW_BOTTOM && switch_now == RCSW_MID)//�ڱ����_������
				digitalHi(&shootData.fricMotorFlag); 
		}
			else if(robotmode == MODE_KM){
								if(ROBOT==INFANTRY_ID){             //PY: KMģʽ�µ�Ħ���ֿ����߼�
			if((switch_now == RCSW_MID && ROBOT != SENTRY_ID && getinfantryFricState()->infantryFricStateFlag == 1)||(lastswitch == RCSW_BOTTOM && switch_now == RCSW_MID)){												
                //KMģʽSW1���м䵵ʱ����Ħ���ֱ�־λΪ1ʱĦ���ֿ���ͬ�r�_������			
				getinfantryFricState()->infantryFricStateFlag = 1;
				digitalHi(&shootData.fricMotorFlag); 
        digitalLo(&shootData.laser_close);				
			}
			else if(switch_now == RCSW_BOTTOM || getinfantryFricState()->infantryFricStateFlag == 0){               											
                //�����
				digitalHi(&shootData.laser_close);																			
                //Ħ���ֹر�
				digitalLo(&shootData.fricMotorFlag);												
			} 
                }
            if(ROBOT==F_TANK_ID){
			if(switch_now == RCSW_BOTTOM || getftankFricState()->ftankFricStateFlag == 0){               											
                //�����
				digitalHi(&shootData.laser_close);																			
                //Ħ���ֹر�
				digitalLo(&shootData.fricMotorFlag);												
			}
			else if(switch_now == RCSW_MID && ROBOT != SENTRY_ID && getftankFricState()->ftankFricStateFlag == 1){												
                //KMģʽSW1���м䵵ʱ����Ħ���ֱ�־λΪ1ʱĦ���ֿ���ͬ�r�_������									
				digitalHi(&shootData.fricMotorFlag); 
        digitalLo(&shootData.laser_close);				
			}
                }
		}			
	
		
		else{
			digitalHi(&shootData.laser_close);
            //��ҡ�˻��߼���ģʽ��Ħ���ֶ��ر�
            digitalLo(&shootData.fricMotorFlag);              
		}
	}

	//������
	laser_working(shootData.laser_close);
    //��ִ�еĻ�����ģʽ�����ϴε�ģʽ
	lastmode = robotmode;																	
    //��ִ�е�SW1�����ϴε�SW1		RC_GEAR	
	lastswitch = switch_now;															
}

static void shoot_in_modeRC(uint8_t lastSwitch){
	//RCģʽ�Ҳ��ֲ������������嵯ģʽ
	//������������嵯
	if(RC_ROTATE < -100 && ROBOT == INFANTRY_ID){					
		shootData.shootMode_17mm = AUTO_CONTINUOUS;
		shootData.clearBulletFlag = true;
	}
	//RCģʽ��ֻ�е���
	else{
		shootData.shootMode_17mm = MANUAL_SINGLE;
		shootData.clearBulletFlag = false;
	}
	//�󲦸˴��м䲦������
	if(lastSwitch == RCSW_MID && RC_GEAR == RCSW_TOP){
		if (ROBOT == F_TANK_ID)
			digitalHi(&shootData.fireFlag_42mm);
		else
			if(ROBOT == P_TANK_ID){
		        digitalHi(&shootData.fireFlag_42mm);	//LCM: RCģʽ���˻���
		        digitalHi(&shootData.fireFlag_17mm);	//LCM: RCģʽ���˻���
			}
		else
			digitalHi(&shootData.fireFlag_17mm);		//LCM: RCģʽ���˻���
	}
	// ���˻�RCģʽ������ 
	else if(RC_GEAR == RCSW_TOP&&lastSwitch == RCSW_TOP){
		if(ROBOT == UAV_ID){
			digitalHi(&shootData.shootTrigger_17mm);	//LCM: RCģʽ���˻���
			digitalHi(&shootData.fireFlag_17mm);		//LCM: RCģʽ���˻���
		}
	}
	//�����������
	else if(RC_GEAR != RCSW_TOP) shootDataReset();
	else if(RC_GEAR == RCSW_BOTTOM) shootData.gsTime = 0;
}

double time1;
static void  shoot_in_modeKM(uint8_t lastPress,uint8_t lastSwitch){
	//����û���嵯����
	shootData.clearBulletFlag = false;
		//�󲦸˴��м䲦������
	if(lastSwitch == RCSW_MID && RC_GEAR == RCSW_TOP){
		if (ROBOT == F_TANK_ID)
		digitalHi(&shootData.fireFlag_42mm);
		else
			if(ROBOT == P_TANK_ID){
		        digitalHi(&shootData.fireFlag_42mm);	//LCM: KMģʽ���˻���
		        digitalHi(&shootData.fireFlag_17mm);	//LCM: KMģʽ���˻���
			}
		else
			digitalHi(&shootData.fireFlag_17mm);		//LCM: KMģʽ���˻���
	}
	if(KB_TYPY_SHOOT && !lastPress){         //����ģʽ����B���ı����״̬
		digitalIncreasing(&shootData.shootMode_17mm);
		if(shootData.shootMode_17mm > AUTO_CONTINUOUS)
			shootData.shootMode_17mm = MANUAL_SINGLE;
	}
	if(ROBOT == UAV_ID || ROBOT == SENTRY_ID || ROBOT == SMALLGIMBAL_ID)
		shootData.shootMode_17mm = AUTO_CONTINUOUS;
	//��ȫģʽ��Σ��ģʽ֮���л�
	else if(KB_TYPY_HEAT_PROTECT)
		//�޾���ģʽ						 
		shootData.shootStatusMode_17mm = DANGEROUS;
	else
		shootData.shootStatusMode_17mm = SAFE;//����������ģʽ
	
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
        //�ͷſ���Ȩ��ص�����ģʽ
		shootData.shootMode_17mm = MANUAL_SINGLE;								
        //��ʼ�������
		digitalHi(&shootData.ballisticFill);									
	}	
    //Ħ���ֿ���״̬�ſ��Է����ӵ�
	if(shootData.fricMotorFlag){											
        //ң��������ģʽ
		if(getrobotMode() == MODE_RC)
			shoot_in_modeRC(lastswitch);
        //���̿���ģʽ
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
/* ����������� */
void shootUpdate(void){
    //����ʣ��ǹ�������͵�ǰĦ����ת��
	 fric_wheel_calc();			
    //������յ������źţ����ݵ�ǰ�����ģʽ�ж�Ӧ������ӵ����������㰲ȫ��
	if(FIRE_FLAG_17MM || FIRE_FLAG_42MM){						
		shootManualUpdate();
		shootTimesCount();	
	}
	if(ROBOT == P_TANK_ID){
		p_heroAutoLoad();
	}
	shootProtect();
    //�����ʱ���,�����յ������źŵ�ʱ��ȷ����������Ƿ�ʱ�������ʱ�������������
	shootOverTimeCheck_17mm();
	shootOverTimeCheck_42mm();
    //����ʣ���ӵ�,���ж��Ƿ�ֹͣ������
	bulletRemainCount();	
  if(!SHOOTER_SWITCH)	{
		time2 = getClockCount();
		time3 = time2 - time1;
		time1 ++;
	}
	
#if SNAIL_MOTOR 
	if(!(getcontrolData()->loops % 5)){
        //Ħ���ֵ�����10msһ��
		shootFrictiongearUpdate(shootData.fricMotorFlag);	
	}
#else
	fricWheel_update(shootData.fricMotorFlag);
#endif
    //��������ת���
	pokeStallCheck();         
    //�����̵�����,���տ���ָ��
	shootpokeMoterUpdate(shootData.fricMotorFlag);
    //������ȴ
	shooterHeatCooling();			
	digitalIncreasing(&shootData.loops);
	shooterHeatAbjust();
	shooterHeatIncreasing(); 	
//    //�򵯺���
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
//Ħ����PID����	
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
	//������PID����
    //������ͷſ���Ȩģʽ��ֹͣģʽ
	if(getrobotMode() == MODE_RELAX||getrobotMode() == MODE_STOP){																
        //Ħ���ֲ������������Ҳ����				
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
    //С���ָ�PID����
    shootData.supplySpeedOut = pidUpdate(shootData.shootsupplyPID,\
										  shootData.supplyRef,\
										 (f32_t)motorHandleClass.Speed(&commandoMotorConfig[SUPPLYMOTOR]),\
										 shootData.supplyIntervalTime);
}

static void safe_heat_set(void){
	//���ڲ���ϵͳʱ�а�ȫ��������
	if(get_judgeData()->extGameRobotState.max_HP){
		getshootData()->safe_heat_17mm = __17MM_HEAT;
		getshootData()->safe_heat_42mm = __42MM_HEAT;
	}
	else{
		getshootData()->safe_heat_17mm = __NO_HEAT;
		getshootData()->safe_heat_42mm = __NO_HEAT;
	}
	
}

/* ���������ʼ�� */
void shootInit(void){		
#if SNAIL_MOTOR 	
	//������ˢ���,Ƶ��400Hz																								
	BSP_TIM_PWM_Init( TIM9, 2500-1, 168-1, BSP_GPIOE5, BSP_GPIOE6, NULL,NULL );	
	FricMotor_L = FricMotor_R = 1000;
#endif	
	//�����ʼ��	
	BSP_GPIO_Init(LASER_GPIO,GPIO_Mode_Out_PP);
	//�ᴥ���س�ʼ��
	BSP_GPIO_Init(SHOOTER_SWITCH_GPIO,GPIO_Mode_IPU);
	//���䰲ȫ����
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
    //Ĭ�ϵ���
	shootData.shootMode_17mm = MANUAL_SINGLE;
    //Ĭ�ϲ����� ����Ħ���� û���յ�������־ ������ȫģʽ
	shootData.shootStatusMode_17mm = SAFE;
	if(ROBOT == SENTRY_ID){
		shootData.shootMode_17mm = AUTO_CONTINUOUS;	//����
        //17mmĦ����ת��Ĭ��ֵ
		shootData.fricSpeedSet_17mm = 6600;
	}
	/**/
	else{
        //17mmĦ����ת��Ĭ��ֵ
		shootData.fricSpeedSet_17mm = 6560;
	}
	/**/
    //42mmĦ����ת��Ĭ��ֵ
	shootData.fricSpeedSet_42mm = 5650;
	shootData.pokeSpeedSet = POKE_STATIC_SPEED;
	shootData.slave_pokeStall = MOTOR_FOWARD;											//Ȼ����ת
	shootData.timeing = 0;
	shootData.gsTime = 0;
	shootData.lasttimez = 0;
	shootData.speeding = 0;
}
