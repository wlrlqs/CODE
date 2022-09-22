#include "warning.h"
#include "config.h"
#include "supervisor.h"
#include "shoot.h"
#include "auto_task.h"
#include "type_robot.h"
#include "local_id.h"
#include "imu.h"
#include "rc.h"
#include "judge.h"
#include "vision.h"
#include "power.h"

warningStruct_t warningData;

//灯条闪烁
void light_blink_switch(uint32_t loops,uint8_t freq,uint8_t __switch){
	uint8_t invertFreq = 0;
	invertFreq = (1000 / WARNING_STACK_PERIOD) / freq;      //10/频率
	if(!(loops % invertFreq)){
		__switch = !__switch;
		if(invertFreq == 10)            //频率（freq）为1则常量
			__switch = ENABLE;
	}
	if(!__switch)
		warningData.displayColor = SK6812_DARK;
}

//单独灯珠显示
void light_alone_control(uint8_t light_pos,hsvColor_t *op_display_color){
	setOneLedHsv(light_pos,op_display_color);
}

//灯条序列连续显示
void light_sequence_control(uint8_t light_index,uint8_t display_num,hsvColor_t *op_display_color,hsvColor_t *cl_display_color){
	while(light_index < SK6812_LED_STRIP_LENGTH){
		if(light_index < display_num)
			setOneLedHsv(light_index,op_display_color);
		else
			setOneLedHsv(light_index,cl_display_color);
		light_index ++;
	}
}

//用于切换灯条状态机状态的		
void lightBarsStateSwitch(uint16_t state,uint8_t valve){																																			
	if (valve)
        //把对应位置1
		warningData.lightBarsState.u16_temp |= state;											
	else
        //把对应位清0
		warningData.lightBarsState.u16_temp &= ~state;											
}

//用于检查当前错误状态
void lightBarsErrorCheck(void){																		
    //(1)检测遥控器是否丢失  0x0001		
	if(supervisorData.state & STATE_RADIO_LOSS){												
		lightBarsStateSwitch(RC_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(RC_FAULT,DISABLE);
	}
    //(2)检测裁判系统是否故障  0x0002
	if(supervisorData.state & STATE_JUDGE_ERROR){												
		lightBarsStateSwitch(JUDGE_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(JUDGE_FAULT,DISABLE);
	}
	//(3)检测陀螺仪是否故障  0x0004
	if(supervisorData.state & STATE_SENSOR_IMU_ERROR){											
		lightBarsStateSwitch(SENSOR_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(SENSOR_FAULT,DISABLE);
	}
    //(4)检测动力系统和云台电机是否故障  0x0008
	if(supervisorData.state & STATE_MOTOR_ERROR){												
		lightBarsStateSwitch(MOTOR_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(MOTOR_FAULT,DISABLE);
	}
    //(5)检测TX2是否通信  0x0010
	if(supervisorData.state & STATE_VISION_ERROR){											
		lightBarsStateSwitch(VISION_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(VISION_FAULT,DISABLE);
	}
	//(6)双控通信检测	0x0020
	if(supervisorData.state & STATE_TRANS_ERROR){											
		lightBarsStateSwitch(DOUBLE_TRANS_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(DOUBLE_TRANS_FAULT,DISABLE);
	}
    //(7)检测电容板是否故障  0x0040
	if(supervisorData.state & STATE_CAPACITANCE_ERROR){																						
		lightBarsStateSwitch(POWER_LIMIT_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(POWER_LIMIT_FAULT,DISABLE);
	}
}
//发射机构工作状态
uint8_t check_shooter_working(void){
	static uint8_t result = 0x00;
	uint32_t lastErr_fricL   = 0;
	uint32_t lastErr_fricR   = 0;
	uint32_t lastErr_trigger = 0;
	uint32_t lastErr_supply  = 0;
	//轮流排除故障
	//左摩擦轮故障
	if(commandoMotorConfig[FRICLMOTOR].errorCount - lastErr_fricL == 0)
		result = 1 << 0;
	//右摩擦轮故障
	else if(commandoMotorConfig[FRICRMOTOR].errorCount - lastErr_fricR == 0)
		result = 1 << 1;
	//拨弹盘故障
	else if(commandoMotorConfig[POKEMOTOR].errorCount - lastErr_trigger == 0)
		result = 1 << 2;
	//弹舱盖
	else if(commandoMotorConfig[SUPPLYMOTOR].errorCount - lastErr_supply == 0)
		result = 1 << 3;
	else
		result = 0x00;
	
	//更新计数
	lastErr_fricL   = commandoMotorConfig[FRICLMOTOR].errorCount;
	lastErr_fricR   = commandoMotorConfig[FRICRMOTOR].errorCount;
	lastErr_trigger = commandoMotorConfig[POKEMOTOR].errorCount;
	lastErr_supply  = commandoMotorConfig[SUPPLYMOTOR].errorCount;
	
	return result;
}
//云台机构工作状态
uint8_t check_gimbal_working(void){
	static uint8_t result = 0x00;
	uint32_t lastErr_pitch = 0;
	uint32_t lastErr_yaw = 0;
	//轮流排除故障
	//yaw轴故障
	if(commandoMotorConfig[YAWMOTOR].errorCount - lastErr_yaw == 0)
		result = 1 << 0;
	//pitch轴故障
	else if(commandoMotorConfig[PITCHMOTOR].errorCount - lastErr_pitch == 0)
		result = 1 << 1;
	else
		result = 0x00;
	
	//更新计数
	lastErr_yaw   = commandoMotorConfig[YAWMOTOR].errorCount;
	lastErr_pitch = commandoMotorConfig[PITCHMOTOR].errorCount;
	
	return result;
}
//灯带状态机更新
void lightBarsStateUpdata(void){
    static uint8_t lightBarsSwitch = ENABLE;
	uint8_t shooter_process = 0x00;
	uint8_t gimbal_process = 0x00;
	/*****************************************1*****************************************/
	//摩擦轮使能
	if(getshootData()->fricMotorFlag)
		warningData.displayColor = SK6812_GREEN;    //使能绿灯
	else
		warningData.displayColor = SK6812_RED;      //不使能绿灯
	light_alone_control(0,&warningData.displayColor);  //0代表第一颗灯
	/*****************************************2*****************************************/
	//射击安全等级
	if(getshootData()->shootStatusMode_17mm == SAFE){
		warningData.displayColor = SK6812_GREEN;
		warningData.blinkFrequency = 1;             //射击安全绿灯
	}
	else{
		warningData.displayColor = SK6812_RED;
		warningData.blinkFrequency = 5;             //不安全红灯闪
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);  //灯条闪烁函数
	light_alone_control(1,&warningData.displayColor); //1代表第二颗灯
	/*****************************************3*****************************************/
	//射击等级
	switch(getshootData()->speed_limit){
		case SPEED_17MM_LOW:
			warningData.displayColor = SK6812_GREEN;     //17mm 15m/s  绿灯
			break;
		case SPEED_17MM_MID:
			warningData.displayColor = SK6812_YELLOW;    //17mm 18m/s  黄灯
			break;
		case SPEED_17MM_HIGH:
			warningData.displayColor = SK6812_BLUE;      //17mm 22m/s  蓝灯
			break;
		case SPEED_17MM_EXTREME:
			warningData.displayColor = SK6812_WHITE;     //17mm 30m/s  白灯
			break;  
		case SPEED_42MM_LOW:
			warningData.displayColor = SK6812_GREEN;     //42mm 10m/s  绿灯
			break;
		case SPEED_42MM_MID:
			warningData.displayColor = SK6812_YELLOW;    //42mm 12m/s  黄灯
			break;
		case SPEED_42MM_HIGH:
			warningData.displayColor = SK6812_BLUE;      //42mm 14m/s  蓝灯
			break;
		case SPEED_42MM_EXTREME:
			warningData.displayColor = SK6812_WHITE;     //42mm 16m/s  白灯
			break;
		default:
			warningData.displayColor = SK6812_PINK;      //无裁判系统时的射速 粉灯
			break;
	}
	light_alone_control(2,&warningData.displayColor);//2代表第三颗灯
	/*****************************************4*****************************************/
	//功率等级
	switch(get_judgeData( )->extGameRobotState.chassis_power_limit){
		case 40:
			warningData.displayColor = SK6812_GREEN;    //功率40 ：绿灯闪
		  warningData.blinkFrequency = 5;  
      break;		
		case 45:
			warningData.displayColor = SK6812_GREEN;    //功率45 ：绿灯常亮
		  warningData.blinkFrequency = 1;
			break;
		case 50:
			warningData.displayColor = SK6812_YELLOW;   //功率50 ：黄灯闪
		  warningData.blinkFrequency = 5; 
			break;
		case 55:
			warningData.displayColor = SK6812_YELLOW;   //功率55 ：黄灯常亮
		  warningData.blinkFrequency = 1; 
			break;
		case 60: 
			warningData.displayColor = SK6812_BLUE;     //功率60 ：蓝灯闪
	  	warningData.blinkFrequency = 5;     
			break;
		case 65: 
			warningData.displayColor = SK6812_BLUE;     //功率65 ：蓝灯常亮
	  	warningData.blinkFrequency = 1;     
			break;		
		case 70:
			warningData.displayColor = SK6812_PURPLE;   //功率70 ：紫灯闪
		  warningData.blinkFrequency = 5;      
			break;
		case 80:
			warningData.displayColor = SK6812_PURPLE;   //功率80 ：紫灯常亮
			warningData.blinkFrequency = 1; 
		break;
		case 90:
			warningData.displayColor = SK6812_WHITE;    //功率90 ：白灯闪
		  warningData.blinkFrequency = 5;  
			break;
		case 100:
			warningData.displayColor = SK6812_WHITE;    //功率100 ：白灯常亮
		  warningData.blinkFrequency = 5;  
			break;		
		case 120:
			warningData.displayColor = SK6812_PINK;     //功率120 ：粉灯闪
		  warningData.blinkFrequency = 5;  
			break;		
		default:
			warningData.displayColor = SK6812_PINK;     //裁判系统不返回功率 ：粉灯常亮
		  warningData.blinkFrequency = 1;
			break;
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);  //灯条闪烁函数
	light_alone_control(3,&warningData.displayColor);//3代表第四颗灯
	/*****************************************5*****************************************/
	//发射机构工作状态
	shooter_process = check_shooter_working();
	switch(shooter_process){
		//正常：绿灯
		case 0x00:
			warningData.displayColor = SK6812_GREEN;
			warningData.blinkFrequency = 1;
			break;
		//左摩擦轮故障：黄灯闪
		case 0x01:
			warningData.displayColor = SK6812_YELLOW;
			warningData.blinkFrequency = 2;
			break;
		//右摩擦轮故障：蓝灯闪
		case 0x02:
			warningData.displayColor = SK6812_BLUE;
			warningData.blinkFrequency = 2;
			break;
		//拨弹盘故障：  红灯闪
		case 0x04:
			warningData.displayColor = SK6812_RED;
			warningData.blinkFrequency = 2;
			break;
		//弹舱盖故障：  粉灯闪
		case 0x08:
			warningData.displayColor = SK6812_PINK;
			warningData.blinkFrequency = 2;
			break;
		//两个以上机构出现故障：红灯常量
		default:
			warningData.displayColor = SK6812_RED;
		  warningData.blinkFrequency = 1;
		  break;
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);//灯条闪烁函数
	light_alone_control(4,&warningData.displayColor);//4代表第五颗灯
	/*****************************************6*****************************************/
	//云台机构工作状态
	gimbal_process = check_gimbal_working();
	switch(gimbal_process){
		//正常
		case 0x00:
			warningData.displayColor = SK6812_GREEN;
			warningData.blinkFrequency = 1;
			break;
		//yaw轴故障
		case 0x01:
			warningData.displayColor = SK6812_BLUE;
			warningData.blinkFrequency = 3;
			break;
		//pitch轴故障
		case 0x02:
			warningData.displayColor = SK6812_WHITE;
			warningData.blinkFrequency = 3;
			break;
	  //两个机构出现故障：红灯常量
		default:
			warningData.displayColor = SK6812_RED;
		  warningData.blinkFrequency = 1;
		  break;
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);//灯条闪烁函数
	light_alone_control(5,&warningData.displayColor);//5代表第六颗灯
	/*****************************************7*****************************************/
	//底盘机构工作状态
	switch(getchassisData()->chassis_fault){
		//正常：绿灯
		case 0x00:
			warningData.displayColor = SK6812_GREEN;
			warningData.blinkFrequency = 1;
			break;
		//右前轮故障：黄灯闪
		case 0x01:
			warningData.displayColor = SK6812_YELLOW;
			warningData.blinkFrequency = 5;
			break;
		//左前轮故障：蓝灯闪
		case 0x02:
			warningData.displayColor = SK6812_BLUE;
			warningData.blinkFrequency = 5;
			break;
		//左后轮故障：粉灯闪
		case 0x04:
			warningData.displayColor = SK6812_PINK;
			warningData.blinkFrequency = 5;
			break;
		//右后轮故障：红灯闪
		case 0x08:
			warningData.displayColor = SK6812_RED;
			warningData.blinkFrequency = 5;
			break;
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);//灯条闪烁函数
	light_alone_control(6,&warningData.displayColor);//6代表第七颗灯
}


//用于报错
/*
一个灯珠位表示一种机器人的错误状态
红灯表示存在错误、绿灯表示正常
最高错误位用于进度条效果,进度条会进展到错误优先级最高的灯珠
进度条效果:暗色->绿色(表示无错误)，依次亮起绿灯,知道有错误的灯时全部变为暗色,再次进行循环,有错误的灯为红色
错误优先级（最高错误状态）:1灯错误>2灯错误>3灯错误>4灯错误>5灯错误>6灯错误>7灯错误......>16灯错误(最大支持16个灯珠)
*/
void lightBarsReportErrors(void){
	uint8_t index = 0;//索引
	uint8_t nowFaultBit = 0; 
	uint16_t remainFaultSate;
	//存在有需要点亮的灯珠位(有错误状态)
	if(warningData.lightBarsState.u16_temp != 0){
		//当前错误状态
		remainFaultSate = warningData.lightBarsState.u16_temp;
        /**从最低位灯珠开始遍历到最后一颗灯珠去找到最高错误状态**/
	 	while(index < SK6812_LED_STRIP_LENGTH && !warningData.highestFaultFlag){    
			//当前第index颗灯珠状态
			nowFaultBit = remainFaultSate % 2;
			//如果状态触发
			if(nowFaultBit){
				//最高错误状态就是这颗灯珠,获取索引号
				warningData.highestFault = index;
				//最高错误状态已获取到，触发标志跳出循环
				digitalHi(&warningData.highestFaultFlag);
			}
			//当前错误状态右移，读取下一颗灯珠状态
			remainFaultSate = remainFaultSate >> 1;
			//索引+1
			index++;
		}
		digitalLo(&warningData.highestFaultFlag);
	}
	//清除索引
	index = 0;
    /**在第一位和最高错误位间来一个渐变效果,类似进度条**/
	//存在最高错误状态
	if(warningData.highestFault != 0){
		//索引<=最高错误位的灯珠
		while(index < warningData.highestFault){
			//索引>=最高状态循环位(最高状态循环位<=最高状态位,每次循环逐次+1)
			if(index >= warningData.highestFaultLoop)
				//设置该灯珠为暗灯
				setOneLedHsv(index,&SK6812_DARK);
			else
				//设置该灯珠为绿灯
				setOneLedHsv(index,&SK6812_GREEN);
			//索引+1
			index ++;
		}
	}
	//将最高错误位给索引
	index = warningData.highestFault;
	//当前剩余的错误状态
	remainFaultSate = warningData.lightBarsState.s16_temp >> warningData.highestFault;
	/**索引小于最大灯珠个数，从当前最高错误位开始遍历到最后一个灯珠**/
	//显示错误状态的灯珠（红色）和无错误状态的灯珠（绿色）
	while(index < SK6812_LED_STRIP_LENGTH){
		//当前灯珠位的状态
		nowFaultBit = remainFaultSate % 2;
		//如果当前位存在错误状态
		if(nowFaultBit)
			//设置该灯珠为红灯
			setOneLedHsv(index,&SK6812_RED);
		//不存在错误状态
		else
			//设置该灯珠为绿灯
			setOneLedHsv(index,&SK6812_GREEN);
		//当前错误状态右移1位，读取下一颗灯珠位的状态
		remainFaultSate = remainFaultSate >> 1;
		//索引+1
		index ++;
	}
    //500ms刷新
	if(!(warningData.loops % 5)){
		//最高状态循环位自增
		digitalIncreasing(&warningData.highestFaultLoop);
		//最高状态循环位大于最高状态位清除循环位
		if(warningData.highestFaultLoop > warningData.highestFault)
			digitalClan(&warningData.highestFaultLoop);
	}
	else
		//清除最高错误状态
		digitalClan(&warningData.highestFault);
}

//裁判系统发送数据更新
void lightBarsJudgeUpdate(void){
//	if(getvisionData()->captureFlag&&getvisionData()->cailSuccess)
//		judgeData.extShowData.data1 = UKF_POS_Z;
//	else
//		judgeData.extShowData.data1 = 0.0;
//	
//	//电容电量百分比，17v为保护电压
//	judgeData.extShowData.data2 = (float)(100 * (getpowerData() ->capVol - 17) / (getpowerData() ->capVolMax - 17));
//	//没开启摩擦轮灯不亮
//	if(!getshootData()->fricMotorFlag)	
//		//红灯全亮		
//		judgeData.extShowData.mask = 0x00;									
//	else{
//		if(getshootData()->shootMode_17mm == MANUAL_SINGLE){	
//			//单发1颗灯
//			judgeData.extShowData.mask |= 0x01;
//			judgeData.extShowData.mask &= 0xF9;
//		}
//		else if(getshootData()->shootMode_17mm == MANUAL_CONTINUOUS){
//			//三连发2颗灯
//			judgeData.extShowData.mask |= 0x03;
//			judgeData.extShowData.mask &= 0xFB;
//		}
//		else if(getshootData()->shootMode_17mm == AUTO_CONTINUOUS){
//			//连发三颗灯
//			judgeData.extShowData.mask |= 0x07;
//		}
//		if(getinfantryAutoData()->rotateFlag)
//            //小陀螺模式			
//			judgeData.extShowData.mask |= 0x08;
//		else
//			judgeData.extShowData.mask &= 0xF7;
//		
//		if(getinfantryAutoData()->aviodFlag)
//			//扭腰模式			
//			judgeData.extShowData.mask |= 0x10;
//		else
//			judgeData.extShowData.mask &= 0xEF;
//			//开电容放电
//		if(openCap)							   													
//			judgeData.extShowData.mask |= 0x20;
//		else
//			judgeData.extShowData.mask &= 0xDF;
//	}
}

/****************参数说明******************
    "SK6812_LED_STRIP_LENGTH"       ----> 灯带的长度  7
	"warningData.displayNumber"     ----> 点亮灯的数目（从左数起）
	"warningData.displayColor"      ----> 灯带的颜色 
    "setOneLedHsv(index,&warningData.displayColor);" ----> 此函数为点灯函数 index（点亮灯的数目），warningData.displayColor 灯带的颜色
    "safeMode" ----> 安保等级   SAFE（安全）：灯带全亮绿灯  DANGEROUS（危险）：灯带全亮红灯
    "contrlMode" ----> 控制模式  MANUAL_SINGLE（单发）左起亮一颗灯  MANUAL_CONTINUOUS（三连发）左起亮两颗灯  AUTO_CONTINUOUS（连发）左起亮三颗灯  
    灯带各个灯的作用（从左到右）：
    单发1颗灯    灯带颜色由射击安全状态决定
    三连发2颗灯  灯带颜色由射击安全状态决定
    连发3颗灯    灯带颜色由射击安全状态决定
    从左数起第4颗灯显示扭腰,小陀螺    扭腰（红灯）  小陀螺（绿灯）
    从左数起第5颗灯显示自动任务状态	 X_TASK(粉灯)  R_TASK（红灯）  V_TASK（绿灯） Z_TASK（黄灯） G_TASK(蓝灯)  C_TASK(白灯)无任务时灯灭
	从左数起第6颗灯显示步兵上台阶机构的可用状态	 禁止(蓝灯闪烁)	 可用(白灯常亮)
    从左数起第7颗灯显示电容状态  满电或者为安全电量长亮（绿灯）   电容放电（绿灯，闪烁）  电容即将没电，警告（红灯） 连底盘时灯不亮
    在键鼠模式下没开启摩擦轮灯全为红色    
*******************************************/


//用于控制状态
void lightBarsOfContrl(uint8_t shootContrlMode,uint8_t shootSafeMode){					
	uint8_t index = 0;//索引
	static uint8_t lightBarsSwitch = ENABLE;
	/*****************************************1~3*****************************************/
    //射击模式
	switch(shootContrlMode){
		//单发射击
		case MANUAL_SINGLE:
			warningData.displayNumber = 1;//亮1颗
			break;
		//三连发射击
		case MANUAL_CONTINUOUS:
			warningData.displayNumber = 2;//亮2颗
			break;
		//连发射击
		case AUTO_CONTINUOUS:
			warningData.displayNumber = 3;//亮3颗
			break;
	}
    //识别射击安全状态
	switch(shootSafeMode){
		//安全
		case SAFE:  
			warningData.blinkFrequency = 1;              //绿色常亮
			warningData.displayColor = SK6812_GREEN;
			break;
		//危险
		case DANGEROUS:
			warningData.blinkFrequency = 5;              //红色闪烁
			warningData.displayColor = SK6812_RED;
			break;
	}
	//灯条闪烁表现效果
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);
	//从左到右显示显示射击模式
	light_sequence_control(index,warningData.displayNumber,&warningData.displayColor,&SK6812_DARK);
	/*****************************************4*****************************************/
	//右侧第4个灯显示扭腰,小陀螺
	//扭腰：红灯
	if(autoTaskData->aviodFlag)
		warningData.displayColor = SK6812_RED;
	//小陀螺：绿灯
	else if(autoTaskData->rotateFlag)
		warningData.displayColor = SK6812_GREEN;
	else
		warningData.displayColor = SK6812_DARK;

	light_alone_control(SK6812_LED_STRIP_LENGTH - 4,&warningData.displayColor);
	/*****************************************5*****************************************/
	//右侧第3个灯珠用于显示任务状态
    //最左侧和最右侧的灯珠用于显示任务状态
	switch(autoTaskData->currentTask){
		case X_TASK:     //粉灯  步兵：调射速
			warningData.displayColor = SK6812_PINK;													
			break;
		case R_TASK:     //红灯  步兵：子弹交接
			warningData.displayColor = SK6812_RED;													
			break;	
		case V_TASK:     //绿灯  步兵：辅助瞄准
			warningData.displayColor = SK6812_GREEN;
			break;
		case Z_TASK:     //蓝灯  步兵：击打神符
			warningData.displayColor = SK6812_BLUE;
			break;
		case G_TASK:     //黄灯  步兵：击打大符
			warningData.displayColor = SK6812_YELLOW;
			break;
		case C_TASK:     //白灯  步兵：上台阶
			warningData.displayColor = SK6812_WHITE;
			break;
		default:         //无任务 不亮
			warningData.displayColor = SK6812_DARK;
			break;
	}
	
	light_alone_control(SK6812_LED_STRIP_LENGTH - 3,&warningData.displayColor);
	/*****************************************6*****************************************/
	//右侧第2颗用于显示步兵的上台阶机构
	if(ROBOT == INFANTRY_ID){
		if(getrobotMode() == MODE_KM){                   
			if(get_infDeforming()->up_step_switch){
				warningData.displayColor = SK6812_WHITE;     //白灯 步兵：开启同步带并且可以控制机械臂
				warningData.blinkFrequency = 1;
			}
			else{
				warningData.displayColor = SK6812_BLUE;     //蓝灯  步兵：同步带停止运动此时不可控制机械臂
				warningData.blinkFrequency = 3;
			}
		}
		else
			warningData.displayColor = SK6812_DARK;
		light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);
		light_alone_control(SK6812_LED_STRIP_LENGTH - 2,&warningData.displayColor);
	}
	/*****************************************7*****************************************/
	//最右边的灯显示电容状态
  if(ROBOT == INFANTRY_ID || ROBOT == F_TANK_ID){
		 if(getcapacitorData()->percentage < 40)
			  //百分之四十：红色
				warningData.displayColor = SK6812_RED;
         else if(getcapacitorData()->percentage < 60)
			 //百分之六十：黄色
				warningData.displayColor = SK6812_YELLOW;
         else if(getcapacitorData()->percentage < 80)
			 //百分之八十：粉色 
				warningData.displayColor = SK6812_PINK;
		 else if(getcapacitorData()->percentage < 85)
			 //百分之八十五：白色 
				warningData.displayColor = SK6812_WHITE;
		 else if(getcapacitorData()->percentage < 90)
			 //百分之九十：蓝色
				warningData.displayColor = SK6812_BLUE;
         else if(getcapacitorData()->percentage <= 100)
	         //百分百：    绿色
				warningData.displayColor = SK6812_GREEN;
		 
	switch(getcapacitorData()->capacitor_status){
			case Charging:      //充电：闪烁
				warningData.blinkFrequency = 2;
			break;
			case Discharging:   //放电：常量
				warningData.blinkFrequency = 1;
			break;
			case Protection:   //保护：不亮
				warningData.displayColor = SK6812_DARK;
			break;
		}
		light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);
		light_alone_control(SK6812_LED_STRIP_LENGTH - 1,&warningData.displayColor);
	}
	/*****************************************ALL*****************************************/
	if(getrobotMode() == MODE_KM && !getshootData()->fricMotorFlag){   //KM键鼠模式下没有开摩擦轮：灯带全红
		setAllLedColors(&SK6812_RED);
	}
}


//灯带状态更新
void lightBarsUpdate(void){																						
    //获取错误
	lightBarsErrorCheck();																		
    //如果处于有遥控的状态	没有丢控						
	if(!(supervisorData.state & STATE_RADIO_LOSS)){											
		if(supervisorData.state & STATE_DISARMED || remoteControlData.dt7Value.keyBoard.bit.CTRL)  
            //没有解锁(下控)或在键鼠模式下按下CTRL显示当前机器人工作状态
			lightBarsReportErrors();
        //如果在拥有控制权的情况下并且没有按下CTRL
		else{
			//遥控器模式
			if(getrobotMode() == MODE_RC)
				//灯带状态机更新            
				lightBarsStateUpdata();
			//键鼠模式
			else if(getrobotMode() == MODE_KM){
				switch(robotConfigData.typeOfRobot){
					case INFANTRY_ID:
						lightBarsOfContrl(getshootData()->shootMode_17mm,getshootData()->shootStatusMode_17mm); 
						break;
					case P_TANK_ID:
						lightBarsOfContrl(getshootData()->shootMode_17mm,getshootData()->shootStatusMode_17mm);
						break;
					//工程车的人机界面还包括登岛界面	
					case AUXILIARY_ID:																							
						break;
					//哨兵不用灯带
					case SENTRY_ID:																								
						break;
					case UAV_ID:
						lightBarsOfContrl(getshootData()->shootMode_17mm,getshootData()->shootStatusMode_17mm);
						break;
					case F_TANK_ID:
						lightBarsOfContrl(getshootData()->shootMode_17mm,getshootData()->shootStatusMode_17mm);
				}
			}
		}
	}
	else
	{
		lightBarsReportErrors();    //丢控
	}
	SK6812UpdateStrip();
    
} 


void warningUpdate(void){
    //灯带标准色初始化  填写色域
	colorStdInit();
    //sk6812 更新
    lightBarsUpdate();																									
    //裁判系统发送数据更新
//	lightBarsJudgeUpdate();														
    //LED闪动		主控板上3色LED闪动指示状态										
	appSightClass.led(supervisorData.ledState);                           
    //蜂鸣器正常响起
	appSightClass.beep(supervisorData.beepState);												 		
    //状态清零
	digitalClan(&supervisorData.beepState);													  	
	digitalIncreasing(&warningData.loops);
}
