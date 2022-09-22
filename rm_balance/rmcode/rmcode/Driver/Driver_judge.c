#include "Driver_Judge.h"
#include "judge.h"
#include "crc.h"
#include "math.h"
#include "chassis.h"
#define POSITION_DEFINE 	1	//是否读取position数据
#define READBULLET_DEFINE 1	//是否读取子弹数据
#define READGOLF_DEFINE 	1	//是否读取高尔夫数据



buffefLoop_t bufferLoop = {
	.header = 0,
	.tail		= 0,
	.buffer	= {0},
};

/*
***************************************************
函数名：Driver_Judge_Init
功能：裁判系统串口初始化
入口参数：	Judge_USARTx：裁判系统串口号
					Judge_USART_TX：串口发送引脚号
					Judge_USART_RX：串口接收引脚号
					PreemptionPriority：抢占优先级
					SubPriority：次优先级
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void Driver_Judge_Init(USART_TypeDef *Judge_USARTx,
												BSP_GPIOSource_TypeDef *Judge_USART_TX,
												BSP_GPIOSource_TypeDef *Judge_USART_RX,
												uint8_t PreemptionPriority,uint8_t SubPriority){
	BSP_USART_TypeDef Judge_USART = {
		.USARTx = Judge_USARTx,				//串口号
		.USART_RX = Judge_USART_RX,		//引脚
		.USART_TX = Judge_USART_TX,
		.USART_InitStructure = {
			.USART_BaudRate = 115200,										/*波特率设置*/					
			.USART_WordLength = USART_WordLength_8b,		/*字长为8位数据格式*/	
			.USART_StopBits = USART_StopBits_1,					/*一个停止位*/					
			.USART_Parity = USART_Parity_No,						/*无校验位*/						
			.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,/*接收和发送模式*/						
			.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*无硬件数据流控制*/	
		}
	};
	BSP_USART_Init(&Judge_USART,PreemptionPriority,SubPriority);
	/*- 暂时屏蔽，有冲突 -*/
	BSP_USART_TX_DMA_Init(&Judge_USART);
	BSP_USART_RX_DMA_Init(&Judge_USART);
}

/*
***************************************************
函数名：judgeExtGameStateInfo
功能：获取裁判系统比赛状态数据(0x0001)
入口参数：	Array：数据段数组指针
					extGameState：比赛状态数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameStateInfo(uint8_t *Array,ext_game_state_t *extGameState){
	formatTrans64Struct_t dataTrans;
	extGameState->game_type    = (uint8_t)Array[0]&0x0f;                   //0-3 bit：比赛类型 
	extGameState->game_progress   = ((uint8_t)Array[0]&0xf0)>>4;           //4-7 bit：当前比赛阶段
	extGameState->stage_remain_time = (uint16_t)Array[1]<<8 | Array[2];	
	dataTrans.u8_temp[0] = Array[3];
	dataTrans.u8_temp[1] = Array[4];
	dataTrans.u8_temp[2] = Array[5];
	dataTrans.u8_temp[3] = Array[6];
	dataTrans.u8_temp[4] = Array[7];
	dataTrans.u8_temp[5] = Array[8];
	dataTrans.u8_temp[6] = Array[9];
	dataTrans.u8_temp[7] = Array[10];
	extGameState->SyncTimeStamp = dataTrans.u64_temp;
}

/*
***************************************************
函数名：judgeExtGameResultInfo
功能：获取裁判系统比赛结果数据 (0x0002)
入口参数：	Array：数据段数组指针
					extGameResult：比赛结果数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameResultInfo(uint8_t *Array,ext_game_result_t *extGameResult){
   extGameResult->winner = (uint8_t)Array[0];
}

/*
***************************************************
函数名：judgeExtGameRobotSurvivorsInfo
功能：获取裁判系统比赛机器人血量数据 1HZ (0x0003)
入口参数：	Array：数据段数组指针
					extGameRobotSurvivors：机器人存活数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameRobotHpInfo(uint8_t *Array,ext_game_robot_HP_t *extGameRobotHp){	
	extGameRobotHp->red_1_robot_HP 	= (uint16_t) Array[0]<<8 | Array[1];
	extGameRobotHp->red_2_robot_HP 	= (uint16_t) Array[2]<<8 | Array[3];
	extGameRobotHp->red_3_robot_HP 	= (uint16_t) Array[4]<<8 | Array[5];
	extGameRobotHp->red_4_robot_HP 	= (uint16_t) Array[6]<<8 | Array[7];
	extGameRobotHp->red_5_robot_HP 	= (uint16_t) Array[8]<<8 | Array[9];
	extGameRobotHp->red_7_robot_HP 	= (uint16_t)Array[10]<<8 | Array[11];
	extGameRobotHp->red_outpost_HP  = (uint16_t)Array[12]<<8 | Array[13];
	extGameRobotHp->red_base_HP 	= (uint16_t)Array[14]<<8 | Array[15];
	extGameRobotHp->blue_1_robot_HP = (uint16_t)Array[16]<<8 | Array[17];
	extGameRobotHp->blue_2_robot_HP = (uint16_t)Array[18]<<8 | Array[19];
	extGameRobotHp->blue_3_robot_HP = (uint16_t)Array[20]<<8 | Array[21];
	extGameRobotHp->blue_4_robot_HP = (uint16_t)Array[22]<<8 | Array[23];
	extGameRobotHp->blue_5_robot_HP = (uint16_t)Array[24]<<8 | Array[25];
	extGameRobotHp->blue_7_robot_HP = (uint16_t)Array[26]<<8 | Array[27];
	extGameRobotHp->blue_outpost_HP = (uint16_t)Array[28]<<8 | Array[29];
	extGameRobotHp->blue_base_HP = (uint16_t)Array[30]<<8 | Array[31];
}

/*
***************************************************
函数名：judgeExtGameRobotDartStatusInfo
功能：获取裁判系统飞镖发射状态 1HZ (0x0004)
入口参数：	Array：数据段数组指针
			extDartStatus: 飞镖发送状态指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameRobotDartStatusInfo(uint8_t *Array,ext_dart_status_t *extDartStatus){
	extDartStatus->dart_belong = (uint8_t)Array[0];
	extDartStatus->stage_remaining_time = (uint16_t) Array[1]<<8 | Array[2];
}

/*
***************************************************
函数名：judgeExtEventDataInfo
功能：获取裁判系统场地事件数据 (0x0101)
入口参数：	Array：数据段数组指针
					extEventData：场地事件数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtEventDataInfo(uint8_t *Array,ext_event_data_t *extEventData){
	FormatTrans dataTrans;
	dataTrans.u8_temp[0] = Array[0];
	dataTrans.u8_temp[1] = Array[1];
	dataTrans.u8_temp[2] = Array[2];
	dataTrans.u8_temp[3] = Array[3];
	extEventData->event_type = dataTrans.u32_temp;
}

/*
***************************************************
函数名：judgeExtSupplyProjectileActionInfo
功能：获取裁判系统场地补给站动作标识数据(0x0102)
入口参数：	Array：数据段数组指针
					extSupplyProjectileAction：场地补给站动作标识数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtSupplyProjectileActionInfo(uint8_t *Array,ext_supply_projectile_action_t *extSupplyProjectileAction){
	extSupplyProjectileAction->supply_projectile_id   = (uint8_t)Array[0];
	extSupplyProjectileAction->supply_robot_id 		  = (uint8_t)Array[1];
	extSupplyProjectileAction->supply_projectile_step = (uint8_t)Array[2];
	extSupplyProjectileAction->supply_projectile_num  = (uint8_t)Array[3];
}

/*
***************************************************
函数名：judgeExtSupplyProjectileBookingInfo
功能：获取裁判系统警告数据(0x0104)
入口参数：	Array：数据段数组指针
					extRefereeWarning：裁判系统警告数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtRefereeWarningInfo(uint8_t *Array,ext_referee_warning_t *extRefereeWarning){
	extRefereeWarning->level 		 = (uint8_t)Array[0];
	extRefereeWarning->foul_robot_id = (uint8_t)Array[1];
}

/*
***************************************************
函数名：judgeExtGameRobotDartRemainingInfo
功能：飞镖发射口倒计时 1HZ (0x0105)
入口参数：	Array：数据段数组指针
			extDartStatus: 飞镖发射口倒计时
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameRobotDartRemainingInfo(uint8_t *Array,ext_dart_remaining_time_t *extDartRemainingTime){
	extDartRemainingTime->dart_remaining_time = (uint8_t)Array[0];
}

/*
***************************************************
函数名：judgeExtGameRobotStateInfo
功能：获取裁判系统机器人状态数据（0x0201）
入口参数：	Array：数据段数组指针
					extGameRobotState：机器人状态数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameRobotStateInfo(uint8_t *Array,ext_game_robot_state_t *extGameRobotState){
	extGameRobotState->robot_id = (uint8_t)Array[0];
	extGameRobotState->robot_level = (uint8_t)Array[1];
	extGameRobotState->remain_HP = (uint16_t)Array[3]<<8 | Array[2];
	extGameRobotState->max_HP = (uint16_t)Array[5]<<8 | Array[4];
	extGameRobotState->shooter_id1_17mm_cooling_rate = (uint16_t)Array[7]<<8 | Array[6];
	extGameRobotState->shooter_id1_17mm_cooling_limit = (uint16_t)Array[9]<<8 | Array[8];
	extGameRobotState->shooter_id1_17mm_speed_limit = (uint16_t)Array[11]<<8 | Array[10];
	extGameRobotState->shooter_id2_17mm_cooling_rate = (uint16_t)Array[12]<<8 | Array[13];
	extGameRobotState->shooter_id2_17mm_cooling_limit = (uint16_t)Array[14]<<8 | Array[15];
	extGameRobotState->shooter_id2_17mm_speed_limit = (uint16_t)Array[16]<<8 | Array[17];
	extGameRobotState->shooter_id1_42mm_cooling_rate = (uint16_t)Array[19]<<8 | Array[18];
	extGameRobotState->shooter_id1_42mm_cooling_limit = (uint16_t)Array[21]<<8 | Array[20];
	extGameRobotState->shooter_id1_42mm_speed_limit = (uint16_t)Array[23]<<8 | Array[22];

	extGameRobotState->chassis_power_limit = (uint16_t)Array[25]<<8 | Array[24];
	extGameRobotState->mains_power_gimbal_output = ((uint8_t)Array[26] & 0x01);
	extGameRobotState->mains_power_chassis_output = ((uint8_t)Array[26] & 0x02)<<1;
	extGameRobotState->mains_power_shooter_output = ((uint8_t)Array[26] & 0x04)<<2;
}

/*
***************************************************
函数名：judgeExtPowerHeatDataInfo
功能：获取裁判系统实时功率热量数据数据（0x0202）
入口参数：	Array：数据段数组指针
					extPowerHeatData：实时功率热量数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtPowerHeatDataInfo(uint8_t *Array,ext_power_heat_data_t *extPowerHeatData){
	FormatTrans dataTrans;
	extPowerHeatData->chassis_volt = (uint16_t)Array[1]<<8 | Array[0];   
	extPowerHeatData->chassis_current = (uint16_t)Array[3]<<8 | Array[2];    
	dataTrans.u8_temp[0] = Array[4];
	dataTrans.u8_temp[1] = Array[5];
	dataTrans.u8_temp[2] = Array[6];
	dataTrans.u8_temp[3] = Array[7];
	extPowerHeatData->chassis_power = dataTrans.float_temp;        
	extPowerHeatData->chassis_power_buffer = (uint16_t)Array[9]<<8 | Array[8];   
	extPowerHeatData->shooter_id1_17mm_cooling_heat = (uint16_t)Array[11]<<8 | Array[10];
	extPowerHeatData->shooter_id2_17mm_cooling_heat = (uint16_t)Array[13]<<8 | Array[12];
	extPowerHeatData->shooter_id1_42mm_cooling_heat = (uint16_t)Array[15]<<8 | Array[14];
}

/*
***************************************************
函数名：judgeExtGameRobotPosInfo
功能：获取机器人位置朝向信息(0x0203)
入口参数：	Array：数据段数组指针
					  extGameRobotPos 位置朝向信息数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameRobotPosInfo(uint8_t *Array,ext_game_robot_pos_t *extGameRobotPos ){
#if POSITION_DEFINE
	FormatTrans dataTrans;
	dataTrans.u8_temp[0] = Array[0];
	dataTrans.u8_temp[1] = Array[1];
	dataTrans.u8_temp[2] = Array[2];
	dataTrans.u8_temp[3] = Array[3];
	extGameRobotPos -> x = dataTrans.float_temp;
	
	dataTrans.u8_temp[0] = Array[4];
	dataTrans.u8_temp[1] = Array[5];
	dataTrans.u8_temp[2] = Array[6];
	dataTrans.u8_temp[3] = Array[7];
	extGameRobotPos -> y = dataTrans.float_temp;
	
	dataTrans.u8_temp[0] = Array[8];
	dataTrans.u8_temp[1] = Array[9];
	dataTrans.u8_temp[2] = Array[10];
	dataTrans.u8_temp[3] = Array[11];
	extGameRobotPos -> z = dataTrans.float_temp;	
	
	dataTrans.u8_temp[0] = Array[12];
	dataTrans.u8_temp[1] = Array[13];
	dataTrans.u8_temp[2] = Array[14];
	dataTrans.u8_temp[3] = Array[15];
	extGameRobotPos -> yaw = dataTrans.float_temp;
	
#endif
}
/*
***************************************************
函数名：judgeExtBuffMuskInfo
功能：获取裁判系统机器人增益数据（0x0204）
入口参数：	Array：数据段数组指针
					extBuffMusk：机器人增益数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtBuffMuskInfo(uint8_t *Array,ext_buff_musk_t *extBuffMusk ){
	extBuffMusk->power_rune_buff = (uint8_t)Array[0];
}
/*
***************************************************
函数名：judgeaerialRobotEnergyInfo
功能：获取裁判系统空中机器人能量状态数据（0x0205）
入口参数：	Array：数据段数组指针
					aerialRobotEnergy：空中机器人能量状态数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeaerialRobotEnergyInfo(uint8_t *Array,aerial_robot_energy_t *aerialRobotEnergy ){
	aerialRobotEnergy->attack_time = (uint8_t)Array[0];
}
/*
***************************************************
函数名：judgeExtRobotHurtInfo
功能：获取裁判系统伤害状态数据（0x0206）
入口参数：	Array：数据段数组指针
					extRobotHurt：伤害状态数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtRobotHurtInfo(uint8_t *Array,ext_robot_hurt_t *extRobotHurt ){
	extRobotHurt->armor_id = (uint8_t)Array[0]&0x0f;
	extRobotHurt->hurt_type = ((uint8_t)Array[0]&0xf0)>>4;
}
/*
***************************************************
函数名：judgeExtShootDataInfo
功能：获取裁判系统实时射击数据（0x0207）
入口参数：	Array：数据段数组指针
					extShootData：实时射击数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
#include "Driver_USBVCP.h"
void judgeExtShootDataInfo(uint8_t *Array,ext_shoot_data_t *extShootData ){
	FormatTrans dataTrans;
    extShootData->bullet_type = (uint8_t)Array[0];
	extShootData->shooter_id  = (uint8_t)Array[1];
	extShootData->bullet_freq = (uint8_t)Array[2];
	dataTrans.u8_temp[0] = Array[3];
	dataTrans.u8_temp[1] = Array[4];
	dataTrans.u8_temp[2] = Array[5];
	dataTrans.u8_temp[3] = Array[6];
	extShootData->bullet_speed = dataTrans.float_temp;
	
usbVCP_Printf("speed_out = %f\r\n",	extShootData->bullet_speed);
}
/*
***************************************************
函数名：judgeExtShootDataInfo
功能：获取裁判系统子弹剩余发射数数据（0x0208）
入口参数：	Array：数据段数组指针
					extShootData：实时射击数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtBulletRemainingInfo(uint8_t *Array,ext_bullet_remaining_t *extBulletRemaining ){
	extBulletRemaining->bullet_remaining_num_17mm = (uint16_t)Array[0]<<8 | Array[1];
	extBulletRemaining->bullet_remaining_num_42mm = (uint16_t)Array[2]<<8 | Array[3];
	extBulletRemaining->coin_remaining_num = (uint16_t)Array[4]<<8 | Array[5];
}

/*
***************************************************
函数名：judgeExtRFIDStatus
功能：机器人 RFID 状态（0x0209）
入口参数：	Array：数据段数组指针
					extRfidStatus：接收数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtRFIDStatus(uint8_t *Array,ext_rfid_status_t* extRfidStatus){
	FormatTrans dataTrans;
	dataTrans.u8_temp[0] = Array[0];
	dataTrans.u8_temp[1] = Array[1];
	dataTrans.u8_temp[2] = Array[2];
	dataTrans.u8_temp[3] = Array[3];
	extRfidStatus->rfid_status = dataTrans.u32_temp;
}

/*
***************************************************
函数名：judgeExtDartClientCmd
功能：飞镖机器人客户端接收指令数据（0x020A）
入口参数：	Array：数据段数组指针
					extDartClientCmd：飞镖机器人客户端接收指令数据
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtDartClientCmd(uint8_t *Array,ext_dart_client_cmd_t* extDartClientCmd ){
	extDartClientCmd->dart_launch_opening_status = (uint8_t)Array[0];
	extDartClientCmd->dart_attack_target = (uint8_t)Array[1];
	extDartClientCmd->target_change_time = (uint16_t)Array[2]<<8 | Array[3];
	extDartClientCmd->first_dart_speed = (uint8_t)Array[4];
	extDartClientCmd->second_dart_speed = (uint8_t)Array[5];
	extDartClientCmd->third_dart_speed = (uint8_t)Array[6];
	extDartClientCmd->fourth_dart_speed = (uint8_t)Array[7];
	extDartClientCmd->last_dart_launch_time = (uint16_t)Array[8]<<8 | Array[9];
	extDartClientCmd->operate_launch_cmd_time = (uint16_t)Array[10]<<8 | Array[11];
}
/*
***************************************************
函数名：judgeOtherRobotSendDataInfo
功能：获取其他机器人发送来数据（0x0301）
入口参数：	Array：数据段数组指针
					extReceiveData：接收数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeOtherRobotSendDataInfo(uint8_t *Array,ext_receive_data_t *extReceiveData){	
	uint8_t index = 0;
	for(index = 0; index < dataLen; index ++) 
	extReceiveData->data[index] = Array[index];
}

/*
***************************************************
函数名：Driver_Judge_SendData
功能：向裁判系统发送数据
入口参数：	senddata：数据段数组指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
//void judgeExtShowData(extShowData_t *extShowData,ext_game_robot_state_t *extGameRobotState){
//	static unsigned char Seq=0;
//	FormatTrans dataTrans;
//	uint8_t i;
//	JUDGE_SEND_BUFF[0] = 0xA5;
//	JUDGE_SEND_BUFF[1] = 0x13;				//data长度包含ID在内 长度固定
//	JUDGE_SEND_BUFF[2] = 0x00;
//	JUDGE_SEND_BUFF[3] = Seq;
//	JUDGE_SEND_BUFF[5] = 0x01;
//	JUDGE_SEND_BUFF[6] = 0x03;
//	/***内容ID****/
//	JUDGE_SEND_BUFF[7] = 0x80;
//	JUDGE_SEND_BUFF[8] = 0xD1;
//	/***发送者ID**/
//	JUDGE_SEND_BUFF[9] = (uint8_t)extGameRobotState->robot_id;
//	JUDGE_SEND_BUFF[10] = 0X00;
//	/***客户端ID**/
//	if(extGameRobotState->robot_id > 9){        //说明是蓝方
//		JUDGE_SEND_BUFF[11] = (uint8_t)(extGameRobotState->robot_id + 6);
//		JUDGE_SEND_BUFF[12] = 0X01;
//	}
//	else{
//		JUDGE_SEND_BUFF[11] = (uint8_t)extGameRobotState->robot_id;
//		JUDGE_SEND_BUFF[12] = 0X01;
//	}
//	
//	dataTrans.float_temp = extShowData->data1;
//	for(i=0;i<4;i++){
//		JUDGE_SEND_BUFF[13+i] = dataTrans.u8_temp[i];
//	}
//	
//	dataTrans.float_temp = extShowData->data2;
//	for(i=0;i<4;i++){
//		JUDGE_SEND_BUFF[17+i] = dataTrans.u8_temp[i];
//	}
//	
//	dataTrans.float_temp = extShowData->data3;
//	for(i=0;i<4;i++){
//		JUDGE_SEND_BUFF[21+i] = dataTrans.u8_temp[i];
//	}
//	
//	JUDGE_SEND_BUFF[25] = extShowData->mask;
//	Append_CRC8_Check_Sum(JUDGE_SEND_BUFF,5);
//	Append_CRC16_Check_Sum(JUDGE_SEND_BUFF,28);
//	Seq++;
//	BSP_USART1_DMA_SendData(JUDGE_SEND_BUFF,28);
//}

/*
***************************************************
函数名：Driver_Judge_SendData
功能：机器人间交互数据
入口参数：	senddata：数据段数组指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void robotExchangeData(ext_student_interacttive_header_data_t *extStudentData,ext_game_robot_state_t *extGameRobotState,uint8_t robotID){
	static unsigned char Seq=0;
	uint8_t count = 0;
	extStudentData->data_cmd_id = 0x0200; // 0x0200 ~ 0x02FF
	extStudentData->send_id = extGameRobotState->robot_id;
	if(extGameRobotState->robot_id > 9) //己方是蓝方
		//只能发送给己方
		extStudentData->receive_id = robotID + 0x0a;
	else
		//己方是红方
		extStudentData->receive_id = robotID;
	//如果是自己发送给自己
	if(extGameRobotState->robot_id == extStudentData->receive_id ){
		//发给自己什么都不做
	}
	else{
		//填装数据长度小于113 个字节
		JUDGE_SEND_BUFF[0] = 0xA5;
		JUDGE_SEND_BUFF[2] = 0x00;
		JUDGE_SEND_BUFF[3] = Seq;
		JUDGE_SEND_BUFF[5] = 0x01;
		JUDGE_SEND_BUFF[6] = 0x03;
		/****内容ID****/
		JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)(extStudentData->data_cmd_id & 0x00ff);
		JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)((extStudentData->data_cmd_id & 0xff00)>>8);

		/***发送者ID***/
		JUDGE_SEND_BUFF[7+(count++)] = extStudentData->send_id;
		JUDGE_SEND_BUFF[7+(count++)] = 0x00;
		
		/***接收者ID***/
		JUDGE_SEND_BUFF[7+(count++)] = extStudentData->receive_id;
		JUDGE_SEND_BUFF[7+(count++)] = 0x00;
		
		/**数据段***/
		JUDGE_SEND_BUFF[7+(count++)] = 0x00;   // 数据填装  数据带宽最大为112byte  测试数据“1”正式填装将其覆盖
		/*..........*/
		/**********/
		
		JUDGE_SEND_BUFF[1] = count;						//数据段长度
		
		Append_CRC8_Check_Sum(JUDGE_SEND_BUFF,5);
		Append_CRC16_Check_Sum(JUDGE_SEND_BUFF,count+9);
		Seq++;
		BSP_USART1_DMA_SendData(JUDGE_SEND_BUFF,count+9);	
	}
}

//射击模式
graphic_data_struct_t UI_CONFING_SHOOTMODE_NUM[1] = {
{
	.graphic_name = {0,0,2},
	{
		.operate_tpye = add_ui,
		.graphic_tpye = integer_number,
		.layer = 5,
		.color = yellow_ui,
		.width = 2,
		.start_angle = 20,
		.end_angle = 0,
		.start_x = 1400,
		.start_y = 460,
	},
	.Integer_type = 0,
},
};

//电容容量
graphic_data_struct_t UI_CONFING_EDLC_NUM[1] = {
{
	.graphic_name = {1,0,2},
	{
		.operate_tpye = add_ui,
		.graphic_tpye = integer_number,
		.layer = 5,
		.color = yellow_ui,
		.width = 3,
		.start_angle = 32,
		.end_angle = 0,
		.start_x = 1200,
		.start_y = 300,
	},
	.Integer_type = 0,
},
};
//摩擦轮1
graphic_data_struct_t UI_CONFING_FRIC_NUM1[1] = {
{
	.graphic_name = {1,0,3},
	{
		.operate_tpye = add_ui,
		.graphic_tpye = integer_number,
		.layer = 5,
		.color = yellow_ui,
		.width = 5,
		.start_angle = 50,
		.end_angle = 0,
		.start_x = 50,
		.start_y = 750,
	},
	.Integer_type = 0,
},
};
//摩擦轮2
graphic_data_struct_t UI_CONFING_FRIC_NUM2[1] = {
{
	.graphic_name = {1,0,4},
	{
		.operate_tpye = add_ui,
		.graphic_tpye = integer_number,
		.layer = 5,
		.color = yellow_ui,
		.width = 5,
		.start_angle = 50,
		.end_angle = 0,
		.start_x = 50,
		.start_y = 680,
	},
	.Integer_type = 0,
},
};

//辅助瞄准1
graphic_data_struct_t UI_CONFING_aim_assist1[7] = {
	{   .graphic_name = {0,1,1},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = green_ui,
			.width = 0,
			.start_x = 950,
			.start_y = 400,
			.end_x	 = 970,
			.end_y	 = 400,
		},
	},

	{  	.graphic_name = {0,1,2},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = circle,
			.layer = 1,
			.color = white_ui,
			.width = 0,
			.start_x = 960,
			.start_y = 540,
			.radius = 76,
		},
	},

	{  	.graphic_name = {0,1,3},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = circle,
			.layer = 1,	
			.color = white_ui,
			.width = 2,
			.start_x = 960,
			.start_y = 540,
			.radius = 20,
		},
	},

	{  	.graphic_name = {0,1,4},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = circle,
			.layer = 2,
			.color = black_ui,
			.width = 4,
			.start_x = 960,
			.start_y = 140,
			.radius = 40,
		},
	},

	{   .graphic_name = {0,1,5},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = cyan_ui,
			.width = 3,
			.start_x = 960,
			.start_y = 140,
			.end_x	 = 960,
			.end_y	 = 180,
		},
	},

	{		.graphic_name = {0,1,6},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = red_blue_ui,
			.width = 6,
			.start_x = 960,
			.start_y = 230,
			.end_x = 920,
			.end_y = 280,
		},	
	},

	{		.graphic_name = {0,1,7},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = red_blue_ui,
			.width = 6,
			.start_x = 960,
			.start_y = 230,
			.end_x = 1000,
			.end_y = 280,
		},	
	},
};

//辅助瞄准2
graphic_data_struct_t UI_CONFING_aim_assist2[7] = {
	{   .graphic_name = {0,2,1},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = white_ui,
			.width = 1,
			.start_x = 960,
			.start_y = 500,
			.end_x	 = 960,
			.end_y	 = 240,	
		},
	},
	{   .graphic_name = {0,2,2},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = red_blue_ui,
			.width = 1,
			.start_x = 900,
			.start_y = 500,
			.end_x	 = 1020,
			.end_y	 = 500,
		},
	},
	{   .graphic_name = {0,2,3},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = green_ui,
			.width = 1,
			.start_x = 940,
			.start_y = 460,
			.end_x	 = 980,
			.end_y	 = 460,
		},
	},
	{   .graphic_name = {0,2,4},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = yellow_ui,
			.width = 1,
			.start_x = 940,
			.start_y = 420,
			.end_x	 = 980,
			.end_y	 = 420,
		},
	},
	{   .graphic_name = {0,2,5},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = orange_ui,
			.width = 1,
			.start_x = 940,
			.start_y = 380,
			.end_x	 = 980,
			.end_y	 = 380,
		},
	},
	{   .graphic_name = {0,2,6},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = pink_ui,
			.width = 1,
			.start_x = 940,
			.start_y = 340,
			.end_x	 = 980,
			.end_y	 = 340,
		},
	},
	{   .graphic_name = {0,2,7},
		{
			.operate_tpye = add_ui,
			.graphic_tpye = straight_line,
			.layer = 1,
			.color = cyan_ui,
			.width = 1,
			.start_x = 940,
			.start_y = 300,
			.end_x	 = 980,
			.end_y	 = 300,
		},
	},	
};

//超级电容电量显示
void cap_energy_UI(int32_t cap_energy,graphic_data_struct_t *UI_CONFING){
	UI_CONFING[0].bitSet.operate_tpye = amend_ui;
	if(cap_energy >= 90)
		UI_CONFING[0].bitSet.color = green_ui;
	else if(cap_energy >= 85 && cap_energy < 90)
		UI_CONFING[0].bitSet.color = cyan_ui;
	else if(cap_energy >= 80 && cap_energy < 85)
		UI_CONFING[0].bitSet.color = white_ui;
	else if(cap_energy >= 60 && cap_energy < 80)
		UI_CONFING[0].bitSet.color = pink_ui;
	else if(cap_energy >= 40 && cap_energy < 60)
		UI_CONFING[0].bitSet.color = yellow_ui;
	else if(cap_energy < 40)
		UI_CONFING[0].bitSet.color = amaranth_ui;
	else
		UI_CONFING[0].bitSet.color = black_ui;
	UI_CONFING[0].Integer_type = cap_energy;
}

//射击模式ui
void shootMode_UI(u8 flag1,graphic_data_struct_t *UI_CONFING){
	UI_CONFING[0].bitSet.operate_tpye = amend_ui;
	switch(flag1){
		case 1:
			UI_CONFING[0].bitSet.color = cyan_ui;
			UI_CONFING[0].bitSet.start_x = 1400;
			UI_CONFING[0].Integer_type = 1;
			break;
		case 2:
			UI_CONFING[0].bitSet.color = orange_ui;
			UI_CONFING[0].bitSet.start_x = 1400;
			UI_CONFING[0].Integer_type = 3;
			break;
		case 3:
			UI_CONFING[0].bitSet.color = white_ui;
			UI_CONFING[0].bitSet.start_x = 1380;
			UI_CONFING[0].Integer_type = 999;
			break;
		default:
		break;
	}
}

//射击模式ui
void fricLspeed_UI(int16_t speed,graphic_data_struct_t *UI_CONFING){
	UI_CONFING[0].bitSet.operate_tpye = amend_ui;
	UI_CONFING[0].bitSet.color = cyan_ui;
	UI_CONFING[0].Integer_type = (int32_t)speed;
}
void fricRspeed_UI(int16_t speed,graphic_data_struct_t *UI_CONFING){
	UI_CONFING[0].bitSet.operate_tpye = amend_ui;
	UI_CONFING[0].bitSet.color = cyan_ui;
	UI_CONFING[0].Integer_type = (int32_t)speed;
}


//移动靶心
void aim_move_UI(f32_t pitch_angle,f32_t yaw_angle,bool flag1,bool flag2,bool flag3,bool flag4,graphic_data_struct_t *UI_CONFING){
	UI_CONFING[1].bitSet.operate_tpye = amend_ui;
	if(flag4){
		UI_CONFING[1].bitSet.color = green_ui;
		UI_CONFING[1].bitSet.width = 5;
	}
	else{
		UI_CONFING[1].bitSet.color = white_ui;
		UI_CONFING[1].bitSet.width = 0;
	}
	UI_CONFING[2].bitSet.operate_tpye = amend_ui;
	if(!flag1 && !flag2){
		UI_CONFING[2].bitSet.start_x = 960 - yaw_angle * 4.142f;
		UI_CONFING[2].bitSet.start_y = 540 - pitch_angle * 6.142f;
	}
	else{
		UI_CONFING[2].bitSet.start_x = 960;
		UI_CONFING[2].bitSet.start_y = 540;
	}
	if(flag3)
		UI_CONFING[2].bitSet.color = amaranth_ui;
	else
		UI_CONFING[2].bitSet.color = white_ui;
	
	UI_CONFING[3].bitSet.operate_tpye = amend_ui;
	if(flag1)
		UI_CONFING[3].bitSet.color = green_ui;
	else if(flag2)
		UI_CONFING[3].bitSet.color = orange_ui;
	else
		UI_CONFING[3].bitSet.color = black_ui;
	UI_CONFING[4].bitSet.operate_tpye = amend_ui;
	if(!flag1 && !flag2)
		UI_CONFING[4].bitSet.end_x = 960 - 40 * sinf(yaw_angle * 0.02f);
	else
		UI_CONFING[4].bitSet.end_x = 960 + 40 * sinf(yaw_angle * 0.02f);
	UI_CONFING[4].bitSet.end_y = 140 + 40 * cosf(yaw_angle * 0.02f);

	UI_CONFING[5].bitSet.operate_tpye = amend_ui;	
	UI_CONFING[5].bitSet.start_x = 960;
	UI_CONFING[5].bitSet.end_x = 960 - 50 * sin((50.0f + getchassisData()->pitchGyroAngle) * 0.02f);
	UI_CONFING[5].bitSet.start_y = 230;
	UI_CONFING[5].bitSet.end_y = 230 + 50 * cos((50.0f + getchassisData()->pitchGyroAngle) * 0.02f);
	
	UI_CONFING[6].bitSet.operate_tpye = amend_ui;
	UI_CONFING[6].bitSet.start_x = 960;
	UI_CONFING[6].bitSet.end_x = 960 + 50 * sin((50.0f - getchassisData()->pitchGyroAngle) * 0.02f);
	UI_CONFING[6].bitSet.start_y = 230;
	UI_CONFING[6].bitSet.end_y = 230 + 50 * cos((50.0f - getchassisData()->pitchGyroAngle) * 0.02f);
}
//目前UI界面不支持浮点数和负整形
void robot_ui_setting(ext_student_interacttive_header_data_t *extStudentData,ext_game_robot_state_t *extGameRobotState,   \
	graphic_data_struct_t *graphicData,int16_t cmd_id,uint8_t Number_graphics){
	static unsigned char Seq = 0;
	uint8_t count = 0;
	uint8_t IDX = 1;
	if(cmd_id == draw_character_shape_id)  IDX = 0;
	extStudentData->data_cmd_id = cmd_id;  //命令id
	extStudentData->send_id = extGameRobotState->robot_id;
	extStudentData->receive_id = extStudentData->send_id + 0x100;
	JUDGE_SEND_BUFF[0] = 0xA5;      //SOF 起始帧固定为0xA5
  //JUDGE_SEND_BUFF[1]、[2]是数据帧中 data 的长度（两个字节）
	JUDGE_SEND_BUFF[2] = 0x00; 
	JUDGE_SEND_BUFF[3] = Seq;   //包序号
  //JUDGE_SEND_BUFF[4] 帧头 CRC8 校验
	 
  //命令码 ID:0x0301   机器人间交互数据，发送方触发发送，上限 10Hz
	JUDGE_SEND_BUFF[5] = 0x01;  
	JUDGE_SEND_BUFF[6] = 0x03;
	/****内容ID****/
	JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)(extStudentData->data_cmd_id & 0x00ff);
	JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)((extStudentData->data_cmd_id & 0xff00)>>8);
	/***发送者ID***/
	JUDGE_SEND_BUFF[7+(count++)] = extStudentData->send_id;
	JUDGE_SEND_BUFF[7+(count++)] = 0x00;
	/***接收者ID***/
	JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)(extStudentData->receive_id & 0x00ff);
	JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)((extStudentData->receive_id & 0xff00)>>8);     //12
	/***绘制图形***/
	formatTrans32Struct_t trans32_data;	
	for(;Number_graphics > 0;Number_graphics--,IDX++){
		for(uint8_t index = 0; index < 3; index++)
			//图形名字  在删除，修改等操作中，作为客户端的索引。
			JUDGE_SEND_BUFF[7+(count++)] = graphicData[IDX].graphic_name[index];       
				
		trans32_data.u32_temp = graphicData[IDX].bitSet.operate_tpye |//图形操作	
							(graphicData[IDX].bitSet.graphic_tpye << 3) |//图形类型
							(graphicData[IDX].bitSet.layer << 6) |//图层数
						    (graphicData[IDX].bitSet.color << 10) |//图形颜色
							(graphicData[IDX].bitSet.start_angle << 14) |//起始角度
							(graphicData[IDX].bitSet.end_angle << 23);//结束角度
												 
		for(uint8_t index = 0; index < 4; index++)
			JUDGE_SEND_BUFF[7+(count++)] = trans32_data.u8_temp[index];                 
		trans32_data.u32_temp = graphicData[IDX].bitSet.width |  //线宽
								(graphicData[IDX].bitSet.start_x << 10) |  //起点 x 坐标
								(graphicData[IDX].bitSet.start_y << 21);   //起点 y 坐标
		for(uint8_t index = 0; index < 4; index++)
			JUDGE_SEND_BUFF[7+(count++)] = trans32_data.u8_temp[index];               
			
		if(graphicData[IDX].bitSet.graphic_tpye == floating_number)
			trans32_data.u32_temp = graphicData[IDX].floating_point_type;
		else if(graphicData[IDX].bitSet.graphic_tpye == integer_number)
			trans32_data.u32_temp = graphicData[IDX].Integer_type;
		else{
			trans32_data.u32_temp = graphicData[IDX].bitSet.radius |          //字体大小或者半径
									(graphicData[IDX].bitSet.end_x << 10) |   //终点 x 坐标；
									(graphicData[IDX].bitSet.end_y << 21);    //终点 y 坐标;
		}
		for(uint8_t index = 0; index < 4; index++)
			JUDGE_SEND_BUFF[7+(count++)] = trans32_data.u8_temp[index];        
   }
//	if(cmd_id == draw_character_shape_id){
//		for(uint8_t index = 0; index < character_length; index++)
//			JUDGE_SEND_BUFF[7+(count++)] =graphicData[0].ui_character[index]; 
//	}
		
		
	JUDGE_SEND_BUFF[1] = count;                                            
	Append_CRC8_Check_Sum(JUDGE_SEND_BUFF,5);//JUDGE_SEND_BUFF[4] 帧头 CRC8 校验                               
	Append_CRC16_Check_Sum(JUDGE_SEND_BUFF,count + 9);
	Seq++;
	BSP_USART_DMA_SendData(JUDGE_USARTX,JUDGE_SEND_BUFF,count+9);
}

