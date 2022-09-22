#include "Driver_Judge.h"
#include "judge.h"
#include "crc.h"
#include "math.h"
#include "chassis.h"
#define POSITION_DEFINE 	1	//�Ƿ��ȡposition����
#define READBULLET_DEFINE 1	//�Ƿ��ȡ�ӵ�����
#define READGOLF_DEFINE 	1	//�Ƿ��ȡ�߶�������



buffefLoop_t bufferLoop = {
	.header = 0,
	.tail		= 0,
	.buffer	= {0},
};

/*
***************************************************
��������Driver_Judge_Init
���ܣ�����ϵͳ���ڳ�ʼ��
��ڲ�����	Judge_USARTx������ϵͳ���ں�
					Judge_USART_TX�����ڷ������ź�
					Judge_USART_RX�����ڽ������ź�
					PreemptionPriority����ռ���ȼ�
					SubPriority�������ȼ�
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void Driver_Judge_Init(USART_TypeDef *Judge_USARTx,
												BSP_GPIOSource_TypeDef *Judge_USART_TX,
												BSP_GPIOSource_TypeDef *Judge_USART_RX,
												uint8_t PreemptionPriority,uint8_t SubPriority){
	BSP_USART_TypeDef Judge_USART = {
		.USARTx = Judge_USARTx,				//���ں�
		.USART_RX = Judge_USART_RX,		//����
		.USART_TX = Judge_USART_TX,
		.USART_InitStructure = {
			.USART_BaudRate = 115200,										/*����������*/					
			.USART_WordLength = USART_WordLength_8b,		/*�ֳ�Ϊ8λ���ݸ�ʽ*/	
			.USART_StopBits = USART_StopBits_1,					/*һ��ֹͣλ*/					
			.USART_Parity = USART_Parity_No,						/*��У��λ*/						
			.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,/*���պͷ���ģʽ*/						
			.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
		}
	};
	BSP_USART_Init(&Judge_USART,PreemptionPriority,SubPriority);
	/*- ��ʱ���Σ��г�ͻ -*/
	BSP_USART_TX_DMA_Init(&Judge_USART);
	BSP_USART_RX_DMA_Init(&Judge_USART);
}

/*
***************************************************
��������judgeExtGameStateInfo
���ܣ���ȡ����ϵͳ����״̬����(0x0001)
��ڲ�����	Array�����ݶ�����ָ��
					extGameState������״̬����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeExtGameStateInfo(uint8_t *Array,ext_game_state_t *extGameState){
	formatTrans64Struct_t dataTrans;
	extGameState->game_type    = (uint8_t)Array[0]&0x0f;                   //0-3 bit���������� 
	extGameState->game_progress   = ((uint8_t)Array[0]&0xf0)>>4;           //4-7 bit����ǰ�����׶�
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
��������judgeExtGameResultInfo
���ܣ���ȡ����ϵͳ����������� (0x0002)
��ڲ�����	Array�����ݶ�����ָ��
					extGameResult�������������ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeExtGameResultInfo(uint8_t *Array,ext_game_result_t *extGameResult){
   extGameResult->winner = (uint8_t)Array[0];
}

/*
***************************************************
��������judgeExtGameRobotSurvivorsInfo
���ܣ���ȡ����ϵͳ����������Ѫ������ 1HZ (0x0003)
��ڲ�����	Array�����ݶ�����ָ��
					extGameRobotSurvivors�������˴������ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
��������judgeExtGameRobotDartStatusInfo
���ܣ���ȡ����ϵͳ���ڷ���״̬ 1HZ (0x0004)
��ڲ�����	Array�����ݶ�����ָ��
			extDartStatus: ���ڷ���״ָ̬��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeExtGameRobotDartStatusInfo(uint8_t *Array,ext_dart_status_t *extDartStatus){
	extDartStatus->dart_belong = (uint8_t)Array[0];
	extDartStatus->stage_remaining_time = (uint16_t) Array[1]<<8 | Array[2];
}

/*
***************************************************
��������judgeExtEventDataInfo
���ܣ���ȡ����ϵͳ�����¼����� (0x0101)
��ڲ�����	Array�����ݶ�����ָ��
					extEventData�������¼�����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
��������judgeExtSupplyProjectileActionInfo
���ܣ���ȡ����ϵͳ���ز���վ������ʶ����(0x0102)
��ڲ�����	Array�����ݶ�����ָ��
					extSupplyProjectileAction�����ز���վ������ʶ����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
��������judgeExtSupplyProjectileBookingInfo
���ܣ���ȡ����ϵͳ��������(0x0104)
��ڲ�����	Array�����ݶ�����ָ��
					extRefereeWarning������ϵͳ��������ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeExtRefereeWarningInfo(uint8_t *Array,ext_referee_warning_t *extRefereeWarning){
	extRefereeWarning->level 		 = (uint8_t)Array[0];
	extRefereeWarning->foul_robot_id = (uint8_t)Array[1];
}

/*
***************************************************
��������judgeExtGameRobotDartRemainingInfo
���ܣ����ڷ���ڵ���ʱ 1HZ (0x0105)
��ڲ�����	Array�����ݶ�����ָ��
			extDartStatus: ���ڷ���ڵ���ʱ
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeExtGameRobotDartRemainingInfo(uint8_t *Array,ext_dart_remaining_time_t *extDartRemainingTime){
	extDartRemainingTime->dart_remaining_time = (uint8_t)Array[0];
}

/*
***************************************************
��������judgeExtGameRobotStateInfo
���ܣ���ȡ����ϵͳ������״̬���ݣ�0x0201��
��ڲ�����	Array�����ݶ�����ָ��
					extGameRobotState��������״̬����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
��������judgeExtPowerHeatDataInfo
���ܣ���ȡ����ϵͳʵʱ���������������ݣ�0x0202��
��ڲ�����	Array�����ݶ�����ָ��
					extPowerHeatData��ʵʱ������������ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
��������judgeExtGameRobotPosInfo
���ܣ���ȡ������λ�ó�����Ϣ(0x0203)
��ڲ�����	Array�����ݶ�����ָ��
					  extGameRobotPos λ�ó�����Ϣ����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
��������judgeExtBuffMuskInfo
���ܣ���ȡ����ϵͳ�������������ݣ�0x0204��
��ڲ�����	Array�����ݶ�����ָ��
					extBuffMusk����������������ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeExtBuffMuskInfo(uint8_t *Array,ext_buff_musk_t *extBuffMusk ){
	extBuffMusk->power_rune_buff = (uint8_t)Array[0];
}
/*
***************************************************
��������judgeaerialRobotEnergyInfo
���ܣ���ȡ����ϵͳ���л���������״̬���ݣ�0x0205��
��ڲ�����	Array�����ݶ�����ָ��
					aerialRobotEnergy�����л���������״̬����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeaerialRobotEnergyInfo(uint8_t *Array,aerial_robot_energy_t *aerialRobotEnergy ){
	aerialRobotEnergy->attack_time = (uint8_t)Array[0];
}
/*
***************************************************
��������judgeExtRobotHurtInfo
���ܣ���ȡ����ϵͳ�˺�״̬���ݣ�0x0206��
��ڲ�����	Array�����ݶ�����ָ��
					extRobotHurt���˺�״̬����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeExtRobotHurtInfo(uint8_t *Array,ext_robot_hurt_t *extRobotHurt ){
	extRobotHurt->armor_id = (uint8_t)Array[0]&0x0f;
	extRobotHurt->hurt_type = ((uint8_t)Array[0]&0xf0)>>4;
}
/*
***************************************************
��������judgeExtShootDataInfo
���ܣ���ȡ����ϵͳʵʱ������ݣ�0x0207��
��ڲ�����	Array�����ݶ�����ָ��
					extShootData��ʵʱ�������ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
��������judgeExtShootDataInfo
���ܣ���ȡ����ϵͳ�ӵ�ʣ�෢�������ݣ�0x0208��
��ڲ�����	Array�����ݶ�����ָ��
					extShootData��ʵʱ�������ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeExtBulletRemainingInfo(uint8_t *Array,ext_bullet_remaining_t *extBulletRemaining ){
	extBulletRemaining->bullet_remaining_num_17mm = (uint16_t)Array[0]<<8 | Array[1];
	extBulletRemaining->bullet_remaining_num_42mm = (uint16_t)Array[2]<<8 | Array[3];
	extBulletRemaining->coin_remaining_num = (uint16_t)Array[4]<<8 | Array[5];
}

/*
***************************************************
��������judgeExtRFIDStatus
���ܣ������� RFID ״̬��0x0209��
��ڲ�����	Array�����ݶ�����ָ��
					extRfidStatus����������ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
��������judgeExtDartClientCmd
���ܣ����ڻ����˿ͻ��˽���ָ�����ݣ�0x020A��
��ڲ�����	Array�����ݶ�����ָ��
					extDartClientCmd�����ڻ����˿ͻ��˽���ָ������
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
��������judgeOtherRobotSendDataInfo
���ܣ���ȡ���������˷��������ݣ�0x0301��
��ڲ�����	Array�����ݶ�����ָ��
					extReceiveData����������ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void judgeOtherRobotSendDataInfo(uint8_t *Array,ext_receive_data_t *extReceiveData){	
	uint8_t index = 0;
	for(index = 0; index < dataLen; index ++) 
	extReceiveData->data[index] = Array[index];
}

/*
***************************************************
��������Driver_Judge_SendData
���ܣ������ϵͳ��������
��ڲ�����	senddata�����ݶ�����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
//void judgeExtShowData(extShowData_t *extShowData,ext_game_robot_state_t *extGameRobotState){
//	static unsigned char Seq=0;
//	FormatTrans dataTrans;
//	uint8_t i;
//	JUDGE_SEND_BUFF[0] = 0xA5;
//	JUDGE_SEND_BUFF[1] = 0x13;				//data���Ȱ���ID���� ���ȹ̶�
//	JUDGE_SEND_BUFF[2] = 0x00;
//	JUDGE_SEND_BUFF[3] = Seq;
//	JUDGE_SEND_BUFF[5] = 0x01;
//	JUDGE_SEND_BUFF[6] = 0x03;
//	/***����ID****/
//	JUDGE_SEND_BUFF[7] = 0x80;
//	JUDGE_SEND_BUFF[8] = 0xD1;
//	/***������ID**/
//	JUDGE_SEND_BUFF[9] = (uint8_t)extGameRobotState->robot_id;
//	JUDGE_SEND_BUFF[10] = 0X00;
//	/***�ͻ���ID**/
//	if(extGameRobotState->robot_id > 9){        //˵��������
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
��������Driver_Judge_SendData
���ܣ������˼佻������
��ڲ�����	senddata�����ݶ�����ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void robotExchangeData(ext_student_interacttive_header_data_t *extStudentData,ext_game_robot_state_t *extGameRobotState,uint8_t robotID){
	static unsigned char Seq=0;
	uint8_t count = 0;
	extStudentData->data_cmd_id = 0x0200; // 0x0200 ~ 0x02FF
	extStudentData->send_id = extGameRobotState->robot_id;
	if(extGameRobotState->robot_id > 9) //����������
		//ֻ�ܷ��͸�����
		extStudentData->receive_id = robotID + 0x0a;
	else
		//�����Ǻ췽
		extStudentData->receive_id = robotID;
	//������Լ����͸��Լ�
	if(extGameRobotState->robot_id == extStudentData->receive_id ){
		//�����Լ�ʲô������
	}
	else{
		//��װ���ݳ���С��113 ���ֽ�
		JUDGE_SEND_BUFF[0] = 0xA5;
		JUDGE_SEND_BUFF[2] = 0x00;
		JUDGE_SEND_BUFF[3] = Seq;
		JUDGE_SEND_BUFF[5] = 0x01;
		JUDGE_SEND_BUFF[6] = 0x03;
		/****����ID****/
		JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)(extStudentData->data_cmd_id & 0x00ff);
		JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)((extStudentData->data_cmd_id & 0xff00)>>8);

		/***������ID***/
		JUDGE_SEND_BUFF[7+(count++)] = extStudentData->send_id;
		JUDGE_SEND_BUFF[7+(count++)] = 0x00;
		
		/***������ID***/
		JUDGE_SEND_BUFF[7+(count++)] = extStudentData->receive_id;
		JUDGE_SEND_BUFF[7+(count++)] = 0x00;
		
		/**���ݶ�***/
		JUDGE_SEND_BUFF[7+(count++)] = 0x00;   // ������װ  ���ݴ������Ϊ112byte  �������ݡ�1����ʽ��װ���串��
		/*..........*/
		/**********/
		
		JUDGE_SEND_BUFF[1] = count;						//���ݶγ���
		
		Append_CRC8_Check_Sum(JUDGE_SEND_BUFF,5);
		Append_CRC16_Check_Sum(JUDGE_SEND_BUFF,count+9);
		Seq++;
		BSP_USART1_DMA_SendData(JUDGE_SEND_BUFF,count+9);	
	}
}

//���ģʽ
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

//��������
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
//Ħ����1
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
//Ħ����2
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

//������׼1
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

//������׼2
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

//�������ݵ�����ʾ
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

//���ģʽui
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

//���ģʽui
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


//�ƶ�����
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
//ĿǰUI���治֧�ָ������͸�����
void robot_ui_setting(ext_student_interacttive_header_data_t *extStudentData,ext_game_robot_state_t *extGameRobotState,   \
	graphic_data_struct_t *graphicData,int16_t cmd_id,uint8_t Number_graphics){
	static unsigned char Seq = 0;
	uint8_t count = 0;
	uint8_t IDX = 1;
	if(cmd_id == draw_character_shape_id)  IDX = 0;
	extStudentData->data_cmd_id = cmd_id;  //����id
	extStudentData->send_id = extGameRobotState->robot_id;
	extStudentData->receive_id = extStudentData->send_id + 0x100;
	JUDGE_SEND_BUFF[0] = 0xA5;      //SOF ��ʼ֡�̶�Ϊ0xA5
  //JUDGE_SEND_BUFF[1]��[2]������֡�� data �ĳ��ȣ������ֽڣ�
	JUDGE_SEND_BUFF[2] = 0x00; 
	JUDGE_SEND_BUFF[3] = Seq;   //�����
  //JUDGE_SEND_BUFF[4] ֡ͷ CRC8 У��
	 
  //������ ID:0x0301   �����˼佻�����ݣ����ͷ��������ͣ����� 10Hz
	JUDGE_SEND_BUFF[5] = 0x01;  
	JUDGE_SEND_BUFF[6] = 0x03;
	/****����ID****/
	JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)(extStudentData->data_cmd_id & 0x00ff);
	JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)((extStudentData->data_cmd_id & 0xff00)>>8);
	/***������ID***/
	JUDGE_SEND_BUFF[7+(count++)] = extStudentData->send_id;
	JUDGE_SEND_BUFF[7+(count++)] = 0x00;
	/***������ID***/
	JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)(extStudentData->receive_id & 0x00ff);
	JUDGE_SEND_BUFF[7+(count++)] = (uint8_t)((extStudentData->receive_id & 0xff00)>>8);     //12
	/***����ͼ��***/
	formatTrans32Struct_t trans32_data;	
	for(;Number_graphics > 0;Number_graphics--,IDX++){
		for(uint8_t index = 0; index < 3; index++)
			//ͼ������  ��ɾ�����޸ĵȲ����У���Ϊ�ͻ��˵�������
			JUDGE_SEND_BUFF[7+(count++)] = graphicData[IDX].graphic_name[index];       
				
		trans32_data.u32_temp = graphicData[IDX].bitSet.operate_tpye |//ͼ�β���	
							(graphicData[IDX].bitSet.graphic_tpye << 3) |//ͼ������
							(graphicData[IDX].bitSet.layer << 6) |//ͼ����
						    (graphicData[IDX].bitSet.color << 10) |//ͼ����ɫ
							(graphicData[IDX].bitSet.start_angle << 14) |//��ʼ�Ƕ�
							(graphicData[IDX].bitSet.end_angle << 23);//�����Ƕ�
												 
		for(uint8_t index = 0; index < 4; index++)
			JUDGE_SEND_BUFF[7+(count++)] = trans32_data.u8_temp[index];                 
		trans32_data.u32_temp = graphicData[IDX].bitSet.width |  //�߿�
								(graphicData[IDX].bitSet.start_x << 10) |  //��� x ����
								(graphicData[IDX].bitSet.start_y << 21);   //��� y ����
		for(uint8_t index = 0; index < 4; index++)
			JUDGE_SEND_BUFF[7+(count++)] = trans32_data.u8_temp[index];               
			
		if(graphicData[IDX].bitSet.graphic_tpye == floating_number)
			trans32_data.u32_temp = graphicData[IDX].floating_point_type;
		else if(graphicData[IDX].bitSet.graphic_tpye == integer_number)
			trans32_data.u32_temp = graphicData[IDX].Integer_type;
		else{
			trans32_data.u32_temp = graphicData[IDX].bitSet.radius |          //�����С���߰뾶
									(graphicData[IDX].bitSet.end_x << 10) |   //�յ� x ���ꣻ
									(graphicData[IDX].bitSet.end_y << 21);    //�յ� y ����;
		}
		for(uint8_t index = 0; index < 4; index++)
			JUDGE_SEND_BUFF[7+(count++)] = trans32_data.u8_temp[index];        
   }
//	if(cmd_id == draw_character_shape_id){
//		for(uint8_t index = 0; index < character_length; index++)
//			JUDGE_SEND_BUFF[7+(count++)] =graphicData[0].ui_character[index]; 
//	}
		
		
	JUDGE_SEND_BUFF[1] = count;                                            
	Append_CRC8_Check_Sum(JUDGE_SEND_BUFF,5);//JUDGE_SEND_BUFF[4] ֡ͷ CRC8 У��                               
	Append_CRC16_Check_Sum(JUDGE_SEND_BUFF,count + 9);
	Seq++;
	BSP_USART_DMA_SendData(JUDGE_USARTX,JUDGE_SEND_BUFF,count+9);
}

