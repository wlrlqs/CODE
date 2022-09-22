#ifndef __DRIVER_JUDGE_H
#define __DRIVER_JUDGE_H

#include "BSP.h"
#include "Util.h"
#include "stdbool.h"
#define JUDGE_USARTX				  UART4			//接收机串口号
#define JUDGE_USARTX_TX_PIN			  BSP_GPIOA0	//接收机发送引脚
#define JUDGE_USARTX_RX_PIN			  BSP_GPIOA1	//接收机接收引脚
#define JUDGE_USART_PRE_PRIORITY 3					//SBUS_USART中断抢占优先级
#define JUDGE_USART_SUB_PRIORITY 0					//SBUS_USART中断响应优先级
//裁判系统发送数组
#define JUDGE_SEND_BUFF Array_USART1_TX
#define Armor_0 0
#define Armor_1 1
#define Armor_2 2
#define Armor_3 3
#define Armor_4 4
#define Armor_5 5

#define Armor_Front 	Armor_0            //正装甲板  	  0			
#define Armor_Left 		Armor_1            //左装甲板     1
#define Armor_Behind 	Armor_2            //后装甲板     2
#define Armor_Right		Armor_3            //后装甲板     3
#define Armor_UP1 		Armor_4
#define Armor_UP2 		Armor_5

#define Damage_Armor 		0x00
#define Damage_OverSpeed 	0x01
#define Damage_OverFreq 	0x02
#define Damage_OverPower 	0x03
#define Damage_Offline 		0x04
#define Damage_Foul  		0x06
#define Heal_Park 			0x0A
#define Heal_Engineer 		0x0B



#define LEN           118
#define dataLen       112
#pragma pack(1)


//比赛机器人状态 (0x0001)
typedef struct 
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
}ext_game_state_t; 
 
//比赛结果数据 (0x0002)
typedef struct 
{    
 uint8_t winner; 
}ext_game_result_t; 

//机器人血量数据 (0x0003)
typedef struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP;
 uint16_t red_3_robot_HP;
 uint16_t red_4_robot_HP;
 uint16_t red_5_robot_HP;
 uint16_t red_7_robot_HP;
 uint16_t red_outpost_HP;
 uint16_t red_base_HP;
 uint16_t blue_1_robot_HP;
 uint16_t blue_2_robot_HP;
 uint16_t blue_3_robot_HP;
 uint16_t blue_4_robot_HP;
 uint16_t blue_5_robot_HP;
 uint16_t blue_7_robot_HP;
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} ext_game_robot_HP_t;

//飞镖发射状态（0x0004）
typedef struct
{
 uint8_t dart_belong;
 uint16_t stage_remaining_time;
} ext_dart_status_t;

//场地事件数据 (0x0101)
typedef struct
{
	uint32_t event_type;
}ext_event_data_t;

//补给站动作标识（0x0102）
typedef struct 
{   
	uint8_t supply_projectile_id;    
	uint8_t supply_robot_id;    
	uint8_t supply_projectile_step;  
	uint8_t supply_projectile_num;
}ext_supply_projectile_action_t;


//飞镖发射倒计时（0x0105）
typedef struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

//裁判系统警告信息
typedef struct 
{
uint8_t level;
uint8_t foul_robot_id;  
}ext_referee_warning_t; 

//比赛机器人状态（0x0201）
typedef struct 
{   
  uint8_t robot_id;                 //英雄ID
  uint8_t robot_level;              //英雄等级
  uint16_t remain_HP;         		//剩余血量
  uint16_t max_HP;             		//最大血量
  uint16_t shooter_id1_17mm_cooling_rate;          //17mm冷却速度
  uint16_t shooter_id1_17mm_cooling_limit;         //17mm冷却限制
  uint16_t shooter_id1_17mm_speed_limit;           //17mm 射速限制
  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;
  uint16_t chassis_power_limit;             		//底盘功率限制
  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
}ext_game_robot_state_t; 

//实时功率热量数据(0x0202)
typedef struct 
{   
	uint16_t chassis_volt;			//电压
	uint16_t chassis_current;  		//电流
	float 	 chassis_power;         //功率
	uint16_t chassis_power_buffer;  //缓冲功率
	uint16_t shooter_id1_17mm_cooling_heat;    
	uint16_t shooter_id2_17mm_cooling_heat;  
	uint16_t shooter_id1_42mm_cooling_heat;
}ext_power_heat_data_t; 

//机器人位置(0x0203)
typedef struct
{
	float x;
	float y;
	float z;
	float yaw;
}ext_game_robot_pos_t;

//机器人增益(0x0204)	
typedef struct
{
	uint8_t power_rune_buff;	
}ext_buff_musk_t;

//空中机器人的状态(0x0205)
typedef struct
{
	uint8_t attack_time;
}aerial_robot_energy_t;

//伤害状态(0x0206)
typedef struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
}ext_robot_hurt_t;

//实时射击信息(0x0207)
typedef struct
{
   uint8_t bullet_type;
   uint8_t shooter_id;
   uint8_t bullet_freq;
   float bullet_speed;
}ext_shoot_data_t;

//子弹剩余发射数 (0x0208 空中机器人及哨兵机器人主控发送)
typedef struct 
{   
   uint16_t bullet_remaining_num_17mm;
   uint16_t bullet_remaining_num_42mm;
   uint16_t coin_remaining_num;   
}ext_bullet_remaining_t; 

//机器人RFID状态（0x0209）
typedef struct
{
	uint32_t rfid_status;
}ext_rfid_status_t;

//飞镖机器人客户端指令数据(0x020a)
typedef struct
{
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint8_t first_dart_speed;
	uint8_t second_dart_speed;
	uint8_t third_dart_speed;
	uint8_t fourth_dart_speed;
	uint16_t last_dart_launch_time;
	uint16_t operate_launch_cmd_time;
}ext_dart_client_cmd_t;

//一下发送id 为0x0301
//机器人交互 内容ID 发送者ID 接收者ID
typedef struct
{
	uint16_t data_cmd_id;
	uint16_t send_id;
	uint16_t receive_id;
}ext_student_interacttive_header_data_t;

typedef struct
{
	uint8_t data[dataLen];
}ext_receive_data_t;

//学生机器人间通信
typedef struct
{
	uint8_t data[dataLen];
}robot_interactive_data_t;
#pragma pack()

//ui界面图形个数
enum{
one_ui   = 1,
two_ui   = 2,
five_ui  = 5,
seven_ui = 7,
};
//operate_tpye：图形操作
enum{
No_Operation   = 0, //空操作
add_ui         = 1, //增加
amend_ui       = 2, //修改
delete_ui      = 3, //删除
};
//graphic_tpye：图形类型
enum{
straight_line    = 0, //直线
orthogon         = 1, //矩形
circle           = 2, //整圆
ellipse          = 3, //椭圆
arc              = 4, //圆弧
floating_number  = 5, //浮点数
integer_number   = 6, //整型数
chars            = 7, //字符
};

//color：颜色
enum{
red_blue_ui       = 0, //红蓝主色
yellow_ui         = 1, //黄色
green_ui          = 2, //绿色
orange_ui         = 3, //橙色
amaranth_ui       = 4, //紫红色
pink_ui           = 5, //粉色
cyan_ui           = 6, //青色
black_ui          = 7, //黑色
white_ui          = 8, //白色
};

#define remove_graphics_id 		    0x0100
#define draw_one_shape_id	        0x0101
#define draw_two_shape_id	        0x0102
#define draw_five_shape_id	        0x0103
#define draw_seven_shape_id	        0x0104
#define draw_character_shape_id	    0x0110
//图形数据
typedef struct
{
	uint8_t graphic_name[3];
	struct{
		uint32_t operate_tpye:3;//图形操作
		uint32_t graphic_tpye:3;//图形类型
		uint32_t layer:4;//图层数
		uint32_t color:4;//颜色
		uint32_t start_angle:9;//起始角度，单位：°，范围[0,360]
		uint32_t end_angle:9;//终止角度，单位：°，范围[0,360]
		uint32_t width:10;//线宽
		uint32_t start_x:11;//起点 x 坐标
		uint32_t start_y:11;//起点 y 坐标
		uint32_t radius:10;//字体大小或者半径
		uint32_t end_x:11;//终点 x 坐标
		uint32_t end_y:11;//终点 y 坐标
	}bitSet;
   int32_t  Integer_type;//整型数
   float    floating_point_type;//浮点数
   uint8_t *ui_character;//字符
 } graphic_data_struct_t;

typedef struct 
{
	unsigned char header;
	unsigned char tail;
	unsigned char buffer[256];
}buffefLoop_t;


extern graphic_data_struct_t UI_CONFING_EDLC_NUM[1];
extern graphic_data_struct_t UI_CONFING_FRIC_NUM1[1];
extern graphic_data_struct_t UI_CONFING_FRIC_NUM2[1];
extern graphic_data_struct_t UI_CONFING_aim_assist1[7];
extern graphic_data_struct_t UI_CONFING_aim_assist2[7];
extern graphic_data_struct_t UI_CONFING_SHOOTMODE_NUM[1];
extern buffefLoop_t bufferLoop;
void Driver_Judge_Init(USART_TypeDef *Judge_USARTx,BSP_GPIOSource_TypeDef *Judge_USART_TX,BSP_GPIOSource_TypeDef *Judge_USART_RX,uint8_t PreemptionPriority,uint8_t SubPriority);
void judgeExtGameStateInfo(uint8_t *Array,ext_game_state_t *extGameState);
void judgeExtGameResultInfo(uint8_t *Array,ext_game_result_t *extGameResult);
void judgeExtGameRobotHpInfo(uint8_t *Array,ext_game_robot_HP_t *extgameRobotHP);
void judgeExtGameRobotDartStatusInfo(uint8_t *Array,ext_dart_status_t *extDartStatus);
void judgeExtEventDataInfo(uint8_t *Array,ext_event_data_t *extEventData);   
void judgeExtSupplyProjectileActionInfo(uint8_t *Array,ext_supply_projectile_action_t *extSupplyProjectileAction);
void judgeExtGameRobotDartRemainingInfo(uint8_t *Array,ext_dart_remaining_time_t *extDartRemainingTime);
void judgeExtRefereeWarningInfo(uint8_t *Array,ext_referee_warning_t *extRefereeWarning);
void judgeExtGameRobotStateInfo(uint8_t *Array,ext_game_robot_state_t *extGameRobotState); 
void judgeExtPowerHeatDataInfo(uint8_t *Array,ext_power_heat_data_t *extPowerHeatData);
void judgeExtGameRobotPosInfo(uint8_t *Array,ext_game_robot_pos_t *extGameRobotPos );
void judgeExtBuffMuskInfo(uint8_t *Array,ext_buff_musk_t *extBuffMusk );
void judgeaerialRobotEnergyInfo(uint8_t *Array,aerial_robot_energy_t *aerialRobotEnergy );
void judgeExtRobotHurtInfo(uint8_t *Array,ext_robot_hurt_t *extRobotHurt );
void judgeExtShootDataInfo(uint8_t *Array,ext_shoot_data_t *extShootData );
void judgeExtBulletRemainingInfo(uint8_t *Array,ext_bullet_remaining_t *extBulletRemaining );
void judgeExtRFIDStatus(uint8_t *Array,ext_rfid_status_t* extRfidStatus);
void judgeExtDartClientCmd(uint8_t *Array,ext_dart_client_cmd_t* extDartClientCmd );
void judgeOtherRobotSendDataInfo(uint8_t *Array,ext_receive_data_t *extReceiveData);
void robotExchangeData(ext_student_interacttive_header_data_t *extStudentData,ext_game_robot_state_t *extGameRobotState,uint8_t robotID);
void cap_energy_UI(int32_t cap_energy,graphic_data_struct_t *UI_CONFING);
void shootMode_UI(u8 flag1,graphic_data_struct_t *UI_CONFING);
void aim_move_UI(f32_t pitch_angle,f32_t yaw_angle,bool flag1,bool flag2,bool flag3,bool flag4,graphic_data_struct_t *UI_CONFING);
void fricLspeed_UI(int16_t speed,graphic_data_struct_t *UI_CONFING);
void fricRspeed_UI(int16_t speed,graphic_data_struct_t *UI_CONFING);
void robot_ui_setting(ext_student_interacttive_header_data_t *extStudentData,ext_game_robot_state_t *extGameRobotState,graphic_data_struct_t *graphicData,int16_t cmd_id,uint8_t Number_graphics);
//CRC
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
#endif
