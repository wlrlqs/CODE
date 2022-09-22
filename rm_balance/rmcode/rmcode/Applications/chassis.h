#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "Util.h"
#include "pid.h"
#include "imu.h"
#include "clockcount.h"
#include "Driver_Power.h"
#include "power.h"


//麦轮直径
#define DIAMETER_OF_MOTOR  				0.187f
//电机减速比，如果换减速箱请更改此数，优化后可能需要通过上位机来确定此值
#define GEAR_RATIO						13.70f      //3508：（3591/187）≈19.20
//机电转化效率，一般情况下电机不可能达到最大速率，这个值需要负载后跑动测得，其决定电机满速时底盘的速度，需要更具减速箱来更改
#define ELETRO_MECHANICAL_EFFICIENCY 	0.98f
//键盘控制的最小速度m/s
#define REAL_MOTOR_SPEED_SCALE 			(PI * DIAMETER_OF_MOTOR / (60.0f * GEAR_RATIO)) * ELETRO_MECHANICAL_EFFICIENCY //实际电机转速比
#define MIN_CHASSIS_SPEED  				0.2f	  //底盘最小速度   
#define JOINT_ANGLE_RANGE         31000

//#define POWER_LIMIT   				   getpowerData() ->powerLimit              	//80.0f
//#define WARNING_POWER 				   getpowerData() ->warningPower 				//40.0f
//#define WARNING_POWER_BUFF 			   getpowerData() ->WarningPowerBuff 			//50.0f
//#define NO_JUDGE_TOTAL_CURRENT_LIMIT   getpowerData() ->noJudgeTotalCurrentLimit 	//64000.0f
//#define JUDGE_TOTAL_CURRENT_LIMIT      getpowerData() ->judgeTotalCurrentLimit 		//38000.0f 更改这个可以改变启动功率
//#define ADD_POWER_CURRENT              getpowerData() ->addPowerCurrent            	//18000.0f

#define POWER_LIMIT   				   get_judgeData()->extGameRobotState.chassis_power_limit    //最大功率限制
#define WARNING_POWER 				   get_power_judge()->warning_power		//40.0f
#define WARNING_POWER_BUFF 			   get_power_judge()->warning_power_buff			      //缓冲功率
#define NO_JUDGE_TOTAL_CURRENT_LIMIT   64000 	//64000.0f
#define JUDGE_TOTAL_CURRENT_LIMIT      get_power_judge()->total_current_limit 		//38000.0f 更改这个可以改变启动功率
#define ADD_POWER_CURRENT              get_power_judge()->add_power_limit            	//18000.0f   电流加成值

#define WHEEL_OF_AUX                   2      //辅助车轮
#define LEFT_WHEEL_ENCODER			   ((f32_t)motorHandleClass.Encoder(&commandoMotorConfig[9]))
#define RIGHT_WHEEL_ENCODER			   ((f32_t)motorHandleClass.Encoder(&commandoMotorConfig[10]))

#define LQR_K1 -0.035355339059329f
#define LQR_K2 -2.649978767404223f	//速度反馈系数
#define LQR_K3 -12.551414269395824f //pitch轴角度反馈系数
#define LQR_K4 -1.961874622097917f	//pitch轴角速度反馈系数
#define LQR_K15 2.958039891549834f	//yaw轴角度差反馈系数
#define LQR_K16 0.895171277508466f	//yaw轴角速度反馈系数
#define LQR_K25	LQR_K15
#define LQR_K26	LQR_K16

#define SAFTY_ANGLE	25.0f	// 角度保护
#define STUCK_ANGLE 10.0f // 卡住角度
#define MED_POS_LEFT	149
#define MED_POS_RIGHT	150

enum {
	L_WHEEL = 0,
	R_WHEEL,
	NUMBER_OF_WHEEL           //2
};

typedef enum {
	CHASSIS_RELAX = 0,
	CHASSIS_INIT,
	CHASSIS_STOP,
	CHASSIS_SEPARATE_GIMBAL,
	CHASSIS_FOLLOW_GIMBAL,
	CHASSIS_AVOID_MODE,
} chassisMode_e;

enum floorType{
	LEVEL_LAND = 0,
	SLOPE_LAND
};

typedef struct{	//new power control
	f32_t		 		wReal[2];
	f32_t				Force[2];
	f32_t				ForceTotal_2;
	f32_t				wRealTotal_2;
	f32_t				A;
	f32_t				B;
	f32_t				C;
	f32_t				Pmax;
	f32_t				Pin;
	f32_t				K1;
	f32_t				K2;
	f32_t				K_law;              //
}powerJudgeStruct_t;

typedef struct{
	f32_t 	speed;
	f32_t 	omega;
	f32_t 	torque;
	f32_t 	torque_set;
	int16_t current_set;
}chassis_motor_t;

typedef struct{
	uint8_t		ID[2];
	f32_t		distance[2];//dis*1000
	uint16_t	signal_strengthen[2];
	uint16_t	reserved[2];
		//滑块sliding_blocks
	f32_t				encoder[2];		//滑块电机编码器
	f32_t				encoderLast[2];
	f32_t				angle[2];			//test
	f32_t				angleReal[2]; 	//test
	f32_t				angleSpeed[2];	//滑块角速度
	f32_t				positionOut[2];	//滑块位置环输出
	f32_t				angleOut[2];	//滑块角度环输出，速度闭环
	f32_t				rateOut[2];		//滑块速度环输出
	f32_t				position[2];		//滑块位置
	//自适应重心
	f32_t				mid_position[2];
}sliding_blocks_t;

typedef struct{
	f32_t lqr_pitch_gain;
	f32_t lqr_speed_gain;
	f32_t lqr_pitchspeed_gain;
	f32_t	lqr_yaw_gain;
	f32_t	lqr_yawspeed_gain;
}chassisLQRGainStruct_t;

typedef struct{
	chassis_motor_t	motor_chassis[2];	
								//轮毂电机
	f32_t	vx_l;				//底盘速度，左单位m/s
	f32_t	vx_r;				//底盘速度，右单位m/s
	f32_t	vx;					//底盘速度
	f32_t	omega;				//底盘轮毂角速度,单位rad/s
	f32_t	vx_set;				//底盘速度设定，速度期望单位m/s
	f32_t	chassisYawSet;		//底盘yaw轴角度设定
	f32_t	delta_angle;		//底盘yaw轴角度差	
	f32_t	s;
	f32_t	vx_max_speed;		//前进方向最大速度，单位m/s
	f32_t	vx_min_speed;		//后退方向最大速度，单位m/s
	f32_t	chassis_yaw;		//底盘imu反馈的yaw轴角度
	f32_t	chassis_yaw_last;	
	f32_t	chassis_pitch;		//底盘imu反馈的pitch轴角度
	f32_t	chassis_pitch_last;
	f32_t	chassis_roll;		//底盘反馈的roll轴角度
	f32_t	chassis_yaw_speed;	//底盘yaw轴角速度
	f32_t	chassis_pitch_speed;//底盘pitch轴角速度
	f32_t	chassis_roll_speed;	//底盘roll轴角速度
	f32_t	chassis_high_left;	//底盘左关节高度
	f32_t	chassis_high_right;	//底盘右关节高度
	f32_t	chassis_vx_limit;
	f32_t	speed_temp;//加速比例
	//
	f32_t	lqr_k1;
	f32_t	lqr_k2;
	f32_t	lqr_k3;
	f32_t	lqr_k4;
	f32_t	lqr_k15;
	f32_t	lqr_k16;
	f32_t	lqr_k25;
	f32_t	lqr_k26;
	//
	u8		fall;
	u8		stuck;
	u8		autoCenterFlag;
	f32_t	CNTR;
}chassis_LQR_t;

typedef struct {	
	coordinateFloat_t	sumofChssisTarget; 
	coordinateFloat_t	manualSpeedTarget; 
	coordinateFloat_t	autoSpeedTarget;
	uint8_t 			autoMode;
	uint8_t 			changeHeadOrder;
	uint8_t 			changeHeadSchedule;
	uint8_t 			changeChassisSchedule;
	uint8_t 			changeMode;
	uint8_t 			jointInitFlag;
	chassisMode_e 		ctrlMode;   
	chassisMode_e 		lastCtrlMode;
	int8_t  			direction; 
	
	//陀螺仪角度，PY
	f32_t 				yawGyroAngle;         
	f32_t 				pitchGyroAngle;
	f32_t 				rollGyroAngle;
	
	//角度期望，PY
	f32_t 				yawAngleRef;
	f32_t 				pitchAngleRef;
	f32_t 				rollAngleRef;
	f32_t        	jointAngleRef[4];
	
	//角度反馈，PY
	f32_t         lastYawAngle;
	f32_t 				yawAngleSave;
	f32_t 				yawAngleFbd;
	f32_t 				pitchAngleFbd;
	f32_t 				rollAngleFbd;
	f32_t				pitchGyroFbd;
	f32_t				motorAngleSave[2];
	f32_t				motorAngleRef[2];
	u8					stopFlag;
	
	f32_t         		slantAngle;
	f32_t				slantAngle_rate;//LJL
	f32_t				chassis_velocity_out;//LJL
	f32_t				chassis_vertical_out;//LJL
	f32_t				chassis_med_angle;
	f32_t				left_wheel_vel;	//左轮轮速
	f32_t				right_wheel_vel;
	f32_t				Encoder_Intergal;
	f32_t				Encoder_speed;
	f32_t				Encoder_speed_last;
	f32_t				Car_position;
	f32_t				Speed_mid;
	f32_t				left_wheel_encoder_last;
	f32_t				right_wheel_encoder_last;
	f32_t				left_wheel_angleFbd;
	f32_t				right_wheel_angleFbd;
	f32_t				left_wheel_angle_reduceFbd;
	f32_t				right_wheel_angle_reduceFbd;
	f32_t 				positionOut[2];

	f32_t 				landingSpeedx;
	f32_t 				landingSpeedy;
	f32_t 				landingSpeedz;
	f32_t 				speedLimit;
	f32_t 				powerCurrent[4];
	f32_t 				current[2];	
	f32_t 				chaseRef;
	f32_t 				chaseFbd;
	f32_t 				chaseAngleOut;
	f32_t 				chaseSpeedRef;
	f32_t 				chaseSpeedFbd;
	f32_t 				autoSpeedMote;
	f32_t 				speedFbdMax;
	f32_t 				speedLimitFloor;
	f32_t 				DecelerateRatio;
	f32_t 				rotate;
	uint8_t 			floorType;
	f32_t 				yawCenterSave;
	coordinateFloat_t 	posRef;
	coordinateFloat_t 	posFbd;
	uint32_t			chassis_wheel_error_last[2];
	uint8_t				chassis_fault;
	f32_t 				speedFbd[2];
	f32_t 				speedRef[2];
	f32_t				anti_speedRef[2];
	f32_t 				jointSpeedRef[4];//PY
	f32_t 				drivenRef[2];
	f32_t 				drivenFbd[2];
	f32_t 				drivenOut[2];
	int16_t       		jointOut[4];//PY
	int32_t       		jointLastOut[4];//PY
	int32_t       		jointRefGain[4];//PY
	int32_t       		jointPitchGain[4];//PY
	int32_t       		jointGainMid[4];//PY
	int32_t       		jointRollGain[4];//PY
	//位置环相关参数
	int32_t         	position[2];//PY
	uint16_t         	positionSave[2];//PY
	int32_t        		lastPosition[2];//PY
	uint8_t       	 	positionSchedule;
	
	f32_t				armWheelRef;
	f32_t				armWheelFbd;
	f32_t				armWheelOut;
	f32_t				syncWheelRef;
	f32_t				syncWheelFbd;
	f32_t				syncWheelOut;
	f32_t 				scale[2];
	f32_t 				averageScale;
	f32_t				manualSpeedTargetBuf[2];
	uint8_t				suspend_chassis;
	errorScanStruct_t 	wheelError[2];
	errorScanStruct_t 	currentError;
	pidStruct_t 		*chasePID;
	pidStruct_t 		*chaseSpeedPID;
	pidStruct_t 		*posPID;
	pidStruct_t 		*speedAllPID;
	pidStruct_t 		*speedPID[2];
	pidStruct_t 		*currentPID[2];
	pidStruct_t 		*jointAnglePID[4];
	pidStruct_t 		*jointPitchPID[4];
	
	pidStruct_t     	*positionPID;//PY
						//pitch轴PID结构体
	pidStruct_t 		*pitchAnglePID;//PY
						//roll轴PID结构体
	pidStruct_t 		*rollAnglePID[4];//PY
	
	pidStruct_t 		*auxDriverWheelPID[2];
	pidStruct_t			*armWheelPID;
	pidStruct_t			*syncWheelPID;
    float				diffSpeedLimit;		//工程辅助轮限速
	int8_t 				turnswitch; 		//工程抓弹方向调换
	float				roDir;
	f32_t 				intervalTime;
	double 				time[2];
	uint8_t       		ctalSchedule;
	float         		outRestrain;
	float         		lineRestrain[2];
	int32_t       		jointPosInit[4];
	int32_t       		jointPosMid[4];
	float         		flashTime;
	uint8_t       		chassisFlash;
	//supercap puls
	u8					chassis_fast_back;
	u8					chassis_boost;
} chassisStruct_t;



powerJudgeStruct_t* get_power_judge(void);
chassisStruct_t* getchassisData(void);
chassis_LQR_t* getchassisLQRData(void);
sliding_blocks_t* getsliding_blocksData(void);
void chassisUpdate(void);
static void chassisStopHandle(void);
static void chassisRelaxHandle(void);
static void followGimbalHandle(void);
static void separateGimbalHandle(void);
void avoidHandle(void);
static void powerLimitHandle(void);
void chassisInit(void);
void relax_chassis(void);
f32_t Vertical(float Med_angle,float Pitch_angle,float Ptich_rate);
f32_t Velocity(float Traget, float Encoder_left,float Encoder_right);
f32_t anti_Velocity(float Traget, float Encoder_left,float Encoder_right);
void left_wheelMotorAngleUpdate(void);
void right_wheelMotorAngleUpdate(void);
void getSlideMotorAngle(void);//滑块角度
void chassis_wheel_control(chassis_LQR_t *chassis_wheel_control_loop);
void NoopLoop_CanRecieve(sliding_blocks_t* tofRecieve,CanRxMsg* CanRevData);
void NoopLoop_UsartRecieve(uint8_t * array,uint16_t len);
void NoopLoop_Usart6Recieve(uint8_t * array,uint16_t len);
f32_t slidingBlocksControl(pidStruct_t* posPid,f32_t setpos,f32_t posFbd,f32_t med_pos);
f32_t LevelSpeed(uint16_t level);
static void autoGravityCenter(void);
void 	blocksFeedback(void);
void  lqrFeedbackUpdate(void);
void  fallDownProtection(void);
void  directionChosen(void);
void 	speed_temp_choose(void);
#endif
