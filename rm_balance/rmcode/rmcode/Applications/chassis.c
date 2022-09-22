#include "chassis.h"
#include "Driver_M1502AMotor.h"
#include "Driver_USBVCP.h"
#include "gimbal.h"
#include "rc.h"
#include "keyboard.h"
#include "config.h"
#include "Driver_RMMotor.h"
#include "judge.h"
#include "cansend.h"
#include "control.h"
#include "auto_auxiliary.h"
#include "auto_infantry.h"
#include "auto_sentry.h"
#include "imu.h"
#include "motorHandle.h"
#include "supercapacitor.h"

#define ANTI_SLIP
#define CHASSIS_SPEED_PID
#define ADD_IMU_RATE_PID

#define DOUBLE_GUN_HERO

//大腿长度
#define  THIGH_LEN                 0.10f
//小腿长度
#define  SHANK_LEN                 0.19f
//关节电机间距
#define  JOINT_BETWEEN_LEN         0.12f
//滑块位置
#define SLID_BLOCK_LEFT_FORWARD		(sliding_blocks.position[0] = 20)
#define SLID_BLOCK_RIGHT_FORWARD	(sliding_blocks.position[1] = 360 - 20)
#define SLID_BLOCK_LEFT_MID			(sliding_blocks.position[0] = 180)
#define SLID_BLOCK_RIGHT_MID		(sliding_blocks.position[1] = 360 - 180)
#define SLID_BLOCK_LEFT_BACKWARD	(sliding_blocks.position[0] = 340)
#define SLID_BLOCK_RIGHT_BACKWARD	(sliding_blocks.position[1] = 360 - 340)	

chassisStruct_t chassisData;
chassis_LQR_t 	chassisLQRData;
sliding_blocks_t	sliding_blocks;
uint16_t K_p[4]={50},speedKp=1000;
powerJudgeStruct_t powerJudgeData;
chassisLQRGainStruct_t	chassisLQRGain;
chassisStruct_t* getchassisData(){
    return &chassisData;
}
powerJudgeStruct_t* get_power_judge(void){
	return &powerJudgeData;
}
chassis_LQR_t* getchassisLQRData(void){
	return &chassisLQRData;
}
sliding_blocks_t* getsliding_blocksData(void){
	return &sliding_blocks;
}
static f32_t slantAngleCount(f32_t theta1, f32_t theta2, f32_t imuAngle);
	
//检测底盘故障
void chassis_check_error(void){
	chassisData.chassis_fault = 0x00;
	for(uint8_t index = 0; index < 4; index++){
		if(commandoMotorConfig[CHASSISMOTOR_RF + index].errorCount - chassisData.chassis_wheel_error_last[index] == 0)
			chassisData.chassis_fault = 1 << index;
	}
	//更新数据
	for(uint8_t index = 0; index < 4; index++)
		chassisData.chassis_wheel_error_last[index] = commandoMotorConfig[CHASSISMOTOR_RF + index].errorCount;
}
/*
***************************************************
函 数 名：	chassisDataUpdate
功		能：底盘任务更新
入口参数：	chassisData.autoSpeedTarget 底盘旋转运动速度目标(结构体，在任务中赋值x,y,z)
					chassisData.autoMode：切换自动任务标志
返 回 值：	无
应用范围：	外部调用
备		注：
**************************************************
*/
f32_t theta1,theta2,theta3,theta4;
f32_t speedOut[2],pitchOut[2],posOut[2]={0},yawOut[2],speedMid,s_out=15000,_out=0,speed_ref=0;
f32_t	distance_last[2];
f32_t	angle_rotate;
f32_t	slidKp[2];
void chassisUpdate(void){
	if(getrobotMode() == MODE_RC || getrobotMode() == MODE_KM){
		cap_energy_UI(getcapacitorData()->percentage,UI_CONFING_EDLC_NUM);//超级电容电量显示
		shootMode_UI(getshootData()->shootmode_ui,UI_CONFING_SHOOTMODE_NUM);
		fricLspeed_UI(getshootData()->L_speed,UI_CONFING_FRIC_NUM1);
		fricRspeed_UI(getshootData()->R_speed,UI_CONFING_FRIC_NUM2);//没起作用
		aim_move_UI(getGimbalData()->pitchGyroAngle,getGimbalData()->yawMotorAngle,\
		getinfantryAutoData()->rotateFlag,getinfantryAutoData()->aviodFlag,getshootData()->fricMotorFlag,getvisionData()->captureFlag,UI_CONFING_aim_assist1);
	}
	for(u8 i = 0;i < 2;i++){
		chassisData.speedFbd[i] = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[i+9]) * REAL_MOTOR_SPEED_SCALE;
		if(fabs(chassisData.speedFbd[i]) >= chassisData.speedFbdMax)
			chassisData.speedFbdMax = fabs(chassisData.speedFbd[i]);
	}	
	chassisData.time[0] = getClockCount();
	chassisData.intervalTime = (f32_t)(chassisData.time[0] - chassisData.time[1]);
	chassisData.time[1] = chassisData.time[0];
	chassisData.pitchAngleRef = 0;
	//底盘imu角度
	chassisData.yawGyroAngle = SC_YAW;
	chassisData.pitchGyroAngle = SC_PITCH;
	chassisData.pitchGyroFbd = IMU_RATEX;
	//速度上限和轮速
	chassisData.speedLimitFloor =  chassisData.speedLimit * getConfigData()->chassisRCSpeed * REAL_MOTOR_SPEED_SCALE;
	chassisData.left_wheel_vel = -(f32_t)motorHandleClass.Speed(&commandoMotorConfig[9]) * REAL_MOTOR_SPEED_SCALE;
	chassisData.right_wheel_vel = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[10]) * REAL_MOTOR_SPEED_SCALE;
	chassisData.Encoder_speed = chassisData.left_wheel_vel + chassisData.right_wheel_vel;
	chassisData.Encoder_speed_last *= 0.7f;
	chassisData.Encoder_speed_last += chassisData.Encoder_speed * 0.3f;
	chassisData.speedRef[0] = -chassisData.sumofChssisTarget.y * 2.0f;
	if(getlastrobotMode() == MODE_RELAX && getrobotMode() == MODE_RC && chassisData.ctalSchedule == 0){
		digitalIncreasing(&chassisData.ctalSchedule);
		chassisData.yawAngleSave = chassisData.yawGyroAngle;
	}
	//方向选择 
	directionChosen();
	//滑块	
	blocksFeedback();
	//LQR参数	
	lqrFeedbackUpdate();
	//倒地保护	
	fallDownProtection();
	//上控更新函数
	if(getrobotMode() != MODE_RELAX){
		chassisData.pitchAngleFbd = chassisData.pitchGyroAngle;
		chassisData.yawAngleFbd = chassisData.yawGyroAngle - chassisData.yawAngleSave;
		chassisData.yawAngleRef += chassisData.sumofChssisTarget.x;
		chassisData.pitchAngleRef = chassisData.sumofChssisTarget.y;
		chassisData.motorAngleRef[0]= chassisData.left_wheel_angle_reduceFbd - chassisData.motorAngleSave[0];
		chassisData.anti_speedRef[0] = -chassisData.left_wheel_vel;
		//速度与滑块
		speed_temp_choose();
		/*LQR*/		
		//倒地保护
		if(chassisLQRData.fall){
			chassisLQRData.vx_set = 0.0f;
			chassisLQRGain.lqr_pitch_gain = 1.0f;
			chassisLQRGain.lqr_pitchspeed_gain = 1.0f;
			chassisLQRGain.lqr_yaw_gain = 0.4f;
			chassisLQRGain.lqr_yawspeed_gain = 1.0f;
			chassisLQRGain.lqr_speed_gain = 1.0f;
			if(ABS(chassisData.pitchAngleFbd) < 4.0f){
				chassisLQRGain.lqr_yaw_gain = 6.0f;
				digitalLo(&chassisLQRData.fall);
			}
		}	
			
		if(chassisLQRData.stuck){
			chassisLQRGain.lqr_yaw_gain = 6.0f;
			digitalHi(&getinfantryAutoData()->rotateFlag);
		}
		
		chassisLQRData.delta_angle = (-chassisLQRData.chassisYawSet + chassisLQRData.chassis_yaw);
		chassisLQRData.vx = (chassisLQRData.vx_l - chassisLQRData.vx_r) / 2.0f;
		chassisLQRData.lqr_k25 = chassisLQRData.lqr_k15;
		chassisLQRData.lqr_k26 = chassisLQRData.lqr_k16;
		

		if(!getinfantryAutoData()->aviodFlag){
			chassisLQRData.chassisYawSet = 0.0f;
		}
		
		if(!getinfantryAutoData()->aviodFlag){
			chassisLQRData.vx_set = chassisData.sumofChssisTarget.y * chassisLQRData.speed_temp * chassisData.roDir;
		}
		else
			chassisLQRData.vx_set = chassisData.sumofChssisTarget.x * chassisLQRData.speed_temp * chassisData.roDir;
		
		
		chassisLQRData.chassis_vx_limit = LevelSpeed(powerJudgeData.Pmax);
		
		//自适应重心
		//autoGravityCenter();
	}
	else{
		chassisData.yawAngleRef = 0;
		chassisData.pitchAngleFbd = SC_PITCH;
		chassisData.yawAngleFbd = 0;
		chassisData.yawAngleSave = chassisData.yawGyroAngle;
		chassisLQRData.vx_set = 0;
		chassisLQRData.chassis_yaw = 0;
		chassisLQRData.chassisYawSet = 0;
		chassisLQRData.motor_chassis[0].torque_set = 0;
		chassisLQRData.motor_chassis[1].torque_set = 0;
	}		
	/**********************机体倾角***************************/
	chassisData.slantAngle = SC_PITCH;//不带轮腿
	chassisData.slantAngle_rate = IMU_RATEY;//角速度
	/*********************************************************/
	/*           记录电机行驶距离，用于定点稳定               */
	/*********************************************************/
	#ifdef	PID
	chassisData.chassis_vertical_out = Vertical(/*chassisData.pitchAngleRef + */chassisData.chassis_med_angle,chassisData.pitchAngleFbd,chassisData.slantAngle_rate);
	chassisData.chassis_velocity_out = Velocity(chassisData.speedRef[0],chassisData.left_wheel_vel,chassisData.right_wheel_vel);	
	speedOut[0] = -(chassisData.chassis_velocity_out + chassisData.chassis_vertical_out);
	speedOut[1] = -speedOut[0];
	#else
	#endif
	/**************************************************/
	yawOut[0] = (int16_t)pidUpdate(chassisData.chasePID, chassisData.yawAngleRef, chassisData.yawAngleFbd, chassisData.intervalTime);
	yawOut[1] = yawOut[0];
	/*************************************************/
	usbVCP_Printf("motorRef = %f",chassisData.motorAngleRef);
	usbVCP_Printf("accel = %f",getScimuData()->accel[0]);	
	//yaw轴 atan(x/y) 求解算角度  
	//yawAngle = atan(x/y) * (180.0/PI);yaw轴期望角度
	//角度解算到编码器码盘值
	/*********************************************************/
	if(getrobotMode() == MODE_RC || getrobotMode() == MODE_KM){
		#ifdef PID
		chassisData.current[0] = speedOut[0] + yawOut[0] + chassisData.positionOut[0]; //+ pitchOut[0] + yawOut[0] + chassisData.lineRestrain[0];  //
		chassisData.current[1] = speedOut[1] + yawOut[1] + chassisData.positionOut[1]; //+ pitchOut[1] + yawOut[1] + chassisData.lineRestrain[1];  // 
		chassisData.angleOut[0] = slidingBlocksControl(&getConfigData()->chassisPosPID,chassisData.position[0],sliding_blocks.distance[0],MED_POS_LEFT);
		chassisData.angleOut[1] = slidingBlocksControl(&getConfigData()->chassisPosPID,chassisData.position[1],sliding_blocks.distance[1],MED_POS_RIGHT);
		/*
		  加速时车头前倾，在加速的一段时间内滑块的位置直接到指定位置，在速度达到近似值后，
		滑块回到中值。在速度期望为0后，但是速度反馈还没能到0时，滑块到指定位置，然后再次回中。	
		*/
		#else
		if(getinfantryAutoData()->rotateFlag){
			chassisLQRData.delta_angle = angle_rotate * DEG_TO_RAD;//小陀螺
			sliding_blocks.rateOut[1] = 0;
			sliding_blocks.rateOut[0] = 0;
		}
		else if(getinfantryAutoData()->aviodFlag){
			chassisLQRData.chassisYawSet = 90.0f * DEG_TO_RAD;//此面向敌
		}
		chassis_wheel_control(getchassisLQRData());
		if(!chassisLQRData.autoCenterFlag){
			sliding_blocks.angleOut[1] = pidUpdate(chassisData.positionPID,sliding_blocks.position[1],sliding_blocks.angleReal[1],chassisData.intervalTime);		//chassisPositionPID
			sliding_blocks.angleOut[0] = pidUpdate(chassisData.posPID,sliding_blocks.position[0],sliding_blocks.angleReal[0],chassisData.intervalTime);					//chassisPosPID
			sliding_blocks.rateOut[1] = pidUpdate(chassisData.chasePID,sliding_blocks.angleOut[1],0/*sliding_blocks.angleSpeed[1]*/,chassisData.intervalTime);				//chassisChasePID
			sliding_blocks.rateOut[0] = pidUpdate(chassisData.chaseSpeedPID,sliding_blocks.angleOut[0],0/*sliding_blocks.angleSpeed[0]*/,chassisData.intervalTime);	//chassisRatePID
		}
		#endif
	}
	else{
		chassisData.current[0] = 0;
		chassisData.current[1] = 0;
		sliding_blocks.rateOut[0] = 0;
		sliding_blocks.rateOut[1] = 0;
	}
	
	powerJudgeData.ForceTotal_2 = 0;
	powerJudgeData.wRealTotal_2 = 0;
	powerJudgeData.Pin = 0;
	powerJudgeData.A = 0;
	powerJudgeData.B = 0;
	powerJudgeData.C = 0;
	for(uint8_t index = 0; index < NUMBER_OF_WHEEL; index++){
		powerJudgeData.Force[index] = fabs(chassisData.current[index] *(20.0f / 16384.0f * 0.0156223893f ) * 19.20f);
		powerJudgeData.wReal[index] = fabs(motorHandleClass.Speed(&commandoMotorConfig[CHASSISMOTOR_L + index]) * PI / 30.0f / 19.20f);
		powerJudgeData.ForceTotal_2 += pow(powerJudgeData.Force[index],2.0f);
		powerJudgeData.wRealTotal_2 += powerJudgeData.wReal[index];
		powerJudgeData.B += fabs(powerJudgeData.Force[index] * powerJudgeData.wReal[index]);
	}
	if(chassisData.chassis_boost == 1 /*&& chassisData.chassis_fast_back == 1 && getcapacitorData()->percentage > 30*/){
		powerJudgeData.Pmax = 300;
		chassisLQRData.speed_temp = 1.5f;
		chassisLQRGain.lqr_speed_gain = 1.9f;
		chassisLQRGain.lqr_pitch_gain = 1.0f;	
		powerJudgeData.K_law = 1;
	}
	else{
		powerJudgeData.Pmax = get_judgeData()->extGameRobotState.chassis_power_limit;
	}
	powerJudgeData.A = powerJudgeData.K1 * powerJudgeData.ForceTotal_2;
	powerJudgeData.C = powerJudgeData.K2 * powerJudgeData.wRealTotal_2 - powerJudgeData.Pmax;
	powerJudgeData.Pin = pow(powerJudgeData.B,2.0f) - 4 * (powerJudgeData.A * powerJudgeData.C);
	if(powerJudgeData.Pin > 0){
		powerJudgeData.K_law = (-powerJudgeData.B + sqrtf(powerJudgeData.Pin))/(2 * powerJudgeData.A);
		if(powerJudgeData.K_law > 1){
			powerJudgeData.K_law = 1;
		}
	}
	else
		powerJudgeData.K_law = 1;

	chassisData.powerCurrent[0] = chassisData.current[0];
	chassisData.powerCurrent[1] = chassisData.current[1];
//	
//	chassisData.powerCurrent[0] *= 0;//powerJudgeData.K_law;
//	chassisData.powerCurrent[1] *= 0;//powerJudgeData.K_law;
	chassisData.powerCurrent[0] *= powerJudgeData.K_law;
	chassisData.powerCurrent[1] *= powerJudgeData.K_law;
	
	chassisData.lastYawAngle = chassisData.yawAngleFbd;
}
//LJL
//PID
/*********************************************************************************************/
//直立环(加入机械中值是为了将来的自适应重心/狗头/)
f32_t Vertical(float Med_angle,float Pitch_angle,float Ptich_rate){
	static float pitchout;
	pitchout = getConfigData()->chassisPitchPID.PID_P * (Pitch_angle - Med_angle) + \
				getConfigData()->chassisPitchPID.PID_D * (Ptich_rate - 0);
	return pitchout;
}
//速度环(Encoder_left,Encoder_right为电机转速)
f32_t Velocity(float Traget, float Encoder_left,float Encoder_right){
	static float Encoder_S,Encoder_Err_Lowout_last,PWM_out,Encoder_Err,Encoder_Err_Lowout,pitchout;
	Encoder_Err = (Encoder_left + Encoder_right) - Traget;
	Encoder_Err_Lowout = (1 - 0.7f) * Encoder_Err + 0.7f * Encoder_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变(没啥用)
	Encoder_Err_Lowout_last = Encoder_Err_Lowout;
	Encoder_S += Encoder_Err_Lowout;
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	pitchout = getConfigData()->chassisAllSpeedPID.PID_P * Encoder_Err_Lowout + getConfigData()->chassisAllSpeedPID.PID_I * Encoder_S;
	return pitchout;
}
f32_t anti_Velocity(float Traget, float Encoder_left,float Encoder_right){
	static float Encoder_S,Encoder_Err_Lowout_last,PWM_out,Encoder_Err,Encoder_Err_Lowout,pitchout;
	Encoder_Err = (Encoder_left + Encoder_right) - Traget;
	Encoder_Err_Lowout = (1 - 0.7f) * Encoder_Err + 0.7f * Encoder_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变(没啥用)
	Encoder_Err_Lowout_last = Encoder_Err_Lowout;
	Encoder_S += Encoder_Err_Lowout;
	Encoder_S=Encoder_S>100?100:(Encoder_S<(-100)?(-100):Encoder_S);
	pitchout = getConfigData()->chassisSpeedPID.PID_P * Encoder_Err_Lowout + getConfigData()->chassisSpeedPID.PID_I * Encoder_S;
	return pitchout;
}
//转向环
//电机减速箱角度
/*********************************************************************************************/
void left_wheelMotorAngleUpdate(){
	f32_t	res1,res2,delta;
	if(LEFT_WHEEL_ENCODER < chassisData.left_wheel_encoder_last){
		res1 = LEFT_WHEEL_ENCODER + 8192 - chassisData.left_wheel_encoder_last;
		res2 = LEFT_WHEEL_ENCODER - chassisData.left_wheel_encoder_last;
	}
	else {
		res1 = LEFT_WHEEL_ENCODER - 8192 - chassisData.left_wheel_encoder_last;
		res2 = LEFT_WHEEL_ENCODER - chassisData.left_wheel_encoder_last;
	}
	if(ABS(res1) < ABS(res2)){
		delta = res1;
	}
	else{
		delta = res2;
	}
	chassisData.left_wheel_angleFbd += delta;
	chassisData.left_wheel_angle_reduceFbd = chassisData.left_wheel_angleFbd * (360.0f / 8192.0f) / 19.20f;
	chassisData.left_wheel_encoder_last = LEFT_WHEEL_ENCODER;
}
/*********************************************************************************************/
void right_wheelMotorAngleUpdate(){
	f32_t	res1,res2,delta;
	if(RIGHT_WHEEL_ENCODER < chassisData.right_wheel_encoder_last){
		res1 = RIGHT_WHEEL_ENCODER + 8192 - chassisData.right_wheel_encoder_last;
		res2 = RIGHT_WHEEL_ENCODER - chassisData.right_wheel_encoder_last;
	}
	else {
		res1 = RIGHT_WHEEL_ENCODER - 8192 - chassisData.right_wheel_encoder_last;
		res2 = RIGHT_WHEEL_ENCODER - chassisData.right_wheel_encoder_last;
	}
	if(ABS(res1) < ABS(res2)){
		delta = res1;
	}
	else{
		delta = res2;
	}
	chassisData.right_wheel_angleFbd += delta;
	chassisData.right_wheel_angle_reduceFbd = chassisData.right_wheel_angleFbd * (360.0f / 8192.0f) / 19.20f;
	chassisData.right_wheel_encoder_last = RIGHT_WHEEL_ENCODER;
}
/*********************************************************************************************/
void getSlideMotorAngle(){
	f32_t	res1,res2,delta;
	f32_t	res3,res4,delta2;
	if(sliding_blocks.encoder[0] < sliding_blocks.encoderLast[0]){
		res1 = sliding_blocks.encoder[0] + 8191 - sliding_blocks.encoderLast[0];
		res2 = sliding_blocks.encoder[0] - sliding_blocks.encoderLast[0];
	}
	else{
		res1 = sliding_blocks.encoder[0] - 8191 - sliding_blocks.encoderLast[0];
		res2 = sliding_blocks.encoder[0] - sliding_blocks.encoderLast[0];
	}
	if(ABS(res1) < ABS(res2)){
		delta = res1;
	}
	else{
		delta = res2;
	}
	sliding_blocks.angle[0] += delta;
	sliding_blocks.angleReal[0] = sliding_blocks.encoder[0] * (360.0f / 8191.0f);
	sliding_blocks.encoderLast[0] = sliding_blocks.encoder[0];//314.6f
	///
	if(sliding_blocks.encoder[1] < sliding_blocks.encoderLast[1]){
		res3 = sliding_blocks.encoder[1] + 8191 - sliding_blocks.encoderLast[1];
		res4 = sliding_blocks.encoder[1] - sliding_blocks.encoderLast[1];
	}
	else{
		res3 = sliding_blocks.encoder[1] - 8191 - sliding_blocks.encoderLast[1];
		res4 = sliding_blocks.encoder[1] - sliding_blocks.encoderLast[1];
	}
	if(ABS(res3) < ABS(res4)){
		delta2 = res3;
	}
	else{
		delta2 = res4;
	}
	sliding_blocks.angle[1] += delta2;
	sliding_blocks.angleReal[1] = sliding_blocks.encoder[1] * (360.0f / 8191.0f);//right
	sliding_blocks.encoderLast[1] = sliding_blocks.encoder[1];//359-15
}
/*********************************************************************************************/
f32_t slidingBlocksControl(pidStruct_t* posPid,f32_t setpos,f32_t posFbd,f32_t med_pos){
	f32_t outPut,Ref,rateOut;
	setpos = constrainFloat(setpos,-100,100);
	Ref = setpos - med_pos;
	outPut = pidUpdate(posPid,Ref,posFbd,chassisData.intervalTime);
}
/*********************************************************************************************/
//PY：机体倾角计算，推导过程及matlab计算文件网站“”
static f32_t slantAngleCount(f32_t theta1, f32_t theta2, f32_t imuAngle){
  double x1,x2,y1,y2,mid,x,y;
	f32_t kneeBtwLen,alpha,beta,angle;
	
	if(((theta1-theta2) < 1.5f && (theta1-theta2) > -1.5f)||(theta1 == 0 && theta2 == 0)) 
		return imuAngle;
	
	theta1 = (180 - theta1)/180 * PI;
  theta2 = theta2/180 * PI;
	
	x1 = THIGH_LEN * cos(theta1);
	y1 = THIGH_LEN * sin(theta1);
  x2 = THIGH_LEN * cos(theta2) + JOINT_BETWEEN_LEN;
	y2 = THIGH_LEN * sin(theta2);
	
	mid = pow(x2-x1,2)+pow(y2-y1,2);
	kneeBtwLen = sqrt(mid);
	
	alpha = atan((y2-y1)/(x2-x1));
  beta = acos(kneeBtwLen/(2 * SHANK_LEN));
	
	x = x1 + SHANK_LEN * cos(alpha + beta) - 0.065;
  y = y1 + SHANK_LEN * sin(alpha + beta);
	
	angle = imuAngle + atan(x/y)/PI * 180;
	
	return angle;
}
/***************************************LQR*****************************************/
/*
***************************************************
函 数 名：	chassis_wheel_control
功		能：LQR轮毂力矩更新
入口参数：	chassisLQRData	LQR结构体
返 回 值：	无
应用范围：	外部调用
备		注：
***************************************************
*/
//额外三个参数，乘上速度，pitch角度，pitch角速度    全局变量
static void chassis_wheel_control(chassis_LQR_t *chassis_wheel_control_loop){
	f32_t		max_torque = 0.0f;
	f32_t 	torque_rate = 0.0f;
	f32_t 	temp = 0.0f;
	u8 		i = 0;
	if(chassisLQRData.vx_set > chassisLQRData.vx_max_speed){
		chassisLQRData.vx_set = chassisLQRData.vx_max_speed;
	}
	chassis_wheel_control_loop->motor_chassis[0].torque_set = (-(chassisLQRGain.lqr_speed_gain * chassis_wheel_control_loop->lqr_k2 * (chassis_wheel_control_loop->vx_set - (chassis_wheel_control_loop->vx_l - chassis_wheel_control_loop->vx_r)/2.0f)) + \
																															chassisLQRGain.lqr_pitch_gain * chassis_wheel_control_loop->lqr_k3 * (chassis_wheel_control_loop->chassis_pitch) + \
																															chassisLQRGain.lqr_pitchspeed_gain * chassis_wheel_control_loop->lqr_k4 * (chassis_wheel_control_loop->chassis_pitch_speed) - \
																															chassisLQRGain.lqr_yaw_gain * chassis_wheel_control_loop->lqr_k15 * chassisLQRData.delta_angle + \
																															chassisLQRGain.lqr_yawspeed_gain * chassis_wheel_control_loop->lqr_k16 * chassis_wheel_control_loop->chassis_yaw_speed);
	chassis_wheel_control_loop->motor_chassis[1].torque_set = (chassisLQRGain.lqr_speed_gain * (chassis_wheel_control_loop->lqr_k2 * (chassis_wheel_control_loop->vx_set - (chassis_wheel_control_loop->vx_l - chassis_wheel_control_loop->vx_r)/2.0f)) + \
																															chassisLQRGain.lqr_pitch_gain * chassis_wheel_control_loop->lqr_k3 * (-chassis_wheel_control_loop->chassis_pitch) + \
																															chassisLQRGain.lqr_pitchspeed_gain * chassis_wheel_control_loop->lqr_k4 * (-chassis_wheel_control_loop->chassis_pitch_speed) - \
																															chassisLQRGain.lqr_yaw_gain * chassis_wheel_control_loop->lqr_k25 * chassisLQRData.delta_angle + \
																															chassisLQRGain.lqr_yawspeed_gain * chassis_wheel_control_loop->lqr_k26 * chassis_wheel_control_loop->chassis_yaw_speed);				
	//3508反解电流
	for(uint8_t index = 0; index < 2; index++){
		chassisData.current[index] = (int16_t)(chassis_wheel_control_loop->motor_chassis[index].torque_set / 0.3f * (16384.0f/20.0f));
		chassisData.current[index] = constrainFloat(chassisData.current[index],-16000,16000);
	}
}																							
/**************************************NoopLoop_TOF*************************************/
void NoopLoop_CanRecieve(sliding_blocks_t* tofRecieve,CanRxMsg* CanRevData){
	if(CanRevData->StdId == INFANTRY_SLIDE_LEFT){
		tofRecieve->distance[0] = (uint32_t)(CanRevData->Data[0]<<8|CanRevData->Data[1]<<16|CanRevData->Data[2]<<24)/256;
	}
	else if(CanRevData->StdId == INFANTRY_SLIDE_RIGHT){
		tofRecieve->distance[1] = (uint32_t)(CanRevData->Data[0]<<8|CanRevData->Data[1]<<16|CanRevData->Data[2]<<24)/256;
	}
}
/**************************************NoopLoop_TOF************************************/
void NoopLoop_UsartRecieve(uint8_t * array,uint16_t len){
	if(len == 16){
		if(array[0] == 0x57 && array[1] == 0x00){
			sliding_blocks.distance[0] = (uint32_t)(array[8]<<8|array[9]<<16|array[10]<<24)/256;
		}
	}
}
/***************************************NoopLoop_TOF***********************************/
void NoopLoop_Usart6Recieve(uint8_t * array,uint16_t len){
	if(len == 16){
		if(array[0] == 0x57 && array[1] == 0x00){
			sliding_blocks.distance[1] = (uint32_t)(array[8]<<8|array[9]<<16|array[10]<<24)/256;
		}
	}
}
/***************************************功率速度选择****************************************/
f32_t LevelSpeed(uint16_t level){
	f32_t	speed;
	switch(level){
		case 60:	speed = 0.5f;		break;
		case 80:	speed = 0.6f;		break;
		case 120:	speed = 0.9f;		break;
		case 300:	speed = 10.0f;		break;
		default:	speed = 1.0f;		break;
	}
	return speed;
}
/***************************************滑块反馈*******************************************/
void blocksFeedback(){
	sliding_blocks.encoder[0] = (f32_t)motorHandleClass.Encoder(&commandoMotorConfig[11]) - 4650;//右滑块
	if(sliding_blocks.encoder[0] < 0){
		sliding_blocks.encoder[0] += 8191.0f;
	}
	sliding_blocks.encoder[1] = (f32_t)motorHandleClass.Encoder(&commandoMotorConfig[12]) - 1100;//左滑块
	if(sliding_blocks.encoder[1] < 0){
		sliding_blocks.encoder[1] += 8191.0f;
	}
	sliding_blocks.angleSpeed[0] = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[11]);
	sliding_blocks.angleSpeed[1] = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[12]);
	getSlideMotorAngle();
}
/***************************************LQR参数更新****************************************/
void lqrFeedbackUpdate(){
	chassisLQRData.chassis_pitch = chassisData.pitchAngleFbd * PI/180;
	chassisLQRData.chassis_pitch_speed = (chassisLQRData.chassis_pitch - chassisLQRData.chassis_pitch_last)/chassisData.intervalTime;//试试gyroFbd
	chassisLQRData.chassis_pitch_last = chassisLQRData.chassis_pitch;
	chassisLQRData.chassis_roll = SC_ROLL * PI/180;
	chassisLQRData.chassis_roll_speed = IMU_RATEY;
	chassisLQRData.chassis_yaw_speed = IMU_RATEZ;
	chassisLQRData.vx_l = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[9]) * PI / 30.0f / 19.20f * 0.1f;
	chassisLQRData.vx_r = (f32_t)motorHandleClass.Speed(&commandoMotorConfig[10]) * PI / 30.0f / 19.20f * 0.1f;
	chassisLQRData.vx_min_speed = (chassisLQRData.vx_l - chassisLQRData.vx_r)/2.0f;
	chassisLQRData.vx_max_speed = 8000.0f * PI / 30.0f / 19.20f * 0.1f;
	if(chassisData.roDir){
		chassisLQRData.chassis_yaw = getInstallDirect(YAW_INSTALL_CONFIG, INSTALL_TURN) * getGimbalData()->yawMotorAngle * DEG_TO_RAD;
	}
	else
		chassisLQRData.chassis_yaw = getInstallDirect(YAW_INSTALL_CONFIG, INSTALL_TURN) * (getGimbalData()->yawMotorAngle + 180.0f) * DEG_TO_RAD;
	if(!getinfantryAutoData()->aviodFlag && chassisData.roDir){
		chassisLQRData.chassisYawSet = 0.0f;
	}
	else if(!getinfantryAutoData()->aviodFlag && !chassisData.roDir){
		chassisLQRData.chassisYawSet = 180.0f * DEG_TO_RAD;
	}
}
/***************************************倒地疯车保护*****************************************/
void fallDownProtection(){
	if(ABS(chassisData.pitchAngleFbd) > SAFTY_ANGLE && ABS(chassisLQRData.delta_angle) > (10.0f * DEG_TO_RAD)){
		chassisLQRGain.lqr_pitch_gain = 1.2f;
		chassisLQRGain.lqr_pitchspeed_gain = 0.2f;
		chassisLQRGain.lqr_speed_gain = 1.0f;
		if(getrobotMode() != MODE_RELAX){
			digitalHi(&chassisLQRData.fall);				
		}
	}
	else{
		chassisLQRGain.lqr_pitch_gain = 1.7f;
		chassisLQRGain.lqr_pitchspeed_gain = 0.8f;	
		chassisLQRGain.lqr_speed_gain = 0.5f;	
		chassisLQRGain.lqr_yaw_gain = 2.0f;
		if(getinfantryAutoData()->rotateFlag){
			chassisLQRGain.lqr_speed_gain = 0.3f;
		}
	}

	if(ABS(chassisLQRData.chassis_pitch) > STUCK_ANGLE && ABS(chassisLQRData.vx) < 0.5f){
		digitalHi(&chassisLQRData.stuck);
	}
	else{
		digitalLo(&chassisLQRData.stuck);
	}
}
/****************************************方向选择*******************************************/
void directionChosen(){
	if(chassisData.direction == 0){
		chassisData.roDir = -1;
	}
	else
		chassisData.roDir = 1;
}
/****************************************速度惯量选择****************************************/
void speed_temp_choose(){
	if(getrobotMode() == MODE_KM){
		chassisLQRData.speed_temp = 0.5f;
	}
	else
		chassisLQRData.speed_temp = 0.45f;
	
	if(!chassisLQRData.autoCenterFlag){
		if(chassisLQRData.vx_set < 0 && chassisLQRData.vx > -chassisLQRData.chassis_vx_limit){
			SLID_BLOCK_LEFT_BACKWARD;	
			SLID_BLOCK_RIGHT_BACKWARD;
			chassisLQRData.speed_temp = 0.6f;	
			chassisLQRGain.lqr_speed_gain = 0.5f;				
		}
		else if(chassisLQRData.vx_set > 0 && chassisLQRData.vx < chassisLQRData.chassis_vx_limit){
			SLID_BLOCK_LEFT_FORWARD;	
			SLID_BLOCK_RIGHT_FORWARD;	
			chassisLQRData.speed_temp = 0.6f;
			chassisLQRGain.lqr_speed_gain = 0.5f;
		}
		else{
			SLID_BLOCK_LEFT_MID;	
			SLID_BLOCK_RIGHT_MID;			
		}
	}
}
/***********************************自适应重心******************************************/
u8 flag;
static void autoGravityCenter(){
	if(chassisLQRData.vx_set != 0){
		digitalLo(&chassisLQRData.autoCenterFlag);
	}
	else
		digitalHi(&chassisLQRData.autoCenterFlag);
	
	if(chassisLQRData.autoCenterFlag){		
		//限位
		if((sliding_blocks.angleReal[0] > 350.0f || sliding_blocks.angleReal[0] < 10.0f) && (sliding_blocks.angleReal[1] > 350.0f	|| sliding_blocks.angleReal[1] < 10.0f)){
			if(sliding_blocks.angleOut[0] > 0 && sliding_blocks.angleOut[1] > 0){
				digitalLo(&chassisLQRData.autoCenterFlag);
				if(sliding_blocks.angleReal[0] > 350.0f)	sliding_blocks.position[0] = 350.0f;
				if(sliding_blocks.angleReal[1] > 350.0f)	sliding_blocks.position[1] = 350.0f;
				if(sliding_blocks.angleReal[0] < 10.0f)		sliding_blocks.position[0] = 10.0f;
				if(sliding_blocks.angleReal[1] < 10.0f)		sliding_blocks.position[1] = 10.0f;
				}
			}
		digitalHi(&flag);
	}
	/* 判断角度是否在限位里面，不在就判断超出限位后的角度输出有多大，如果大于0，就用位置环替代输出。*/
	if(chassisLQRData.vx == 0){
		sliding_blocks.angleOut[0] = 0;
		sliding_blocks.angleOut[1] = 0;
	}
	else{
		sliding_blocks.angleOut[0] = pidUpdate(chassisData.pitchAnglePID,0,chassisLQRData.vx,chassisData.intervalTime);
		sliding_blocks.angleOut[1] = -pidUpdate(chassisData.pitchAnglePID,0,chassisLQRData.vx,chassisData.intervalTime);		
	}
	sliding_blocks.rateOut[0] = pidUpdate(chassisData.speedAllPID,sliding_blocks.angleOut[0],sliding_blocks.angleSpeed[0],chassisData.intervalTime);//sliding_blocks.angleOut[0] * slidKp[0];
	sliding_blocks.rateOut[1] = pidUpdate(chassisData.speedAllPID,sliding_blocks.angleOut[1],sliding_blocks.angleSpeed[1],chassisData.intervalTime);	
}
/**************************************************************************************/
static void chassisStopHandle(void){
    //丢控停止模式期望速度全为零
    chassisData.manualSpeedTarget.y = 0;
    chassisData.manualSpeedTarget.x = 0;
    chassisData.manualSpeedTarget.z = 0;
}

//解除控制权模式复位pid
static void chassisRelaxHandle(void){
	pidZeroState(chassisData.chasePID);
	pidZeroState(chassisData.armWheelPID);
	pidZeroState(chassisData.syncWheelPID);
	pidZeroState(chassisData.chaseSpeedPID);
	pidZeroState(chassisData.pitchAnglePID);
	pidZeroState(chassisData.positionPID);
	pidZeroState(chassisData.posPID);
	pidZeroState(chassisData.speedAllPID);
	for(uint8_t index = 0;index < NUMBER_OF_WHEEL;index++){
		pidZeroState(chassisData.speedPID[index]);
		pidZeroState(chassisData.rollAnglePID[index]);
	}
}
//跟随云台模式
static void followGimbalHandle(void){
	if(getinfantryAutoData()->separate_flag){
	  chassisData.manualSpeedTarget.x = (chassisData.sumofChssisTarget.x);
	  chassisData.manualSpeedTarget.y = (chassisData.sumofChssisTarget.y);
	  chassisData.manualSpeedTarget.z = 0;
	}
	else{
	f32_t angleError;
	f32_t sinAngle;
	f32_t cosAngle;
        //底盘跟随反馈赋值	
	chassisData.chaseFbd = getInstallDirect(YAW_INSTALL_CONFIG, INSTALL_TURN)*getGimbalData()->yawMotorAngle;     
	angleError = chassisData.chaseFbd * DEG_TO_RAD;
	sinAngle = sinf(angleError);
	cosAngle = cosf(angleError);
	chassisData.manualSpeedTarget.x = (chassisData.sumofChssisTarget.x) * cosAngle \
							          - (chassisData.sumofChssisTarget.y) * sinAngle;
	chassisData.manualSpeedTarget.y = (chassisData.sumofChssisTarget.x) * sinAngle \
									  + (chassisData.sumofChssisTarget.y) * cosAngle;
	/**/
#ifdef ADD_IMU_RATE_PID
	chassisData.chaseAngleOut = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) *pidUpdate(chassisData.chasePID, chassisData.chaseFbd, \
											chassisData.chaseRef, chassisData.intervalTime);
	/**/
	chassisData.chaseSpeedRef = chassisData.chaseAngleOut;   //底盘角度环输出   赋给   
	/**/
	if(ROBOT == INFANTRY_ID &&getpowerData() -> rotateFast && getinfantryAutoData()->rotateFlag)
		chassisData.chaseSpeedRef *= 1.5f;
	//正常陀螺变速处理
	else if((ROBOT == INFANTRY_ID && getinfantryAutoData()->rotateFlag)||(ROBOT == F_TANK_ID && getTankAutoData()->rotateFlag)){   //小陀螺旋转标志位
		chassisData.chaseSpeedRef *= chassisData.rotate;
	}
	else{
		chassisData.chaseSpeedRef *= 1.0f;
	}
	 if(!getinfantryAutoData()->rotateFlag )
		chassisData.chaseSpeedFbd = -chassis_angular_velocity(chassisData.speedFbd);    //麦轮逆解算
	 else
		 chassisData.chaseSpeedFbd = IMU_RATEZ;    //进入这里表明进入小陀螺状态，为了让小陀螺更快速，因为没有底盘陀螺仪所以会超调，能让小陀螺转得更快
	chassisData.manualSpeedTarget.z = pidUpdate(chassisData.chaseSpeedPID, chassisData.chaseSpeedFbd, \
												chassisData.chaseSpeedRef, chassisData.intervalTime);
	chassisData.manualSpeedTarget.z = chassisData.manualSpeedTarget.z * REAL_MOTOR_SPEED_SCALE;		 									
#else
	chassisData.manualSpeedTarget.z = getInstallDirect(parameter[YAW_INSTALL],INSTALL_TURN) \
									* pidUpdate(chassisData.chasePID, chassisData.chaseFbd, \
												chassisData.chaseRef, chassisData.intervalTime);
#endif
 }
}

//底盘分离云台模式
static void separateGimbalHandle(void){
		chassisData.manualSpeedTarget.x = (chassisData.sumofChssisTarget.x);
		chassisData.manualSpeedTarget.y = (chassisData.sumofChssisTarget.y);
		chassisData.manualSpeedTarget.z = (chassisData.sumofChssisTarget.z);
}


void relax_chassis(void){
	digitalClan(&chassisData.chaseRef);
	digitalClan(&remoteControlData.chassisSpeedTarget.x);
	digitalClan(&remoteControlData.chassisSpeedTarget.y);
	digitalClan(&remoteControlData.chassisSpeedTarget.z);
	digitalClan(&keyBoardCtrlData.chassisSpeedTarget.x);
	digitalClan(&keyBoardCtrlData.chassisSpeedTarget.y);
	digitalClan(&keyBoardCtrlData.chassisSpeedTarget.z);
}


//底盘初始化
void chassisInit(void){							
	chassisData.ctrlMode      = CHASSIS_RELAX;
	chassisData.lastCtrlMode  = CHASSIS_STOP;
	//FirstOrderFilter
	M1502A_MotorInit(1);
	powerInitFirstOrderFilter();					
	initFirstOrderFilter();
	chassisData.posPID = pidInit(&(getConfigData()->chassisPosPID)); 
	
	for(uint8_t index = 0;index < NUMBER_OF_WHEEL;index++){		
    chassisData.speedPID[index] = pidInit(&getConfigData()->chassisSpeedPID);		
		if(robotConfigData.robotDeviceList & DEVICE_CURRENT){
			chassisData.currentPID[index] = pidInit(&getConfigData()->powerLimitPID);  
		}
	}
	
	for(uint8_t index = 0;index < 4;index++){		
    chassisData.jointAnglePID[index] = pidInit(&getConfigData()->jointAnglePID);
		chassisData.rollAnglePID[index] = pidInit(&getConfigData()->chassisRollPID);
	}
	
	chassisData.chasePID = pidInit(&getConfigData()->chassisChasePID);				//
	chassisData.speedAllPID = pidInit(&getConfigData()->chassisAllSpeedPID);	//自适应重心滑块速度环	
	chassisData.chaseSpeedPID = pidInit(&getConfigData()->chassisRatePID);
	chassisData.positionPID = pidInit(&getConfigData()->chassisPositionPID);
	chassisData.pitchAnglePID = pidInit(&getConfigData()->chassisPitchPID);
	chassisData.armWheelPID = pidInit(&getConfigData()->shootFricPID);
	chassisData.syncWheelPID = pidInit(&getConfigData()->shootFricPID);
	for(uint8_t index=0; index<4; index++){
	  chassisData.jointPosInit[index] = getConfigData()->jointPosition[index];
	}
	chassisData.turnswitch = 1.0f;
	chassisData.speedLimit = 1.0f;
	chassisData.roDir = 1.0f;
	chassisData.landingSpeedx = 0.0f;
	chassisData.landingSpeedy = 0.0f;
	chassisData.landingSpeedz = 0.0f;
	chassisData.changeHeadOrder = 0.0f;
	chassisData.changeHeadSchedule = 0.0f;
	chassisData.changeChassisSchedule = 0.0f;
	chassisData.rotate = 1.0f;
	chassisData.positionSchedule = 0;
	chassisData.lastYawAngle = 0.0f;
	chassisData.yawAngleRef = 0;
	chassisData.ctalSchedule = 0;
	chassisData.speedRef[0] = 0;
	chassisData.flashTime = 0.0f;
	powerJudgeData.K1 = 1.0f; //
	powerJudgeData.K2 = 0.03f;
	for(uint8_t index = 0;index < 4;index++){
	  chassisData.jointOut[index] = 0;
		chassisData.jointPitchGain[index] = 0;
		chassisData.jointRollGain[index] = 0;
	}
	//存储校准好的中值
	chassisData.yawCenterSave = getConfigData()->yawCenter; 		
	chassisData.chassis_med_angle = 0;
	chassisLQRData.lqr_k1 = LQR_K1;
	chassisLQRData.lqr_k2 = LQR_K2;
	chassisLQRData.lqr_k3 = LQR_K3;
	chassisLQRData.lqr_k4 = LQR_K4;
	chassisLQRData.lqr_k15 = LQR_K15;
	chassisLQRData.lqr_k16 = LQR_K16;
	chassisLQRData.lqr_k25 = chassisLQRData.lqr_k15;
	chassisLQRData.lqr_k26 = chassisLQRData.lqr_k16;
	jointInit();
	//启动参数
	chassisLQRGain.lqr_pitch_gain = 1.0f;
	chassisLQRGain.lqr_pitchspeed_gain = 1.0f;
	chassisLQRGain.lqr_speed_gain = 1.0f;
	chassisLQRGain.lqr_yaw_gain = 0.3f;
	chassisLQRGain.lqr_yawspeed_gain = 1.0f;
	
	angle_rotate = 100.0f;
	
	sliding_blocks.position[0] = 180;	
	sliding_blocks.position[1] = 180;	
	digitalLo(&chassisLQRData.autoCenterFlag);
	
	slidKp[0] = slidKp[1] = 1500;
} 

