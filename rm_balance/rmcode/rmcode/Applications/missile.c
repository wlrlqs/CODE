#include "application.h"
#include "missile.h"

/* 导弹测试代码
导弹分为RC模式、 KM模式

RC模式下可以通过S1来切换调试对象
S1底部：步进电机、直流电机受控
S1中间：直流电机自动上膛
S1上部：摇动右摇杆来选择发射导弹

KM模式下由操作手终端控制
步进电机自动校准
*/

missileDataStruct_t missileData;	//导弹数据结构体

static void DCMotorUpdate(void);
static void stepMotorUpdate(void);
static void controlStateUpdate(void);
static void triggerUpdate(void);

/*********************************************************************/

missileDataStruct_t* getMissileData() {
	return &missileData;
}

void missileUpdate() {
	controlStateUpdate();	//首先更新状态，机构根据状态来更新  /*注意*/ 在非更新状态下将发送默认数据。 如：直流电机保持静止。这要求从控板选择性使用主控数据
	stepMotorUpdate();
	DCMotorUpdate();
	triggerUpdate();
}

static void controlStateUpdate() {
	
	if( getrobotMode() == MODE_RC ) {
		
		if( RC_GEAR == RCSW_BOTTOM ) {
			missileData.missileSend[CONTROL_STATE] = RC_CONTROL_MOVE;	//遥控器控制步进电机与直流电机	
			missileData.missileSend[FIRE_STATE] = NULL;					//开火标志置0
		}
		else if( RC_GEAR == RCSW_MID ) {
			missileData.missileSend[CONTROL_STATE] = RC_CONTROL_AUTOLOAD;	//该状态下由从控板执行自动上膛 
			missileData.missileSend[FIRE_STATE] = NULL;						//开火标志置0
		}
		else if( RC_GEAR == RCSW_TOP ) {
			missileData.missileSend[CONTROL_STATE] = RC_CONTROL_TRIGGER;	
		}
		
	}
	else if( getrobotMode() == MODE_KM ) {
        if (RC_GEAR == RCSW_BOTTOM) {
            missileData.missileSend[CONTROL_STATE] = KM_CONTROL;
        }
        else if (RC_GEAR == RCSW_MID) {
             missileData.missileSend[CONTROL_STATE] = KM_CALI;
        }
        
	}
	else {	//非控制状态，下控
		missileData.missileSend[CONTROL_STATE] = RELX_CONTROL;
	}
	
}

static void stepMotorUpdate() {

	if( missileData.missileSend[CONTROL_STATE] == RC_CONTROL_MOVE ||  missileData.missileSend[CONTROL_STATE] == RC_CONTROL_AUTOLOAD ) {
		
		if( RC_RUDD >= 200 ) {
			missileData.missileSend[STEP_RC_YAW_STATE] = STEP_YAW_CW;
		}
		else if( RC_RUDD <= -200 ) {
			missileData.missileSend[STEP_RC_YAW_STATE] = STEP_YAW_CCW;
		}
		else {
			missileData.missileSend[STEP_RC_YAW_STATE] = STEP_YAW_STOP;
		}
		
		if( RC_PITCH >= 200 ) {
			missileData.missileSend[STEP_RC_PITCH_STATE] = STEP_PITCH_UP;
		}
		else if( RC_PITCH <= -200 ) {
			missileData.missileSend[STEP_RC_PITCH_STATE] = STEP_PITCH_DOWN;
		}
		else {
			missileData.missileSend[STEP_RC_PITCH_STATE] = STEP_PITCH_STOP;
		}
		
	}
	else if( missileData.missileSend[CONTROL_STATE] == KM_CONTROL ) {

		//导弹测试代码，用摇杆代替操作手按键
		if( RC_RUDD >= 200 || (remoteControlData.dt7Value.keyBoard.bit.D) ) { missileData.missileSend[ STEP_KM_TARGET ] = TARGET_BASE; }
		else if ( RC_RUDD <= -200 || (remoteControlData.dt7Value.keyBoard.bit.A) ) { missileData.missileSend[ STEP_KM_TARGET ] = TARGET_OUTPOST; }
		else { missileData.missileSend[ STEP_KM_TARGET ] = TARGET_NULL; }
		
	}
	else {	//在其他状态下，发送给从控板的是默认数据
		missileData.missileSend[STEP_RC_YAW_STATE] = STEP_YAW_STOP;
		missileData.missileSend[STEP_RC_PITCH_STATE] = STEP_PITCH_STOP;
	}
}

static void DCMotorUpdate() {
	
	if( missileData.missileSend[CONTROL_STATE] == RC_CONTROL_MOVE ) {
		if( RC_LONGITUDINAL >= 200 ) {
			missileData.missileSend[DC_RC_STATE] = DC_RC_FORWARD;
		}
		else if( RC_LONGITUDINAL <= -200 ) {
			missileData.missileSend[DC_RC_STATE] = DC_RC_BACK;
		}
		else {
			missileData.missileSend[DC_RC_STATE] = DC_RC_STOP;
		}
	}
	else { //在其他状态下，发送给从控板的是默认数据
		missileData.missileSend[DC_RC_STATE] = DC_RC_STOP;
	}
}

static void triggerUpdate() {
	
	if( missileData.missileSend[CONTROL_STATE] == RC_CONTROL_TRIGGER || missileData.missileSend[CONTROL_STATE] == KM_CONTROL ) {
		if( (RC_TRANSVERSE >= 300 && RC_LONGITUDINAL <= -300 ) || (remoteControlData.dt7Value.keyBoard.bit.Q) ) {		//右下方，对应一号导弹
			missileData.missileSend[FIRE_STATE] |= ( 1 << 0 );
		}
		else { missileData.missileSend[FIRE_STATE] &= ~( 1 << 0 ); }
		
		if( ( RC_TRANSVERSE >= 300 && RC_LONGITUDINAL >= 300 ) || (remoteControlData.dt7Value.keyBoard.bit.W) ) {		//右上方，对应二号导弹
			missileData.missileSend[FIRE_STATE] |= ( 1 << 1 );
		}
		else { missileData.missileSend[FIRE_STATE] &= ~( 1 << 1 ); }
		
		if( ( RC_TRANSVERSE <= -300 && RC_LONGITUDINAL <= -300 ) || (remoteControlData.dt7Value.keyBoard.bit.E) )  {	//左下方，对应三号导弹
			missileData.missileSend[FIRE_STATE] |= ( 1 << 2 );
		}
		else { missileData.missileSend[FIRE_STATE] &= ~( 1 << 2 ); }
		
		if( ( RC_TRANSVERSE <= -300  && RC_LONGITUDINAL >= 300 ) || (remoteControlData.dt7Value.keyBoard.bit.R) )  {	//左上方，对应四号导弹
			missileData.missileSend[FIRE_STATE] |= ( 1 << 3 );
		}
		else { missileData.missileSend[FIRE_STATE] &= ~( 1 << 3 ); }
	}
	else {	//在其他状态下，发送给从控板的是默认数据
		missileData.missileSend[FIRE_STATE] = 0;
	}
}

void missileCanSend(void) {
	uint8_t mbox;                                   
    volatile uint16_t i=0;
	
	CanTxMsg txMessage;
	txMessage.StdId = 0x407;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = 0xAC;/*******校验*********/
	txMessage.Data[1] = 0xAD;/*******校验*********/
	
	txMessage.Data[2] = (uint8_t)missileData.missileSend[0];
	txMessage.Data[3] = (uint8_t)missileData.missileSend[1];
	txMessage.Data[4] = (uint8_t)missileData.missileSend[2];
	txMessage.Data[5] = (uint8_t)missileData.missileSend[3];
	txMessage.Data[6] = (uint8_t)missileData.missileSend[4];
	txMessage.Data[7] = (uint8_t)missileData.missileSend[5];
	
	mbox= CAN_Transmit(CAN1, &txMessage);   
	
	//等待发送结束
    while(CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

