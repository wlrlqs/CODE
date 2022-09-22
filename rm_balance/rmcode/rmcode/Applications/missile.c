#include "application.h"
#include "missile.h"

/* �������Դ���
������ΪRCģʽ�� KMģʽ

RCģʽ�¿���ͨ��S1���л����Զ���
S1�ײ������������ֱ������ܿ�
S1�м䣺ֱ������Զ�����
S1�ϲ���ҡ����ҡ����ѡ���䵼��

KMģʽ���ɲ������ն˿���
��������Զ�У׼
*/

missileDataStruct_t missileData;	//�������ݽṹ��

static void DCMotorUpdate(void);
static void stepMotorUpdate(void);
static void controlStateUpdate(void);
static void triggerUpdate(void);

/*********************************************************************/

missileDataStruct_t* getMissileData() {
	return &missileData;
}

void missileUpdate() {
	controlStateUpdate();	//���ȸ���״̬����������״̬������  /*ע��*/ �ڷǸ���״̬�½�����Ĭ�����ݡ� �磺ֱ��������־�ֹ����Ҫ��ӿذ�ѡ����ʹ����������
	stepMotorUpdate();
	DCMotorUpdate();
	triggerUpdate();
}

static void controlStateUpdate() {
	
	if( getrobotMode() == MODE_RC ) {
		
		if( RC_GEAR == RCSW_BOTTOM ) {
			missileData.missileSend[CONTROL_STATE] = RC_CONTROL_MOVE;	//ң�������Ʋ��������ֱ�����	
			missileData.missileSend[FIRE_STATE] = NULL;					//�����־��0
		}
		else if( RC_GEAR == RCSW_MID ) {
			missileData.missileSend[CONTROL_STATE] = RC_CONTROL_AUTOLOAD;	//��״̬���ɴӿذ�ִ���Զ����� 
			missileData.missileSend[FIRE_STATE] = NULL;						//�����־��0
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
	else {	//�ǿ���״̬���¿�
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

		//�������Դ��룬��ҡ�˴�������ְ���
		if( RC_RUDD >= 200 || (remoteControlData.dt7Value.keyBoard.bit.D) ) { missileData.missileSend[ STEP_KM_TARGET ] = TARGET_BASE; }
		else if ( RC_RUDD <= -200 || (remoteControlData.dt7Value.keyBoard.bit.A) ) { missileData.missileSend[ STEP_KM_TARGET ] = TARGET_OUTPOST; }
		else { missileData.missileSend[ STEP_KM_TARGET ] = TARGET_NULL; }
		
	}
	else {	//������״̬�£����͸��ӿذ����Ĭ������
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
	else { //������״̬�£����͸��ӿذ����Ĭ������
		missileData.missileSend[DC_RC_STATE] = DC_RC_STOP;
	}
}

static void triggerUpdate() {
	
	if( missileData.missileSend[CONTROL_STATE] == RC_CONTROL_TRIGGER || missileData.missileSend[CONTROL_STATE] == KM_CONTROL ) {
		if( (RC_TRANSVERSE >= 300 && RC_LONGITUDINAL <= -300 ) || (remoteControlData.dt7Value.keyBoard.bit.Q) ) {		//���·�����Ӧһ�ŵ���
			missileData.missileSend[FIRE_STATE] |= ( 1 << 0 );
		}
		else { missileData.missileSend[FIRE_STATE] &= ~( 1 << 0 ); }
		
		if( ( RC_TRANSVERSE >= 300 && RC_LONGITUDINAL >= 300 ) || (remoteControlData.dt7Value.keyBoard.bit.W) ) {		//���Ϸ�����Ӧ���ŵ���
			missileData.missileSend[FIRE_STATE] |= ( 1 << 1 );
		}
		else { missileData.missileSend[FIRE_STATE] &= ~( 1 << 1 ); }
		
		if( ( RC_TRANSVERSE <= -300 && RC_LONGITUDINAL <= -300 ) || (remoteControlData.dt7Value.keyBoard.bit.E) )  {	//���·�����Ӧ���ŵ���
			missileData.missileSend[FIRE_STATE] |= ( 1 << 2 );
		}
		else { missileData.missileSend[FIRE_STATE] &= ~( 1 << 2 ); }
		
		if( ( RC_TRANSVERSE <= -300  && RC_LONGITUDINAL >= 300 ) || (remoteControlData.dt7Value.keyBoard.bit.R) )  {	//���Ϸ�����Ӧ�ĺŵ���
			missileData.missileSend[FIRE_STATE] |= ( 1 << 3 );
		}
		else { missileData.missileSend[FIRE_STATE] &= ~( 1 << 3 ); }
	}
	else {	//������״̬�£����͸��ӿذ����Ĭ������
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
	
	txMessage.Data[0] = 0xAC;/*******У��*********/
	txMessage.Data[1] = 0xAD;/*******У��*********/
	
	txMessage.Data[2] = (uint8_t)missileData.missileSend[0];
	txMessage.Data[3] = (uint8_t)missileData.missileSend[1];
	txMessage.Data[4] = (uint8_t)missileData.missileSend[2];
	txMessage.Data[5] = (uint8_t)missileData.missileSend[3];
	txMessage.Data[6] = (uint8_t)missileData.missileSend[4];
	txMessage.Data[7] = (uint8_t)missileData.missileSend[5];
	
	mbox= CAN_Transmit(CAN1, &txMessage);   
	
	//�ȴ����ͽ���
    while(CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

