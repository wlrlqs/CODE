#ifndef __MISSILE_H
#define __MISSILE_H

enum {	//����������˶�����
	STEP_YAW_STOP = 0,
	STEP_YAW_CW,
	STEP_YAW_CCW,
	STEP_PITCH_STOP,
	STEP_PITCH_UP,
	STEP_PITCH_DOWN,
};

enum {	//KMģʽ�µĴ��Ŀ��
	TARGET_NULL = 0,
	TARGET_BASE,
	TARGET_OUTPOST,
};

enum {	//��ǰ����״̬
	RELX_CONTROL = 0,			//ң�������� �¿�״̬
	RC_CONTROL_MOVE,			//ң�������� ���������ֱ�����
	RC_CONTROL_TRIGGER,			//ң�������� ��������Խ׶�ʹ�ã�
	RC_CONTROL_AUTOLOAD,		//�Զ�����(�Զ�����)
	KM_CONTROL,					//�������
    KM_CALI,                    //����ģʽ��У׼���غ�ǰ��վλ��
};

enum {	//ֱ������ֶ�����ʱ���͵�ָ��
	DC_RC_STOP = 0,
	DC_RC_FORWARD,
	DC_RC_BACK
};

enum {	//missileData������λ
	CONTROL_STATE = 0,		//��ǰ����״̬						������RC_CONTROL_MOVE
	STEP_RC_YAW_STATE,		//�������YAW��RC״̬�µ��˶�״̬		������STEP_YAW_STOP
	STEP_RC_PITCH_STATE,	//�������PITCH��RC״̬�µ��˶�״̬	������STEP_PITCH_STOP
	DC_RC_STATE,			//ֱ�������RC״̬�µ��˶�״̬			������DC_RC_STOP
	FIRE_STATE,				//��ӦҪ����ĵ���
	STEP_KM_TARGET,			//���������KM״̬�µĴ��Ŀ��			������TARGET_NULL
};

typedef struct{
	uint8_t missileSend[6];	// 0����ǰ����״̬ 1���������RC״̬�µ��˶�״̬ 2��ֱ�������RC״̬�µ��˶�״̬ 3����ӦҪ����ĵ��� 4�����������KM״̬�µĴ��Ŀ��
} missileDataStruct_t;

void missileUpdate(void);
missileDataStruct_t* getMissileData(void);
void missileCanSend(void);
#endif
