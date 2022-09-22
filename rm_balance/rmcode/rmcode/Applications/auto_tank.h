#ifndef __AUTO_TANK_H
#define __AUTO_TANK_H

#include "auto_task.h"
//��Ӣ��
enum{

	TANK_MANUAL = 0,											//�ֶ�0								
	TANK_MORTAR,												//CTRL:�Ȼ���ģʽ1
	TANK_BULLET_SUPPLY,										    //����վ����2
    TANK_BULLET_TRANSFER,									    //R:�Զ�תͷ42mm��ҩ����3	
	TANK_CHANGE_HIT_MODE,										//Z:��`��ģʽ4
	TANK_AUTOMATIC_AIM,										    //V:������׼5
	TANK_TURN_AROUND											//Q:������6
																//G:7
};
//��Ӣ��
enum{
	F_TANK_MANUAL = 0,											//�ֶ�0							
    F_TANK_BULLET_TRANSFER = 3,								    //R:�Զ�תͷ42mm��ҩ����3	
	F_TANK_AUTOMATIC_AIM = 5,									//V:������׼5
	F_TANK_TURN_AROUND = 6										//Q:������6													
};

typedef struct _AutotankStruct_t{
	void (*p_tank_x_task)		(void);
	void (*p_tank_ctrl_task)	(void);
	void (*p_tank_r_task)		(void);
	void (*p_tank_z_task)		(void);
	void (*p_tank_v_task)		(void);
	void (*p_tank_q_task)		(void);
	void (*p_tank_g_task)		(void);
	void (*p_tank_c_task)		(void);
//	X_TASK = 1,					    //1
//	CTRL_TASK,					    //2
//	R_TASK,							//3
//	Z_TASK,							//4
//	V_TASK,							//5
//	Q_TASK,							//6
//	G_TASK,							//7
//	C_TASK,							//8
}AutotankStruct_t;
typedef struct{
	//Ħ�����Զ�������־
	uint8_t ftankFricNumFlag;
	//Ħ����״̬��־
	uint8_t ftankFricStateFlag;
}ftankfricFlagStruct_t;

extern AutotankStruct_t tankAutoSerial;
ftankfricFlagStruct_t* getftankFricState(void);
AutoTaskStruct_t* getTankAutoData(void);
void f_tankAutoTaskUpdate(void);
void p_tankAutoTaskUpdate(void);
#endif

