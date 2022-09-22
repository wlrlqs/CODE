#ifndef __RMDT7_H
#define __RMDT7_H
#include "bsp.h"
#include "BSP_GPIO.h"
#include "type_robot.h"
#include "driver.h"
#include "BSP_USART.h"
/**********************************SBUS��ʼ������****************************************/
//���ջ����ں�
#define SBUS_USARTX							USART2
//���ջ���������
#define SBUS_USARTX_RX_PIN			        BSP_GPIOA3
//SBUS_USART�ж���ռ���ȼ�
#define SBUS_USART_PRE_PRIORITY 2			
//SBUS_USART�ж���Ӧ���ȼ�
#define SBUS_USART_SUB_PRIORITY 0						    
//���ջ���ȡ����
#define RECEIVER_BUFF Array_USART2_RX

#if UAV_SBUS
//SBUSң����ͨ������
#define CH_NUMBER 9	
#else
//DBUSң����ͨ������
#define CH_NUMBER 5	
#endif

//ҡ�����м��ֵ
#define REMOTE_CONTROLLER_STICK_OFFSET 1024

//����ģʽKM
#define RCSW_TOP 		1
//����ģʽRC
#define RCSW_MID		3
//����ģʽRELAX
#define	RCSW_BOTTOM		2

/*************** ���̰�����ֵ *****************/
#define KEY_W_Press 		0x0001
#define KEY_S_Press 		0x0002
#define KEY_A_Press 		0x0004
#define KEY_D_Press 		0x0008
#define KEY_Shitf_Press     0x0010
#define KEY_Ctrl_Press 		0x0020
#define KEY_Q_Press 		0x0040
#define KEY_E_Press 		0x0080
#define KEY_R_Press 		0x0100
#define KEY_F_Press 		0x0200
#define KEY_G_Press 		0x0400
#define KEY_Z_Press 		0x0800
#define KEY_X_Press 		0x1000
#define KEY_C_Press 		0x2000
#define KEY_V_Press 		0x4000
#define KEY_B_Press 		0x8000
/*************** ң�����ṹ�嶨�� *****************/
#pragma pack(1)
typedef struct
{
	//��ҡ�����ң�364->1024->1684
	int16_t CH0;
	//��ҡ�����£�364->1024->1684	
	int16_t CH1;
	//��ҡ�����ң�364->1024->1684
	int16_t CH2;	
	//��ҡ�����£�364->1024->1684
	int16_t CH3;	
	//���֣�-660->660
	int16_t CH4;	
	//��߿��أ�1���ϣ�2���£�3����
	uint8_t  S1;	
	//�ұ߿��أ�1���ϣ�2���£�3����
	uint8_t  S2;	
}dt7Struct_t;

typedef struct
{
	int16_t CH0;	
	int16_t CH1;	
	int16_t CH2;	
	int16_t CH3;	
	int16_t CH4;	
	int16_t CH5;	
	int16_t CH6;	
	int16_t CH7;	
	int16_t CH8;	
	int16_t CH9;	
	int16_t CH10;	
	int16_t CH11;	
	int16_t CH12;	
	int16_t CH13;	
	int16_t CH14;	
}sbusStruct_t;

typedef struct
{
	//���X�᷽���ٶ� -32768(��)->0->32768(��)
	int16_t X;					
	//���Y�᷽���ٶ� -32768(��)->0->32768(��)
	int16_t Y;					
	//���Z�᷽���ٶ� -32768(��)->0->32768(��)
	int16_t Z;					
	//���X�᷽���ٶ� -32768(��)->0->32768(��)
	int16_t X_Last;			
	//���Y�᷽���ٶ� -32768(��)->0->32768(��)
	int16_t Y_Last;				
	//�������Ƿ��� 0���ް��£�1������
	uint8_t	Press_L;		
	//����Ҽ��Ƿ��� 0���ް��£�1������
	uint8_t	Press_R;		
}mouseStruct_t;

typedef union
{
    uint16_t key_code;
    struct 
    {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    } bit;
}keyBoardStruct_t;
//DUBS_DT7����
typedef struct
{
	//ң��������
	dt7Struct_t rcRawData;
	dt7Struct_t rcRealData;
	//�������
	mouseStruct_t mouse;
	//���̼�ֵ
	keyBoardStruct_t keyBoard;	
}dt7RcSturct_t;

//SUBS_DT7����
typedef struct
{
	sbusStruct_t rcRawData;
	sbusStruct_t rcRealData;
}sbusRcStruct_t;

#pragma pack()
/*************** �ⲿ�������� *****************/
void Driver_RMDT7_Decode_RemoteData(dt7RcSturct_t *RC_Ctrl,uint8_t *Array_Dbus);
void Driver_SBUS_Decode_RemoteData(sbusStruct_t *RC_Ctrl,uint8_t *Array_Dbus);

#endif
