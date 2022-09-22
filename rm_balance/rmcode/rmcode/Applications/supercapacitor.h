#ifndef __SUPERCAPACITOR_H
#define __SUPERCAPACITOR_H
#include "BSP.h"
#include "Util.h"
#include "stdint.h"
#include <stdbool.h>

#define ID_CAPACITOR    0x702	//��������ID
#define CHARGER         0x56   //���
#define DISCHARGER      0x57   //�ŵ�
#define NOTHING         0x58   //����״̬ʲô������

typedef struct 
{
	uint8_t capacitorFlag;
	formatTrans8Struct_t sendData[2];
	uint8_t send1[6];
	uint8_t send2[6];
	uint8_t send3[6];
	uint8_t send4[6];
	uint8_t read1[6];

	formatTrans8Struct_t readData[2];
}capacitor_state_data_t;

typedef struct 
{
    u8 state_ing;                       //�ŵ�
    u8 charging_stop_discharging;       //���
    u8 percentage;                      //���ݵ���
    u8 capacitor_status;                //���ݳ�ŵ�״̬ 
	capacitor_state_data_t capacitor_1;
	capacitor_state_data_t capacitor_2;
	capacitor_state_data_t capacitor_3;
	
	f32_t CapacitorErrorCount;  //��������ͨ�ż���λ
	f32_t lastErrorCount;    //��һ�μ���ֵ
	f32_t intervalNum;   //�������
}capacitorData_t;

typedef struct{

	uint8_t Stop_discharge; //ֹͣ�ŵ�
	uint8_t charging;       //���
	uint8_t discharge;      //�ŵ�
  uint8_t state;
	bool PressFlag;     //������־
}capacitance_Struct_t;

//�������ݵ���
enum{
  twenty_percent  = 0x01,   //�ٷ�֮��ʮ
	forty_percent   = 0x02,   //�ٷ�֮��ʮ
	sixty_percent   = 0x03,   //�ٷ�֮��ʮ
	eighty_percent  = 0x04,   //�ٷ�֮��ʮ
	hundred_percent = 0x05,   //�ٷ�֮��
};

//��������״̬
enum{
  Charging    = 0x55,  //���
	Discharging = 0x44,  //�ŵ�
	Protection  = 0x66,  //����
};

capacitance_Struct_t *get_capacitance(void);//HXB

void capacitor_can1_sentData(uint32_t ID_CAN);
void capacitor_can2_sentData(uint32_t ID_CAN);
void capacitor_readData(CanRxMsg *can_rx_msg);
void super_capacitorTask(void);
void super_capacitorTask_dowm(void);
void super_capacitorTask_updowm(void);
void capacitance_cansend(uint8_t __state);//HXB


extern capacitorData_t	capacitorData;
capacitorData_t *getcapacitorData(void);
#endif

