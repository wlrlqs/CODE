#ifndef __BROADCAST_DEMO_H
#define __BROADCAST_DEMO_H

#include "custom_broadcast.h"
#include "util.h"

//֡����
typedef enum  {
    DATA_TYPE_SENSOR = 0x01,
    DATA_TYPE_FORWARD0,
    DATA_TYPE_FORWARD1,
    DATA_TYPE_LOG,
} DATA_TYPE_E;

//�����±��ĵĵ�ַ
#define FRAME_RECEIVE_ADDRESS   0xAD
#define FRAME_BROADCAST_ADDRESS 0x3F

#define RECEIVE_HEADER_LENGTH 0x04

enum {
    //֡��ַ�����±�
    FRAME_ADDRESS_INDEX = 0x00,
    //֡���������±�
    FRAME_TYPE_INDEX,
    //֡���������±�
    FRAME_LENGTH_INDEX,
    //֡��������±�
    FRAME_CNTRL_INDEX,
    FRAME_CNTRH_INDEX,
    //֡ͷ����
    FRAME_HEADER_LENGTH = 0x06,
    //֡ͷCRC8�����±�
    FRAME_HEADER_CRC8_INDEX = FRAME_HEADER_LENGTH - 0x01,
    
};

//֡βCRC16�����±�
#define FRAME_TAIL_CRC16_INDEX(length)  { length - 0x02 }
//���������ݰ������������±�
//#define PACKET_AMOUNT_INDEX FRAME_HEADER_LENGTH

#define CUSTOM_MAX_TRANSFER_COUNT 256

//�������ݽṹ��
typedef struct {
    //CNTR
    format16BitStruct_t imuCNTR_2byte;
    //�û�frame����
    customFrame_t customFrame;
    //�������鷢�͵��±�
    uint16_t sendPtr;
    //���ĵ�ַ
    uint8_t *_broadcast;
} customContent_t;

/**
 *******************************************************************************
 * @brief      �����û����ݵ���λ����ѭ������						   
 * @param[in]  None
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description     ����һ������������ִ���������������Ƶ����ѭ��Ƶ�ʾ�������λ�����
 *                      �ȶ���ʾ2kHz�����������ϴ󣬽���ʹ��USB VCP���Դ�����У�飩
 * @details     ��demo�У����Ժ���д�ڱ������ڲ�
 *******************************************************************************
 */
void uploadCustomDataLoop(void);
/**
 *******************************************************************************
 * @brief     ��ʼ�����ݽṹ����������������ַ						   
 * @param[in]  None
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description      ��ʹ�÷���֮ǰ������س�ʼ����������
 * @details     
 *******************************************************************************
 */
void initCustomContent(void);

#endif

