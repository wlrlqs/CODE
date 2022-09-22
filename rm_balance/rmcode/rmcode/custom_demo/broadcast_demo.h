#ifndef __BROADCAST_DEMO_H
#define __BROADCAST_DEMO_H

#include "custom_broadcast.h"
#include "util.h"

//帧类型
typedef enum  {
    DATA_TYPE_SENSOR = 0x01,
    DATA_TYPE_FORWARD0,
    DATA_TYPE_FORWARD1,
    DATA_TYPE_LOG,
} DATA_TYPE_E;

//串口下报文的地址
#define FRAME_RECEIVE_ADDRESS   0xAD
#define FRAME_BROADCAST_ADDRESS 0x3F

#define RECEIVE_HEADER_LENGTH 0x04

enum {
    //帧地址所在下标
    FRAME_ADDRESS_INDEX = 0x00,
    //帧种类所在下标
    FRAME_TYPE_INDEX,
    //帧长度所在下标
    FRAME_LENGTH_INDEX,
    //帧序号所在下标
    FRAME_CNTRL_INDEX,
    FRAME_CNTRH_INDEX,
    //帧头长度
    FRAME_HEADER_LENGTH = 0x06,
    //帧头CRC8所在下标
    FRAME_HEADER_CRC8_INDEX = FRAME_HEADER_LENGTH - 0x01,
    
};

//帧尾CRC16所在下标
#define FRAME_TAIL_CRC16_INDEX(length)  { length - 0x02 }
//传感器数据包的数量所在下标
//#define PACKET_AMOUNT_INDEX FRAME_HEADER_LENGTH

#define CUSTOM_MAX_TRANSFER_COUNT 256

//报文内容结构体
typedef struct {
    //CNTR
    format16BitStruct_t imuCNTR_2byte;
    //用户frame数据
    customFrame_t customFrame;
    //报文数组发送的下标
    uint16_t sendPtr;
    //报文地址
    uint8_t *_broadcast;
} customContent_t;

/**
 *******************************************************************************
 * @brief      发送用户数据到上位机的循环函数						   
 * @param[in]  None
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description     请在一个周期任务中执行这个函数，发送频率由循环频率决定，上位机最高
 *                      稳定显示2kHz，如数据量较大，建议使用USB VCP（自带丢包校验）
 * @details     在demo中，测试函数写在本函数内部
 *******************************************************************************
 */
void uploadCustomDataLoop(void);
/**
 *******************************************************************************
 * @brief     初始化数据结构，给发送数组分配地址						   
 * @param[in]  None
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description      在使用发送之前，请务必初始化发送内容
 * @details     
 *******************************************************************************
 */
void initCustomContent(void);

#endif

