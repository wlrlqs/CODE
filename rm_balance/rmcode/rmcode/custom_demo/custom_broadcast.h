#ifndef __CUSTOM_BROADCAST_H
#define __CUSTOM_BROADCAST_H

#include "stm32f4xx.h"
#include <stdlib.h>
#include <util.h>

//�û�������������������Ҫ�������ö��٣�v2.2.5����λ�����֧��32���Զ�������
//ע�⣺����32�飬����ÿ��packet�ڲ���n��ͬ��Ĵ��������ݣ�m��packetռ�� m * n ������
#define CUSTOM_PACKET_MAX_COUNT   20

typedef struct {
    uint8_t baseIndex;  //packet����ʼ���ݵ�index
    uint8_t cellSize;
    uint8_t cellCount;
    uint8_t dataConfig;
    void *dataAddress;  //����Դ
} customPacket_t;

typedef struct {
    uint8_t packetCount; //frame��packet�ĸ���
    customPacket_t packet[CUSTOM_PACKET_MAX_COUNT];
} customFrame_t;


/**
 *******************************************************************************
 * @brief      ��ȡһ��customFrame�ڵ�packet����						   
 * @param[in]  frame    ��Ӧ���û����ݿ�ܵ�ַ
 * @param[out] count    �û����ݿ���ڵ�packet����   
 * @retval     None    	 
 *
 * @par Description
* @details      ȷ��ÿ�η���ǰframe���Ǵ������ݵ�һ��������
 *******************************************************************************
 */
uint8_t getCustomPacketCount(customFrame_t *frame);
/**
 *******************************************************************************
 * @brief      ���һ��customFrame						   
 * @param[in]  frame    ��Ӧ���û����ݿ�ܵ�ַ
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description
* @details      ��ÿ�����ڷ���ǰ����ʹ�øú����Զ�Ӧ�û����ݿ�ܽ����������
 *******************************************************************************
 */
void resetCustomFrame(customFrame_t *frame);
/**
 *******************************************************************************
 * @brief      ���ض��û����ݿ�ܺ�׷��һ�鴫��������						   
 * @param[in]  frame    ��Ӧ���û����ݿ�ܵ�ַ
 * @param[in]  data     ��Ҫװ�ش�������������/�ṹ���׵�ַ
 * @param[in]  baseIndex    ���鴫�������ݵĻ�����ţ���ΧΪ0��31
 * @param[in]  cellSize     ÿ�������������ݴ�С����sizeof()���룬��λΪbyte
 * @param[in]  cellCount    ��װ�ش����������м���ͬ���͵Ĵ���������
 * @param[in]  properties   �����û��ӿ���ʱûʲô�ã�������NULL
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description
 * @details     װ�صĴ�����������ȷ������������һ�µģ�ͬ���͵����ݷ���һ������/���ݽṹ�����У�
 *              ����λ����ʾ�л���baseIndexΪ��׼�����ζ�һ��packet�е����ݽ�������
 *******************************************************************************
 */
void appendCustomCell(customFrame_t *frame, void *data, uint8_t baseIndex, uint8_t cellSize, uint8_t cellCount, uint8_t properties);
/**
 *******************************************************************************
 * @brief      ��һ���û����ݿ��׷�ӵ�Ϊ׼�����͵ı���������						   
 * @param[in]  frame    ��Ҫת�����û����ݿ�ܵ�ַ
 * @param[in]  array    Ŀ�������ַ���׵�ַ��
 * @param[in]  ptr      �����±�ĵ�ַ������±�ᱻ����������Ķ���
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description
 * @details    
 *******************************************************************************
 */
void addCustomFrameToBroadcast(customFrame_t *frame, uint8_t *array, uint16_t *ptr);

#endif
