#ifndef __CUSTOM_BROADCAST_H
#define __CUSTOM_BROADCAST_H

#include "stm32f4xx.h"
#include <stdlib.h>
#include <util.h>

//用户数据组的最大组量，需要多少设置多少，v2.2.5的上位机最大支持32个自定义数据
//注意：不是32组，假设每个packet内部有n个同类的传感器数据，m个packet占用 m * n 个数据
#define CUSTOM_PACKET_MAX_COUNT   20

typedef struct {
    uint8_t baseIndex;  //packet中起始数据的index
    uint8_t cellSize;
    uint8_t cellCount;
    uint8_t dataConfig;
    void *dataAddress;  //数据源
} customPacket_t;

typedef struct {
    uint8_t packetCount; //frame中packet的个数
    customPacket_t packet[CUSTOM_PACKET_MAX_COUNT];
} customFrame_t;


/**
 *******************************************************************************
 * @brief      获取一个customFrame内的packet数量						   
 * @param[in]  frame    对应的用户数据框架地址
 * @param[out] count    用户数据框架内的packet数量   
 * @retval     None    	 
 *
 * @par Description
* @details      确保每次发送前frame中是存在数据的一个服务函数
 *******************************************************************************
 */
uint8_t getCustomPacketCount(customFrame_t *frame);
/**
 *******************************************************************************
 * @brief      清空一个customFrame						   
 * @param[in]  frame    对应的用户数据框架地址
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description
* @details      在每次周期发送前，请使用该函数对对应用户数据框架进行清零操作
 *******************************************************************************
 */
void resetCustomFrame(customFrame_t *frame);
/**
 *******************************************************************************
 * @brief      在特定用户数据框架后追加一组传感器数据						   
 * @param[in]  frame    对应的用户数据框架地址
 * @param[in]  data     所要装载传感器数据数组/结构的首地址
 * @param[in]  baseIndex    本组传感器数据的基础序号，范围为0到31
 * @param[in]  cellSize     每个传感器的数据大小，用sizeof()输入，单位为byte
 * @param[in]  cellCount    所装载传感器数组有几个同类型的传感器数据
 * @param[in]  properties   面向用户接口暂时没什么用，请输入NULL
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description
 * @details     装载的传感器数据请确保数据类型是一致的，同类型的数据放在一个数组/数据结构中排列，
 *              在上位机显示中会以baseIndex为基准，依次对一个packet中的数据进行排序
 *******************************************************************************
 */
void appendCustomCell(customFrame_t *frame, void *data, uint8_t baseIndex, uint8_t cellSize, uint8_t cellCount, uint8_t properties);
/**
 *******************************************************************************
 * @brief      将一个用户数据框架追加到为准备发送的报文数组中						   
 * @param[in]  frame    需要转换的用户数据框架地址
 * @param[in]  array    目标数组地址（首地址）
 * @param[in]  ptr      数组下标的地址（这个下标会被这个函数所改动）
 * @param[out] None   
 * @retval     None    	 
 *
 * @par Description
 * @details    
 *******************************************************************************
 */
void addCustomFrameToBroadcast(customFrame_t *frame, uint8_t *array, uint16_t *ptr);

#endif
