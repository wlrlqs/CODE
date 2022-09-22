#include "custom_broadcast.h"
#include <util.h>
#include "arm_math.h"

/**
 *******************************************************************************
 * @brief      将多字节的数据转换成字节单位						   
 * @param[in]  input    输入的数据地址
 * @param[out] size     每个数据的大小
 * @param[in]  output   输出的数据地址
 * @param[out] ptr      输出的数据下标地址（这个下标会被这个函数所改动）
 * @retval     None    	 
 *
 * @par Description
* @details      这是一个本文件的静态函数
 *******************************************************************************
 */
static void multipleBytesToSingleByte(void *input, size_t size, uint8_t *output, uint16_t *ptr) {
    switch (size) {
        case (sizeof(format64BitStruct_t)) : {
            format64BitStruct_t *data_64bit;
            data_64bit = input;
            for(size_t i = 0; i < sizeof(format64BitStruct_t); i++) {
                output[(*ptr)++] = data_64bit->u8Union[i];
            }     
            break;
        }
        case (sizeof(format32BitStruct_t)) : {
            format32BitStruct_t *data_32bit;
            data_32bit = input;
            for(size_t i = 0; i < sizeof(format32BitStruct_t); i++) {
                output[(*ptr)++] = data_32bit->u8Union[i];
            }      
            break;
        }
        case (sizeof(format16BitStruct_t)) : {
            format16BitStruct_t *data_16bit;
            data_16bit = input;
            for(size_t i = 0; i < sizeof(format16BitStruct_t); i++) {
                output[(*ptr)++] = data_16bit->u8Union[i];
            }  
            break;
        }
    }
}

uint8_t getCustomPacketCount(customFrame_t *frame) {
    return frame->packetCount;
}

void resetCustomFrame(customFrame_t *frame) {
    memset(frame, 0, sizeof(customFrame_t));
}

void appendCustomCell(customFrame_t *frame,
                        void *data, \
                        uint8_t baseIndex, \
                        uint8_t cellSize, \
                        uint8_t cellCount, \
                        uint8_t properties) {
    //packet总数自增
    frame->packetCount ++;
    frame->packetCount = constrainInt(frame->packetCount, 0, CUSTOM_PACKET_MAX_COUNT);
    //获取当前packet的地址
    customPacket_t *packet = &frame->packet[frame->packetCount - 1];
    packet->dataAddress = data;
    packet->baseIndex = baseIndex;
    packet->cellSize = cellSize;
    packet->cellCount = cellCount;
    packet->dataConfig = (0x03 & (int)log2f(cellSize)) | (0x3C & (cellCount << 2)) | (0xC0 & (properties << 6));
}

void addCustomFrameToBroadcast(customFrame_t *frame, uint8_t *array, uint16_t *ptr) {
    array[(*ptr)++] = frame->packetCount;
    for(uint8_t j = 0; j < frame->packetCount; j++) {
        customPacket_t *packet = &frame->packet[j];
        array[(*ptr)++] = packet->baseIndex;
        array[(*ptr)++] = packet->dataConfig;
        for(uint8_t i = 0; i < packet->cellCount; i++) {
            multipleBytesToSingleByte(((uint8_t *)packet->dataAddress + i * packet->cellSize), packet->cellSize, array, ptr);
        }
    }
}

