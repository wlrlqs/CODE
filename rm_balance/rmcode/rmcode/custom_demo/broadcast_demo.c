#include "broadcast_demo.h"
#include "crc.h"
#include "util.h"
#include "Driver_USBVCP.h"
#include "arm_math.h"
#include "gimbal.h"

//只是一个测试发送的demo，将会发送3个正弦波和3个余弦波

customContent_t customContent;

//测试的数组
static float cycleTime = 0.0f;
//static float sineWave[3], cosineWave[3];

//帧头初始化
static void initFrameHeader(uint8_t *array, uint16_t *beginPtr, uint16_t *endPtr, FRAME_TYPE_E frameType, format16BitStruct_t *CNTR) {
    memset(&array[*beginPtr], 0, FRAME_HEADER_LENGTH);
    //帧头地址
    array[*beginPtr + FRAME_ADDRESS_INDEX] = FRAME_BROADCAST_ADDRESS;
    //消息类型
    array[*beginPtr + FRAME_TYPE_INDEX] = frameType;
    //时间戳数据
    array[*beginPtr + FRAME_CNTRL_INDEX] = CNTR->u8Union[0];
    array[*beginPtr + FRAME_CNTRH_INDEX] = CNTR->u8Union[1];
    *endPtr += FRAME_HEADER_LENGTH;
}

//填装帧头及帧尾的CRC
static void fillFrameTailCRC(uint8_t *array, uint16_t *beginPtr, uint16_t *endPtr) {
    //填装总长度信息
    array[*beginPtr + FRAME_LENGTH_INDEX] = (*endPtr - *beginPtr) + 2;
    //填装校验位
    Append_CRC8_Check_Sum(&array[*beginPtr], FRAME_HEADER_LENGTH);
    Append_CRC16_Check_Sum(&array[*beginPtr], array[*beginPtr + FRAME_LENGTH_INDEX]);
    *endPtr += 2;
}

//发送的格式函数
static void formatPrepare(uint8_t *array, uint16_t *ptr) {
    uint16_t beginPtr = *ptr;
    customContent_t *_content = &customContent;
    //初始化帧头
    initFrameHeader(array, &beginPtr, ptr, FRAME_TYPE_SENSOR, &_content->imuCNTR_2byte);  
    //转化packet
    if(getCustomPacketCount(&_content->customFrame) > 0) {
        addCustomFrameToBroadcast(&_content->customFrame, array, ptr);
    }
    //帧尾
    fillFrameTailCRC(array, &beginPtr, ptr);
}

//请在一个周期任务中执行这个函数
void uploadCustomDataLoop(void) {
    cycleTime += 0.001f;
//    sineWave[0] = 100 * sinf((2 * 3.1415926f) / 5.0f *  cycleTime);
//    sineWave[1] = 50 * sinf((2 * 3.1415926f) / 0.5f *  cycleTime);
//    sineWave[2] = 10 * sinf((2 * 3.1415926f) / 0.005f *  cycleTime);
//    
//    cosineWave[0] = 100 * cosf((2 * 3.1415926f) / 5.0f *  cycleTime);
//    cosineWave[1] = 50 * cosf((2 * 3.1415926f) / 0.5f *  cycleTime);
//    cosineWave[2] = 10 * cosf((2 * 3.1415926f) / 0.005f *  cycleTime);
    
    customContent_t *_content = &customContent;
    //重置发送ptr
    _content->sendPtr = 0;
    //每发送一次frame，都要对计数器执行一次自增，这是确保实际显示和发送频率同步的关键
    _content->imuCNTR_2byte.u16Union ++;
    //先清空当前frame
    resetCustomFrame(&_content->customFrame);

    //添加数据到frame，源cosineWave，基础下标为3，浮点大小，数量3，无特性
    appendCustomCell(&_content->customFrame, &getGimbalData()->pitchAngleRef, 0, sizeof(float), 1, NULL);
    appendCustomCell(&_content->customFrame, &getGimbalData()->pitchAngleFbd, 1, sizeof(float), 1, NULL);
    //添加数据到frame，源sineWave，基础下标为4，浮点大小，数量1，无特性
    appendCustomCell(&_content->customFrame, &getGimbalData()->pitchSpeedRef, 2, sizeof(float), 1, NULL);
    appendCustomCell(&_content->customFrame, &getGimbalData()->pitchSpeedFbd, 3, sizeof(float), 1, NULL);

    if(customContent._broadcast != NULL) {
        //发送前的完成格式转换，数据存到数组中
        formatPrepare(_content->_broadcast, &_content->sendPtr);
        //==================使用发送函数进行发送，此处请使用你的发送函数进行发送===============//
        
        usbVCP_SendBuffer(_content->_broadcast, _content->sendPtr);
        
        //==================使用发送函数进行发送，此处请使用你的发送函数进行发送===============//
    }
}

//在使用发送之前请务必初始化内容
void initCustomContent(void) {
    memset(&customContent, 0, sizeof(customContent_t));
    customContent._broadcast = calloc(CUSTOM_MAX_TRANSFER_COUNT, sizeof(uint8_t));
}
