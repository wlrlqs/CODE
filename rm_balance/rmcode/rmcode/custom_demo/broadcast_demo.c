#include "broadcast_demo.h"
#include "crc.h"
#include "util.h"
#include "Driver_USBVCP.h"
#include "arm_math.h"
#include "gimbal.h"

//ֻ��һ�����Է��͵�demo�����ᷢ��3�����Ҳ���3�����Ҳ�

customContent_t customContent;

//���Ե�����
static float cycleTime = 0.0f;
//static float sineWave[3], cosineWave[3];

//֡ͷ��ʼ��
static void initFrameHeader(uint8_t *array, uint16_t *beginPtr, uint16_t *endPtr, FRAME_TYPE_E frameType, format16BitStruct_t *CNTR) {
    memset(&array[*beginPtr], 0, FRAME_HEADER_LENGTH);
    //֡ͷ��ַ
    array[*beginPtr + FRAME_ADDRESS_INDEX] = FRAME_BROADCAST_ADDRESS;
    //��Ϣ����
    array[*beginPtr + FRAME_TYPE_INDEX] = frameType;
    //ʱ�������
    array[*beginPtr + FRAME_CNTRL_INDEX] = CNTR->u8Union[0];
    array[*beginPtr + FRAME_CNTRH_INDEX] = CNTR->u8Union[1];
    *endPtr += FRAME_HEADER_LENGTH;
}

//��װ֡ͷ��֡β��CRC
static void fillFrameTailCRC(uint8_t *array, uint16_t *beginPtr, uint16_t *endPtr) {
    //��װ�ܳ�����Ϣ
    array[*beginPtr + FRAME_LENGTH_INDEX] = (*endPtr - *beginPtr) + 2;
    //��װУ��λ
    Append_CRC8_Check_Sum(&array[*beginPtr], FRAME_HEADER_LENGTH);
    Append_CRC16_Check_Sum(&array[*beginPtr], array[*beginPtr + FRAME_LENGTH_INDEX]);
    *endPtr += 2;
}

//���͵ĸ�ʽ����
static void formatPrepare(uint8_t *array, uint16_t *ptr) {
    uint16_t beginPtr = *ptr;
    customContent_t *_content = &customContent;
    //��ʼ��֡ͷ
    initFrameHeader(array, &beginPtr, ptr, FRAME_TYPE_SENSOR, &_content->imuCNTR_2byte);  
    //ת��packet
    if(getCustomPacketCount(&_content->customFrame) > 0) {
        addCustomFrameToBroadcast(&_content->customFrame, array, ptr);
    }
    //֡β
    fillFrameTailCRC(array, &beginPtr, ptr);
}

//����һ������������ִ���������
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
    //���÷���ptr
    _content->sendPtr = 0;
    //ÿ����һ��frame����Ҫ�Լ�����ִ��һ������������ȷ��ʵ����ʾ�ͷ���Ƶ��ͬ���Ĺؼ�
    _content->imuCNTR_2byte.u16Union ++;
    //����յ�ǰframe
    resetCustomFrame(&_content->customFrame);

    //������ݵ�frame��ԴcosineWave�������±�Ϊ3�������С������3��������
    appendCustomCell(&_content->customFrame, &getGimbalData()->pitchAngleRef, 0, sizeof(float), 1, NULL);
    appendCustomCell(&_content->customFrame, &getGimbalData()->pitchAngleFbd, 1, sizeof(float), 1, NULL);
    //������ݵ�frame��ԴsineWave�������±�Ϊ4�������С������1��������
    appendCustomCell(&_content->customFrame, &getGimbalData()->pitchSpeedRef, 2, sizeof(float), 1, NULL);
    appendCustomCell(&_content->customFrame, &getGimbalData()->pitchSpeedFbd, 3, sizeof(float), 1, NULL);

    if(customContent._broadcast != NULL) {
        //����ǰ����ɸ�ʽת�������ݴ浽������
        formatPrepare(_content->_broadcast, &_content->sendPtr);
        //==================ʹ�÷��ͺ������з��ͣ��˴���ʹ����ķ��ͺ������з���===============//
        
        usbVCP_SendBuffer(_content->_broadcast, _content->sendPtr);
        
        //==================ʹ�÷��ͺ������з��ͣ��˴���ʹ����ķ��ͺ������з���===============//
    }
}

//��ʹ�÷���֮ǰ����س�ʼ������
void initCustomContent(void) {
    memset(&customContent, 0, sizeof(customContent_t));
    customContent._broadcast = calloc(CUSTOM_MAX_TRANSFER_COUNT, sizeof(uint8_t));
}
