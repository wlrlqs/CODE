#include "Driver_imuSC.h"
#include "cansend.h"
#include "BSP.h"
#include <string.h>
#include <math.h>
#include "crc.h"
#include "slave_sensor.h"
#include "supervisor.h"
#include "imu.h"
imuBroadcastStruct_t imuBroadcast;
float sensorData[SENSOR_MAX_AMOUNT];
imuBroadcastStruct_t *getScimuData(){
	return &imuBroadcast;
}
//���ֽ�ת��Ϊ������
static float byteConvertFloat(uint8_t *src, int index) {
    formatTrans32Struct_t value;
    for (uint8_t i = 0; i < sizeof(formatTrans32Struct_t); i++) {
        value.u8_temp[i] = src[index + i];
    }
    return value.float_temp;
}
//���ֽ�ת��Ϊ16����
static int16_t byteConvertInt16(uint8_t *src, int index) {
    formatTrans16Struct_t value;
    for (uint8_t i = 0; i < sizeof(formatTrans16Struct_t); i++) {
        value.u8_temp[i] = src[index + i];
    }
    return value.s16_temp;
}

//static uint16_t byteConvertUint16(uint8_t *src, int index) {
//	formatTrans16Struct_t value;
//	for (uint8_t i = 0; i < 2; i++)
//		value.u8_temp[i] = src[index + i];
//	return value.u16_temp;
//}

//static void BytesConvertMultipleByte(uint8_t *intput, uint8_t length, void *output, uint8_t *add) {
//    formatTrans32Struct_t *data_32bit;
//    formatTrans16Struct_t *data_16bit;
//    if(length == 4) {
//        data_32bit = output;
//        for(uint8_t i = 0; i < 4; i++) {
//            data_32bit->u8_temp[i] = intput[(*add)++];
//        }
//    }
//    else if(length == 2) {
//        data_16bit = output;
//        for(uint8_t i = 0; i < 2; i++) {
//            data_16bit->u8_temp[i] = intput[(*add)++];
//        }
//    }
//}
//��ȡĳ����ŵĴ�������ַ
float *getScSensorAdd(uint8_t sequence) {
    demoASSERT(sequence < SENSOR_MAX_AMOUNT);
    return &sensorData[sequence];
}

//��ȡĳ����ŵĴ���������
float getScSensorData(uint8_t sequence) {
    demoASSERT(sequence < SENSOR_MAX_AMOUNT);
    return sensorData[sequence];
}
//��ȡ�ض�λ������
int8_t imuReadConfigRegister(uint16_t reg) {
    uint8_t group = (uint8_t)((reg & 0xF00) >> 8);
    uint8_t regBit = (uint8_t)(reg & 0xFF);
    uint8_t shift = 0;
    //ѭ���ҵ���С����Чλ������¼λ�ƴ�������cmd�������0
    if(regBit > 0) {
        for(; shift < 8 ; shift ++) {
            //�Ҳ�û����λ��1�������λ
            if(regBit & 0x01) {
                break;
            }
           regBit = regBit >> 1;
        }
    }
    else {
        return -1;
    }
    //��cmdֵ��λ
    regBit = (uint8_t)(reg & 0xFF);
    return (imuBroadcast.config.setting[group] & regBit) >> shift;
}

//д���ض�λ������
int8_t imuWirteConfigRegister(imuConfigStruct_t *config, uint16_t reg, uint8_t data) {
    uint8_t group = (uint8_t)((reg & 0xF00) >> 8);
    uint8_t regBit = (uint8_t)(reg & 0xFF);
    uint8_t shift = 0;
    //ѭ���ҵ���С����Чλ����¼λ�ƴ�������cmd�������0
    if(regBit > 0) {
        for(; shift < 8 ; shift ++) {
            //�Ҳ�û����λ��1�������λ
            if(regBit & 0x01) {
                break;
            }
            regBit = regBit >> 1;
        }
    }
    else {
        return -1;
    }
    //��cmdֵ��λ
    regBit = (uint8_t)(reg & 0xFF);
    //��ָ��λ���㲢��ֵ
    uint8_t value = config->setting[group] & (~regBit);
    value |= (data << shift);
    config->setting[group] = value;
    return 0;
}

//��USART��ȡһ��������Ϣ
void imuUsartGetOnceParameter(void) {
    //�����ڴ�
    uint8_t *array = malloc(6 * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
	array[index_ptr++] = SETTING_LOG;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    //��װ����
	array[2] = index_ptr + 2;
    //��װУ��λ
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //��������
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //�ͷ��ڴ�
    free(array);
}

//USART����д��1���Ĵ�����������Ϣ
void imuUsartSetting(uint8_t reg, uint8_t data) {
    //�����ڴ�
    uint8_t *array = malloc(7 * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
	array[index_ptr++] = reg;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    array[index_ptr++] = data;
    //��װ����
	array[2] = index_ptr + 2;
    //��װУ��λ
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //��������
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //�ͷ��ڴ�
    free(array);
}

//USARTд��ȫ���Ĵ�����������Ϣ
void imuUsartAllSetting(uint8_t *data) {
    //�����ڴ�
    uint8_t *array = malloc(14 * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
	array[index_ptr++] = 0xBF;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    for(uint8_t j = 0; j < 8; j++) {
        array[index_ptr++] = data[j];
    }
    //��װ����
	array[2] = index_ptr + 2;
    //��װУ��λ
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //��������
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //�ͷ��ڴ�
    free(array);
}

/*
***************************************************
��������imuUsartForwardTx
���ܣ�����imu��CAN��ת��
��ڲ�����	forwardTarget��ת���Ķ��� FORWARD_DATA0��ת����CAN��  FORWARD_DATA0��ת����VCP��
            data����Ҫת�������ݵ������ַ
            length����Ҫת���ĳ��ȣ����鵥�β�����72Byte��ע��ʣ�����
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע�����ô˺������Զ�������ͷ��ڴ�
***************************************************
*/
void imuUsartForwardTx(uint8_t forwardTarget, uint8_t *data, uint8_t length) {
    //�����ڴ�
    uint8_t *array = malloc((6 + length) * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
	array[index_ptr++] = forwardTarget;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    //��ת������Ϣ��ֵ
    for(uint8_t i = 0; i < length; i++) {
        array[index_ptr++] = data[i];
    }    
    //��װ����
	array[2] = index_ptr + 2;
    //��װУ��λ
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //��������
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //�ͷ��ڴ�
    free(array);
}

//USART��������
void imuUsartSpecialSetting(SPECIAL_COMMAND_E cmd) {
    //�����ڴ�
    uint8_t *array = malloc(6 * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
    array[index_ptr++] = cmd;
    array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    //��װ����
	array[2] = index_ptr + 2;
    //��װУ��λ
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //��������
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //�ͷ��ڴ�
    free(array);
}


//���ڴ����ж��еĽ��պ���
void imuUsartIspFunction(uint8_t *array, uint16_t len)  {
    for(uint16_t i = 0 ; i < len; i++){
		imuBroadcast.bufferLoop.buffer[(uint8_t)(imuBroadcast.bufferLoop.tail + i)] = array[i];
    }
    imuBroadcast.bufferLoop.tail += len;
}
/*���յ���SC��������ر�����ֵ
(ֻ����1.5.9�汾�̼���SC�����ǿ��Խ��յ�����)*/
static void imuUsartSensorReceive() {
	//���ٶ�
   getScimuData()->gyro[0] = getScSensorData(GYRO_X);
   getScimuData()->gyro[1] = getScSensorData(GYRO_Y);
   getScimuData()->gyro[2] = getScSensorData(GYRO_Z);
	//���ٶ�
   getScimuData()->accel[0] = getScSensorData(ACCEL_X);
   getScimuData()->accel[1] = getScSensorData(ACCEL_Y);
   getScimuData()->accel[2] = getScSensorData(ACCEL_Z);	
   // ŷ����
   getScimuData()->euler[0] = getScSensorData(EULER_X);
   getScimuData()->euler[1] = getScSensorData(EULER_Y);
   getScimuData()->euler[2] = getScSensorData(EULER_Z);
   //�����ǵ�ǰ�¶�
   getScimuData()->temperature = getScSensorData(TEMPERATURE);
	
}
//�������������ݣ�32λ��Ƭ��˫������㻺�������ڴ����ֻ�ṩ������ı��ķ���
static void analyseSensorPacket(uint8_t *src, uint8_t length) {
    uint8_t ptr = PACKET_AMOUNT_INDEX;
    //packet�ڲ����������ϵ���Ŀ
    const uint8_t amount = src[ptr++]; 
    for (uint8_t j = 0; j < amount; j++) {
        //��ǰ���ϵĴ���
        uint8_t sequence = src[ptr++];
        //����Ԫ�صĴ�С
        uint8_t unitLength = powf(2, (src[ptr] & 0x03));
        //��ǰ�����ڸ������Ŀ
		uint8_t aggregationSize = (src[ptr] & 0x3C) >> 2;
        //������Ԫ�ص�����
		uint8_t properties = (src[ptr++] & 0xC0) >> 6;
        //��ֵ�������
        for (int i = 0; i < aggregationSize; i++, ptr += unitLength) {
            float value;
            switch (unitLength) {
                case sizeof(int16_t) :
                    value = (float)byteConvertInt16(src, ptr);
                    break;
                case sizeof(float) :
                    value = byteConvertFloat(src, ptr);
                    break;
            }
            //����
            sensorData[sequence + i] = value;
        }
    }
	imuUsartSensorReceive();
}
//��ʶͨ��У��İ���
static void identifyPacket(uint8_t *intput, uint8_t length) {
    switch(intput[HEADER_TYPE_INDEX]) {
        case FRAME_TYPE_SENSOR:
            //��������������
            analyseSensorPacket(intput, length);
            break;
        case FRAME_TYPE_FORWARD0:   //��������ģ����Զ���ת�����ݣ������汾��demo����ϣ�����������������ϵ΢�ź�rime_he
        case FRAME_TYPE_FORWARD1:   //��������ģ����Զ���ת�����ݣ������汾��demo����ϣ�����������������ϵ΢�ź�rime_he
        case FRAME_TYPE_LOG:        //�����ǵ���־���ݣ������汾��demo����ϣ�����������������ϵ΢�ź�rime_he
            break;
        default:
            break;
    }
}
//USART�ػ������������
void imuUsartReceive(void) {
    buffefLoopStruct_t *cycle = &imuBroadcast.bufferLoop;
    while((uint8_t)(cycle->tail - cycle->header) > HEADER_SIZE) {
        //�����ǰ����ͷ��ֵ��Ϊ��ͷ����ǰѰ�ҵ���ͷ
        while(((uint8_t)(cycle->tail - cycle->header) > HEADER_SIZE) \
            && (cycle->buffer[cycle->header] != BROADCAST_COM_ADDRESS)) {
            digitalIncreasing(&cycle->header);
        }
        if((uint8_t)(cycle->tail - cycle->header) > HEADER_SIZE) {
            //��У��CRC8�Ƿ�ͨ����������ʱ�����ڴ棨��̬���룩
            uint8_t *crc8Check = calloc(HEADER_SIZE, sizeof(uint8_t));
            for (uint8_t i = 0; i < HEADER_SIZE; i++) {
                crc8Check[i] = cycle->buffer[(uint8_t)(cycle->header + i)];
            }
            //CRC8У�飬��ͨ��ֱ��������
            if (Verify_CRC8_Check_Sum(crc8Check, HEADER_SIZE)) {
                uint8_t packetLength = cycle->buffer[(uint8_t)(cycle->header + HEADER_LENGTH_INDEX)];
                //ʣ�೤�ȱ����㹻ȡ�굱ǰ��,��������ֱ�ӽ�����ǰѭ��
                if ((uint8_t)(cycle->tail - cycle->header) < packetLength) {
                    free(crc8Check);
                    break;
                }
                //���ͨ��У��İ�����������ʱ�����ڴ棨��̬���룩
                uint8_t *accuratePacket = calloc(packetLength, sizeof(uint8_t));
                //��������ȡ������
                for(uint8_t i = 0; i < packetLength; i++) {
                    accuratePacket[i] = cycle->buffer[(uint8_t)(cycle->header + i)];
                }
                //16λCRCУ��
                if(Verify_CRC16_Check_Sum(accuratePacket, packetLength)) {
                    //У��ͨ������ϳ���
                    cycle->header += packetLength;
                    //��ʶ����İ���
                    identifyPacket(accuratePacket, packetLength);
					digitalIncreasing(&imuBroadcast.errorCount);
                }
                else {
                    digitalIncreasing(&cycle->header);
                }
                free(accuratePacket);
            }
            else {
                digitalIncreasing(&cycle->header);
            }
            free(crc8Check);
        }
    }
}
 
//can���ͺ���
uint8_t canTransferPack(uint32_t canID, uint8_t *src,CAN_TypeDef *CANx) {
    uint8_t mbox;
    uint16_t i = 0;     
	CanTxMsg txMessage;
	txMessage.StdId = canID;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
    for(uint8_t i = 0; i < 8; i++) {
        txMessage.Data[i] = (uint8_t)src[i];
    }
    
    mbox = CAN_Transmit(CANx, &txMessage);   
	
	//�ȴ����ͽ���
    while(CAN_TransmitStatus(CANx, mbox) == CAN_TxStatus_Failed) {
        i++;	
		if(i >= 0xFFF)
			return 1;
	}
    return 0;
}

//CAN��ȡһ��������Ϣ
void imuCanGetOnceParameter(void) {
    uint8_t array[8] = {0};
    array[0] = 0x01;
    canTransferPack(SETTING_ID + 1, array,CAN_SEND_TYPE_DEF);
}

//CAN���պ���
void imuCanReceive(CanRxMsg *can_rx_msg) {
    switch(can_rx_msg->StdId) {
        case BRAODCAST_RECEIVE_ID : {
            formatTrans16Struct_t gyro_16bit[3];
            for(uint8_t i = 0; i < 3; i++) {
                for(uint8_t j = 0; j < 2; j++) {
                    gyro_16bit[i].u8_temp[j] = can_rx_msg->Data[i * 2 + j];
                }
                if(imuReadConfigRegister(IMU_GYRO_TYPE_RW)) {
                    imuBroadcast.gyro[i] = (float)gyro_16bit[i].s16_temp * 0.001f;
                }
                else {
                    imuBroadcast.gyro[i] = (float)gyro_16bit[i].s16_temp * 0.1f;
                }
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 1) : {
            formatTrans16Struct_t accel_16bit[3];
            for(uint8_t i = 0; i < 3; i++) {
                for(uint8_t j = 0; j < 2; j++) {
                    accel_16bit[i].u8_temp[j] = can_rx_msg->Data[i * 2 + j];
                }
                imuBroadcast.accel[i] = (float)accel_16bit[i].s16_temp * 0.01f;
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 2) : {
            formatTrans16Struct_t mag_16bit[3];
            for(uint8_t i = 0; i < 3; i++) {
                for(uint8_t j = 0; j < 2; j++) {
                    mag_16bit[i].u8_temp[j] = can_rx_msg->Data[i * 2 + j];
                }
                imuBroadcast.mag[i] = (float)mag_16bit[i].s16_temp * 0.01f;
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 3) : {
            formatTrans32Struct_t pres_32bit;
            for(uint8_t j = 0; j < 4; j++) {
                pres_32bit.u8_temp[j] = can_rx_msg->Data[j];
            }
            imuBroadcast.presure = pres_32bit.float_temp;
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 4) : {
            if(imuReadConfigRegister(IMU_ATTITUDE_TYPE_RW)) {
                formatTrans16Struct_t q_16bit[4];
                for(uint8_t i = 0; i < 4; i++) {
                    for(uint8_t j = 0; j < 2; j++) {
                        q_16bit[i].u8_temp[j] = can_rx_msg->Data[i * 2 + j];
                    }
                    imuBroadcast.q[i] = (float)q_16bit[i].s16_temp * 0.0001f;
                }
            }
            else {
                formatTrans32Struct_t euler_32bit[2];
                for(uint8_t i = 0; i < 2; i++) {
                    for(uint8_t j = 0; j < 4; j++) {
                        euler_32bit[i].u8_temp[j] = can_rx_msg->Data[i * 4 + j];
                    }
                    imuBroadcast.euler[i] = euler_32bit[i].float_temp;
                }
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 5) : {
            formatTrans32Struct_t euler_32bit;
            formatTrans16Struct_t CNTR_16bit;
            if(imuReadConfigRegister(IMU_ATTITUDE_TYPE_RW)) {
                for(uint8_t j = 0; j < 2; j++) {
                    CNTR_16bit.u8_temp[j] = can_rx_msg->Data[j];
                }
                imuBroadcast.CNTR = CNTR_16bit.u16_temp;
            }
            else {
                for(uint8_t j = 0; j < 4; j++) {
                    euler_32bit.u8_temp[j] = can_rx_msg->Data[j];
                }
                imuBroadcast.euler[2] = euler_32bit.float_temp;
                for(uint8_t j = 0; j < 2; j++) {
                    CNTR_16bit.u8_temp[j] = can_rx_msg->Data[4 + j];
                }
                imuBroadcast.CNTR = CNTR_16bit.u16_temp;
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 0x06) : {
            for(uint8_t j = 0; j < 8; j++) {
                imuBroadcast.config.setting[j] = can_rx_msg->Data[j];
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 0x07) : {
            
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 0x08) :
        case (BRAODCAST_RECEIVE_ID + 0x09) : {
            buffefLoopStruct_t *forwardContent = &imuBroadcast.forwardContent[USART_TRANS_CAN];
            forwardContent->buffer[(uint8_t)(forwardContent->tail++)] = can_rx_msg->StdId - BRAODCAST_RECEIVE_ID;
            for(uint8_t i = 0 ; i < 8; i++) {
                forwardContent->buffer[(uint8_t)(forwardContent->tail + i)] = can_rx_msg->Data[i];
            }
            forwardContent->tail += 8;
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 0x0A) :
        case (BRAODCAST_RECEIVE_ID + 0x0B) : {
            buffefLoopStruct_t *forwardContent = &imuBroadcast.forwardContent[VCP_TRANS_CAN];
            forwardContent->buffer[(uint8_t)(forwardContent->tail++)] = can_rx_msg->StdId - BRAODCAST_RECEIVE_ID;
            for(uint8_t i = 0 ; i < 8; i++) {
                forwardContent->buffer[(uint8_t)(forwardContent->tail + i)] = can_rx_msg->Data[i];
            }
            forwardContent->tail += 8;
            break;
        }
    }
	digitalIncreasing(&imuBroadcast.errorCount);
}

//���ձ��ĳ�ʼ��
void imuBroadcastInit(void) {  
//	memset(&imuBroadcast, 0, sizeof(imuBroadcastStruct_t));
//    imuConfigStruct_t writeConfig;
//    memset(&writeConfig, 0, sizeof(imuConfigStruct_t));
//    //���ò�����һ�£�ע�⣬�˲��ز�����
//    imuWirteConfigRegister(&writeConfig, IMU_USART_BAUDRATE_RW, USART_BAUDRATE_921600);
//    //���ٶȲ�����̬�ں�
//    imuWirteConfigRegister(&writeConfig, IMU_ACCEL_FUSION_RW, ENABLE);
//    //�������ڱ���
//    imuWirteConfigRegister(&writeConfig, IMU_UASRT_BROADCAST_RW, ENABLE);
//    //����CAN����
//    imuWirteConfigRegister(&writeConfig, IMU_CAN_BROADCAST_RW, ENABLE);
//    //�������ٶȱ������
//    imuWirteConfigRegister(&writeConfig, IMU_GYRO_OUTPUT_RW, ENABLE);
//    //�������ٶȱ������
//    //imuWirteConfigRegister(&writeConfig, IMU_ACCEL_OUTPUT_RW, DISABLE);
//    //������̬�������
//    imuWirteConfigRegister(&writeConfig, IMU_ATTITUDE_OUTPUT_RW, ENABLE);
//    //���ٶȱ��������λΪdps
//    imuWirteConfigRegister(&writeConfig, IMU_GYRO_TYPE_RW, GYRO_RPS);
//    //���ٶȱ����������Ϊ��������ϵ
//    imuWirteConfigRegister(&writeConfig, IMU_ACCEL_TYPE_RW, ACCEL_BODY);
//    //��̬�����������Ϊŷ����
//    imuWirteConfigRegister(&writeConfig, IMU_ATTITUDE_TYPE_RW, ATTITUDE_EULER);
//    //���ٶ�32λ���
//    imuWirteConfigRegister(&writeConfig, IMU_GYRO_LENGTH_RW, OUTPUT_16BIT);
//    //���ٶ�32λ���
//    imuWirteConfigRegister(&writeConfig, IMU_ACCEL_LENGTH_RW, OUTPUT_16BIT);
//    //��̬����32λ���
//    imuWirteConfigRegister(&writeConfig, IMU_ATTITUDE_LENGTH_RW, OUTPUT_16BIT);

//    //CAN�������õ�ģ��
//    canTransferPack(SETTING_ID, writeConfig.setting,CAN_SEND_TYPE_DEF);

    //USART�������õ�ģ��
    //imuUsartAllSetting(writeConfig.setting);
}

//��ȡת����Ϣ�ĳ���
uint8_t getForwardReceiveLength(FORWARD_ENUM group) {
    buffefLoopStruct_t *forwardContent = &imuBroadcast.forwardContent[group];
    return (uint8_t)(forwardContent->tail - forwardContent->header);
}

//��ȡת����Ϣ����
bool getDataFromForward(uint8_t *dst, FORWARD_ENUM group) {
    buffefLoopStruct_t *forwardContent = &imuBroadcast.forwardContent[group];
    if(getForwardReceiveLength(group) > 0) {
        uint8_t length = getForwardReceiveLength(group);
        for(uint8_t i = 0; i < length; i ++) {
            dst[i] = forwardContent->buffer[forwardContent->header++];
        }
        return true;
    }
    else {
        return false;
    }
}

/*���������ADIS������*/
void modularDataDecoding(uint8_t array){
	supervisorStateSwitch(STATE_IMUCALI, array & 0x04);
}
static void slaveImuDataRead(uint8_t *array){
	uint8_t index_ptr = 4;
	uint8_t index = 0;
	modularDataDecoding(array[index_ptr++]);
	for(uint8_t j = 0; j < 3; j++){
		for(index = 0; index < 2; index++){
			imuData.gyo[j].u8_temp[index] = array[index_ptr++];
		}
	}
	for(uint8_t j = 0; j < 3; j++)
        imuSensorOfChassisData.gyo[j] = imuData.gyo[j].s16_temp;
	
	if(array[1] & TRANS_ADD_ANGLE){
		for(index=0;index < 4;index++)
			imuData.pitch.u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			imuData.roll.u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			imuData.yaw.u8_temp[index] = array[index_ptr++];
		for(index=0;index < 2;index++)
			imuData.CNTR.u8_temp[index] = array[index_ptr++];
        
		imuSensorOfChassisData.mag[0] = imuData.roll.float_temp;
		imuSensorOfChassisData.mag[1] = imuData.pitch.float_temp;
		imuSensorOfChassisData.mag[2] = imuData.yaw.float_temp;
	}	
}
//��ȡADI����������
void slaveSensorRead(void){
	uint8_t *array;
	 buffefLoopStruct_t *bufferLoop = &imuBroadcast.bufferLoop;
	 while(((uint8_t)(bufferLoop->tail - bufferLoop->header) > 4) && (bufferLoop->buffer[bufferLoop->header] != BROADCAST_COM_ADDRESS)) {
        digitalIncreasing(&bufferLoop->header);
    }
    array = &bufferLoop->buffer[bufferLoop->header];		
	if(array[0] == BROADCAST_COM_ADDRESS && (array[1] & MAIN_CONTROL_ADDRESS) ){
		if(!Verify_CRC8_Check_Sum(array, 4) && !Verify_CRC16_Check_Sum(array, array[2])){
		}
		else{//��ȡ�ӻ�imu����
			slaveImuDataRead(array);
            bufferLoop->header += array[2];
		}
	}
}
#define ADISINS_HEADER_SIZE 4
#define ADISINS_LENGTH_INDEX 2
//adisINS���ػ�����
void adisInsDecode(void) {
    buffefLoopStruct_t *cycle = &imuBroadcast.bufferLoop;
    while((uint8_t)(cycle->tail - cycle->header) > ADISINS_HEADER_SIZE) {
        //�����ǰ����ͷ��ֵ��Ϊ��ͷ����ǰѰ�ҵ���ͷ
        while(((uint8_t)(cycle->tail - cycle->header) > ADISINS_HEADER_SIZE) \
            && (cycle->buffer[cycle->header] != BROADCAST_COM_ADDRESS)) {
            digitalIncreasing(&cycle->header);
        }
        if((uint8_t)(cycle->tail - cycle->header) > ADISINS_HEADER_SIZE) {
            //��У��CRC8�Ƿ�ͨ����������ʱ�����ڴ棨��̬���룩
            uint8_t *crc8Check = calloc(ADISINS_HEADER_SIZE, sizeof(uint8_t));
            for (uint8_t i = 0; i < ADISINS_HEADER_SIZE; i++) {
                crc8Check[i] = cycle->buffer[(uint8_t)(cycle->header + i)];
            }
            //CRC8У�飬��ͨ��ֱ��������
            if (Verify_CRC8_Check_Sum(crc8Check, ADISINS_HEADER_SIZE)) {
                uint8_t packetLength = cycle->buffer[(uint8_t)(cycle->header + ADISINS_LENGTH_INDEX)];
                //ʣ�೤�ȱ����㹻ȡ�굱ǰ��,��������ֱ�ӽ�����ǰѭ��
                if ((uint8_t)(cycle->tail - cycle->header) < packetLength) {
                    free(crc8Check);
                    break;
                }
                //���ͨ��У��İ�����������ʱ�����ڴ棨��̬���룩
                uint8_t *accuratePacket = calloc(packetLength, sizeof(uint8_t));
                //��������ȡ������
                for(uint8_t i = 0; i < packetLength; i++) {
                    accuratePacket[i] = cycle->buffer[(uint8_t)(cycle->header + i)];
                }
                //16λCRCУ��
                if(Verify_CRC16_Check_Sum(accuratePacket, packetLength)) {
                    //У��ͨ������ϳ���
                    cycle->header += packetLength;
                    //��ʶ����İ���
                    
                    uint8_t index_ptr = 4;
                    modularDataDecoding(accuratePacket[index_ptr++]);
										for(uint8_t j = 0; j < 3; j++){
											for(uint8_t i = 0; i < 2; i++){
												imuData.gyo[j].u8_temp[i] = accuratePacket[index_ptr++];
											}
											imuData.gyroF32[j] = (float)imuData.gyo[j].s16_temp / 1000;
										}
										for(uint8_t j = 0; j < 3; j++)
											imuSensorOfChassisData.gyo[j] = imuData.gyo[j].s16_temp;
                    if(accuratePacket[1] & TRANS_ADD_ANGLE){
                        for(uint8_t i = 0;i < 2;i++)
                            imuData.pitch_an.u8_temp[i] = accuratePacket[index_ptr++];
                        for(uint8_t i = 0;i < 2;i++)
                            imuData.roll_an.u8_temp[i] = accuratePacket[index_ptr++];
                        for(uint8_t i = 0;i < 4;i++)
                            imuData.yaw.u8_temp[i] = accuratePacket[index_ptr++];
                        for(uint8_t i = 0;i < 2;i++)
                            imuData.CNTR.u8_temp[i] = accuratePacket[index_ptr++];
                    }
                    digitalIncreasing(&imuBroadcast.errorCount);
                }
                else {
                    digitalIncreasing(&cycle->header);
                }
                free(accuratePacket);
            }
            else {
                digitalIncreasing(&cycle->header);
            }
            free(crc8Check);
        }
    }
}



