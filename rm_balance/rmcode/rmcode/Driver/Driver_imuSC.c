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
//将字节转化为浮点型
static float byteConvertFloat(uint8_t *src, int index) {
    formatTrans32Struct_t value;
    for (uint8_t i = 0; i < sizeof(formatTrans32Struct_t); i++) {
        value.u8_temp[i] = src[index + i];
    }
    return value.float_temp;
}
//将字节转化为16整形
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
//获取某个序号的传感器地址
float *getScSensorAdd(uint8_t sequence) {
    demoASSERT(sequence < SENSOR_MAX_AMOUNT);
    return &sensorData[sequence];
}

//获取某个序号的传感器数据
float getScSensorData(uint8_t sequence) {
    demoASSERT(sequence < SENSOR_MAX_AMOUNT);
    return sensorData[sequence];
}
//读取特定位的配置
int8_t imuReadConfigRegister(uint16_t reg) {
    uint8_t group = (uint8_t)((reg & 0xF00) >> 8);
    uint8_t regBit = (uint8_t)(reg & 0xFF);
    uint8_t shift = 0;
    //循环找到最小的有效位，并记录位移次数，但cmd必须大于0
    if(regBit > 0) {
        for(; shift < 8 ; shift ++) {
            //右侧没有移位到1则继续移位
            if(regBit & 0x01) {
                break;
            }
           regBit = regBit >> 1;
        }
    }
    else {
        return -1;
    }
    //将cmd值复位
    regBit = (uint8_t)(reg & 0xFF);
    return (imuBroadcast.config.setting[group] & regBit) >> shift;
}

//写入特定位的配置
int8_t imuWirteConfigRegister(imuConfigStruct_t *config, uint16_t reg, uint8_t data) {
    uint8_t group = (uint8_t)((reg & 0xF00) >> 8);
    uint8_t regBit = (uint8_t)(reg & 0xFF);
    uint8_t shift = 0;
    //循环找到最小的有效位，记录位移次数，但cmd必须大于0
    if(regBit > 0) {
        for(; shift < 8 ; shift ++) {
            //右侧没有移位到1则继续移位
            if(regBit & 0x01) {
                break;
            }
            regBit = regBit >> 1;
        }
    }
    else {
        return -1;
    }
    //将cmd值复位
    regBit = (uint8_t)(reg & 0xFF);
    //将指定位清零并赋值
    uint8_t value = config->setting[group] & (~regBit);
    value |= (data << shift);
    config->setting[group] = value;
    return 0;
}

//从USART获取一次配置信息
void imuUsartGetOnceParameter(void) {
    //申请内存
    uint8_t *array = malloc(6 * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
	array[index_ptr++] = SETTING_LOG;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    //填装长度
	array[2] = index_ptr + 2;
    //填装校验位
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //发送数据
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //释放内存
    free(array);
}

//USART单次写入1个寄存器的配置信息
void imuUsartSetting(uint8_t reg, uint8_t data) {
    //申请内存
    uint8_t *array = malloc(7 * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
	array[index_ptr++] = reg;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    array[index_ptr++] = data;
    //填装长度
	array[2] = index_ptr + 2;
    //填装校验位
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //发送数据
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //释放内存
    free(array);
}

//USART写入全部寄存器的配置信息
void imuUsartAllSetting(uint8_t *data) {
    //申请内存
    uint8_t *array = malloc(14 * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
	array[index_ptr++] = 0xBF;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    for(uint8_t j = 0; j < 8; j++) {
        array[index_ptr++] = data[j];
    }
    //填装长度
	array[2] = index_ptr + 2;
    //填装校验位
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //发送数据
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //释放内存
    free(array);
}

/*
***************************************************
函数名：imuUsartForwardTx
功能：测试imu到CAN的转发
入口参数：	forwardTarget：转发的对象， FORWARD_DATA0：转发到CAN；  FORWARD_DATA0：转发到VCP；
            data：需要转发的内容的数组地址
            length：需要转发的长度，建议单次不超过72Byte，注意剩余带宽
返回值：无
应用范围：外部调用
备注：调用此函数会自动申请和释放内存
***************************************************
*/
void imuUsartForwardTx(uint8_t forwardTarget, uint8_t *data, uint8_t length) {
    //申请内存
    uint8_t *array = malloc((6 + length) * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
	array[index_ptr++] = forwardTarget;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    //给转发的消息赋值
    for(uint8_t i = 0; i < length; i++) {
        array[index_ptr++] = data[i];
    }    
    //填装长度
	array[2] = index_ptr + 2;
    //填装校验位
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //发送数据
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //释放内存
    free(array);
}

//USART特殊命令
void imuUsartSpecialSetting(SPECIAL_COMMAND_E cmd) {
    //申请内存
    uint8_t *array = malloc(6 * sizeof(uint8_t));
    uint8_t index_ptr = 0;
    array[index_ptr++] = CONFIG_COM_ADDRESS;
    array[index_ptr++] = cmd;
    array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    //填装长度
	array[2] = index_ptr + 2;
    //填装校验位
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //发送数据
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
    //释放内存
    free(array);
}


//放在串口中断中的接收函数
void imuUsartIspFunction(uint8_t *array, uint16_t len)  {
    for(uint16_t i = 0 ; i < len; i++){
		imuBroadcast.bufferLoop.buffer[(uint8_t)(imuBroadcast.bufferLoop.tail + i)] = array[i];
    }
    imuBroadcast.bufferLoop.tail += len;
}
/*接收到的SC陀螺仪相关变量赋值
(只含有1.5.9版本固件的SC陀螺仪可以接收到数据)*/
static void imuUsartSensorReceive() {
	//角速度
   getScimuData()->gyro[0] = getScSensorData(GYRO_X);
   getScimuData()->gyro[1] = getScSensorData(GYRO_Y);
   getScimuData()->gyro[2] = getScSensorData(GYRO_Z);
	//加速度
   getScimuData()->accel[0] = getScSensorData(ACCEL_X);
   getScimuData()->accel[1] = getScSensorData(ACCEL_Y);
   getScimuData()->accel[2] = getScSensorData(ACCEL_Z);	
   // 欧拉角
   getScimuData()->euler[0] = getScSensorData(EULER_X);
   getScimuData()->euler[1] = getScSensorData(EULER_Y);
   getScimuData()->euler[2] = getScSensorData(EULER_Z);
   //陀螺仪当前温度
   getScimuData()->temperature = getScSensorData(TEMPERATURE);
	
}
//解析传感器数据，32位单片机双浮点计算缓慢，故在此最大只提供单浮点的报文分析
static void analyseSensorPacket(uint8_t *src, uint8_t length) {
    uint8_t ptr = PACKET_AMOUNT_INDEX;
    //packet内部传感器集合的数目
    const uint8_t amount = src[ptr++]; 
    for (uint8_t j = 0; j < amount; j++) {
        //当前集合的次序
        uint8_t sequence = src[ptr++];
        //单个元素的大小
        uint8_t unitLength = powf(2, (src[ptr] & 0x03));
        //当前集合内个体的数目
		uint8_t aggregationSize = (src[ptr] & 0x3C) >> 2;
        //集合内元素的特性
		uint8_t properties = (src[ptr++] & 0xC0) >> 6;
        //拆分单个个体
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
            //缓存
            sensorData[sequence + i] = value;
        }
    }
	imuUsartSensorReceive();
}
//辨识通过校验的包裹
static void identifyPacket(uint8_t *intput, uint8_t length) {
    switch(intput[HEADER_TYPE_INDEX]) {
        case FRAME_TYPE_SENSOR:
            //解析传感器数据
            analyseSensorPacket(intput, length);
            break;
        case FRAME_TYPE_FORWARD0:   //来自其他模块的自定义转发内容，后续版本的demo会加上，如有特殊需求请联系微信号rime_he
        case FRAME_TYPE_FORWARD1:   //来自其他模块的自定义转发内容，后续版本的demo会加上，如有特殊需求请联系微信号rime_he
        case FRAME_TYPE_LOG:        //陀螺仪的日志内容，后续版本的demo会加上，如有特殊需求请联系微信号rime_he
            break;
        default:
            break;
    }
}
//USART回环数组解析函数
void imuUsartReceive(void) {
    buffefLoopStruct_t *cycle = &imuBroadcast.bufferLoop;
    while((uint8_t)(cycle->tail - cycle->header) > HEADER_SIZE) {
        //如果当前数组头的值不为包头则向前寻找到包头
        while(((uint8_t)(cycle->tail - cycle->header) > HEADER_SIZE) \
            && (cycle->buffer[cycle->header] != BROADCAST_COM_ADDRESS)) {
            digitalIncreasing(&cycle->header);
        }
        if((uint8_t)(cycle->tail - cycle->header) > HEADER_SIZE) {
            //先校验CRC8是否通过，申请临时数组内存（动态申请）
            uint8_t *crc8Check = calloc(HEADER_SIZE, sizeof(uint8_t));
            for (uint8_t i = 0; i < HEADER_SIZE; i++) {
                crc8Check[i] = cycle->buffer[(uint8_t)(cycle->header + i)];
            }
            //CRC8校验，不通过直接则跳过
            if (Verify_CRC8_Check_Sum(crc8Check, HEADER_SIZE)) {
                uint8_t packetLength = cycle->buffer[(uint8_t)(cycle->header + HEADER_LENGTH_INDEX)];
                //剩余长度必须足够取完当前包,不满足则直接结束当前循环
                if ((uint8_t)(cycle->tail - cycle->header) < packetLength) {
                    free(crc8Check);
                    break;
                }
                //存放通过校验的包裹，申请临时数组内存（动态申请）
                uint8_t *accuratePacket = calloc(packetLength, sizeof(uint8_t));
                //拷贝到读取数组中
                for(uint8_t i = 0; i < packetLength; i++) {
                    accuratePacket[i] = cycle->buffer[(uint8_t)(cycle->header + i)];
                }
                //16位CRC校验
                if(Verify_CRC16_Check_Sum(accuratePacket, packetLength)) {
                    //校验通过后加上长度
                    cycle->header += packetLength;
                    //辨识无误的包裹
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
 
//can发送函数
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
	
	//等待发送结束
    while(CAN_TransmitStatus(CANx, mbox) == CAN_TxStatus_Failed) {
        i++;	
		if(i >= 0xFFF)
			return 1;
	}
    return 0;
}

//CAN获取一次配置信息
void imuCanGetOnceParameter(void) {
    uint8_t array[8] = {0};
    array[0] = 0x01;
    canTransferPack(SETTING_ID + 1, array,CAN_SEND_TYPE_DEF);
}

//CAN接收函数
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

//接收报文初始化
void imuBroadcastInit(void) {  
//	memset(&imuBroadcast, 0, sizeof(imuBroadcastStruct_t));
//    imuConfigStruct_t writeConfig;
//    memset(&writeConfig, 0, sizeof(imuConfigStruct_t));
//    //设置波特率一致，注意，此步必不可少
//    imuWirteConfigRegister(&writeConfig, IMU_USART_BAUDRATE_RW, USART_BAUDRATE_921600);
//    //加速度参与姿态融合
//    imuWirteConfigRegister(&writeConfig, IMU_ACCEL_FUSION_RW, ENABLE);
//    //开启串口报文
//    imuWirteConfigRegister(&writeConfig, IMU_UASRT_BROADCAST_RW, ENABLE);
//    //开启CAN报文
//    imuWirteConfigRegister(&writeConfig, IMU_CAN_BROADCAST_RW, ENABLE);
//    //开启角速度报文输出
//    imuWirteConfigRegister(&writeConfig, IMU_GYRO_OUTPUT_RW, ENABLE);
//    //开启加速度报文输出
//    //imuWirteConfigRegister(&writeConfig, IMU_ACCEL_OUTPUT_RW, DISABLE);
//    //开启姿态报文输出
//    imuWirteConfigRegister(&writeConfig, IMU_ATTITUDE_OUTPUT_RW, ENABLE);
//    //角速度报文输出单位为dps
//    imuWirteConfigRegister(&writeConfig, IMU_GYRO_TYPE_RW, GYRO_RPS);
//    //加速度报文输出类型为机体坐标系
//    imuWirteConfigRegister(&writeConfig, IMU_ACCEL_TYPE_RW, ACCEL_BODY);
//    //姿态报文输出类型为欧拉角
//    imuWirteConfigRegister(&writeConfig, IMU_ATTITUDE_TYPE_RW, ATTITUDE_EULER);
//    //角速度32位输出
//    imuWirteConfigRegister(&writeConfig, IMU_GYRO_LENGTH_RW, OUTPUT_16BIT);
//    //加速度32位输出
//    imuWirteConfigRegister(&writeConfig, IMU_ACCEL_LENGTH_RW, OUTPUT_16BIT);
//    //姿态报文32位输出
//    imuWirteConfigRegister(&writeConfig, IMU_ATTITUDE_LENGTH_RW, OUTPUT_16BIT);

//    //CAN发送配置到模块
//    canTransferPack(SETTING_ID, writeConfig.setting,CAN_SEND_TYPE_DEF);

    //USART发送配置到模块
    //imuUsartAllSetting(writeConfig.setting);
}

//获取转发消息的长度
uint8_t getForwardReceiveLength(FORWARD_ENUM group) {
    buffefLoopStruct_t *forwardContent = &imuBroadcast.forwardContent[group];
    return (uint8_t)(forwardContent->tail - forwardContent->header);
}

//获取转发消息内容
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

/*黄锡标添加ADIS陀螺仪*/
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
//读取ADI陀螺仪数据
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
		else{//读取从机imu数据
			slaveImuDataRead(array);
            bufferLoop->header += array[2];
		}
	}
}
#define ADISINS_HEADER_SIZE 4
#define ADISINS_LENGTH_INDEX 2
//adisINS拆解回环数组
void adisInsDecode(void) {
    buffefLoopStruct_t *cycle = &imuBroadcast.bufferLoop;
    while((uint8_t)(cycle->tail - cycle->header) > ADISINS_HEADER_SIZE) {
        //如果当前数组头的值不为包头则向前寻找到包头
        while(((uint8_t)(cycle->tail - cycle->header) > ADISINS_HEADER_SIZE) \
            && (cycle->buffer[cycle->header] != BROADCAST_COM_ADDRESS)) {
            digitalIncreasing(&cycle->header);
        }
        if((uint8_t)(cycle->tail - cycle->header) > ADISINS_HEADER_SIZE) {
            //先校验CRC8是否通过，申请临时数组内存（动态申请）
            uint8_t *crc8Check = calloc(ADISINS_HEADER_SIZE, sizeof(uint8_t));
            for (uint8_t i = 0; i < ADISINS_HEADER_SIZE; i++) {
                crc8Check[i] = cycle->buffer[(uint8_t)(cycle->header + i)];
            }
            //CRC8校验，不通过直接则跳过
            if (Verify_CRC8_Check_Sum(crc8Check, ADISINS_HEADER_SIZE)) {
                uint8_t packetLength = cycle->buffer[(uint8_t)(cycle->header + ADISINS_LENGTH_INDEX)];
                //剩余长度必须足够取完当前包,不满足则直接结束当前循环
                if ((uint8_t)(cycle->tail - cycle->header) < packetLength) {
                    free(crc8Check);
                    break;
                }
                //存放通过校验的包裹，申请临时数组内存（动态申请）
                uint8_t *accuratePacket = calloc(packetLength, sizeof(uint8_t));
                //拷贝到读取数组中
                for(uint8_t i = 0; i < packetLength; i++) {
                    accuratePacket[i] = cycle->buffer[(uint8_t)(cycle->header + i)];
                }
                //16位CRC校验
                if(Verify_CRC16_Check_Sum(accuratePacket, packetLength)) {
                    //校验通过后加上长度
                    cycle->header += packetLength;
                    //辨识无误的包裹
                    
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



