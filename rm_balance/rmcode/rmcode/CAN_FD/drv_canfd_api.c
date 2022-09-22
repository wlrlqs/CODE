#include "drv_canfd_api.h"
#include "drv_canfd_register.h"
#include "util.h"

//MCP2517FD SPI配置
BSP_SPI_TypeDef canfd_control = CAN_FD_SPI_DEFAULT;
/******************************************CAN-FD CONTROL******************************************/
//MCP2517FD复位
void drv_can_fd_reset(void){
	uint8_t spiTransmitBuffer[2];
	spiTransmitBuffer[0] = (uint8_t)(cINSTRUCTION_RESET << 4);
	spiTransmitBuffer[1] = 0x00;
	CAN_FD_CS = 0;
	for(uint8_t index = 0; index < 2; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
}
/******************************************CAN-FD BYTE******************************************/
//MCP2517FD写入1字节
void drv_can_fd_spi_writeByte(uint16_t address,uint8_t data){
	uint8_t spiTransmitBuffer[3];
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	spiTransmitBuffer[2] = data;
	CAN_FD_CS = 0;
	for(uint8_t index = 0; index < 3; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
}

//MCP2517FD读取1字节
uint8_t drv_can_fd_spi_readByte(uint16_t address){
	uint8_t spiTransmitBuffer[3];
	uint8_t spiReceiveBuffer;
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	spiTransmitBuffer[2] = 0x00;//假写数据
	CAN_FD_CS = 0;
	for(uint8_t index = 0; index < 3; index++)
		spiReceiveBuffer = BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
	return spiReceiveBuffer;
}

//MCP2517FD安全写入1字节
void drv_can_fd_spi_writeByte_safe(uint16_t address,uint8_t data){
	uint8_t spiTransmitBuffer[5];
	uint16_t crc_checksum;
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_WRITE_SAFE << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	spiTransmitBuffer[2] = data;
	//生成CRC校验
	crc_checksum = drv_can_fd_crc16_checksum(spiTransmitBuffer,3);
	spiTransmitBuffer[3] = (crc_checksum >> 8) & 0xFF;//CRC16 H 
	spiTransmitBuffer[4] = crc_checksum & 0xFF;//CRC16 L
	
	CAN_FD_CS = 0;
	for(uint8_t index = 0; index < 4; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
}
/******************************************CAN-FD WORD******************************************/
//MCP2517FD写入1个字
void drv_can_fd_spi_writeWord(uint16_t address,uint32_t data){
	uint8_t spiTransmitBuffer[6];
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	//装填
	for(uint8_t index = 0; index < 4; index++)
#ifndef LSB_SENDING
		spiTransmitBuffer[index + 2] = (uint8_t)((data >> (8 * (3 - index))) & 0xFF);
#else
		spiTransmitBuffer[index + 2] = (uint8_t)((data >> (8 * index)) & 0xFF);
#endif
	CAN_FD_CS = 0;
	for(uint8_t index = 0; index < 6; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
}
//MCP2517FD读取1个字
uint32_t drv_can_fd_spi_readWord(uint16_t address){
	uint8_t spiTransmitBuffer[6] = {0};
	uint8_t spiReceiveBuffer[4];
	uint32_t orchidData = 0;
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	CAN_FD_CS = 0;
	for(uint8_t index = 0; index < 6; index++)
		spiReceiveBuffer[index] = BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
	//合并
	for(uint8_t index = 0; index < 4; index++)
#ifndef LSB_SENDING
		orchidData |= (uint32_t)((spiReceiveBuffer[index + 2] & 0xFF) << (8 * (3 - index)));
#else
		orchidData |= (uint32_t)((spiReceiveBuffer[index + 2] & 0xFF) << (8 * index));
#endif
	return orchidData;
}
//MCP2517FD安全写入1个字
void drv_can_fd_spi_writeWord_safe(uint16_t address,uint32_t data){
	uint8_t spiTransmitBuffer[8];
	uint16_t crc_checksum;
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_WRITE_SAFE << 4) + (address >> 8) & 0xF);
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	//装填
	for(uint8_t index = 0; index < 4; index++)
#ifndef LSB_SENDING
		spiTransmitBuffer[index + 2] = (uint8_t)((data >> (8 * (3 - index))) & 0xFF);
#else
		spiTransmitBuffer[index + 2] = (uint8_t)((data >> (8 * index)) & 0xFF);
#endif
	//生成CRC校验
	crc_checksum = drv_can_fd_crc16_checksum(spiTransmitBuffer,6);
	spiTransmitBuffer[6] = (crc_checksum >> 8) & 0xFF;//CRC16 H 
	spiTransmitBuffer[7] = crc_checksum & 0xFF;//CRC16 L
	CAN_FD_CS = 0;
	for(uint8_t index = 0; index < 8; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
}
/******************************************CAN-FD HALF WORD******************************************/
//MCP2517FD写入半个字
void drv_can_fd_spi_writeHalfWord(uint16_t address,uint32_t data){
	uint8_t spiTransmitBuffer[4];
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	//装填
	for(uint8_t index = 0; index < 2; index++)
#ifndef LSB_SENDING
		spiTransmitBuffer[index + 2] = (uint8_t)((data >> (8 * (1 - index))) & 0xFF);
#else
		spiTransmitBuffer[index + 2] = (uint8_t)((data >> (8 * index)) & 0xFF);
#endif
	CAN_FD_CS = 0;
	for(uint8_t index = 0; index < 4; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
}
//MCP2517FD读取半个字
uint16_t drv_can_fd_spi_readHalfWord(uint16_t address){
	uint8_t spiTransmitBuffer[4] = {0};
	uint8_t spiReceiveBuffer[2];
	uint16_t orchidData = 0;
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	CAN_FD_CS = 0;
	for(uint8_t index = 0; index < 4; index++)
		spiReceiveBuffer[index] = BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
	//合并
	for(uint8_t index = 0; index < 2; index++)
#ifndef LSB_SENDING
		orchidData |= (uint32_t)((spiReceiveBuffer[index + 2] & 0xFF) << (8 * (1 - index)));
#else
		orchidData |= (uint32_t)((spiReceiveBuffer[index + 2] & 0xFF) << (8 * index));
#endif
	return orchidData;
}
/******************************************CAN-FD BYTE ARRAY******************************************/
//MCP2517FD写入不定长字节
void drv_can_fd_spi_writeByteArray(uint16_t address,uint8_t *transimit,uint16_t len){
	uint16_t spiTransmitSize = len + 2;
	uint8_t spiTransmitBuffer[spiTransmitSize];
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	//装填
	for(uint16_t index = 0; index < len; index++)
		spiTransmitBuffer[index + 2] = transimit[index];
	CAN_FD_CS = 0;
	for(uint16_t index = 0; index < spiTransmitSize; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
}
//MCP2517FD带CRC校验的写入不定长字节
void drv_can_fd_spi_writeByteArrayCRC(uint16_t address,uint8_t *transimit,uint16_t len,bool fromRAM){
	uint16_t spiTransmitSize = len + 5;
	uint16_t crc_checksum;
	uint8_t spiTransmitBuffer[spiTransmitSize];
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_WRITE_CRC << 4) + (address >> 8) & 0xF);
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	//写入RAM时字节长度需要转换为字长度(byte = 4 * word)
	if(fromRAM) spiTransmitBuffer[2] = len >> 2;
	else spiTransmitBuffer[2] = len;
	//装填数据位
	for(uint16_t index = 0; index < len; index++)
		spiTransmitBuffer[index + 3] = transimit[index];
	//生成CRC16校验位
	crc_checksum = drv_can_fd_crc16_checksum(spiTransmitBuffer,spiTransmitSize - 2);
	spiTransmitBuffer[spiTransmitSize - 2] = (crc_checksum >> 8) & 0xFF;//CRC16 H 
	spiTransmitBuffer[spiTransmitSize - 1] = crc_checksum & 0xFF;//CRC16 L
	CAN_FD_CS = 0;
	for(uint16_t index = 0; index < spiTransmitSize; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
}
//MCP2517FD读取不定长字节
void drv_can_fd_spi_readByteArray(uint16_t address,uint8_t *receive,uint16_t len){
	uint16_t spiTransmitSize = len + 2;
	uint8_t spiTransmitBuffer[spiTransmitSize];
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	//初始化假写数据
	for(uint16_t index = 0; index < len; index++)
		spiTransmitBuffer[index + 2] = 0;
	CAN_FD_CS = 0;
	//先发送命令和地址
	for(uint16_t index = 0; index < 2; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	//读取数据
	for(uint16_t index = 0; index < len; index++)
		receive[index] = BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index + 2]);
	CAN_FD_CS = 1;
}
//MCP2517FD带CRC校验读取不定长字节
void drv_can_fd_spi_readByteArrayCRC(uint16_t address,uint8_t *receive,uint16_t len,bool fromRAM,bool *crc_error){
	uint16_t spiTransmitSize = len + 5;
	uint16_t crc_checksum = 0;
	uint16_t crc_controller = 0;
	uint8_t spiTransmitBuffer[spiTransmitSize];
	uint8_t spiReceiveBuffer[spiTransmitSize];
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_READ_CRC << 4) + (address >> 8) & 0xF);
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	//写入RAM时字节长度需要转换为字长度(byte = 4 * word)
	if(fromRAM) spiTransmitBuffer[2] = len >> 2;
	else spiTransmitBuffer[2] = len;
	//初始化假写数据
	for(uint16_t index = 0; index < len + 2; index++)
		spiTransmitBuffer[index + 3] = 0;
	CAN_FD_CS = 0;
	for(uint16_t index = 0; index < spiTransmitSize; index++)
		spiReceiveBuffer[index] = BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
	
	//获取CRC校验位
	crc_checksum = (uint16_t)(spiReceiveBuffer[spiTransmitSize - 2] << 8) | (uint16_t)(spiReceiveBuffer[spiTransmitSize - 1]);
	//计算CRC
	for(uint8_t index = 0; index < 3; index++)
		spiReceiveBuffer[index] = spiTransmitBuffer[index];
	crc_controller = drv_can_fd_crc16_checksum(spiReceiveBuffer,len + 3);
	//验证CRC正确性
	if(crc_checksum == crc_controller)
		*crc_error = true;//CRC正确
	else
		*crc_error = false;//CRC错误
	
	//校验正确更新数据
	if(crc_error)
		for(uint16_t index = 0; index < len; index++)
			receive[index] = spiReceiveBuffer[index + 3];
}
/******************************************CAN-FD WORD ARRAY******************************************/
//MCP2517FD写入不定长字
void drv_can_fd_spi_writeWordArray(uint16_t address,uint32_t *transimit,uint16_t len){
	uint16_t spiTransmitSize = len * 4 + 2;
	uint16_t Irbis = 2;
	formatTrans32Struct_t shiftingTide;
	uint8_t spiTransmitBuffer[spiTransmitSize];
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	
	//拆解后装填
	for(uint16_t index = 0; index < len; index++){
		shiftingTide.u32_temp = transimit[index];
		for(uint8_t _index = 0; _index < 4; _index++,Irbis++)
			spiTransmitBuffer[Irbis] = shiftingTide.u8_temp[_index];
	}
	CAN_FD_CS = 0;
	for(uint16_t index = 0; index < spiTransmitSize; index++)
		BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
}
//MCP2517FD读取不定长字
void drv_can_fd_spi_readWordArray(uint16_t address,uint32_t *receive,uint16_t len){
	uint16_t spiTransmitSize = len * 4 + 2;
	uint16_t Irbis = 2;
	formatTrans32Struct_t shiftingTide;
	uint8_t spiTransmitBuffer[spiTransmitSize];
	uint8_t spiReceiveBuffer[spiTransmitSize];
	spiTransmitBuffer[0] = (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
	spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
	//初始化假写数据
	for(uint16_t index = Irbis; index < spiTransmitSize; index++)
		spiTransmitBuffer[index] = 0;
	CAN_FD_CS = 0;
	for(uint16_t index = 0; index < spiTransmitSize; index++)
		spiReceiveBuffer[index] = BSP_SPI_ReadWriteByte(&canfd_control,spiTransmitBuffer[index]);
	CAN_FD_CS = 1;
	//拆解
	for(uint16_t index = 0; index < len; index++){
		shiftingTide.u32_temp = 0;
		for(uint8_t _index = 0; _index < 4; _index++,Irbis++)
			shiftingTide.u8_temp[_index] = spiReceiveBuffer[Irbis];
		receive[index] = shiftingTide.u32_temp;
	}
}
/******************************************CAN-FD CONTROL CONFIG******************************************/
//配置MCP2517FD控制寄存器
void drv_can_fd_controlConfig(CAN_CONFIG *configSetting){
	REG_CiCON ciCon;
	//控制寄存器初始化
	ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];
	//开始配置
	ciCon.bitSet.DNetFilterCount 				= configSetting->DNetFilterCount;
	ciCon.bitSet.IsoCrcEnable 					= configSetting->IsoCrcEnable;
    ciCon.bitSet.ProtocolExceptionEventDisable  = configSetting->ProtocolExpectionEventDisable;
    ciCon.bitSet.WakeUpFilterEnable 			= configSetting->WakeUpFilterEnable;
    ciCon.bitSet.WakeUpFilterTime			 	= configSetting->WakeUpFilterTime;
    ciCon.bitSet.BitRateSwitchDisable 			= configSetting->BitRateSwitchDisable;
    ciCon.bitSet.RestrictReTxAttempts 			= configSetting->RestrictReTxAttempts;
    ciCon.bitSet.EsiInGatewayMode 				= configSetting->EsiInGatewayMode;
    ciCon.bitSet.SystemErrorToListenOnly 		= configSetting->SystemErrorToListenOnly;
    ciCon.bitSet.StoreInTEF 					= configSetting->StoreInTEF;
    ciCon.bitSet.TXQEnable 						= configSetting->TXQEnable;
    ciCon.bitSet.TxBandWidthSharing 			= configSetting->TxBandWidthSharing;
	//写入控制寄存器
	drv_can_fd_spi_writeWord(cREGADDR_CiCON,ciCon.word);
}
//获取MCP2517FD控制寄存器初始化配置
void drv_can_fd_controlConfig_objReset(CAN_CONFIG *configSetting){
	REG_CiCON ciCon;
	//控制寄存器初始化
	ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];
	//获取配置
	configSetting->DNetFilterCount 					= ciCon.bitSet.DNetFilterCount;
    configSetting->IsoCrcEnable 					= ciCon.bitSet.IsoCrcEnable;
    configSetting->ProtocolExpectionEventDisable 	= ciCon.bitSet.ProtocolExceptionEventDisable;
    configSetting->WakeUpFilterEnable 				= ciCon.bitSet.WakeUpFilterEnable;
    configSetting->WakeUpFilterTime 				= ciCon.bitSet.WakeUpFilterTime;
    configSetting->BitRateSwitchDisable 			= ciCon.bitSet.BitRateSwitchDisable;
    configSetting->RestrictReTxAttempts 			= ciCon.bitSet.RestrictReTxAttempts;
    configSetting->EsiInGatewayMode 				= ciCon.bitSet.EsiInGatewayMode;
    configSetting->SystemErrorToListenOnly 			= ciCon.bitSet.SystemErrorToListenOnly;
    configSetting->StoreInTEF 						= ciCon.bitSet.StoreInTEF;
    configSetting->TXQEnable 						= ciCon.bitSet.TXQEnable;
    configSetting->TxBandWidthSharing 				= ciCon.bitSet.TxBandWidthSharing;
}
//选择MCP2517FD工作模式
void drv_can_fd_controlConfig_modeSelect(CAN_OPERATION_MODE opMode){
	uint8_t dCon = 0;
	dCon = drv_can_fd_spi_readByte(cREGADDR_CiCON + 3);
	dCon &= ~0x07;
    dCon |= opMode;
	drv_can_fd_spi_writeByte(cREGADDR_CiCON + 3,dCon);
}
//获取MCP2517FD工作模式
CAN_OPERATION_MODE drv_can_fd_controlConfig_modeGet(void){
	uint8_t dCon;
	CAN_OPERATION_MODE mode = CAN_INVALID_MODE;
	dCon = drv_can_fd_spi_readByte(cREGADDR_CiCON + 2);
	dCon = (dCon >> 5) & 0x7;
	
	switch(dCon){
		case CAN_NORMAL_MODE:
			mode = CAN_NORMAL_MODE;
			break;
		case CAN_SLEEP_MODE:
            mode = CAN_SLEEP_MODE;
            break;
        case CAN_INTERNAL_LOOPBACK_MODE:
            mode = CAN_INTERNAL_LOOPBACK_MODE;
            break;
        case CAN_EXTERNAL_LOOPBACK_MODE:
            mode = CAN_EXTERNAL_LOOPBACK_MODE;
            break;
        case CAN_LISTEN_ONLY_MODE:
            mode = CAN_LISTEN_ONLY_MODE;
            break;
        case CAN_CONFIGURATION_MODE:
            mode = CAN_CONFIGURATION_MODE;
            break;
        case CAN_CLASSIC_MODE:
            mode = CAN_CLASSIC_MODE;
            break;
        case CAN_RESTRICTED_MODE:
            mode = CAN_RESTRICTED_MODE;
            break;
        default:
            mode = CAN_INVALID_MODE;
            break; 
	}
	return mode;
}
/******************************************CAN-FD TX CONFIG******************************************/
//配置MCP2517FD发送FIFO
void drv_can_fd_TXfifoConfig(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_CONFIG *configSetting){
	REG_CiFIFOCON ciFifoCon;
	uint16_t address = 0;
	ciFifoCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];
	
    ciFifoCon.txBitSet.TxEnable 	= ENABLE;
    ciFifoCon.txBitSet.FifoSize 	= configSetting->FifoSize;
    ciFifoCon.txBitSet.PayLoadSize 	= configSetting->PayLoadSize;
    ciFifoCon.txBitSet.TxAttempts 	= configSetting->TxAttempts;
    ciFifoCon.txBitSet.TxPriority 	= configSetting->TxPriority;
    ciFifoCon.txBitSet.RTREnable 	= configSetting->RTREnable;
	
	address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
	drv_can_fd_spi_writeWord(address,ciFifoCon.word);
}
//获取MCP2517FD发送FIFO初始化配置
void drv_can_fd_TXfifoConfig_objReset(CAN_TX_FIFO_CONFIG *configSetting){
	REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];

    configSetting->RTREnable 	= ciFifoCon.txBitSet.RTREnable;
    configSetting->TxPriority 	= ciFifoCon.txBitSet.TxPriority;
    configSetting->TxAttempts 	= ciFifoCon.txBitSet.TxAttempts;
    configSetting->FifoSize 	= ciFifoCon.txBitSet.FifoSize;
    configSetting->PayLoadSize 	= ciFifoCon.txBitSet.PayLoadSize;
}
//配置MCP2517FD发送队列
void drv_can_fd_TXqueueConfig(CAN_TX_QUEUE_CONFIG *configSetting){
#ifndef CAN_TXQUEUE_IMPLEMENTED
	configSetting;
	return;
#else
	uint16_t address = 0;
	
    REG_CiTXQCON ciFifoCon;
    ciFifoCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];

    ciFifoCon.txBitSet.TxEnable 	= ENABLE;
    ciFifoCon.txBitSet.FifoSize 	= configSetting->FifoSize;
    ciFifoCon.txBitSet.PayLoadSize 	= configSetting->PayLoadSize;
    ciFifoCon.txBitSet.TxAttempts 	= configSetting->TxAttempts;
    ciFifoCon.txBitSet.TxPriority 	= configSetting->TxPriority;

    address = cREGADDR_CiTXQCON;
	drv_can_fd_spi_writeWord(address,ciFifoCon.word);
#endif
}
//获取MCP2517FD发送队列初始化配置
void drv_can_fd_TXqueueConfig_objReset(CAN_TX_QUEUE_CONFIG *configSetting){
	REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];

    configSetting->TxPriority 	= ciFifoCon.txBitSet.TxPriority;
    configSetting->TxAttempts 	= ciFifoCon.txBitSet.TxAttempts;
    configSetting->FifoSize 	= ciFifoCon.txBitSet.FifoSize;
    configSetting->PayLoadSize 	= ciFifoCon.txBitSet.PayLoadSize;
}
//MCP2517FD发送通道装填数据帧
void drv_can_fd_txChannelLoad(CAN_FIFO_CHANNEL channel,CAN_TX_MSGOBJ *txObj,uint8_t *txData,uint32_t txLen,bool flush){
	uint16_t address;
    uint32_t fifoReg[3];
    uint32_t dataBytesInObject;
    REG_CiFIFOCON ciFifoCon;
//    REG_CiFIFOSTA ciFifoSta;
    REG_CiFIFOUA ciFifoUa;
	
	address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
	drv_can_fd_spi_readWordArray(address,fifoReg,3);
	ciFifoCon.word = fifoReg[0];
	//确定是发送缓存区
	if(!ciFifoCon.txBitSet.TxEnable) return;
	//检查配置的DLC是否满足发送长度
	dataBytesInObject = drv_can_fd_dlcToDataBytes((CAN_DLC)txObj->bitSet.ctrl.DLC);
	if(dataBytesInObject < txLen) return;
	//获取FIFO状态
//	ciFifoSta.word = fifoReg[1];
	//获取FIFO地址
	ciFifoUa.word = fifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
    address = 4 * ciFifoUa.bF.UserAddress;
#else
    address = ciFifoUa.bitSet.UserAddress;
#endif
	address += cRAMADDR_START;
	 
	 //装填发送缓存区
	uint8_t txBuffer[MAX_MSG_SIZE];
	//装填信息帧
	txBuffer[0] = txObj->byte[0];
    txBuffer[1] = txObj->byte[1];
    txBuffer[2] = txObj->byte[2];
    txBuffer[3] = txObj->byte[3];

    txBuffer[4] = txObj->byte[4];
    txBuffer[5] = txObj->byte[5];
    txBuffer[6] = txObj->byte[6];
    txBuffer[7] = txObj->byte[7];
	
	//装填数据帧
	uint8_t index;
	for(index = 0; index < txLen; index++)
		txBuffer[index + 8] = txData[index];
	//写入RAM
	uint16_t Irbis = 0;
	uint8_t grim = 0;
	if(txLen % 4){
		Irbis = 4 - (txLen % 4);
		index = txLen + 8;
		for(grim = 0; grim < Irbis; grim++)
			txBuffer[index + 8 + grim] = 0;
	}
	
	drv_can_fd_spi_writeByteArray(address,txBuffer,txLen + 8 + Irbis);
	
	drv_can_fd_txChannelUpdate(channel,flush);
}
//MCP2517FD发送通道更新
void drv_can_fd_txChannelUpdate(CAN_FIFO_CHANNEL channel,bool flush){
	uint16_t address;
	REG_CiFIFOCON ciFifoCon;
	address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1;
    ciFifoCon.word = 0;
    ciFifoCon.txBitSet.UINC = 1;

    // Set TXREQ
    if(flush) ciFifoCon.txBitSet.TxRequest = 1;
	drv_can_fd_spi_writeByte(address,ciFifoCon.byte[1]);
}
//打开MCP2517FD发送FIFO请求
void drv_can_fd_txChannelFlush(CAN_FIFO_CHANNEL channel){
	uint8_t dCon = 0;
	uint16_t address = 0;
	
	address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
	address += 1;
	
	dCon = 0x02;
	drv_can_fd_spi_writeByte(address,dCon);
}
//获取MCP2517FD的发送FIFO状态
void drv_can_fd_txChannelStatusGet(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_STATUS *status){
	uint16_t address = 0;
    uint32_t state = 0;
    uint32_t fifoReg[2];
    REG_CiFIFOSTA ciFifoSta;
    REG_CiFIFOCON ciFifoCon;
	
	address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
	drv_can_fd_spi_readWordArray(address,fifoReg,2);
	
	ciFifoCon.word = fifoReg[0];
	ciFifoSta.word = fifoReg[1];
	
	state = ciFifoSta.byte[0];
	if(ciFifoCon.txBitSet.TxRequest)
		state |= CAN_TX_FIFO_TRANSMITTING;
	*status = (CAN_TX_FIFO_STATUS)(state & CAN_TX_FIFO_STATUS_MASK);
}
//MCP2517FD发送通道重置
void drv_can_fd_txChannelReset(CAN_FIFO_CHANNEL channel){
	drv_can_fd_rxChannelReset(channel);
}
//设置MCP2517FD发送请求
void drv_can_fd_txReqSet(uint32_t txreq){
	drv_can_fd_spi_writeWord(cREGADDR_CiTXREQ,txreq);
}
//获取当前MCP2517FD发送请求
void drv_can_fd_txReqGet(uint32_t *txreq){
	*txreq = drv_can_fd_spi_readWord(cREGADDR_CiTXREQ);
}
//中止MCP2517FD发送通道
void drv_can_fd_txChannelAbort(CAN_FIFO_CHANNEL channel){
	uint16_t address;
    uint8_t dCon;
	
	address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    address += 1;
	dCon = 0x00;
	
	drv_can_fd_spi_writeByte(address,dCon);
}
//中止所有MCP2517FD发送通道
void drv_can_fd_txChannelAbortAll(CAN_FIFO_CHANNEL channel){
	uint8_t dCon;
	dCon = drv_can_fd_spi_readByte(cREGADDR_CiCON + 3);
	dCon |= 0x8;
	drv_can_fd_spi_writeByte((cREGADDR_CiCON + 3),dCon);
}
//配置MCP2517FD带宽
void drv_can_fd_txBandWidthSharingSet(CAN_TX_BANDWITH_SHARING txbws){
	uint8_t dCon = 0;
	dCon = drv_can_fd_spi_readByte(cREGADDR_CiCON + 3);
	dCon &= 0x0f;
	dCon |= (txbws << 4);
	drv_can_fd_spi_writeByte((cREGADDR_CiCON + 3),dCon);
}
/******************************************CAN-FD RX CONFIG******************************************/
//配置MCP2517FD ID报文过滤器
void drv_can_fd_filterObjConfig(CAN_FILTER filter,CAN_FILTEROBJ_ID *id){
	uint16_t address;
    REG_CiFLTOBJ fObj;
	fObj.word = 0;
    fObj.bitSet = *id;
    address = cREGADDR_CiFLTOBJ + (filter * CiFILTER_OFFSET);
	drv_can_fd_spi_writeWord(address,fObj.word);
}
//配置MCP2517FD 掩码ID过滤器
void drv_can_fd_filterMaskConfig(CAN_FILTER filter,CAN_MASKOBJ_ID *mask){
	uint16_t address = 0;
    REG_CiMASK mObj;
	mObj.word = 0;
    mObj.bitSet = *mask;
    address = cREGADDR_CiMASK + (filter * CiFILTER_OFFSET);
    drv_can_fd_spi_writeWord(address,mObj.word);
}
//MCP2517FD过滤器指向FIFO
void drv_can_fd_filter_fifoLink(CAN_FILTER filter,CAN_FIFO_CHANNEL channel,bool enable){
	uint16_t address;
    REG_CiFLTCON_BYTE fCtrl;
	if(enable) fCtrl.bitSet.Enable = ENABLE;
	else fCtrl.bitSet.Enable = DISABLE;
	fCtrl.bitSet.BufferPointer = channel;
	address = cREGADDR_CiFLTCON + filter;
	drv_can_fd_spi_writeByte(address,fCtrl.byte);
}
//开启MCP2517FD报文过滤器
void drv_can_fd_filterEnable(CAN_FILTER filter){
	uint16_t address;
    REG_CiFLTCON_BYTE fCtrl;
	address = cREGADDR_CiFLTCON + filter;
	fCtrl.byte = drv_can_fd_spi_readByte(address);
	fCtrl.bitSet.Enable = ENABLE;
	drv_can_fd_spi_writeByte(address,fCtrl.byte);
}
//关闭MCP2517FD报文过滤器
void drv_can_fd_filterDisable(CAN_FILTER filter){
	uint16_t address;
    REG_CiFLTCON_BYTE fCtrl;
	address = cREGADDR_CiFLTCON + filter;
	fCtrl.byte = drv_can_fd_spi_readByte(address);
	fCtrl.bitSet.Enable = DISABLE;
	drv_can_fd_spi_writeByte(address,fCtrl.byte);
}
//配置MCP2517FD设备过滤器
void drv_can_fd_DeviceNetFilterCountSet(CAN_DNET_FILTER_SIZE dnfc){
	uint8_t dCon = 0;
	dCon = drv_can_fd_spi_readByte(cREGADDR_CiCON);
	dCon &= 0x1f;
    dCon |= dnfc;
	drv_can_fd_spi_writeByte(cREGADDR_CiCON,dCon);
}
//配置MCP2517FD接收通道
void drv_can_fd_rxChannelConfig(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_CONFIG *configSetting){
	uint16_t address = 0;
#ifdef CAN_TXQUEUE_IMPLEMENTED
    if(channel == CAN_TXQUEUE_CH0) return;
#endif
	REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];

    ciFifoCon.rxBitSet.TxEnable 		 = DISABLE;
    ciFifoCon.rxBitSet.FifoSize 		 = configSetting->FifoSize;
    ciFifoCon.rxBitSet.PayLoadSize 		 = configSetting->PayLoadSize;
    ciFifoCon.rxBitSet.RxTimeStampEnable = configSetting->RxTimeStampEnable;

    address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
	drv_can_fd_spi_writeWord(address,ciFifoCon.word);
}
//获取MCP2517FD接收通道初始化配置
void drv_can_fd_rxChannelConfig_objReset(CAN_RX_FIFO_CONFIG *configSetting){
	REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];

    configSetting->FifoSize 		 = ciFifoCon.rxBitSet.FifoSize;
    configSetting->PayLoadSize 		 = ciFifoCon.rxBitSet.PayLoadSize;
    configSetting->RxTimeStampEnable = ciFifoCon.rxBitSet.RxTimeStampEnable;
}
//获取当前MCP2517FD接收通道状态
void drv_can_fd_rxChannelStatusGet(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_STATUS *status){
	uint16_t address;
    REG_CiFIFOSTA ciFifoSta;
	ciFifoSta.word = 0;
    address = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
	ciFifoSta.byte[0] = drv_can_fd_spi_readByte(address);
	*status = (CAN_RX_FIFO_STATUS)(ciFifoSta.byte[0] & 0x0F);
}
//MCP2517FD接收消息
void drv_can_fd_rxMsg(CAN_FIFO_CHANNEL channel,CAN_RX_MSGOBJ *rxObj,uint8_t *rxData,uint8_t rxLen){
	uint8_t Irbis = 0;
	uint8_t index = 0;
	uint16_t address;
    uint32_t fifoReg[3];
    REG_CiFIFOCON ciFifoCon;
//    REG_CiFIFOSTA ciFifoSta;
    REG_CiFIFOUA ciFifoUa;
	
	address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
	drv_can_fd_spi_readWordArray(address,fifoReg,3);
	
	ciFifoCon.word = fifoReg[0];
	//确定是接收缓存区
	if(ciFifoCon.txBitSet.TxEnable) return;
//	ciFifoSta.word = fifoReg[1];
	ciFifoUa.word = fifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
    address = 4 * ciFifoUa.bF.UserAddress;
#else
    address = ciFifoUa.bitSet.UserAddress;
#endif
    address += cRAMADDR_START;
	//8个信息帧
	Irbis = rxLen + 8;
	//如果打开了时间戳,加入4个字节的时间戳位
	if(ciFifoCon.rxBitSet.RxTimeStampEnable) Irbis += 4;
	//写入RAM需要确定是以字方式写入
	if(Irbis % 4) Irbis = Irbis + 4 - (Irbis % 4);
	
	uint8_t rxBuffer[MAX_MSG_SIZE];
	if(Irbis > MAX_MSG_SIZE) Irbis = MAX_MSG_SIZE;
	
	drv_can_fd_spi_readByteArray(address,rxBuffer,Irbis);
	formatTrans32Struct_t myReg;
	//信息帧
	myReg.u8_temp[0] = rxBuffer[0];
    myReg.u8_temp[1] = rxBuffer[1];
    myReg.u8_temp[2] = rxBuffer[2];
    myReg.u8_temp[3] = rxBuffer[3];
    rxObj->word[0] = myReg.u32_temp;

    myReg.u8_temp[0] = rxBuffer[4];
    myReg.u8_temp[1] = rxBuffer[5];
    myReg.u8_temp[2] = rxBuffer[6];
    myReg.u8_temp[3] = rxBuffer[7];
    rxObj->word[1] = myReg.u32_temp;
	
	if(ciFifoCon.rxBitSet.RxTimeStampEnable){
		myReg.u8_temp[0] = rxBuffer[8];
        myReg.u8_temp[1] = rxBuffer[9];
        myReg.u8_temp[2] = rxBuffer[10];
        myReg.u8_temp[3] = rxBuffer[11];
        rxObj->word[2] = myReg.u32_temp;
		//数据帧
		for(index = 0; index < rxLen; index++)
			rxData[index] = rxBuffer[index + 12];
	}
	else{
		rxObj->word[2] = 0;
		for(index = 0; index < rxLen; index++)
            rxData[index] = rxBuffer[index + 8];
	}
	//FIFO更新
	drv_can_fd_rxChannelUpdate(channel);
}
//MCP2517FD接收通道更新
void drv_can_fd_rxChannelUpdate(CAN_FIFO_CHANNEL channel){
	uint16_t address = 0;
    REG_CiFIFOCON ciFifoCon;
	
	ciFifoCon.word = 0;
	address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1;
	//接收FIFO尾部递增1个报文
	ciFifoCon.rxBitSet.UINC = 1;
	
	drv_can_fd_spi_writeByte(address,ciFifoCon.byte[1]);
}
//MCP2517FD接收FIFO复位
void drv_can_fd_rxChannelReset(CAN_FIFO_CHANNEL channel){
	uint16_t address = 0;
    REG_CiFIFOCON ciFifoCon;
	address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1;
    ciFifoCon.word = 0;
	//置1后由硬件置0
    ciFifoCon.rxBitSet.FRESET = 1;
	
	drv_can_fd_spi_writeByte(address,ciFifoCon.byte[1]);
}
/******************************************CAN-FD TX EVENT FIFO******************************************/
//获取当前MCP2517FD发送事件FIFO状态
void drv_can_fd_txEvtfifoStatusGet(CAN_TEF_FIFO_STATUS *status){
	uint16_t address = 0;
	REG_CiTEFSTA ciTefSta;
    ciTefSta.word = 0;
    address = cREGADDR_CiTEFSTA;
	
	ciTefSta.byte[0] = drv_can_fd_spi_readByte(address);
	*status = (CAN_TEF_FIFO_STATUS)(ciTefSta.byte[0] & CAN_TEF_FIFO_STATUS_MASK);
}
//MCP2517FD发送事件FIFO消息
void drv_can_fd_txEvtfifoMsg(CAN_TEF_MSGOBJ *tefObj){
	uint16_t address = 0;
    uint32_t fifoReg[3];
    uint8_t Irbis = 0;
	
	address = cREGADDR_CiTEFCON;
	drv_can_fd_spi_readWordArray(address,fifoReg,3);
	//当前控制状态
	REG_CiTEFCON ciTefCon;
    ciTefCon.word = fifoReg[0];
	//当前状态
//	REG_CiTEFSTA ciTefSta;
//    ciTefSta.word = fifoReg[1];
	//当前地址
	REG_CiFIFOUA ciTefUa;
    ciTefUa.word = fifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
    address = 4 * ciTefUa.bF.UserAddress;
#else
    address = ciTefUa.bitSet.UserAddress;
#endif
    address += cRAMADDR_START;
	
	Irbis = 8;
	if(ciTefCon.bitSet.TimeStampEnable) Irbis += 4;
	
	uint8_t tefBuffer[12];
	drv_can_fd_spi_readByteArray(address,tefBuffer,Irbis);
	
	formatTrans32Struct_t myReg;
	myReg.u8_temp[0] = tefBuffer[0];
    myReg.u8_temp[1] = tefBuffer[1];
    myReg.u8_temp[2] = tefBuffer[2];
    myReg.u8_temp[3] = tefBuffer[3];
    tefObj->word[0] = myReg.u32_temp;

    myReg.u8_temp[0] = tefBuffer[4];
    myReg.u8_temp[1] = tefBuffer[5];
    myReg.u8_temp[2] = tefBuffer[6];
    myReg.u8_temp[3] = tefBuffer[7];
    tefObj->word[1] = myReg.u32_temp;
	
	if(ciTefCon.bitSet.TimeStampEnable){
		myReg.u8_temp[0] = tefBuffer[8];
        myReg.u8_temp[1] = tefBuffer[9];
        myReg.u8_temp[2] = tefBuffer[10];
        myReg.u8_temp[3] = tefBuffer[11];
        tefObj->word[2] = myReg.u32_temp;
	}
	else
		tefObj->word[2] = 0;
	drv_can_fd_txEvtfifoUpdate();
}
//MCP2517FD发送事件FIFO复位
void drv_can_fd_txEvtfifoReset(void){
	uint16_t address = 0;

    address = cREGADDR_CiTEFCON + 1;
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = 0;
    ciTefCon.bitSet.FRESET = 1;

	drv_can_fd_spi_writeByte(address,ciTefCon.byte[1]);
}
//MCP2517FD发送事件FIFO更新
void drv_can_fd_txEvtfifoUpdate(void){
	uint16_t address = 0;
	address = cREGADDR_CiTEFCON + 1;
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = 0;
    ciTefCon.bitSet.UINC = 1;
	drv_can_fd_spi_writeByte(address,ciTefCon.byte[1]);
}
//配置MCP2517FD发送事件FIFO
void drv_can_fd_txEvtfifoConfig(CAN_TEF_CONFIG *configSetting){
	REG_CiTEFCON ciTefCon;
    ciTefCon.word = canControlResetValues[cREGADDR_CiTEFCON / 4];
	
	ciTefCon.bitSet.FifoSize 		= configSetting->FifoSize;
    ciTefCon.bitSet.TimeStampEnable = configSetting->TimeStampEnable;

    drv_can_fd_spi_writeWord(cREGADDR_CiTEFCON,ciTefCon.word);
}
//获取MCP2517FD发送事件FIFO配置
void drv_can_fd_txEvtConfig_objReset(CAN_TEF_CONFIG *configSetting){
	REG_CiTEFCON ciTefCon;
    ciTefCon.word = canControlResetValues[cREGADDR_CiFIFOCON / 4];
	
	configSetting->FifoSize 		= ciTefCon.bitSet.FifoSize;
    configSetting->TimeStampEnable 	= ciTefCon.bitSet.TimeStampEnable;
}
/******************************************CAN-FD MODULE EVENT******************************************/
//获取MCP2517FD当前中断类型
void drv_can_fd_modEvtGet(CAN_MODULE_EVENT *flags){
	REG_CiINTFLAG intFlags;
    intFlags.word = 0;
	
	intFlags.word = drv_can_fd_spi_readHalfWord(cREGADDR_CiINTFLAG);
	*flags = (CAN_MODULE_EVENT)(intFlags.word & CAN_ALL_EVENTS);
}
//开启MCP2517FD中断
void drv_can_fd_modEvtEnable(CAN_MODULE_EVENT flags){
	uint16_t address = 0;

    address = cREGADDR_CiINTENABLE;
    REG_CiINTENABLE intEnables;
    intEnables.word = 0;
	
	intEnables.word = drv_can_fd_spi_readHalfWord(address);
	intEnables.word |= (flags & CAN_ALL_EVENTS);
	drv_can_fd_spi_writeHalfWord(address,intEnables.word);
}
//关闭MCP2517FD中断
void drv_can_fd_modEvtDisable(CAN_MODULE_EVENT flags){
	uint16_t address = 0;
	
	address = cREGADDR_CiINTENABLE;
    REG_CiINTENABLE intEnables;
    intEnables.word = 0;
	
	intEnables.word = drv_can_fd_spi_readHalfWord(address);
	intEnables.word &= ~(flags & CAN_ALL_EVENTS);
	drv_can_fd_spi_writeHalfWord(address,intEnables.word);
}
//清除MCP2517FD事件标志
void drv_can_fd_modEvtClear(CAN_MODULE_EVENT flags){
	uint16_t address = 0;
	
    address = cREGADDR_CiINTFLAG;
    REG_CiINTFLAG intFlags;
    intFlags.word = 0;
	
	intFlags.word = CAN_ALL_EVENTS;
    intFlags.word &= ~flags;
	
	drv_can_fd_spi_writeHalfWord(address,intFlags.word);
}
//获取MCP2517FD RX CODE
void drv_can_fd_modEvt_rxCodeGet(CAN_RXCODE *rxCode){
	uint16_t address = 0;
    uint8_t rxCodeByte = 0;
	
	address = cREGADDR_CiVEC + 3;
	rxCodeByte = drv_can_fd_spi_readByte(address);
	
	if((rxCodeByte < CAN_RXCODE_TOTAL_CHANNELS) || (rxCodeByte == CAN_RXCODE_NO_INT))
        *rxCode = (CAN_RXCODE)rxCodeByte;
    else
        *rxCode = CAN_RXCODE_RESERVED;
}
//获取MCP2517FD TX CODE
void drv_can_fd_modEvt_txCodeGet(CAN_TXCODE *txCode){
	uint16_t address = 0;
    uint8_t txCodeByte = 0;
	
	address = cREGADDR_CiVEC + 2;
	txCodeByte = drv_can_fd_spi_readByte(address);
	
	if((txCodeByte < CAN_TXCODE_TOTAL_CHANNELS) || (txCodeByte == CAN_TXCODE_NO_INT))
        *txCode = (CAN_TXCODE)txCodeByte;
    else
        *txCode = CAN_TXCODE_RESERVED;
}
//获取MCP2517FD使用的过滤器
void drv_can_fd_modEvt_filterHitGet(CAN_FILTER *filterHit){
	uint16_t address = 0;
    uint8_t filterHitByte = 0;
	
	address = cREGADDR_CiVEC + 1;
	
	filterHitByte = drv_can_fd_spi_readByte(address);
	*filterHit = (CAN_FILTER)filterHitByte;
}
//获取MCP2517FD I CODE
void drv_can_fd_modEvt_iCodeGet(CAN_ICODE *icode){
	uint16_t address = 0;
    uint8_t icodeByte = 0;
	
	address = cREGADDR_CiVEC;
	icodeByte = drv_can_fd_spi_readByte(address);
	
	if((icodeByte < CAN_ICODE_RESERVED) && ((icodeByte < CAN_ICODE_TOTAL_CHANNELS) || (icodeByte >= CAN_ICODE_NO_INT)))
        *icode = (CAN_ICODE)icodeByte;
    else
        *icode = CAN_ICODE_RESERVED;
}
/******************************************CAN-FD TX FIFO EVENT******************************************/
//获取MCP2517FD发送FIFO中断类型
void drv_can_fd_txChannel_evtGet(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_EVENT *flags){
	uint16_t address = 0;
	REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;
    address = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
	
	ciFifoSta.byte[0] = drv_can_fd_spi_readByte(address);
	*flags = (CAN_TX_FIFO_EVENT)(ciFifoSta.byte[0] & CAN_TX_FIFO_ALL_EVENTS);
}
//获取当前MCP2517FD发送中断
void drv_can_fd_txEvtGet(uint32_t *txif){
	*txif = drv_can_fd_spi_readWord(cREGADDR_CiTXIF);
}
//获取当前MCP2517FD发送尝试中断
void drv_can_fd_txEvtAttemptGet(uint32_t *txatif){
	*txatif = drv_can_fd_spi_readWord(cREGADDR_CiTXATIF);
}
//获取MCP2517FD发送FIFO通道号
void drv_can_fd_txChannel_indexGet(CAN_FIFO_CHANNEL channel,uint8_t *idx){
	uint16_t address = 0;
	REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;
    address = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
	
	ciFifoSta.word = drv_can_fd_spi_readWord(address);
	
	*idx = ciFifoSta.txBitSet.FifoIndex;
}
//使能MCP2517FD发送FIFO中断
void drv_can_fd_txChannel_evtEnable(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_EVENT flags){
	uint16_t address = 0;

    address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = 0;

    ciFifoCon.byte[0] = drv_can_fd_spi_readByte(address);
    ciFifoCon.byte[0] |= (flags & CAN_TX_FIFO_ALL_EVENTS);

    drv_can_fd_spi_writeByte(address,ciFifoCon.byte[0]);
}
//失能MCP2517FD发送FIFO中断
void drv_can_fd_txChannel_evtDisable(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_EVENT flags){
	uint16_t address = 0;

    address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = 0;

    ciFifoCon.byte[0] = drv_can_fd_spi_readByte(address);
    ciFifoCon.byte[0] &= ~(flags & CAN_TX_FIFO_ALL_EVENTS);

    drv_can_fd_spi_writeByte(address,ciFifoCon.byte[0]);
}
//清除MCP2517FD发送FIFO尝试中断
void drv_can_fd_txChannel_evtAttemptClear(CAN_FIFO_CHANNEL channel){
	uint16_t address = 0;
    
    address = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
    REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;
	
	ciFifoSta.byte[0] = drv_can_fd_spi_readByte(address);
	ciFifoSta.byte[0] &= ~CAN_TX_FIFO_ATTEMPTS_EXHAUSTED_EVENT;
	drv_can_fd_spi_writeByte(address,ciFifoSta.byte[0]);
}
/******************************************CAN-FD RX FIFO EVENT******************************************/
//获取MCP2517FD接收FIFO中断类型
void drv_can_fd_rxChannel_evtGet(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_EVENT *flags){
	uint16_t address = 0;
#ifdef CAN_TXQUEUE_IMPLEMENTED
    if(channel == CAN_TXQUEUE_CH0) return;
#endif
	REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;
    address = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

    ciFifoSta.byte[0] = drv_can_fd_spi_readByte(address);
	*flags = (CAN_RX_FIFO_EVENT)(ciFifoSta.byte[0] & CAN_RX_FIFO_ALL_EVENTS);
}
//获取当前MCP2517FD接收中断
void drv_can_fd_rxEvtGet(uint32_t *rxif){
	*rxif = drv_can_fd_spi_readWord(cREGADDR_CiRXIF);
}
//获取当前MCP2517FD接收溢出中断
void drv_can_fd_rxEvtOverflowGet(uint32_t *rxovif){
	*rxovif = drv_can_fd_spi_readWord(cREGADDR_CiRXOVIF);
}
//获取MCP2517FD接收FIFO通道号
void drv_can_fd_rxChannel_indexGet(CAN_FIFO_CHANNEL channel,uint8_t* idx){
	drv_can_fd_txChannel_indexGet(channel,idx);
}
//使能MCP2517FD接收FIFO中断
void drv_can_fd_rxChannel_evtEnable(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_EVENT flags){
	uint16_t address = 0;

#ifdef CAN_TXQUEUE_IMPLEMENTED
    if(channel == CAN_TXQUEUE_CH0) return;
#endif
    address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = 0;

    ciFifoCon.byte[0] = drv_can_fd_spi_readByte(address);
	ciFifoCon.byte[0] |= (flags & CAN_RX_FIFO_ALL_EVENTS);
	
	drv_can_fd_spi_writeByte(address,ciFifoCon.byte[0]);
}
//失能MCP2517FD接收FIFO中断
void drv_can_fd_rxChannel_evtDisable(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_EVENT flags){
	uint16_t address = 0;

#ifdef CAN_TXQUEUE_IMPLEMENTED
    if(channel == CAN_TXQUEUE_CH0) return;
#endif
    address = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = 0;

    ciFifoCon.byte[0] = drv_can_fd_spi_readByte(address);
	ciFifoCon.byte[0] &= ~(flags & CAN_RX_FIFO_ALL_EVENTS);
	
	drv_can_fd_spi_writeByte(address,ciFifoCon.byte[0]);
}
//清除MCP2517FD接收FIFO溢出中断
void drv_can_fd_rxChannel_evtOverflowClear(CAN_FIFO_CHANNEL channel){
	uint16_t address = 0;

#ifdef CAN_TXQUEUE_IMPLEMENTED
    if(channel == CAN_TXQUEUE_CH0) return;
#endif
    REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;
    address = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
	
	ciFifoSta.byte[0] = drv_can_fd_spi_readByte(address);
	ciFifoSta.byte[0] &= ~(CAN_RX_FIFO_OVERFLOW_EVENT);
	
	drv_can_fd_spi_writeByte(address,ciFifoSta.byte[0]);
}
/******************************************CAN-FD TX EVENT FIFO EVENT******************************************/
//获取MCP2517FD发送事件FIFO中断类型
void drv_can_fd_txEvtfifo_evtGet(CAN_TEF_FIFO_EVENT *flags){
	uint16_t address = 0;

    REG_CiTEFSTA ciTefSta;
    ciTefSta.word = 0;
    address = cREGADDR_CiTEFSTA;

    ciTefSta.byte[0] = drv_can_fd_spi_readByte(address);
	*flags = (CAN_TEF_FIFO_EVENT)(ciTefSta.byte[0] & CAN_TEF_FIFO_ALL_EVENTS);
}
//使能MCP2517FD发送事件FIFO中断
void drv_can_fd_txEvtfifo_evtEnable(CAN_TEF_FIFO_EVENT flags){
	uint16_t address = 0;

    address = cREGADDR_CiTEFCON;
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = 0;
	
	ciTefCon.byte[0] = drv_can_fd_spi_readByte(address);
	ciTefCon.byte[0] |= (flags & CAN_TEF_FIFO_ALL_EVENTS);
	
	drv_can_fd_spi_writeByte(address,ciTefCon.byte[0]);
}
//失能MCP2517FD发送事件FIFO中断
void drv_can_fd_txEvtfifo_evtDisable(CAN_TEF_FIFO_EVENT flags){
	uint16_t address = 0;

    address = cREGADDR_CiTEFCON;
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = 0;
	
	ciTefCon.byte[0] = drv_can_fd_spi_readByte(address);
	ciTefCon.byte[0] &= ~(flags & CAN_TEF_FIFO_ALL_EVENTS);
	
	drv_can_fd_spi_writeByte(address,ciTefCon.byte[0]);
}
//清除MCP2517FD发送事件FIFO溢出中断
void drv_can_fd_txEvtfifo_evtOverflowClear(void){
	uint16_t address = 0;
	
    REG_CiTEFSTA ciTefSta;
    ciTefSta.word = 0;
    address = cREGADDR_CiTEFSTA;

    ciTefSta.byte[0] = drv_can_fd_spi_readByte(address);
	ciTefSta.byte[0] &= ~(CAN_TEF_FIFO_OVERFLOW_EVENT);
	
	drv_can_fd_spi_writeByte(address,ciTefSta.byte[0]);
}
/******************************************CAN-FD ERROR MASTER******************************************/
//获取MCP2517FD发送错误计数
void drv_can_fd_errorCount_txGet(uint8_t *tec){
	uint16_t address = 0;

    address = cREGADDR_CiTREC + 1;
    *tec = drv_can_fd_spi_readByte(address);
}
//获取MCP2517FD接收错误计数
void drv_can_fd_errorCount_rxGet(uint8_t *rec){
	uint16_t address = 0;

    address = cREGADDR_CiTREC;
    *rec = drv_can_fd_spi_readByte(address);
}
//获取MCP2517FD错误状态
void drv_can_fd_errorStateGet(CAN_ERROR_STATE *flags){
	uint16_t address = 0;
	uint8_t fault = 0;
    address = cREGADDR_CiTREC + 2;
    
    fault = drv_can_fd_spi_readByte(address);
	*flags = (CAN_ERROR_STATE)(fault & CAN_ERROR_ALL);
}
//获取MCP2517FD错误状态计数
void drv_can_fd_errorCount_stateGet(uint8_t *tec,uint8_t *rec,CAN_ERROR_STATE *flags){
	uint16_t address = 0;

    address = cREGADDR_CiTREC;
    REG_CiTREC ciTrec;
    ciTrec.word = 0;

    ciTrec.word = drv_can_fd_spi_readWord(address);
	*tec = ciTrec.byte[1];
    *rec = ciTrec.byte[0];
    *flags = (CAN_ERROR_STATE)(ciTrec.byte[2] & CAN_ERROR_ALL);
}
//获取MCP2517FD总线诊断
void drv_can_fd_busDiagnosticsGet(CAN_BUS_DIAGNOSTIC *bd){
	uint16_t address = 0;

    address = cREGADDR_CiBDIAG0;
    uint32_t watcher[2];
	drv_can_fd_spi_readWordArray(address,watcher,2);
	
	CAN_BUS_DIAGNOSTIC b;
    b.word[0] = watcher[0];
    b.word[1] |= (uint32_t)(watcher[1] >> 16) & 0x0000ffff;
	b.word[1] = (uint32_t)((b.word[1] << 16) & 0xffff0000) | (watcher[1] & 0x0000ffff);
    *bd = b;
}
//清除MCP2517FD总线诊断
void drv_can_fd_busDiagnosticsClear(void){
	uint8_t address = 0;
	
    address = cREGADDR_CiBDIAG0;
    uint32_t watcher[2];
    watcher[0] = 0;
    watcher[1] = 0;
	
	drv_can_fd_spi_writeWordArray(address,watcher,2);
}
/******************************************CAN-FD ECC******************************************/
//使能MCP2517FD ECC
void drv_can_fd_ecc_enable(void){
	uint8_t dCon = 0;
	dCon = drv_can_fd_spi_readByte(cREGADDR_ECCCON);
	dCon |= 0x01;
	drv_can_fd_spi_writeByte(cREGADDR_ECCCON,dCon);
}
/******************************************CAN-FD OSC/BIT TIME******************************************/
//MCP2517FD开启振荡器
void drv_can_fd_osc_enable(void){
	uint8_t dCon = 0;
	dCon = drv_can_fd_spi_readByte(cREGADDR_OSC);
	dCon &= ~0x4;
	drv_can_fd_spi_writeByte(cREGADDR_OSC,dCon);
}
//MCP2517FD振荡器控制
void drv_can_fd_osc_config(CAN_OSC_CTRL ctrl){
	REG_OSC osc;
    osc.word = 0;

    osc.bitSet.PllEnable = ctrl.PllEnable;
    osc.bitSet.OscDisable = ctrl.OscDisable;
    osc.bitSet.SCLKDIV = ctrl.SclkDivide;
    osc.bitSet.CLKODIV = ctrl.ClkOutDivide;
	
	drv_can_fd_spi_writeByte(cREGADDR_OSC,osc.byte[0]);
}
//获取MCP2517FD振荡器初始化配置
void drv_can_fd_osc_config_objReset(CAN_OSC_CTRL *ctrl){
	REG_OSC osc;
	osc.word = mcp2517ControlResetValues[0];
	
	ctrl->PllEnable = osc.bitSet.PllEnable;
	ctrl->ClkOutDivide = osc.bitSet.CLKODIV;
	ctrl->OscDisable = osc.bitSet.OscDisable;
	ctrl->SclkDivide = osc.bitSet.SCLKDIV;
	
}
//获取MCP2517FD振荡器状态
void drv_can_fd_osc_status(CAN_OSC_STATUS *status){
	REG_OSC osc;
    osc.word = 0;
	
	CAN_OSC_STATUS state;
	osc.byte[1] = drv_can_fd_spi_readByte(cREGADDR_OSC +1);
	
	state.PllReady = osc.bitSet.PllReady;
	state.SclkReady = osc.bitSet.SclkReady;
	state.OscReady = osc.bitSet.OscReady;
	
	*status = state;
}
//配置MCP2517FD位时间
void drv_can_fd_bitTime_config(CAN_BITTIME_SETUP bitTime,CAN_SSP_MODE sspMode,CAN_SYSCLK_SPEED sysclk){
	switch(sysclk){
		case CAN_SYSCLK_40M:
			//仲裁段
			drv_can_fd_bitTime_config_nominal40MHz(bitTime);
			//数据段
			drv_can_fd_bitTime_config_data40MHz(bitTime,sspMode);
			break;
		case CAN_SYSCLK_20M:
			//仲裁段
			drv_can_fd_bitTime_config_nominal20MHz(bitTime);
			//数据段
			drv_can_fd_bitTime_config_data20MHz(bitTime,sspMode);
			break;
		case CAN_SYSCLK_10M:
			//仲裁段
			drv_can_fd_bitTime_config_nominal10MHz(bitTime);
			//数据段
			drv_can_fd_bitTime_config_data10MHz(bitTime,sspMode);
			break;
		default:
			break;
	}
}
//配置MCP2517FD仲裁段位时间(40MHz)
void drv_can_fd_bitTime_config_nominal40MHz(CAN_BITTIME_SETUP bitTime){
	REG_CiNBTCFG ciNbtcfg;
	ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];
	
	 switch(bitTime){
        // All 500K
        case CAN_500K_1M:
        case CAN_500K_2M:
        case CAN_500K_3M:
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 62;
            ciNbtcfg.bitSet.TSEG2 = 15;
            ciNbtcfg.bitSet.SJW = 15;
            break;

        // All 250K
        case CAN_250K_500K:
        case CAN_250K_833K:
        case CAN_250K_1M:
        case CAN_250K_1M5:
        case CAN_250K_2M:
        case CAN_250K_3M:
        case CAN_250K_4M:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 126;
            ciNbtcfg.bitSet.TSEG2 = 31;
            ciNbtcfg.bitSet.SJW = 31;
            break;
		
        case CAN_1000K_4M:
        case CAN_1000K_8M:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 30;
            ciNbtcfg.bitSet.TSEG2 = 7;
            ciNbtcfg.bitSet.SJW = 7;
            break;

        case CAN_125K_500K:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 254;
            ciNbtcfg.bitSet.TSEG2 = 63;
            ciNbtcfg.bitSet.SJW = 63;
            break;

        default:
            break;
    }
	 
	drv_can_fd_spi_writeWord(cREGADDR_CiNBTCFG,ciNbtcfg.word);
}
//配置MCP2517FD数据段位时间(40MHz)
void drv_can_fd_bitTime_config_data40MHz(CAN_BITTIME_SETUP bitTime,CAN_SSP_MODE sspMode){
	REG_CiDBTCFG ciDbtcfg;
    REG_CiTDC ciTdc;
	
	ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
    ciTdc.word = 0;
	
	ciTdc.bitSet.TDCMode = CAN_SSP_MODE_AUTO;
	uint32_t tdcValue = 0;
	 switch(bitTime){
        case CAN_500K_1M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 30;
            ciDbtcfg.bitSet.TSEG2 = 7;
            ciDbtcfg.bitSet.SJW = 7;
            // SSP
            ciTdc.bitSet.TDCOffset = 31;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_2M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 14;
            ciDbtcfg.bitSet.TSEG2 = 3;
            ciDbtcfg.bitSet.SJW = 3;
            // SSP
            ciTdc.bitSet.TDCOffset = 15;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_3M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 8;
            ciDbtcfg.bitSet.TSEG2 = 2;
            ciDbtcfg.bitSet.SJW = 2;
            // SSP
            ciTdc.bitSet.TDCOffset = 9;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_4M:
        case CAN_1000K_4M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 6;
            ciDbtcfg.bitSet.TSEG2 = 1;
            ciDbtcfg.bitSet.SJW = 1;
            // SSP
            ciTdc.bitSet.TDCOffset = 7;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_5M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 4;
            ciDbtcfg.bitSet.TSEG2 = 1;
            ciDbtcfg.bitSet.SJW = 1;
            // SSP
            ciTdc.bitSet.TDCOffset = 5;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_6M7:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 3;
            ciDbtcfg.bitSet.TSEG2 = 0;
            ciDbtcfg.bitSet.SJW = 0;
            // SSP
            ciTdc.bitSet.TDCOffset = 4;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_8M:
        case CAN_1000K_8M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 2;
            ciDbtcfg.bitSet.TSEG2 = 0;
            ciDbtcfg.bitSet.SJW = 0;
            // SSP
            ciTdc.bitSet.TDCOffset = 3;
            ciTdc.bitSet.TDCValue = 1;
            break;
        case CAN_500K_10M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 1;
            ciDbtcfg.bitSet.TSEG2 = 0;
            ciDbtcfg.bitSet.SJW = 0;
            // SSP
            ciTdc.bitSet.TDCOffset = 2;
            ciTdc.bitSet.TDCValue = 0;
            break;

        case CAN_250K_500K:
        case CAN_125K_500K:
            ciDbtcfg.bitSet.BRP = 1;
            ciDbtcfg.bitSet.TSEG1 = 30;
            ciDbtcfg.bitSet.TSEG2 = 7;
            ciDbtcfg.bitSet.SJW = 7;
            // SSP
            ciTdc.bitSet.TDCOffset = 31;
            ciTdc.bitSet.TDCValue = tdcValue;
            ciTdc.bitSet.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_833K:
            ciDbtcfg.bitSet.BRP = 1;
            ciDbtcfg.bitSet.TSEG1 = 17;
            ciDbtcfg.bitSet.TSEG2 = 4;
            ciDbtcfg.bitSet.SJW = 4;
            // SSP
            ciTdc.bitSet.TDCOffset = 18;
            ciTdc.bitSet.TDCValue = tdcValue;
            ciTdc.bitSet.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_1M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 30;
            ciDbtcfg.bitSet.TSEG2 = 7;
            ciDbtcfg.bitSet.SJW = 7;
            // SSP
            ciTdc.bitSet.TDCOffset = 31;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_250K_1M5:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 18;
            ciDbtcfg.bitSet.TSEG2 = 5;
            ciDbtcfg.bitSet.SJW = 5;
            // SSP
            ciTdc.bitSet.TDCOffset = 19;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_250K_2M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 14;
            ciDbtcfg.bitSet.TSEG2 = 3;
            ciDbtcfg.bitSet.SJW = 3;
            // SSP
            ciTdc.bitSet.TDCOffset = 15;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_250K_3M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 8;
            ciDbtcfg.bitSet.TSEG2 = 2;
            ciDbtcfg.bitSet.SJW = 2;
            // SSP
            ciTdc.bitSet.TDCOffset = 9;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_250K_4M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 6;
            ciDbtcfg.bitSet.TSEG2 = 1;
            ciDbtcfg.bitSet.SJW = 1;
            // SSP
            ciTdc.bitSet.TDCOffset = 7;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;

        default:
            break;
    }
	 
	drv_can_fd_spi_writeWord(cREGADDR_CiDBTCFG,ciDbtcfg.word);
	
#ifdef REV_A
    ciTdc.bF.TDCOffset = 0;
    ciTdc.bF.TDCValue = 0;
#endif
	
	drv_can_fd_spi_writeWord(cREGADDR_CiTDC,ciTdc.word);
}
//配置MCP2517FD仲裁段位时间(20MHz)
void drv_can_fd_bitTime_config_nominal20MHz(CAN_BITTIME_SETUP bitTime){
	REG_CiNBTCFG ciNbtcfg;
	ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];
	
	switch(bitTime){
        // All 500K
        case CAN_500K_1M:
        case CAN_500K_2M:
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 30;
            ciNbtcfg.bitSet.TSEG2 = 7;
            ciNbtcfg.bitSet.SJW = 7;
            break;

        // All 250K
        case CAN_250K_500K:
        case CAN_250K_833K:
        case CAN_250K_1M:
        case CAN_250K_1M5:
        case CAN_250K_2M:
        case CAN_250K_3M:
        case CAN_250K_4M:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 62;
            ciNbtcfg.bitSet.TSEG2 = 15;
            ciNbtcfg.bitSet.SJW = 15;
            break;

        case CAN_1000K_4M:
        case CAN_1000K_8M:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 14;
            ciNbtcfg.bitSet.TSEG2 = 3;
            ciNbtcfg.bitSet.SJW = 3;
            break;

        case CAN_125K_500K:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 126;
            ciNbtcfg.bitSet.TSEG2 = 31;
            ciNbtcfg.bitSet.SJW = 31;
            break;

        default:
            break;
    }
	drv_can_fd_spi_writeWord(cREGADDR_CiNBTCFG,ciNbtcfg.word);
}
//配置MCP2517FD数据段位时间(20MHz)
void drv_can_fd_bitTime_config_data20MHz(CAN_BITTIME_SETUP bitTime,CAN_SSP_MODE sspMode){
	REG_CiDBTCFG ciDbtcfg;
    REG_CiTDC ciTdc;
	
	ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
    ciTdc.word = 0;
	ciTdc.bitSet.TDCMode = CAN_SSP_MODE_AUTO;
	uint32_t tdcValue = 0;
	
	switch(bitTime){
        case CAN_500K_1M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 14;
            ciDbtcfg.bitSet.TSEG2 = 3;
            ciDbtcfg.bitSet.SJW = 3;
            // SSP
            ciTdc.bitSet.TDCOffset = 15;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_2M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 6;
            ciDbtcfg.bitSet.TSEG2 = 1;
            ciDbtcfg.bitSet.SJW = 1;
            // SSP
            ciTdc.bitSet.TDCOffset = 7;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_4M:
        case CAN_1000K_4M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 2;
            ciDbtcfg.bitSet.TSEG2 = 0;
            ciDbtcfg.bitSet.SJW = 0;
            // SSP
            ciTdc.bitSet.TDCOffset = 3;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_5M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 1;
            ciDbtcfg.bitSet.TSEG2 = 0;
            ciDbtcfg.bitSet.SJW = 0;
            // SSP
            ciTdc.bitSet.TDCOffset = 2;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
        case CAN_1000K_8M:
            break;

        case CAN_250K_500K:
        case CAN_125K_500K:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 30;
            ciDbtcfg.bitSet.TSEG2 = 7;
            ciDbtcfg.bitSet.SJW = 7;
            // SSP
            ciTdc.bitSet.TDCOffset = 31;
            ciTdc.bitSet.TDCValue = tdcValue;
            ciTdc.bitSet.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_833K:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 17;
            ciDbtcfg.bitSet.TSEG2 = 4;
            ciDbtcfg.bitSet.SJW = 4;
            // SSP
            ciTdc.bitSet.TDCOffset = 18;
            ciTdc.bitSet.TDCValue = tdcValue;
            ciTdc.bitSet.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_1M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 14;
            ciDbtcfg.bitSet.TSEG2 = 3;
            ciDbtcfg.bitSet.SJW = 3;
            // SSP
            ciTdc.bitSet.TDCOffset = 15;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_250K_1M5:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 8;
            ciDbtcfg.bitSet.TSEG2 = 2;
            ciDbtcfg.bitSet.SJW = 2;
            // SSP
            ciTdc.bitSet.TDCOffset = 9;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_250K_2M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 6;
            ciDbtcfg.bitSet.TSEG2 = 1;
            ciDbtcfg.bitSet.SJW = 1;
            // SSP
            ciTdc.bitSet.TDCOffset = 7;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_250K_3M:
            break;
        case CAN_250K_4M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 2;
            ciDbtcfg.bitSet.TSEG2 = 0;
            ciDbtcfg.bitSet.SJW = 0;
            // SSP
            ciTdc.bitSet.TDCOffset = 3;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;

        default:
            break;
    }
	drv_can_fd_spi_writeWord(cREGADDR_CiDBTCFG,ciDbtcfg.word);

#ifdef REV_A
    ciTdc.bF.TDCOffset = 0;
    ciTdc.bF.TDCValue = 0;
#endif
	
	drv_can_fd_spi_writeWord(cREGADDR_CiTDC,ciTdc.word);
}
//配置MCP2517FD仲裁段位时间(10MHz)
void drv_can_fd_bitTime_config_nominal10MHz(CAN_BITTIME_SETUP bitTime){
	REG_CiNBTCFG ciNbtcfg;
	ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];
	
	switch(bitTime){
        // All 500K
        case CAN_500K_1M:
        case CAN_500K_2M:
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 14;
            ciNbtcfg.bitSet.TSEG2 = 3;
            ciNbtcfg.bitSet.SJW = 3;
            break;

        // All 250K
        case CAN_250K_500K:
        case CAN_250K_833K:
        case CAN_250K_1M:
        case CAN_250K_1M5:
        case CAN_250K_2M:
        case CAN_250K_3M:
        case CAN_250K_4M:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 30;
            ciNbtcfg.bitSet.TSEG2 = 7;
            ciNbtcfg.bitSet.SJW = 7;
            break;

        case CAN_1000K_4M:
        case CAN_1000K_8M:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 7;
            ciNbtcfg.bitSet.TSEG2 = 2;
            ciNbtcfg.bitSet.SJW = 2;
            break;

        case CAN_125K_500K:
            ciNbtcfg.bitSet.BRP = 0;
            ciNbtcfg.bitSet.TSEG1 = 62;
            ciNbtcfg.bitSet.TSEG2 = 15;
            ciNbtcfg.bitSet.SJW = 15;
            break;

        default:
            break;
    }
	drv_can_fd_spi_writeWord(cREGADDR_CiNBTCFG,ciNbtcfg.word);
}
//配置MCP2517FD数据段位时间(10MHz)
void drv_can_fd_bitTime_config_data10MHz(CAN_BITTIME_SETUP bitTime,CAN_SSP_MODE sspMode){
	REG_CiDBTCFG ciDbtcfg;
    REG_CiTDC ciTdc;
	
	ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
    ciTdc.word = 0;
	ciTdc.bitSet.TDCMode = CAN_SSP_MODE_AUTO;
	uint32_t tdcValue = 0;
	
	switch(bitTime){
        case CAN_500K_1M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 6;
            ciDbtcfg.bitSet.TSEG2 = 1;
            ciDbtcfg.bitSet.SJW = 1;
            // SSP
            ciTdc.bitSet.TDCOffset = 7;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_2M:
            // Data BR
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 2;
            ciDbtcfg.bitSet.TSEG2 = 0;
            ciDbtcfg.bitSet.SJW = 0;
            // SSP
            ciTdc.bitSet.TDCOffset = 3;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
        case CAN_1000K_4M:
        case CAN_1000K_8M:
            break;

        case CAN_250K_500K:
        case CAN_125K_500K:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 14;
            ciDbtcfg.bitSet.TSEG2 = 3;
            ciDbtcfg.bitSet.SJW = 3;
            // SSP
            ciTdc.bitSet.TDCOffset = 15;
            ciTdc.bitSet.TDCValue = tdcValue;
            ciTdc.bitSet.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_833K:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 7;
            ciDbtcfg.bitSet.TSEG2 = 2;
            ciDbtcfg.bitSet.SJW = 2;
            // SSP
            ciTdc.bitSet.TDCOffset = 8;
            ciTdc.bitSet.TDCValue = tdcValue;
            ciTdc.bitSet.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_1M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 6;
            ciDbtcfg.bitSet.TSEG2 = 1;
            ciDbtcfg.bitSet.SJW = 1;
            // SSP
            ciTdc.bitSet.TDCOffset = 7;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_250K_1M5:
            break;
        case CAN_250K_2M:
            ciDbtcfg.bitSet.BRP = 0;
            ciDbtcfg.bitSet.TSEG1 = 2;
            ciDbtcfg.bitSet.TSEG2 = 0;
            ciDbtcfg.bitSet.SJW = 0;
            // SSP
            ciTdc.bitSet.TDCOffset = 3;
            ciTdc.bitSet.TDCValue = tdcValue;
            break;
        case CAN_250K_3M:
        case CAN_250K_4M:
            break;

        default:
            break;
    }
	drv_can_fd_spi_writeWord(cREGADDR_CiDBTCFG,ciDbtcfg.word);
	
#ifdef REV_A
    ciTdc.bF.TDCOffset = 0;
    ciTdc.bF.TDCValue = 0;
#endif
	
	drv_can_fd_spi_writeWord(cREGADDR_CiTDC,ciTdc.word);
}
/******************************************CAN-FD GPIO******************************************/
//MCP2517FD配置GPIO模式
void drv_can_fd_gpio_modeConfig(GPIO_PIN_MODE gpio0,GPIO_PIN_MODE gpio1){
	uint16_t address = 0;
	address = cREGADDR_IOCON + 3;
    REG_IOCON iocon;
    iocon.word = 0;
	
	iocon.byte[3] = drv_can_fd_spi_readByte(address);
	iocon.bitSet.PinMode0 = gpio0;
    iocon.bitSet.PinMode1 = gpio1;
	
	drv_can_fd_spi_writeByte(address,iocon.byte[3]);
}
//MCP2517FD配置GPIO输出模式
void drv_can_fd_gpio_dirConfig(GPIO_PIN_MODE gpio0,GPIO_PIN_MODE gpio1){
	uint16_t address = 0;
	address = cREGADDR_IOCON;
    REG_IOCON iocon;
    iocon.word = 0;
	
	iocon.byte[0] = drv_can_fd_spi_readByte(address);
	iocon.bitSet.TRIS0 = gpio0;
    iocon.bitSet.TRIS1 = gpio1;
	
	drv_can_fd_spi_writeByte(address,iocon.byte[0]);
}

/******************************************CAN-FD TOOL******************************************/
const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};
//MCP2517FD CRC16校验位生成器
uint16_t drv_can_fd_crc16_checksum(uint8_t* data,uint16_t size){
	uint16_t init = CRC16_BASE;
	uint8_t index;
	 while (size-- != 0){
        index = ((uint8_t*) & init)[CRC16_UPPER] ^ *data++;
        init = (init << 8) ^ crc16_table[index];   
    }
    return init;
}
//MCP2517FD 数据帧长度
uint32_t drv_can_fd_dlcToDataBytes(CAN_DLC dlc){
    uint32_t dataBytesInObject = 0;
	
	__nop();
	__nop();
	
    if (dlc < CAN_DLC_12) {
        dataBytesInObject = dlc;
    } else {
        switch (dlc) {
            case CAN_DLC_12:
                dataBytesInObject = 12;
                break;
            case CAN_DLC_16:
                dataBytesInObject = 16;
                break;
            case CAN_DLC_20:
                dataBytesInObject = 20;
                break;
            case CAN_DLC_24:
                dataBytesInObject = 24;
                break;
            case CAN_DLC_32:
                dataBytesInObject = 32;
                break;
            case CAN_DLC_48:
                dataBytesInObject = 48;
                break;
            case CAN_DLC_64:
                dataBytesInObject = 64;
                break;
            default:
                break;
        }
    }
    return dataBytesInObject;
}

//初始化MCP2517FD RAM
void drv_can_fd_ramInit(uint8_t dCon){
	uint8_t txData[DEFAULT_SPI_BUFF_SIZE];
	uint32_t k;
	for(k = 0; k < DEFAULT_SPI_BUFF_SIZE; k++)
		txData[k] = dCon;
	//RAM起始地址
	uint16_t address = cRAMADDR_START;
	for (k = 0; k < (cRAM_SIZE / DEFAULT_SPI_BUFF_SIZE); k++){
		drv_can_fd_spi_writeByteArray(address,txData,DEFAULT_SPI_BUFF_SIZE);
		address += DEFAULT_SPI_BUFF_SIZE;
	}	
}


