#ifndef __DRV_CANFD_REGISTER_H
#define __DRV_CANFD_REGISTER_H

#include "stm32f4xx.h"
#include "drv_canfd_api.h"

/*SPI指令集*/
//指令格式 C = 命令(4位)  A = 地址(12位)  D = 数据(1~n字节)  N = 字节数(1字节)  CRC(2字节)
#define cINSTRUCTION_RESET			0x00//重置设备,选择模式
#define cINSTRUCTION_READ			0x03//从地址A读取SFR/RAM的内容
#define cINSTRUCTION_READ_CRC       0x0B//从地址A读取SFR/RAM的内容,N个数据字节,2字节CRC;基于C,A,N,D计算CRC
#define cINSTRUCTION_WRITE			0x02//将SFR/RAM内容写入地址A
#define cINSTRUCTION_WRITE_CRC      0x0A//将SFR/RAM内容写入地址A,N个数据字节,2字节CRC;基于C,A,N,D计算CRC
#define cINSTRUCTION_WRITE_SAFE     0x0C//将SFR/RAM内容写入地址A,写入前校验CRC;基于C,A,N,D计算CRC

/* can_fd_ubp */
#define cREGADDR_CiCON  	0x000
#define cREGADDR_CiNBTCFG	0x004
#define cREGADDR_CiDBTCFG	0x008
#define cREGADDR_CiTDC  	0x00C

#define cREGADDR_CiTBC      0x010
#define cREGADDR_CiTSCON    0x014
#define cREGADDR_CiVEC      0x018
#define cREGADDR_CiINT      0x01C
#define cREGADDR_CiINTFLAG      cREGADDR_CiINT
#define cREGADDR_CiINTENABLE    (cREGADDR_CiINT+2)

#define cREGADDR_CiRXIF     0x020
#define cREGADDR_CiTXIF     0x024
#define cREGADDR_CiRXOVIF   0x028
#define cREGADDR_CiTXATIF   0x02C

#define cREGADDR_CiTXREQ    0x030
#define cREGADDR_CiTREC     0x034
#define cREGADDR_CiBDIAG0   0x038
#define cREGADDR_CiBDIAG1   0x03C

#define cREGADDR_CiTEFCON   0x040
#define cREGADDR_CiTEFSTA   0x044
#define cREGADDR_CiTEFUA    0x048
#define cREGADDR_CiFIFOBA   0x04C

#define cREGADDR_CiFIFOCON  0x050
#define cREGADDR_CiFIFOSTA  0x054
#define cREGADDR_CiFIFOUA   0x058
#define CiFIFO_OFFSET       (3*4)

#ifdef CAN_TXQUEUE_IMPLEMENTED
#define cREGADDR_CiTXQCON  0x050
#define cREGADDR_CiTXQSTA  0x054
#define cREGADDR_CiTXQUA   0x058
#endif

#ifdef FIXED_FILTER_ADDRESS
// Up to A1, the filter start address was fixed
#define cREGADDR_CiFLTCON   0x1D0
#define cREGADDR_CiFLTOBJ   0x1F0
#define cREGADDR_CiMASK     0x1F4
#else
// Starting with B0, the filters start right after the FIFO control/status registers
#define cREGADDR_CiFLTCON   (cREGADDR_CiFIFOCON+(CiFIFO_OFFSET*CAN_FIFO_TOTAL_CHANNELS))
#define cREGADDR_CiFLTOBJ   (cREGADDR_CiFLTCON+CAN_FIFO_TOTAL_CHANNELS)
#define cREGADDR_CiMASK     (cREGADDR_CiFLTOBJ+4)
#endif

#define CiFILTER_OFFSET     (2*4)

/* MCP2517 Specific */
#define cREGADDR_OSC        0xE00
#define cREGADDR_IOCON      0xE04
#define cREGADDR_CRC    	0xE08
#define cREGADDR_ECCCON  	0xE0C
#define cREGADDR_ECCSTA  	0xE10

/* RAM addresses */
#define cRAM_SIZE       2048
#define cRAMADDR_START  0x400
#define cRAMADDR_END    (cRAMADDR_START+cRAM_SIZE)

//CANFD控制寄存器
typedef union _REG_CiCON {
    struct {
        uint32_t DNetFilterCount : 5;
        uint32_t IsoCrcEnable : 1;
        uint32_t ProtocolExceptionEventDisable : 1;
        uint32_t unimplemented1 : 1;
        uint32_t WakeUpFilterEnable : 1;
        uint32_t WakeUpFilterTime : 2;
        uint32_t unimplemented2 : 1;
        uint32_t BitRateSwitchDisable : 1;
        uint32_t unimplemented3 : 3;
        uint32_t RestrictReTxAttempts : 1;
        uint32_t EsiInGatewayMode : 1;
        uint32_t SystemErrorToListenOnly : 1;
        uint32_t StoreInTEF : 1;
        uint32_t TXQEnable : 1;
        uint32_t OpMode : 3;
        uint32_t RequestOpMode : 3;
        uint32_t AbortAllTx : 1;
        uint32_t TxBandWidthSharing : 4;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiCON;
//CANFD FIFO控制寄存器
typedef union _REG_CiFIFOCON {
    //接收FIFO
    struct {
        uint32_t RxNotEmptyIE : 1;
        uint32_t RxHalfFullIE : 1;
        uint32_t RxFullIE : 1;
        uint32_t RxOverFlowIE : 1;
        uint32_t unimplemented1 : 1;
        uint32_t RxTimeStampEnable : 1;
        uint32_t unimplemented2 : 1;
        uint32_t TxEnable : 1;
        uint32_t UINC : 1;
        uint32_t unimplemented3 : 1;
        uint32_t FRESET : 1;
        uint32_t unimplemented4 : 13;
        uint32_t FifoSize : 5;
        uint32_t PayLoadSize : 3;
    }rxBitSet;
    //发送FIFO
    struct {
        uint32_t TxNotFullIE : 1;
        uint32_t TxHalfFullIE : 1;
        uint32_t TxEmptyIE : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TxAttemptIE : 1;
        uint32_t unimplemented2 : 1;
        uint32_t RTREnable : 1;
        uint32_t TxEnable : 1;
        uint32_t UINC : 1;
        uint32_t TxRequest : 1;
        uint32_t FRESET : 1;
        uint32_t unimplemented3 : 5;
        uint32_t TxPriority : 5;
        uint32_t TxAttempts : 2;
        uint32_t unimplemented4 : 1;
        uint32_t FifoSize : 5;
        uint32_t PayLoadSize : 3;
    }txBitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiFIFOCON;
//CAN FIFO状态寄存器
typedef union _REG_CiFIFOSTA {
    //接收FIFO
    struct {
        uint32_t RxNotEmptyIF : 1;
        uint32_t RxHalfFullIF : 1;
        uint32_t RxFullIF : 1;
        uint32_t RxOverFlowIF : 1;
        uint32_t unimplemented1 : 4;
        uint32_t FifoIndex : 5;
        uint32_t unimplemented2 : 19;
    }rxBitSet;
	//发送FIFO
    struct {
        uint32_t TxNotFullIF : 1;
        uint32_t TxHalfFullIF : 1;
        uint32_t TxEmptyIF : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TxAttemptIF : 1;
        uint32_t TxError : 1;
        uint32_t TxLostArbitration : 1;
        uint32_t TxAborted : 1;
        uint32_t FifoIndex : 5;
        uint32_t unimplemented2 : 19;
    }txBitSet;
    uint32_t word;
    uint8_t byte[4];
} REG_CiFIFOSTA;
//CAN FIFO用户地址寄存器
typedef union _REG_CiFIFOUA {
    struct {
        uint32_t UserAddress : 12;
        uint32_t unimplemented1 : 20;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
} REG_CiFIFOUA;
//CAN发送队列控制寄存器
typedef union _REG_CiTXQCON {
    struct {
        uint32_t TxNotFullIE : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TxEmptyIE : 1;
        uint32_t unimplemented2 : 1;
        uint32_t TxAttemptIE : 1;
        uint32_t unimplemented3 : 2;
        uint32_t TxEnable : 1;
        uint32_t UINC : 1;
        uint32_t TxRequest : 1;
        uint32_t FRESET : 1;
        uint32_t unimplemented4 : 5;
        uint32_t TxPriority : 5;
        uint32_t TxAttempts : 2;
        uint32_t unimplemented5 : 1;
        uint32_t FifoSize : 5;
        uint32_t PayLoadSize : 3;
    }txBitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiTXQCON;
//CAN ID过滤器
typedef union _REG_CiFLTOBJ {
    CAN_FILTEROBJ_ID bitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiFLTOBJ;
//CAN掩码ID过滤器
typedef union _REG_CiMASK {
    CAN_MASKOBJ_ID bitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiMASK;
//CAN过滤器指向FIFO
typedef union _REG_CiFLTCON_BYTE {
    struct {
        uint32_t BufferPointer : 5;
        uint32_t unimplemented1 : 2;
        uint32_t Enable : 1;
    }bitSet;
    uint8_t byte;
}REG_CiFLTCON_BYTE;
//CAN发送事件FIFO控制寄存器
typedef union _REG_CiTEFCON {
    struct {
        uint32_t TEFNEIE : 1;
        uint32_t TEFHFIE : 1;
        uint32_t TEFFULIE : 1;
        uint32_t TEFOVIE : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TimeStampEnable : 1;
        uint32_t unimplemented2 : 2;
        uint32_t UINC : 1;
        uint32_t unimplemented3 : 1;
        uint32_t FRESET : 1;
        uint32_t unimplemented4 : 13;
        uint32_t FifoSize : 5;
        uint32_t unimplemented5 : 3;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiTEFCON;
//CAN发送事件FIFO状态寄存器
typedef union _REG_CiTEFSTA {
    struct {
        uint32_t TEFNotEmptyIF : 1;
        uint32_t TEFHalfFullIF : 1;
        uint32_t TEFFullIF : 1;
        uint32_t TEFOVIF : 1;
        uint32_t unimplemented1 : 28;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiTEFSTA;
//CAN中断标志组
typedef struct _CAN_INT_FLAGS {
    uint32_t TXIF : 1;
    uint32_t RXIF : 1;
    uint32_t TBCIF : 1;
    uint32_t MODIF : 1;
    uint32_t TEFIF : 1;
    uint32_t unimplemented1 : 3;

    uint32_t ECCIF : 1;
    uint32_t SPICRCIF : 1;
    uint32_t TXATIF : 1;
    uint32_t RXOVIF : 1;
    uint32_t SERRIF : 1;
    uint32_t CERRIF : 1;
    uint32_t WAKIF : 1;
    uint32_t IVMIF : 1;
}CAN_INT_FLAGS;
//CAN中断管理寄存器
typedef union _REG_CiINTFLAG {
    CAN_INT_FLAGS IF;
    uint16_t word;
    uint8_t byte[2];
}REG_CiINTFLAG;
//CAN中断启动标志组
typedef struct _CAN_INT_ENABLES {
    uint32_t TXIE : 1;
    uint32_t RXIE : 1;
    uint32_t TBCIE : 1;
    uint32_t MODIE : 1;
    uint32_t TEFIE : 1;
    uint32_t unimplemented2 : 3;

    uint32_t ECCIE : 1;
    uint32_t SPICRCIE : 1;
    uint32_t TXATIE : 1;
    uint32_t RXOVIE : 1;
    uint32_t SERRIE : 1;
    uint32_t CERRIE : 1;
    uint32_t WAKIE : 1;
    uint32_t IVMIE : 1;
}CAN_INT_ENABLES;
//CAN中断使能寄存器
typedef union _REG_CiINTENABLE {
    CAN_INT_ENABLES IE;
    uint16_t word;
    uint8_t byte[2];
}REG_CiINTENABLE;
//CAN错误计数寄存器
typedef union _REG_CiTREC {
    struct {
        uint32_t RxErrorCount : 8;
        uint32_t TxErrorCount : 8;
        uint32_t ErrorStateWarning : 1;
        uint32_t RxErrorStateWarning : 1;
        uint32_t TxErrorStateWarning : 1;
        uint32_t RxErrorStatePassive : 1;
        uint32_t TxErrorStatePassive : 1;
        uint32_t TxErrorStateBusOff : 1;
        uint32_t unimplemented1 : 10;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiTREC;
//CAN振荡器控制寄存器
typedef union _REG_OSC {
    struct {
        uint32_t PllEnable : 1;
        uint32_t unimplemented1 : 1;
        uint32_t OscDisable : 1;
        uint32_t unimplemented2 : 1;
        uint32_t SCLKDIV : 1;
        uint32_t CLKODIV : 2;
        uint32_t unimplemented3 : 1;
        uint32_t PllReady : 1;
        uint32_t unimplemented4 : 1;
        uint32_t OscReady : 1;
        uint32_t unimplemented5 : 1;
        uint32_t SclkReady : 1;
        uint32_t unimplemented6 : 19;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_OSC;
//CAN数据位时间配置寄存器
typedef union _REG_CiNBTCFG {
    struct {
        uint32_t SJW : 7;
        uint32_t unimplemented1 : 1;
        uint32_t TSEG2 : 7;
        uint32_t unimplemented2 : 1;
        uint32_t TSEG1 : 8;
        uint32_t BRP : 8;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiNBTCFG;
//CAN时基计数寄存器
typedef union _REG_CiDBTCFG {
    struct {
        uint32_t SJW : 4;
        uint32_t unimplemented1 : 4;
        uint32_t TSEG2 : 4;
        uint32_t unimplemented2 : 4;
        uint32_t TSEG1 : 5;
        uint32_t unimplemented3 : 3;
        uint32_t BRP : 8;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
} REG_CiDBTCFG;
//CAN发送器延时补偿寄存器
typedef union _REG_CiTDC {
    struct {
        uint32_t TDCValue : 6;
        uint32_t unimplemented1 : 2;
        uint32_t TDCOffset : 7;
        uint32_t unimplemented2 : 1;
        uint32_t TDCMode : 2;
        uint32_t unimplemented3 : 6;
        uint32_t SID11Enable : 1;
        uint32_t EdgeFilterEnable : 1;
        uint32_t unimplemented4 : 6;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
}REG_CiTDC;
//CAN引脚控制寄存器
typedef union _REG_IOCON {
    struct {
        uint32_t TRIS0 : 1;
        uint32_t TRIS1 : 1;
        uint32_t unimplemented1 : 2;
        uint32_t ClearAutoSleepOnMatch : 1;
        uint32_t AutoSleepEnable : 1;
        uint32_t XcrSTBYEnable : 1;
        uint32_t unimplemented2 : 1;
        uint32_t LAT0 : 1;
        uint32_t LAT1 : 1;
        uint32_t unimplemented3 : 5;
        uint32_t HVDETSEL : 1;
        uint32_t GPIO0 : 1;
        uint32_t GPIO1 : 1;
        uint32_t unimplemented4 : 6;
        uint32_t PinMode0 : 1;
        uint32_t PinMode1 : 1;
        uint32_t unimplemented5 : 2;
        uint32_t TXCANOpenDrain : 1;
        uint32_t SOFOutputEnable : 1;
        uint32_t INTPinOpenDrain : 1;
        uint32_t unimplemented6 : 1;
    }bitSet;
    uint32_t word;
    uint8_t byte[4];
} REG_IOCON;
#define N_CAN_CTRL_REGS  20
static uint32_t canControlResetValues[] = {
    /* Address 0x000 to 0x00C */
#ifdef CAN_TXQUEUE_IMPLEMENTED
    0x04980760, 0x003E0F0F, 0x000E0303, 0x00021000,
#else
    0x04880760, 0x003E0F0F, 0x000E0303, 0x00021000,
#endif
    /* Address 0x010 to 0x01C */
    0x00000000, 0x00000000, 0x40400040, 0x00000000,
    /* Address 0x020 to 0x02C */
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* Address 0x030 to 0x03C */
    0x00000000, 0x00200000, 0x00000000, 0x00000000,
    /* Address 0x040 to 0x04C */
    0x00000400, 0x00000000, 0x00000000, 0x00000000,
	/*extra*/
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
};




#endif
