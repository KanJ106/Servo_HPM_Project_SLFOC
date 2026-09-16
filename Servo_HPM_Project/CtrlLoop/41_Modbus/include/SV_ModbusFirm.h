#ifndef SV_MODBUSFIRM_H_
#define SV_MODBUSFIRM_H_
#include "stdint.h"//

#define  MODBUS_RX_END  1

#define  MODBUS_MAX_LEN     16

#define  MODBUS_ID_ERR      1
#define  MODBUS_FRAME_ERR   2
#define  MODBUS_CODE_ERR    3
#define  MODBUS_CRC_ERR     4
#define  MODBUS_ADDRES_ERR  5
#define  MODBUS_LIM_ERR     6
#define  MODBUS_RW_ERR      7

#define MODBUSMAXADDRES  (100)
typedef struct
{
    uint16_t Addr;
    uint16_t RxByteCnt;			//接收数据的个数
    uint16_t LastRxByteCnt;
    uint16_t RxEndCnt;
    uint16_t SCIDelay;
    uint16_t RxEndFlg;
    uint16_t RxEndNum;
    uint8_t  CrcFlg;
    uint8_t  Conflg;
    
    uint16_t Fault_Code;
    uint16_t CrcData;
    
    uint16_t TxCnt;
    uint16_t TxNum;
    uint16_t TXReady;
    uint16_t Tx_Status;
    uint16_t TxTimeAll;
    uint16_t TxTimeCnt;
    uint16_t TxDelay;
    uint16_t WriteWord[32];
    
    uint8_t  RxData[64];
    uint8_t  TxData[64];

    uint16_t TxCrc;
}ModbusVal_Type;

typedef struct
{
    uint8_t  Type;
    uint8_t  Size;
    uint8_t  RW;
    uint16_t Max;
    uint16_t Min;
}TYPE_AttRI_Modbus;

extern ModbusVal_Type ModbusVal;
extern void Modbus_RXProcess(void);
extern void Modbus_TXProcess(void);
#endif
