#ifndef S_TSENSOR_INIT_H
#define S_TSENSOR_INIT_H
#include "MCU_Fw.h"

#define SENSORSCI     HPM_UART3
#define SENSORSCICLK  clock_uart3

#define SENSORSCI_TxData     (SENSORSCI->THR)
#define SENSORSCI_RxData     (SENSORSCI->RBR & UART_RBR_RBR_MASK)

#define SENSORSCI_FIFO_DEPTH 32

#define SENSORSCI_TXEND_FLAG                       (UART_LSR_TEMT_GET(SENSORSCI->LSR))
#define SENSORSCI_TXFIF0_NUM                       (UART_LSR_TFIFO_NUM_GET(SENSORSCI->LSR))
#define SENSORSCI_RXFIF0_NUM                       (UART_LSR_RFIFO_NUM_GET(SENSORSCI->LSR))
#define SENSORSCI_ResetFIFO                        (SENSORSCI->FCRR |= UART_FCRR_RFIFORST_MASK)
#define SENSORSCI_R_STUATS                         (SENSORSCI->LSR)


typedef struct
{
	uint8_t Delay;				//读编码器EEP数据帧延时
    uint8_t Step;
	uint32_t NoAckTime;			//无数据返回计时
	uint32_t ErrCRCcnt;			//CRC校验错误次数

    float TorqueOffset;
    float TorqueOrg;
    float ToreGainf;

    uint8_t EncRxNum;
    uint8_t EncTxcnt; 
    
    uint8_t CrcErrFlag;
    uint8_t NoAckFlag;

    uint8_t Rxflag;
    uint8_t Initflag;
    uint16_t DelayCnt;
    uint32_t DelayCnt2;
    uint8_t EncRxNum2;
    uint8_t SetADCflag;

    uint8_t Errcnt;
    uint16_t Vendor;
    uint16_t ProId;
    uint16_t Version;

}SENSOR_Type;

typedef struct
{
    uint16_t Low:8;   //字的低位
    uint16_t High:8;  //字的高位
}TYPE_BYTE_S;

typedef union
{
    uint16_t      all;
    TYPE_BYTE_S   Byte;
}SENSOR_WORD;

typedef union
{
	float     F;
	uint8_t   Byte[4];
}TYPE_FLOAT;
extern int32_t TorQueData;
extern SENSOR_Type Sensor;
void Sci_TorSensorInit(void);
void Tor_SensorUart_PerMod(void);
extern uint8_t TorFbFlag;
void Tor_Sensor_Precess(void);


#endif

