#ifndef S_COMM_INIT_H
#define S_COMM_INIT_H
#include "r_cg_project.h"
#include "MCU_Fw.h"

/* Dedicated telemetry build: UART0/485 belongs to VOFA, not HarmoCore.
 * Set to 0 at build time to restore the legacy protocol. */
#ifndef SENSORLESS_VOFA_ENABLE
#if defined(SENSORLESS_CANOPEN_BUILD)
#define SENSORLESS_VOFA_ENABLE 1
#else
#define SENSORLESS_VOFA_ENABLE 0
#endif
#endif
#if SENSORLESS_VOFA_ENABLE
void SensorlessVofa_Service(void);
#endif

#define     SCI_BUFF_SIZE  2070    //数据缓冲区大小
#define     SCI_STOPBITP_2 1     //2位停止位
#define     SCI_STOPBITP_1 0     //1位停止位

#define     SCI_NOPARITY    0   //无校验
#define     SCI_PARITY      1   //有校验
#define     SCI_ODDPARTTY   0   //奇校验
#define     SCI_EVEN_PARITY 1   //偶校验

#define     SCI_BUAD_4800   0	//SCI通信波特率4800
#define     SCI_BUAD_9600   1	//SCI通信波特率9600
#define     SCI_BUAD_19200  2	//SCI通信波特率19200
#define     SCI_BUAD_38400  3	//SCI通信波特率38400
#define     SCI_BUAD_57600  4	//SCI通信波特率57600
#define     SCI_BUAD_115200 5	//SCI通信波特率115200
#define     SCI_BUAD_500000 6	//SCI通信波特率500000

#define     FORMAT_N_STOP1  0
#define     FORMAT_O_STOP1  1
#define     FORMAT_E_STOP1  2
#define     FORMAT_N_STOP2  3
#define     FORMAT_O_STOP2  4
#define     FORMAT_E_STOP2  5

#define     SCI_CHA 0    //
#define     SCI_CHB 1    //
      

typedef struct
{
    uint16_t   RxCnt;
    uint16_t   TxCnt;
    uint16_t   TxType;		//发送类型（0：无；1：正常；2：示波器）
    uint16_t   RxSum;
    uint16_t   TxSum;
    uint16_t   RxBuffull;
    uint8_t    RxBuf[SCI_BUFF_SIZE];             /* RX Buffer Struct */
    uint8_t    TxBuf[SCI_BUFF_SIZE];             /* TX Buffer Struct */
}Uart_INFO;

#if SERVO_MCU == HPM_6E00

#define ModbusSci  HPM_UART0
#define ModbusCLK  clock_uart0 
#define ModbusDMA  HPM_DMA_SRC_UART0_TX

#define SCI_SelectToRx()        do{HPM_GPIO0->DO[GPIO_DI_GPIOA].CLEAR = 1 << 9;}while(0) 
#define SCI_SelectToTx()        do{HPM_GPIO0->DO[GPIO_DI_GPIOA].SET = 1 << 9;}while(0) 

#define FIFO_DEPTH 32

#define SCOPE_TXEND_FLAG                       (UART_LSR_TEMT_GET(ModbusSci->LSR))
#define SCOPE_TXFIF0_NUM                       (UART_LSR_TFIFO_NUM_GET(ModbusSci->LSR))
#define SCOPE_TX_DATA                          (ModbusSci->THR)
#define SCOPE_RXFIF0_NUM                       (UART_LSR_RFIFO_NUM_GET(ModbusSci->LSR))
#define SCOPE_ResetFIFO                        (ModbusSci->FCRR |= UART_FCRR_RFIFORST_MASK)
#define SCOPE_RX_DATA                          (ModbusSci->RBR)
#define SCOPE_R_STUATS                         (ModbusSci->LSR)
#else
extern uint8_t Virtual_Uart_TXData;

#define SCOPE_TXEND_FLAG                       (0)
#define SCOPE_TXFIF0_NUM                       (0)
#define SCOPE_TX_DATA                          (Virtual_Uart_TXData)
#define SCOPE_RXFIF0_NUM                       (0)
#define SCOPE_RX_DATA                          (0)

#define MODBUS_TXEND_FLAG                      (0)
#define MODBUS_TXFIF0_NUM                      (0)
#define MODBUS_TX_DATA                         (Virtual_Uart_TXData)
#define MODBUS_RXFIF0_NUM                      (0)
#define MODBUS_RX_DATA                         (0)

#endif

void SCI_BufTx(uint8_t *pData, uint16_t Size);
void SCI_BufRx(uint8_t *pData, uint16_t size);

uint16_t SCI_GetBufRxNum(void);
uint16_t SCI_GetBufTxNum(void);
uint16_t SCI_GetBufRxbufflag(void);
void SCI_ClrRxBuf(void);
void SCI_ClrTxBuf(void);
void SCI_TxInquire(void);
uint8_t SCI_GetTxType(void);
void SCI_ErrClear(void);
void SCI_RxInquire(void);
void SetBaud(uint8_t Baud);
void SetStopBit(uint8_t StopBit);
void SetParity(uint8_t Parity);

void Scope_Init(uint8_t ch, uint16_t baud, uint8_t data_format);
void Scope_DmaSend(uint8_t * buff,uint16_t Len);

extern uint16_t  CrcCheck;
extern uint16_t  CrcCheck3;
extern uint16_t  CrcCheck5;

#endif
