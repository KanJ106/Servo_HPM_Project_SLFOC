#ifndef SV_MODBUSINIT_H
#define SV_MODBUSINIT_H
#include "r_cg_project.h"
#include "MCU_Fw.h"

#define ModbusSEL     HPM_UART15
#define ModbusSELCLK  clock_uart15 

#define  MODBUSFIFO_DEPTH   (32)

#define MODBUS_TXEND_FLAG                       (UART_LSR_TEMT_GET(ModbusSEL->LSR))
#define MODBUS_TXFIF0_NUM                       (UART_LSR_TFIFO_NUM_GET(ModbusSEL->LSR))
#define MODBUS_TX_DATA                          (ModbusSEL->THR)
#define MODBUS_RXFIF0_NUM                       (UART_LSR_RFIFO_NUM_GET(ModbusSEL->LSR))
#define MODBUS_ResetFIFO                        (ModbusSEL->FCRR |= UART_FCRR_RFIFORST_MASK)
#define MODBUS_RX_DATA                          (ModbusSEL->RBR)
#define MODBUS_R_STUATS                         (ModbusSEL->LSR)

void Modbus_Init(uint16_t baud,uint16_t data_format);
uint8_t Modbus_RxInquire(uint8_t *data,uint16_t *crc,uint8_t *flag);
uint8_t Modbus_TxInquire(uint8_t *data,uint16_t txCnt,uint16_t Len,uint16_t *Crc);
void Modbus_CheckErr(void);
#endif

