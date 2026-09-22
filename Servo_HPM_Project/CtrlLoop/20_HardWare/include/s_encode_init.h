#ifndef S_ENCODE_INIT_H
#define S_ENCODE_INIT_H
#include "r_cg_project.h"
#include "MCU_Fw.h"

#if SERVO_MCU == HPM_6E00
#if   HARDWARE_VER_SEL == HARDWARE_VER_0
#define EncodeSCI  HPM_UART1
#elif HARDWARE_VER_SEL == HARDWARE_VER_1
#define EncodeSCI  HPM_UART2
#else
#define EncodeSCI  HPM_UART1
#endif

#define EncodeSCI_TxData     (EncodeSCI->THR)
#define EncodeSCI_FIFO_NUM   (UART_LSR_RFIFO_NUM_GET(EncodeSCI->LSR))
#define EncodeSCI_RxData     (EncodeSCI->RBR & UART_RBR_RBR_MASK)
#define EncodeSCI_ResetFIFO  (EncodeSCI->FCRR |= UART_FCRR_RFIFORST_MASK)
#define EncodeSCI_R_STUATS   (EncodeSCI->LSR)
#else


#endif


extern void EncodeSci_Main(void);
#if defined(SENSORLESS_CANOPEN_BUILD)
#include "SensorlessEncoderMonitor.h"
extern volatile SL_ENCODER_MONITOR g_sl_encoder;
void SensorlessEncoder_Init(uint16_t type);
void SensorlessEncoder_Service1ms(uint16_t drive_active);
#endif
extern uint8_t  UartRxuff[22];
#endif
