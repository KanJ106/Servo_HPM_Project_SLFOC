#ifndef RSPI_H
#define RSPI_H
#include "r_cg_Project.h"
#include "MCU_Fw.h"

#if SERVO_MCU != NO_MCU

#else


#endif

void Panel_KeyInit(void);
uint16_t Panel_Key_RxTxInquire(uint16_t Txdata);
#endif

