#ifndef __HPM6E__CAN_HEADER__
#define __HPM6E__CAN_HEADER__
#include <stdint.h>//
#include <assert.h>
#include "board.h"
#include "hpm_mcan_drv.h"
#include "r_cg_Project.h"//
/******************************************************************************
Functions Prototypes
**
#include "r_cg_Project.h"//****************************************************************************/ 
 
/* CAN Message definitions ------------------------------------------------- */
#if SERVOTYPE == SERVO_CAN

//mqb CANFD调试
#if HARDWARE_VER_SEL == HARDWARE_VER_0
#define CANSelect      HPM_MCAN4
#define CAN_IRQn       IRQn_MCAN4
#elif HARDWARE_VER_SEL == HARDWARE_VER_1
#define CANSelect      HPM_MCAN0
#define CAN_IRQn       IRQn_MCAN0
#endif
//mqb CANFD调试

typedef union udata {
    uint32_t    LONG;
    uint16_t    Word[2];
    uint8_t     BYTE[4];
} udata_t;

void CanInit(uint16_t Baud1, uint16_t Baud2, uint16_t Id,uint16_t CanMode);
void CanInit1(uint16_t Baud1, uint16_t Baud2, uint16_t Id,uint16_t Canmode);
extern hpm_stat_t MCAN_Read_Rxfifo(MCAN_Type *ptr, mcan_rx_message_t *rx_frame);
extern hpm_stat_t MCAN_Read_Rxbuff(MCAN_Type *ptr, uint32_t index, mcan_rx_message_t *rx_frame);
extern hpm_stat_t MCAN_Tranmit(MCAN_Type *ptr, mcan_tx_frame_t *tx_frame);

extern bool hpm_flag;
extern bool Radio_flag;
extern uint32_t elem_addr;
extern volatile uint32_t txfifo_addr;
extern volatile uint32_t rxbuff_addr;
extern volatile uint32_t rxfifo_addr;

extern void test_gpio_toggle(void);
extern mcan_rx_message_t s_can_rx_buf;
extern mcan_tx_frame_t tx_buf;

#endif

#endif /* __RZT1__CAN_HEADER__ */


/* End of File */
