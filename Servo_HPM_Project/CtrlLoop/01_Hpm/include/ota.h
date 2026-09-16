/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/* this file for eeprom emulation component */

#ifndef OTA_H
#define OTA_H

#include "board.h"//

#define DISABLE_GLOBAL_INTERRUPT() disable_global_irq(CSR_MSTATUS_MIE_MASK)
#define ENABLE_GLOBAL_INTERRUPT() enable_global_irq(CSR_MSTATUS_MIE_MASK)

#define START_BANK1_ADDR       (0x80100000)
#define FOE_BANK1_ADDR_FLAG    (0x80000)
#define FOE_BANK1_ADDR         (0x100000)
#define FOE_BANK1_SIZE         (0x80000)
#define BL_WRITE_BUFFER_SIZE   (1024)
#define FOE_BANK1_STATUS_FLAG  (0xA55A)
#ifdef __cplusplus
extern "C" {
#endif

extern char OtaNameHeader[16];

int8_t foe_support_init(void);
int8_t foe_support_prepare_data_space(uint32_t Addres, uint32_t Len);
int8_t foe_support_store_data(uint32_t Addres, uint32_t * data, uint32_t size);
int8_t foe_support_read_data_space(uint32_t Addres, uint8_t * data,uint32_t Len);
void foe_support_soc_reset(void);
void Bank_Sel_Cacl(void);
uint16_t Foe_EraseBank1(void);
uint16_t Foe_EraseBank1_1(void);
uint16_t Foe_EraseBank1_2(void);
uint16_t Foe_Data_Write(uint8_t *pData, uint16_t Size, uint32_t *CodeSum);
uint16_t Foe_CheckSum(uint32_t Size);
uint16_t Foe_WriteBank1Flag(void);
void BL_Ota_Init(uint32_t Code);
#ifdef __cplusplus
}
#endif

#endif /* FOE_SUPPORT_H */