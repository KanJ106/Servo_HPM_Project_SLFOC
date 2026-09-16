/*
 * ecat_escinit.h
 *
 *  Created on: 2023��12��19��
 *      Author: YH
 */

#ifndef _ECAT_ESCINIT_H_
#define _ECAT_ESCINIT_H_
#include "stdint.h"
#include <string.h>//

#ifndef ESCMEM
#define ESCMEM 
#endif

//
// ESC Register Offsets
//
#define ESC_O_DL_STATUS      0x0110U
#define ESC_O_PDI_CONTROL    0x0140U
#define ESC_O_AL_EVENTMASK   0x0204U
#define ESC_O_EEPROM_STATUS  0x0502U

//
// ESC Register Masks
//
#define ESC_DL_STATUS_EEPROMLOAD_M     0x1U
#define ESC_EEPROM_STATUS_EEPROMLOAD_M 0x1000U

//
// ESC Clock Config
//
#define ESC_DISABLE_INT_PHY_CLK     0x0U
#define ESC_USE_INT_PHY_CLK         0x1U

//
// Defines for Success and Failure returns during hardware initialization
//
#define ESC_HW_INIT_SUCCESS  0x0U
#define ESC_HW_INIT_FAIL     0xFFFFU

//
// Defines to EEPROM Loaded Status
//
#define ESC_EEPROM_SUCCESS    0x0U
#define ESC_EEPROM_NOT_LOADED 0xFFFFU
#define ESC_EEPROM_LOAD_ERROR 0xFFFEU


#define ESC_BASE   (0x50000UL)
#define ESC_M_LSB  (0xFF00)
#define ESC_M_MSB  (0x00FF)

#endif /* _ECAT_ESCINIT_H_ */

#define _ECAT_ESCINIT_ 0
#if _ECAT_ESCINIT_
    #define PROTO
#else
    #define PROTO extern
#endif

PROTO uint16_t *pEsc;            // pointer to the ESC 
/*-----------------------------------------------------------------------------------------
------
------    function prototypes
------
-----------------------------------------------------------------------------------------*/

PROTO uint16_t HW_GetTimer(void);
PROTO void HW_ClearTimer(void);

#define     ECAT_TIMER_INC_P_MS      1 //the timer unit 107ns per tick (75MHz/8)
#define     ESC_RD                    0x02            /* read access to ESC */
#define     ESC_WR                    0x04            /* write access to ESC */

PROTO       void HW_SetLed(uint8_t RunLed,uint8_t ErrLed);
PROTO       uint16_t HW_Init(void);
PROTO       void HW_Release(void);

#define     HW_GetALEventRegister()                    ((((uint16_t ESCMEM *)pEsc)[((0x0220)>>1)]))
#define     HW_GetALEventRegister_Isr()                ((((uint16_t ESCMEM *)pEsc)[((0x0220)>>1)]))

#define     HW_EscRead(pData,Address,Len)              memcpy((uint8_t *)(pData), &((uint8_t ESCMEM *) pEsc)[Address], (Len))
#define     HW_EscReadIsr(pData,Address,Len)           memcpy((uint8_t *)(pData), &((uint8_t ESCMEM *) pEsc)[Address], (Len))
#define     HW_EscReadDWord(DWordValue, Address)       ((DWordValue) = (UINT32)(((UINT32 *)pEsc)[((Address)>>2)]))
#define     HW_EscReadDWordIsr(DWordValue, Address)    ((DWordValue) = (UINT32)(((UINT32 *)pEsc)[((Address)>>2)]))
#define     HW_EscReadWord(WordValue, Address)         ((WordValue) = (((uint16_t *)pEsc)[((Address)>>1)]))    //the esc pointer is handled as a word pointer so the address must be devided by 2
#define     HW_EscReadWordIsr(WordValue, Address)      ((WordValue) = (((uint16_t *)pEsc)[((Address)>>1)]))    //the esc pointer is handled as a word pointer so the address must be devided by 2
#define     HW_EscReadMbxMem(pData,Address,Len)        memcpy((uint8_t *)(pData), &((uint8_t ESCMEM *) pEsc)[((Address))], (Len))

#define     HW_EscWrite(pData,Address,Len)             memcpy(&((uint8_t ESCMEM *) pEsc)[((Address))], (uint8_t *)(pData), (Len))
#define     HW_EscWriteIsr(pData,Address,Len)          memcpy(&((uint8_t ESCMEM *) pEsc)[((Address))], (uint8_t *)(pData), (Len))
#define     HW_EscWriteDWord(DWordValue, Address)      ((((UINT32 *)pEsc)[((Address)>>2)]) = (DWordValue))
#define     HW_EscWriteDWordIsr(DWordValue, Address)   ((((UINT32 *)pEsc)[((Address)>>2)]) = (DWordValue))
#define     HW_EscWriteWord(WordValue, Address)        ((((uint16_t *)pEsc)[((Address)>>1)]) = (WordValue))    //the esc pointer is handled as a word pointer so the address must be devided by 2
#define     HW_EscWriteWordIsr(WordValue, Address)     ((((uint16_t *)pEsc)[((Address)>>1)]) = (WordValue))    //the esc pointer is handled as a word pointer so the address must be devided by 2
#define     HW_EscWriteMbxMem(pData,Address,Len)       memcpy(&((uint8_t ESCMEM *) pEsc)[((Address))],(uint8_t *)(pData), (Len))
 
PROTO void DISABLE_ESC_INT(void);
PROTO void ENABLE_ESC_INT(void);

PROTO uint16_t HW_EepromReload(void);
uint16_t ecat_eeprom_emulation_read(uint32_t wordaddr);
uint16_t ecat_eeprom_emulation_write(uint32_t wordaddr);
uint16_t ecat_eeprom_emulation_reload(void);
void ecat_eeprom_emulation_store(void);
void ecat_Eeprom_Process(void);
#undef PROTO
