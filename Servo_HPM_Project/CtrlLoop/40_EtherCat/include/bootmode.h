/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/

#ifndef _BOOTMODE_H_
#define _BOOTMODE_H_
#include "ecat_def.h"
#include "r_cg_Project.h"//


#endif //_BOOTMODE_H_

#if SERVOTYPE == SERVO_ETHERCAT

#if defined(_BOOTMODE_) && (_BOOTMODE_ == 1)
    #define PROTO
#else
    #define PROTO extern
#endif

/*-----------------------------------------------------------------------------------------
------
------    Global variables
------
-----------------------------------------------------------------------------------------*/

#define ID_VENDOR	(0)
#define ID_PRODUCT	(1)
#define ID_REVISION	(2)
#define ID_SERIAL	(3)

/*-----------------------------------------------------------------------------------------
------
------    Global functions
------
-----------------------------------------------------------------------------------------*/
PROTO   void BL_Start( UINT8 State);
PROTO   void BL_Stop(void);
PROTO   void BL_StartDownload(UINT32 password);
PROTO   UINT16 BL_Data(UINT16 *pData,UINT16 Size);
PROTO   UINT16 BL_CheckSum(void);
PROTO   void BL_SetRebootFlag(BOOL Flag);
PROTO   BOOL BL_CheckRebootFlag(void);
PROTO   void BL_Reboot(void);
#undef PROTO

#endif

/** @}*/
