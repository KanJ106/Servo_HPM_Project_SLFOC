/*
* This source file is part of the EtherCAT Slave Stack Code licensed by Beckhoff Automation GmbH & Co KG, 33415 Verl, Germany.
* The corresponding license agreement applies. This hint shall not be removed.
*/

/**
 * \addtogroup CoE CAN Application Profile over EtherCAT
 * @{
 */

/**
\file coeappl.h
\author EthercatSSC@beckhoff.com

\version 5.12

<br>Changes to version V5.11:<br>
V5.12 COE4: add timestamp object (0x10F8) and update diagnosis handling<br>
V5.12 ECAT5: update Sync error counter/flag handling,check enum memory alignment depending on the processor,in case of a polled timer disable ESC interrupts during DC_CheckWatchdog<br>
<br>Changes to version V5.01:<br>
V5.11 ECAT10: change PROTO handling to prevent compiler errors<br>
<br>Changes to version - :<br>
V5.01 : Start file change log
 */

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#ifndef _COEAPPL_H_
#define _COEAPPL_H_

#include "objdef.h"
#include "cia402appl.h"
#include "r_cg_Project.h"//

/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/
#if SERVOTYPE !=SERVO_CANOPEN
          

#define PDOMAXINDEX  10

/**
 * \brief 0x1010 (Store Parameters) object structure
 */
typedef struct  {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT32   u32StoreAllPara; /**< \brief Store All Parameters */
   UINT32   u32StoreCommuPara; /**< \brief Store communication parameters*/
   UINT32   u32StoreCiA402Para; /**< \brief Store CiA402 parameters*/
   UINT32   u32StoreTC200Specific; /**< \brief Store EA200 specific parameters*/
}
TOBJ1010;
/**
 * \brief 0x1011 (ReStore Default Parameters) object structure
 */
typedef struct  {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT32   u32RStoreAllPara; /**< \brief Store All Parameters */
   UINT32   u32RStoreCommuPara; /**< \brief Store communication parameters*/
   UINT32   u32RStoreCiA402Para; /**< \brief Store CiA402 parameters*/
   UINT32   u32RStoreTC200Specific; /**< \brief Store TC200 specific parameters*/
}
TOBJ1011;

/*---------------------------------------------
-    0x1018
-----------------------------------------------*/
/**
* \brief 0x1018 (Identity) object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
    UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
    UINT32   u32VendorID; /**< \brief EtherCAT vendor ID */
    UINT32   u32Productcode; /**< \brief Product code*/
    UINT32   u32Revision; /**< \brief Revision number*/
    UINT32   u32Serialnumber; /**< \brief Serial number*/
} OBJ_STRUCT_PACKED_END
TOBJ1018;

/*---------------------------------------------
-    0x16XX
-----------------------------------------------*/
/** \brief 0x16XX data structure*/
typedef struct  {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT32   aEntries[PDOMAXINDEX];
//   UINT16   *pObjectData[PDOMAXINDEX];
//   UINT16   BitLength[PDOMAXINDEX];
}
TOBJ16XX;

/*---------------------------------------------
-    0x1AXX
-----------------------------------------------*/
/** \brief 0x1AXX data structure*/
typedef struct  {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT32   aEntries[PDOMAXINDEX];//���ڴ洢
//   UINT16   *pObjectData[PDOMAXINDEX];
//   UINT16   BitLength[PDOMAXINDEX];
}
TOBJ1AXX;

typedef struct  {
   UINT16   IndexNum;   /**< \brief SubIndex 0*/
   UINT16   *pObjectData[((PDOMAXINDEX << 2) + 2)];
   UINT16   BitLength[((PDOMAXINDEX << 2) + 2)];
}
TOBJPDOBUFF;

/** @}*/
/*---------------------------------------------
-    0x1C00
-----------------------------------------------*/
/**
* \brief 0x1C00 (Sync manager type) object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
    UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
/*ECATCHANGE_START(V5.12)*/
    UINT8   aEntries[4]; /**< \brief Entry buffer (one for two SyncManagers)*/
/*ECATCHANGE_END(V5.12)*/
} OBJ_STRUCT_PACKED_END
TOBJ1C00;

/**
 * \addtogroup SmAssignObjects SyncManager Assignment Objects
 * SyncManager 2 : 0x1C12<br>
 * SyncManager 3 : 0x1C13
 * @{
 */
/** \brief 0x1C12 (SyncManager 2 assignment) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT16   aEntries[4]; /**< \brief Entry buffer (number of entries depends on the supported axis, each axis requires one entry)*/
} OBJ_STRUCT_PACKED_END
TOBJ1C12;


/** \brief 0x1C13 (SyncManager 3 assignment) data structure*/
typedef struct OBJ_STRUCT_PACKED_START {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT16   aEntries[4]; /**< \brief Entry buffer (number of entries depends on the supported axis, each axis requires one entry)*/
} OBJ_STRUCT_PACKED_END
TOBJ1C13;


#define CHECK_DC_OVERRUN_IN_MS      4000 /**<\brief cycle time in ms to check the DC system time for overrun*/


typedef struct
{
    UINT32 u32Devicetype;         /**< \brief 0x1000 (Device type)*/
    UINT16 u16ErrorRegister;      /**< \brief 0x1001 (Error Register)*/
    CHAR acDevicename[6];         /**< \brief 0x1008*/
    CHAR acHardwareversion[6];    /**< \brief 0x1009 (Hardware version) variable to handle the object data*/
    CHAR acSoftwareversion[12];   /**< \brief 0x100A*/
    TOBJ1018 sIdentity;           /**< \brief 0x1018*/
    TOBJ10F1 sErrorSettings;      /**< \brief 0x10F1*/
    
    #ifdef OBJ0x10F8
    UINT64   u64Timestamp;        /**< \brief 0x10F8*/
    #endif // #ifdef OBJ0x10F8
	
    TOBJ16XX sRxPDOMap0;          /**< \brief RxPDO (0x1600)*/
    TOBJ16XX sRxPDOMap1;          /**< \brief RxPDO (0x1601)*/
    TOBJ16XX sRxPDOMap2;          /**< \brief RxPDO (0x1602)*/
    TOBJ16XX sRxPDOMap3;          /**< \brief RxPDO (0x1603)*/

    TOBJ1AXX sTxPDOMap0;          /**< \brief TxPDO (0x1A00)*/
    TOBJ1AXX sTxPDOMap1;          /**< \brief TxPDO (0x1A01)*/
    TOBJ1AXX sTxPDOMap2;          /**< \brief TxPDO (0x1A02)*/
    TOBJ1AXX sTxPDOMap3;          /**< \brief TxPDO (0x1A03)*/

    TOBJ1C00 sSyncmanagertype;    /**< \brief 0x1C00*/
    TOBJ1C12 sRxPDOassign;        /**< \brief 0x1C12*/
    TOBJ1C13 sTxPDOassign;        /**< \brief 0x1C13*/
    TSYNCMANPAR MBXMEM  sSyncManOutPar;/**< \brief 0x1C32*/
    TSYNCMANPAR MBXMEM  sSyncManInPar; /**< \brief 0x1C33*/
}
CommunicationObjects;
#endif

#ifdef _COEAPPL_

#if SERVOTYPE == SERVO_ETHERCAT
/*---------------------------------------------
-    0x1000
-----------------------------------------------*/
/**
 * \brief 0x1000 (Device type) entry description
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x1000 = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ};
/**
 * \brief 0x1000 (Device type) object name
 */
OBJCONST UCHAR OBJMEM aName0x1000[] = "Device type";

/*---------------------------------------------
-    0x1001
-----------------------------------------------*/
/**
 * \brief 0x1001 (Error Register) entry description
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x1001 = {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ};

/**
 * \brief 0x1001 (Error Register) object name
 */
OBJCONST UCHAR OBJMEM aName0x1001[] = "Error register";

/*---------------------------------------------
-    0x1008
-----------------------------------------------*/
/**
 * \brief 0x1008 (Device name) entry description
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x1008 = {DEFTYPE_VISIBLESTRING, BYTE2BIT(DEVICE_NAME_LEN), ACCESS_READ};
/**
 * \brief 0x1008 (Device name) object name
 */
OBJCONST UCHAR OBJMEM aName0x1008[] = "Device name";

/*---------------------------------------------
-    0x1009
-----------------------------------------------*/

/**
 * \brief 0x1009 (Hardware version) entry description
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x1009 = {DEFTYPE_VISIBLESTRING,BYTE2BIT(DEVICE_HW_VERSION_LEN), ACCESS_READ};

/**
 * \brief 0x1009 (Hardware version) object name
 */
OBJCONST UCHAR OBJMEM aName0x1009[] = "Hardware version";


/*---------------------------------------------
-    0x100A
-----------------------------------------------*/
/**
 * \brief 0x100A (Software version) entry description
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x100A = {DEFTYPE_VISIBLESTRING,BYTE2BIT(DEVICE_SW_VERSION_LEN), ACCESS_READ};

/**
 * \brief 0x100A (Software version) object name
 */
OBJCONST UCHAR OBJMEM aName0x100A[] = "Software version";

/*---------------------------------------------
-    0x1010
-----------------------------------------------*/
/**
 * \brief 0x1010 (Identity) variable to handle the object data
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1010[] = {
   {DEFTYPE_UNSIGNED8, 0x8,   ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}};
OBJCONST UCHAR OBJMEM aName0x1010[] = "Store Parameters\000Store all parameters\000Store communication parameters\000Store CiA402 parameters\000Store TC200E specific parameters\000\377";

/*---------------------------------------------
-    0x1011
-----------------------------------------------*/
/**
 * \brief 0x1011 (Store Parameters) variable to handle the object data
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1011[] = {
   {DEFTYPE_UNSIGNED8, 0x8,   ACCESS_READ},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE},
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}};
OBJCONST UCHAR OBJMEM aName0x1011[] = "ReStore Default Parameters\000Restore default parameters\000Restore communication default parameters\000Restore CiA402 parameters\000Restore TC200E Specific parameters\000\377";


/*---------------------------------------------
-    0x1018
-----------------------------------------------*/
/**
 * \brief 0x1018 (Identity) entry descriptions
 * 
 * Subindex 000
 * SubIndex 001: Vendor ID
 * SubIndex 002: Product code
 * SubIndex 003: Revision
 * SubIndex 004: Serial number
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1018[] = {
   {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ },
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, 
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, 
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, 
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}};

/**
 * \brief 0x1018 (Identity) object and entry names
 */
OBJCONST UCHAR OBJMEM aName0x1018[] = "Identity\000Vendor ID\000Product code\000Revision number\000Serial number\000\377";

/*---------------------------------------------
-    0x10F1
-----------------------------------------------*/
/**
 * \brief 0x10F1 (Error settings) entry description
 * Subindex 000
 * SubIndex 001: Local Error Reaction
 * SubIndex 002: Sync Error Counter Limit
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x10F1[] = {
   {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ },
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, 
   {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE}};

/**
 * \brief 0x10F1 (Error settings) object and entry names
 */   
OBJCONST UCHAR OBJMEM aName0x10F1[] = "Error Settings\000Local Error Reaction\000Sync Error Counter Limit\000\377";


//object declaration and initialization in objdef.h

#ifdef OBJ0x10F8
/*ECATCHANGE_START(V5.12) COE4*/
/*---------------------------------------------
-    0x10F8 (Timestamp object)
-----------------------------------------------*/
/**
* \brief 0x10F8 (Timestamp object) entry description
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x10F8 = { DEFTYPE_UNSIGNED64, 0x40, ACCESS_READWRITE | OBJACCESS_TXPDOMAPPING } ;

/**
* \brief 0x10F8 (Timestamp object) object name
*/
OBJCONST UCHAR OBJMEM aName0x10F8[] = "Timestamp Object\000\377";
#endif //#ifdef OBJ0x10F8

/**
 * \brief Object 0x16XX (RxPDO) entry descriptions
 */
OBJCONST TSDOINFOENTRYDESC	OBJMEM asEntryDesc0x16XX[] = {
   {DEFTYPE_UNSIGNED8, 0x8,   ACCESS_READWRITE}, /* Subindex 000 */
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 001*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 002*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 003*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 004*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 005*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 006*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 007*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 008*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 009*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}}; /* SubIndex 010*/

/**
 * \brief Object 0x16XX (RxPDO) object and entry names
 *
 * In this example no specific entry name is defined ("SubIndex xxx" is used)
 */
OBJCONST UCHAR OBJMEM aName0x1600[] = "1st Receive PDO Mapping\000\377";
OBJCONST UCHAR OBJMEM aName0x1601[] = "2st Receive PDO Mapping\000\377";
OBJCONST UCHAR OBJMEM aName0x1602[] = "3st Receive PDO Mapping\000\377";
OBJCONST UCHAR OBJMEM aName0x1603[] = "4st Receive PDO Mapping\000\377";

/**
 * \brief Object 0x1AXX (TxPDO) entry descriptions
 */
OBJCONST TSDOINFOENTRYDESC	OBJMEM asEntryDesc0x1AXX[] = {
   {DEFTYPE_UNSIGNED8, 0x8,   ACCESS_READWRITE}, /* Subindex 000 */
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 001*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 002*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 003*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 004*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 005*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 006*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 007*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 008*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 009*/
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}}; /* SubIndex 010*/

/**
 * \brief Object 0x1Axx (TxPDO) object and entry names
 *
 * In this example no specific entry name is defined ("SubIndex xxx" is used)
 */
OBJCONST UCHAR OBJMEM aName0x1A00[] = "1st Transmit PDO Mapping\000\377";
OBJCONST UCHAR OBJMEM aName0x1A01[] = "2st Transmit PDO Mapping\000\377";
OBJCONST UCHAR OBJMEM aName0x1A02[] = "3st Transmit PDO Mapping\000\377";
OBJCONST UCHAR OBJMEM aName0x1A03[] = "4st Transmit PDO Mapping\000\377";

/*---------------------------------------------
-    0x1C00
-----------------------------------------------*/

/**
 * \brief 0x1C00 (Sync manager type) variable to handle the object data
 *
 * SI 0 : 4 (4 SyncManagers available)
 * SI1 (SM0): Mailbox Out (0x1)
 * SI2 (SM1): Mailbox In (0x2)
 * SI3 (SM2): Process data Out (0x3)
 * SI4 (SM3): Process data In (0x4)
 *
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1C00[] = {
   {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ},
   {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ}};

/**
 * \brief 0x1C00 (Sync manager type) object name
 */
OBJCONST UCHAR OBJMEM aName0x1C00[] = "Sync manager type";

/////** \brief Object 0x1C10 (RxPDO(SM0) Assignment) entry description*/
//OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1C10[] = {DEFTYPE_UNSIGNED8, 0x08, (ACCESS_READ|OBJACCESS_NOPDOMAPPING)};
//OBJCONST UCHAR OBJMEM aName0x1C10[] = "RxPDO(SM0) Assignment";
//
/////** \brief Object 0x1C11 (Control Word) entry description*/
//OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1C11[] = {DEFTYPE_UNSIGNED8, 0x08, (ACCESS_READ|OBJACCESS_NOPDOMAPPING)};
//OBJCONST UCHAR OBJMEM aName0x1C11[] = "RxPDO(SM1) Assignment";

/**
 * \brief Entry descriptions of SyncManager assign objects
 *
 * SubIndex0<br>
 * SubIndex1 (for all other entries the same description will be used (because the object code is ARRAY))
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asPDOAssignEntryDesc[] = {
   {DEFTYPE_UNSIGNED8,  0x08, (ACCESS_READ|ACCESS_WRITE_PREOP)},
   {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READ|ACCESS_WRITE_PREOP)},
   {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READ|ACCESS_WRITE_PREOP)},
   {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READ|ACCESS_WRITE_PREOP)},
   {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READ|ACCESS_WRITE_PREOP)},
   };

   
/**
 * \brief Object 0x1C12 (Sync Manager 2 assign) object and entry names
 *
 * In this example no specific entry name is defined ("SubIndex xxx" is used)
 */
OBJCONST UCHAR OBJMEM aName0x1C12[] = "RxPDO(SM2) Assignment\000\377";

/**
 * \brief Object 0x1C13 (Sync Manager 3 assign) object and entry names
 *
 * In this example no specific entry name is defined ("SubIndex xxx" is used)
 */
OBJCONST UCHAR OBJMEM aName0x1C13[] = "TxPDO(SM3) Assignment\000\377";

/*---------------------------------------------
-    0x1C3x
-----------------------------------------------*/
/**
 * \brief 0x1C3x (SyncManager parameter) entry description
 *
 * The entry descriptions are used for 0x1C32 and 0x1C33
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1C3x[] = {
   {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ }, /* Subindex 000 */
   {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READ | ACCESS_WRITE_PREOP)}, /* SubIndex 001: Synchronization Type */
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, /* SubIndex 002: Cycle Time */
   {0x0000, 0x20, 0}, /* SubIndex 003: Shift Time (not supported)*/
   {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ}, /* SubIndex 004: Synchronization Types supported */
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, /* SubIndex 005: Minimum Cycle Time */
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, /* SubIndex 006: Calc and Copy Time */
   {0x0000, 0x20, 0}, /* Subindex 007 Minimum Delay Time (not supported)*/
   {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE}, /* SubIndex 008: Get Cycle Time */
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, /* SubIndex 009: Delay Time */
   {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}, /* SubIndex 010: Sync0 Cycle Time */
   {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ}, /* SubIndex 011: SM-Event Missed */
   {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ}, /* SubIndex 012: Cycle Time Too Small */
   {0x0000, 0x10, 0}, /* SubIndex 013: Shift Too Short Counter (not supported)*/
   {0x0000, 0x10, 0}, /* Subindex 014: RxPDO Toggle Failed (not supported)*/
   {0x0000, 0x20, 0}, /* Subindex 015: Minimum Cycle Distance (not supported)*/
   {0x0000, 0x20, 0}, /* Subindex 016: Maximum Cycle Distance (not supported)*/
   {0x0000, 0x20, 0}, /* Subindex 017: Minimum SM Sync Distance (not supported)*/
   {0x0000, 0x20, 0}, /* Subindex 018: Maximum SM Sync Distance (not supported)*/
   {0x0000, 0, 0}, /* Subindex 019 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 020 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 021 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 022 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 023 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 024 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 025 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 026 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 027 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 028 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 029 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 030 doesn't exist */
   {0x0000, 0, 0}, /* Subindex 031 doesn't exist */
   {DEFTYPE_BOOLEAN, 0x01, ACCESS_READ}}; /* SubIndex 032: Sync error */

/**
 * \brief 0x1C32 (SyncManager 2 parameter) object and entry names
 */
OBJCONST UCHAR OBJMEM aName0x1C32[] = "SM input parameter\000Sync mode\000Cycle Time\000\000Sync mode supported\000Min cycle time\000Calc and copy time\000\000Get Cycle Time\000Delay Time\000Sync0 time\000SM-Event Missed\000Cycle Time Too Small\000Cycle Time Too Small\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000Sync error\000\377";

/**
 * \brief 0x1C33 (SyncManager 3 parameter) object and entry names
 */
OBJCONST UCHAR OBJMEM aName0x1C33[] = "SM output parameter\000Sync mode\000Cycle Time\000\000Sync mode supported\000Min cycle time\000Calc and copy time\000\000Get Cycle Time\000Delay Time\000Sync0 time\000SM-Event Missed\000Cycle Time Too Small\000Cycle Time Too Small\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000Sync error\000\377";

#endif   //#ifdef _COEAPPL_

#endif  //#if SERVOTYPE == SERVO_ETHERCAT
/*-----------------------------------------------------------------------------------------
------
------    Global variables
------
-----------------------------------------------------------------------------------------*/

#if defined(_COEAPPL_) && (_COEAPPL_ == 1)
    #define PROTO
#else
    #define PROTO extern
#endif

#ifdef OBJ0x10F8
/*ECATCHANGE_START(V5.12) COE4*/
PROTO UINT64 u64Timestamp
#ifdef _COEAPPL_
= { 0 }
#endif
;
/*ECATCHANGE_END(V5.12) COE4*/
#endif //#ifdef OBJ0x10F8

#if SERVOTYPE !=SERVO_CANOPEN
PROTO CommunicationObjects       CumObj;
#endif

#if SERVOTYPE == SERVO_ETHERCAT
PROTO UINT32 u32LastDc32Value;
PROTO UINT32 u32CheckForDcOverrunCnt;
PROTO CommunicationObjects const DefCommuObjectsValues;//
//PROTO UINT16 * ODStandardTable0x1000[];
/*-----------------------------------------------------------------------------------------
------
------    Global functions
------
-----------------------------------------------------------------------------------------*/
/*ECATCHANGE_START(V5.12) ECAT5*/
PROTO void COE_UpdateSyncErrorStatus(void);
/*ECATCHANGE_END(V5.12) ECAT5*/

PROTO void COE_ObjInit(void);
PROTO void COE_Main(void);
PROTO UINT16 COE_ObjDictionaryInit(void);
PROTO UINT16 COE_AddObjectToDic(TOBJECT OBJMEM * pNewObjEntry,UINT16 Index);
PROTO void COE_RemoveDicEntry(UINT16 index);
PROTO void COE_ClearObjDictionary(void);
PROTO OBJCONST TOBJECT OBJMEM * COE_GetObjectDictionary(UINT16 index);
PROTO TOBJECT OBJMEM * COE_GetObjectDictionaryRAM(UINT16 index);
PROTO UINT16 AddObjectsToObjDictionary(TOBJECT OBJMEM * pObjEntry);

#ifdef OBJ0x10F8
/*ECATCHANGE_START(V5.12) COE4*/
PROTO void COE_SyncTimeStamp(void);
/*ECATCHANGE_END(V5.12) COE4*/
#endif //#ifdef OBJ0x10F8

PROTO UINT8 Read0x10F8(UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM * pData, UINT8 bCompleteAccess);
#endif //#if SERVOTYPE == SERVO_ETHERCAT

#undef PROTO
/** @}*/

#endif //_COEAPPL_H_
