#ifndef MAIN_H_
#define MAIN_H_
#include "IQmathLib.h" 

extern uint16_t SysFtest;        
extern uint16_t Glo_IIcReadAll;

#if SERVOTYPE == SERVO_ETHERCAT

void EctDcCyc_Cacl(void);
void EcatDcOffsetCacl(void);
void EcatDcCalibCacl(void);

extern uint16_t EctDcRefer;
extern int16_t  EctDcAdjustPmax;
extern int16_t  EctDcAdjustNmax;
extern int16_t  EctDcOffset;
extern int16_t  EctDcOffsetRemder;
extern int16_t  EctDcOffsetime;
#endif

#if SERVOTYPE == SERVO_CAN
extern uint16_t SyncRefer;
extern int16_t  SyncAdjustPmax;
extern int16_t  SyncAdjustNmax;
extern int16_t  SyncOffset;
extern int16_t  SyncOffsetRemder;
extern int16_t  SyncOffsetime;
extern volatile uint8_t SnycCmdFlag;
extern volatile uint8_t SnycFlag;

extern void CanSync_init(void);
extern void CanSyncOffsetCacl(void);
extern void CanSyncCalibCacl(void);
#endif


#endif