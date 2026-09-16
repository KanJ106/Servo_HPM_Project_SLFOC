#ifndef CMT_H
#define CMT_H
#include "r_cg_Project.h"
#include "MCU_Fw.h"

#if SERVO_MCU == HPM_6E00
typedef struct
{
    uint32_t TimerCount; 
    uint32_t ReadCount;
    uint32_t CANisrCount;
}TestGptmr ;

extern TestGptmr  mqbtest;
void CheckTimeInit(void);
void TestTimer_Start(void);
void TestTimer_Stop(void);
uint32_t ReadCount(void);
void ResetCount(void);
#else


#endif

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_CMT0_Create(void);
void R_CMT0_Start(void);
void R_CMT0_Stop(void);
#endif
