#ifndef R_CG_SYS_H
#define R_CG_SYS_H
#include <stdint.h>
#include "r_cg_Project.h"
#include "MCU_Fw.h"

#if SERVO_MCU != NO_MCU
#define EWDG_CNT_CLK_FREQ 32768UL
#else


#endif

void Ewdg_Init(void);
void EnableDog(void);
void DisableDog(void);
void ServiceDog(void);
void ResetCPU(void);
#endif


