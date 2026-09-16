#ifndef _TC200NOBJDIC_H_
#define _TC200NOBJDIC_H_
#include "r_cg_project.h"
/*-----------------------------------------------------------------------------------------
------
------    Includes
------//
-----------------------------------------------------------------------------------------*/
#include "objdef.h"
#include "ecat_def.h"
#include "ecatslv.h"
#include "ecatappl.h"
#include "coeappl.h"//

//最大可以显示14组
#define PFPASSWORDOBJ 0x313F

#define OBJLISTNUM 15
#define OBJLISTGROUP {0x1000,0x6000,0x4000,0x3000,0x3100,0x3400,0x3500,0x3600,0x3700,0x3800,0x3900,0x3C00,0x3D00,0x3e00,0x3f00};

extern UINT16 indexNum[OBJLISTNUM];
extern UINT16 Readsize;
extern UINT16 Objlinkedlist;//代表当前显示多少组，主要进行PE组和PF组的操作
extern UINT16 readgroup;
extern UINT16 indexlist[OBJLISTNUM];

#if SERVOTYPE == SERVO_ETHERCAT
void APPL_FanCodeObjInit(void);
#endif
#endif
