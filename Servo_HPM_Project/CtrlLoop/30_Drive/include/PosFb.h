/*
 * PosFb.h
 *
 *  Created on:
 *      Author:
 *///

#ifndef POSFB_H_
#define POSFB_H_
#include "IQmathLib.h" 
struct POSFBFLG_BITS {
	Uint16 FirstPosFbComeCntFlg:6;
	Uint16 PosFbTotleClrFlg:1;
	Uint16 AbsEncFS:1;
	Uint16 Rsvd1:8;
};

union POSFBFLG {
   Uint16                 all;
   struct POSFBFLG_BITS    bit;
};
typedef struct {
				 union POSFBFLG PosFbFlg;
				 Uint32  EncSinglePosOld;			//编码器位置旧值
				 int32  PosFbTotle;					//编码器转动的总位置
                 int32  ScopeFbTotle;               //对外输出编码器总位置
				 int32  PosFbDelta;					//此次调度周期内的位置变化值
				 int32  PosFbDeltaMax;				//位置变化值正限幅
				 int32  PosFbDeltaMin;				//位置变化值负限幅
				 Uint16 Tsamp;						//调度周期单位0.01us		//z信号出现标志
				 void (*init)();
				 void (*rst)();
				 void (*calc)();					// Pointer to calculation function
				 } POSFB;

typedef POSFB *POSFB_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the POSFB object.
-----------------------------------------------------------------------------*/
#define POSFB_DEFAULTS { \
/*PosFbFlg          */0, \
/*EncSinglePosOld   */0, \
/*PosFbTotle        */0, \
/*ScopeFbTotle      */0, \
/*PosFbDelta        */0, \
/*PosFbDeltaMax     */0, \
/*PosFbDeltaMin     */0, \
/*Tsamp             */0, \
/*(*init)()         */(void (*)(long))PosFbinit, \
/*(*rst)()          */(void (*)(long))PosFbrst, \
/*(*calc)()         */(void (*)(long))PosFbcalc}
/*------------------------------------------------------------------------------
Prototypes for the functions in PosFb.c
------------------------------------------------------------------------------*/
void PosFbinit(POSFB_handle);
void PosFbrst(POSFB_handle);
void PosFbcalc(POSFB_handle);
extern POSFB PosFb;

typedef struct {
				 Uint32  EncSinglePosOld;					//编码器位置旧值
				 int32  PosFbDelta;							//此次调度周期内的位置变化值
				 int32  PdeltaRcd;
				 int32  PosFbDeltaMax;						//位置变化值正限幅
				 int32  PosFbDeltaMin;						//位置变化值负限幅
				 Uint16 Tsamp;								//调度周期单位0.01us
				 Uint16 Zflg;								//z信号出现标志
				 Uint16 Cnt;
				 void (*init)();
				 void (*rst)();
				 void (*calc)();	// Pointer to calculation function
				 } ZCHECK;

typedef ZCHECK *ZCHECK_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the ZCHECK object.
-----------------------------------------------------------------------------*/
#define ZCHECK_DEFAULTS {  \
/*EncSinglePosOld     */0, \
/*PosFbDelta          */0, \
/*PdeltaRcd           */0, \
/*PosFbDeltaMax       */0, \
/*PosFbDeltaMin       */0, \
/*Tsamp               */0, \
/*Zflg                */0, \
/*Cnt                 */0, \
/*(*init)()           */(void (*)(long))Zcheck_init, \
/*(*rst)()            */(void (*)(long))Zcheck_rst, \
/*(*calc)()           */(void (*)(long))Zcheck_calc}
/*------------------------------------------------------------------------------
Prototypes for the functions in PosFb.c
------------------------------------------------------------------------------*/
void Zcheck_init(ZCHECK_handle);
void Zcheck_rst(ZCHECK_handle);
void Zcheck_calc(ZCHECK_handle);
extern ZCHECK Zcheck;
extern void Zcheck_Calc(void);
#endif /* POSFB_H_ */
