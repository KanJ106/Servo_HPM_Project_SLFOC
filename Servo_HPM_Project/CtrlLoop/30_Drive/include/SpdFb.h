/*
 * SpdFb.h
 *
 *  Created on: 2016-1-7
 *      Author: rd0213
 *///

#ifndef SPDFB_H_
#define SPDFB_H_
#include "Filter.h"
#include "PosFb.h"
#include "IQmathLib.h" 
struct SPDFBFLG_BITS {
	Uint16 FilterType:2;				//最终速度来源选择，0时为一阶低通后的结果，1时为二阶低通滤波后的结果
	Uint16 SpdCalType:2;				//观测器测速还是一般测速，默认一般测速
	Uint16 Lpf2ordRunFlg:1;			//二阶低通测速运行标志
	Uint16 Lpf1ordRunFlg:1;			//一阶低通测速运行标志
	Uint16 Rsvd1:10;
};

union SPDFBFLG {
   Uint16                 all;
   struct SPDFBFLG_BITS    bit;
};
typedef struct {
				 union SPDFBFLG SpdFbFlg;
                 Uint16  Tsamp;						//测速周期单位0.01us
				 Uint16  Tao;						//滤波时间常数
				 int16  SpdFbDecm;					//测速十进制值单位1rpm
                 Uint32  PosFbDeltaMotMax;			//测速基值，测速周期内基值转速对应的脉冲数，单位1000ppr
#if FLOAT_TYPE_USE
                 float  FK;							//测速滤波系数
#else
                 _iq  FK;							//测速滤波系数
#endif				 
                 int32  PosFbDelta;					//测速周期内的脉冲数，delta(pos)
				 _iq  Fb_1ordlpf;					//一阶低通滤波器测速结果
				 _iq  Fb;							//速度值，标幺
				 _iq  FbPre;						//未滤波的转速
				 _iq  Wfb;							//测速rad/s，标幺
                 _iq  FbFilterLim;                  //滤波输出最小值				 
				 LPF2ORD SpdLpf2ord;				//测速二阶低通滤波
				 void (*init)();
				 void (*rst)();
				 void (*update)();
				 void (*calc)();	// Pointer to calculation function
				 } SPDFB;

typedef SPDFB *SPDFB_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SPDFB object.
-----------------------------------------------------------------------------*/
#define SPDFB_DEFAULTS {   \
/*SpdFbFlg            */0, \
/*Tsamp               */0, \
/*Tao                 */0, \
/*SpdFbDecm           */0, \
/*PosFbDeltaMotMax    */0, \
/*FK                  */0, \
/*PosFbDelta          */0, \
/*Fb_1ordlpf          */0, \
/*Fb                  */0, \
/*FbPre               */0, \
/*Wfb                 */0, \
/*FbFilterLim         */0, \
/*SpdLpf2ord          */LPF2ORD_DEFAULTS,\
/*(*init)()           */(void (*)(long))SpdFbinit, \
/*(*rst)()            */(void (*)(long))SpdFbrst, \
/*(*update)()         */(void (*)(long))SpdFbupdate, \
/*(*calc)()           */(void (*)(long))SpdFbcalc}
/*------------------------------------------------------------------------------
Prototypes for the functions in SpdFb.c
------------------------------------------------------------------------------*/
void SpdFbinit(SPDFB_handle);
void SpdFbrst(SPDFB_handle);
void SpdFbupdate(SPDFB_handle);
void SpdFbcalc(SPDFB_handle);

void PosFbDeltacalc(POSFB *v);

extern POSFB SpdPosFb;
extern SPDFB SpdFb;
#endif /* SPDFB_H_ */
