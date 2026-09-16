/*
 * Pos_Ref.h
 *
 *  Created on:
 *      Author:
 *///

#ifndef POS_REF_H_
#define POS_REF_H_
#include "IQmathLib.h" 
struct HOMINGFLG_BITS {
    Uint16 Ok:1;							//原点回归完成标志，为1时回归完成
    Uint16 NotlimtFlg:1;					//限位开关是否有效,0时检测not，1时不检测not
    Uint16 PotlimtFlg:1;					//限位开关是否有效,0时检测pot，1时不检测pot
    Uint16 DyFlg:1;					    //减速为0时停止时间标志
    Uint16 PulseProhibit:1;				//脉冲禁止标志，为1时停止发脉冲，电机停止
    Uint16 Zcheck:1;					    //测试用
    Uint16 ZfindMask:1;					//测试用
    Uint16 TestBit2:1;					//测试用
    Uint16 rsd1:8;
};
union HOMINGFLG {
    Uint16                 all;
   struct HOMINGFLG_BITS    bit;
};
typedef struct {
	             union  HOMINGFLG HomingFlg;
	 	 	 	 _iq    Delta;						//原点回归加减速
	 	 	 	Uint16  method;						//原点回归方式
	 	 	 	Uint16  Step;						//原点回归步骤
	 	 	 	Uint16  innerStep;
	 	 	 	 Uint16 Cnt;						//回归过程使用的中间计数器
	 	 	 	 Uint32 Hcnt;						//回零超时计数器
	 	 	 	 Uint32 CntMax;						//1s钟延时计数器最大值
	 	 	 	 int32  HighSpeed;					//原点回归第一高速
	 	 	 	 int32  LowSpeed;					//原点回归第二低速
				 //int32  RecodPos;					//记录当前发出的位置指令值
				 int32  ThisTimePos;				//记录当前电机的位置
				 int32  ThisTimePos1;
				 int32 offsetpos;
				 int32  Out;
				 int32  Srem;
				 int32  Target;
				 int32  OutPut;
				 int32  Rem;
				 int64  ThisTimePosTotle;			//当前电机的位置n次求和值
				 Uint32 dyCnt;

	             Uint16 MultiTurn_HomeStep;

}HOMING;


struct ADJUSTFLG_BITS {
    Uint16 Ok:1;
    Uint16 NOTStOld:1;
    Uint16 POTStOld:1;
    Uint16 HOMStOld:1;
    Uint16 PulseProhibit:1;
    Uint16 TestBit:1;
    Uint16 TestBit1:1;
    Uint16 TestBit2:1;
};
union ADJUSTFLG {
    Uint16                 all;
   struct ADJUSTFLG_BITS    bit;
};

typedef struct {
	             union  ADJUSTFLG AdjustFlg;
	 	 	 	 _iq    Delta;
	 	 	 	Uint16  method;
	 	 	 	Uint16  Step;
	 	 	 	 Uint16 Cnt;
	 	 	 	 Uint16 CntMax;
	 	 	 	 int16  HighSpeed;
	 	 	 	 int16  LowSpeed;
				 int32  RecodPos;
				 int32  ThisTimePos;
				 int64  ThisTimePosTotle;
}SYSADJUST;

typedef struct {
				 Uint32 DeltaMax;				//位置环内最大转速对应的脉冲数，单位1024ppr
				 int32 PosrefDeltaMax;
				 int32 PosrefDeltaMin;
				 int32  PosRefTotle;			//位置指令总和
				 int32  PosNow;					//这次的位置指令
				 int32  Interpdata;				//插补值
                 int32  VelOffset;
				 int32  InterpdataRem;
                 int32  EcatRem;
				 int32  PosOld;					//上次的位置指令
				 int32  Delta;					//此次位置环调度内步进的位置指令
                 int32  AccDec;                 //增量脉冲
                 int32 TargetSpd;
				 HOMING Homing;					//原点回归
				 SYSADJUST SysAdjust;			//系统自整定 
                 Uint16 PosFlag;
                 int16  SYNCPeriodRatio;
                 Uint16 SyncCnt;
                 Uint16 SyncFlag;
				 void (*init)();
				 void (*rst)();
				 void (*offrst)();
                 void (*calc)();	// Pointer to calculation function
				 } POS_REF;

typedef POS_REF *POS_REF_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the POS_REF object.
-----------------------------------------------------------------------------*/
#define POS_REF_DEFAULTS { \
/*DeltaMax            */0, \
/*PosrefDeltaMax      */0, \
/*PosrefDeltaMin      */0, \
/*PosRefTotle         */0, \
/*PosNow              */0, \
/*Interpdata          */0, \
/*velOffset           */0, \
/*InterpdataRem       */0, \
/*EcatRem             */0, \
/*PosOld              */0, \
/*Delta               */0, \
/*AccDec              */0, \
/*TargetSpd           */0, \
/*Homing              */{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, \
/*SysAdjust           */{0,0,0,0,0,0,0,0,0,0,0}, \
/*PosFlag             */0,\
/*SYNCPeriodRatio     */0,\
/*SyncCnt             */0,\
/*SyncFlag            */0,\
/*(*init)()           */(void (*)(long))PosRefinit, \
/*(*rst)()            */(void (*)(long))PosRefrst, \
/*(*offrst)()         */(void (*)(long))PosRefOffrst, \
/*(*calc)()           */(void (*)(long))PosRefcalc}
/*------------------------------------------------------------------------------
Prototypes for the functions in Pos_Ref.c
------------------------------------------------------------------------------*/
void PosRefinit(POS_REF_handle);
void PosRefrst(POS_REF_handle);
void PosRefOffrst(POS_REF_handle);
void PosRefcalc(POS_REF_handle);
//extern void PosRef_Calc(void);
extern POS_REF	PosRef;
#endif /* POS_REF_H_ */
