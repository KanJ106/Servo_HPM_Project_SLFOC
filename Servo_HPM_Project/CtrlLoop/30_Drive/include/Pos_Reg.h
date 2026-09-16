/*
 * PosReg.h
 *
 *  Created on:
 *      Author:
 *///

#ifndef POS_REG_H_
#define POS_REG_H_
#include "IQmathLib.h" 
typedef struct 	{
                  #if FLOAT_TYPE_USE
				  float SpdFwdCoffe;		//速度前馈计算测速基值
				  float Ka;					//速度前馈低通滤波器系数
                  float SpdFwdInLatch;		//速度前馈输入锁存
				  float SpdFwdOutLatch;		//速度前馈输出锁存 
                  float PosToSpdCoffe;
                  #else
  				  _iq SpdFwdCoffe;		    //速度前馈计算测速基值
				  _iq Ka;					//速度前馈低通滤波器系数
                  _iq SpdFwdInLatch;		//速度前馈输入锁存
				  _iq SpdFwdOutLatch;		//速度前馈输出锁存  
                  _iq PosToSpdCoffe;
                  #endif
				  int32 Pospd;				//齿轮比之后测速值，不经过后面指令平滑处理的，可作为位置指令对应的转速监控值
                  Uint16 Tao;				//速度前馈测速后一阶低通滤波器时间常数
                  int16 PospdDcm;			//齿轮比之后测速值单位rpm，不经过后面指令平滑处理的
				  int32 Ref;				//位置调节器输入的位置指令
				  int32 Fb;					//位置调节器反馈的位置
				  int32 Err;				//位置偏差Ref-Fb
                  int32 TotalErr;           //内外偏差之和
				  Uint16 PGain;				//位置P调节器增益
                  int16 SpdFwdRefDcm;		//速度前馈测速真实值单位rpm
				  int32 Pout;				//位置P调节器输出
				  int32 FwdOut;				//速度前馈输出
				  int32 Out;				//位置调节器总输出
				  int32 Max;				//位置调节器输出最大值，也就是最大速度指令给定值
				  int32 Min;				//位置调节器输出最小值，也就是最小速度指令给定值
				  int32 Coeff;				//位置环中间系数
				  void (*init)();
				  void (*rst)();
				  void (*update)();
				  void (*fwdcalc)();	    /* */
				  void (*calc)();	    	/* */
				} POS_REG;

typedef POS_REG *POS_REG_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the POS_REG object.
-----------------------------------------------------------------------------*/
#define POS_REG_DEFAULTS { \
/*PDeltaBase          */0, \
/*FK                  */0, \
/*SpdFwdOutLatch      */0, \
/*SpdFwdInLatch       */0, \
/*PosToSpdCoffe       */0, \
/*Pospd               */0, \
/*Tao                 */0, \
/*PospdDcm            */0, \
/*Ref                 */0, \
/*Fb                  */0, \
/*Err                 */0, \
/*Tatal Err           */0, \
/*PGain               */0, \
/*SpdFwdRefDcm        */0, \
/*Pout                */0, \
/*FwdOut              */0, \
/*Out                 */0, \
/*Max                 */0, \
/*Min                 */0, \
/*Coeff               */0, \
/*(*init)()           */(void (*)(long)) Pos_Reg_init ,\
/*(*rst)()            */(void (*)(long)) Pos_Reg_rst ,\
/*(*update)()         */(void (*)(long)) Pos_Reg_update ,\
/*(*fwdcalc)()        */(void (*)(long)) PosFwd_calc ,\
/*(*calc)()           */(void (*)(long)) Pos_Reg_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in Pos_Reg.c
------------------------------------------------------------------------------*/
void Pos_Reg_init(POS_REG_handle);
void Pos_Reg_rst(POS_REG_handle);
void Pos_Reg_update(POS_REG_handle);
void PosFwd_calc(POS_REG_handle);
void Pos_Reg_calc(POS_REG_handle);
extern void PosFwd_Calc(void);
extern void PosReg_Calc(void);
extern POS_REG PosReg;
#endif /* POS_REG_H_ */
