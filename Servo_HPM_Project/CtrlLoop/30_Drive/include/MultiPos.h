/*
 * MultiPos.h
 *
 *  Created on:
 *      Author:
 *///

#ifndef MULTIPOS_H_
#define MULTIPOS_H_
#include "IQmathLib.h" 
struct PMUL_BITS {
    Uint16 PcmpFlg:1;
    Uint16 PcomeinFlg:1;
    Uint16 Rsvd1:14;
};

union PMUL {
    Uint16              all;
   struct PMUL_BITS    bit;
};
typedef struct 	{
	              union PMUL Pmul;			//中间过渡标志位
	              Uint16 Segment;			//选择的位置片段记录值
				  int32 Output;				//规划后的位置指令值，单位为1ppr
				  int32 OutOld;				//上一次规划的位置指令值
				  int32 Delta;				//此次位置调度周期内步进的位移量
				  int32 Totle;				//位置指令规划器总共发出的位置指令数
				  Uint32 DelayCnt;			//每个位置片段走完后停留的时间计数器
				  Uint32 DelayMax;			//每个位置片段走完后停留的时间计数器计数最大值
				  int32  Pos[18];			//实际16段位置值
                  int32  Spd[18];			//实际16段位置值
                  Uint32  dey[18];			//实际16段位置值
				  int32  timcoeff;			//时间系数
				  int32  delta;				//加速度
				  int32  decdelta;			//减速度
                  int32  Accdec;            //减减速2
                  int32  Scopedelta;		//加速度
				  int32  Scopedecdelta;		//减速度
                  Uint16 TwoSegment;
				  void (*init)();
				  void (*rst)();
				  void (*update)();
				  void (*calc)();	    /* */
				} MULTIPOS;

typedef MULTIPOS *MULTIPOS_handle;

#define MULTIPOS_DEFAULTS {  \
/*Pmul                  */0, \
/*Segment               */0, \
/*Output                */0, \
/*OutOld                */0, \
/*Delta                 */0, \
/*Totle                 */0, \
/*DelayCnt              */0, \
/*DelayMax              */0, \
/*Pos[16]               */{0}, \
/*Pos[16]               */{0}, \
/*Pos[16]               */{0}, \
/*timcoeff              */0, \
/*delta                 */0, \
/*decdelta              */0, \
/*Accdec                */0, \
/*Scopedelta            */0, \
/*Scopedecdelta         */0, \
/*TwoSegment            */0, \
/*void (*init)()        */(void (*)(long)) MultiPos_init, \
/*void (*rst)()         */(void (*)(long)) MultiPos_rst, \
/*void (*update)()      */(void (*)(long)) MultiPos_update,\
/*void (*calc)()        */(void (*)(long)) MultiPos_calc }

void MultiPos_init(MULTIPOS_handle);
void MultiPos_rst(MULTIPOS_handle);
void MultiPos_update(MULTIPOS_handle);
void MultiPos_calc(MULTIPOS_handle);
int32 MultiDelataPos_Calc(Uint16 Mode);
extern void MultiPos_Calc(void);
extern void MultiPos2_Calc(void);
extern MULTIPOS MultiPos;
#endif /* MULTIPOS_H_ */
