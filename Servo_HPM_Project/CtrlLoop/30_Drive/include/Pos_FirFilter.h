/*
 * Pos_FirFilter.h
 *
 *  Created on: 2016-5-12
 *      Author: rd0213
 *///

#ifndef POS_FIRFILTER_H_
#define POS_FIRFILTER_H_
#include "IQmathLib.h" 
typedef struct 	{
                  Uint16 Mask;				//位置指令平滑旁路选择，为1时旁路
                  Uint16 UnfullBuffCnt;     //数组填充计数
				  int32 in;					//位置平滑输入指令值
				  int32 Rem;				//位置平滑余数
				  int32 out;				//位置平滑输出指令值
				  int32 intotle;			//rsvd
				  int32 outotle;			//rsvd
				  int32 Detotle;			//累积误差
				  int32 Sum;				//fifo总和
				  int16 fK;					//滤波器系数
				  Uint16 cnt;				//指针计数器
				  Uint16 cntmax;			//指针计数器极限值
                  Uint16 cntdiv;
				  int32 infifo[1028];		//fir滤波器内存
				  void (*init)();
                  void (*rst)();
				  void (*clr)();
				  void (*calc)();	    /* */
				} POS_FIRFILTER;

typedef POS_FIRFILTER *POS_FIRFILTER_handle;

#define POS_FIRFILTER_DEFAULTS { \
/*Mask                      */0, \
/*UnfullBuffCnt             */0, \
/*in                        */0, \
/*Rem                       */0, \
/*out                       */0, \
/*intotle                   */0, \
/*outotle                   */0, \
/*Detotle                   */0, \
/*Sum                       */0, \
/*fK                        */0, \
/*cnt                       */0, \
/*cntmax                    */0, \
/*cntdiv                    */0, \
/*infifo[1024]              */{{0}},\
/*(*init)()                 */(void (*)(long)) Pos_Firflt_init ,\
/*(*rst)()                  */(void (*)(long)) Pos_Firflt_rst ,\
/*(*rst)()                  */(void (*)(long)) Pos_Firflt_clr ,\
/*(*calc)()                 */(void (*)(long)) Pos_Firflt_calc}

void Pos_Firflt_init(POS_FIRFILTER_handle);
void Pos_Firflt_rst(POS_FIRFILTER_handle);
void Pos_Firflt_clr(POS_FIRFILTER_handle);
void Pos_Firflt_calc(POS_FIRFILTER_handle);
extern POS_FIRFILTER PosFirflt;
#endif /* POS_FIRFILTER_H_ */
