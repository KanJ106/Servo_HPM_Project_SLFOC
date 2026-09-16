/*
 * MultiSpd.h
 *
 *  Created on: 2016-1-13
 *      Author: rd0213
 *///

#ifndef MULTISPD_H_
#define MULTISPD_H_
#include "IQmathLib.h" 
typedef struct 	{
				int16 Segment;			//选择的速度片段记录值
				Uint16 cmpflg;			//走完所有速度段后标志位，为1说明一个来回完毕
                Uint32 Time;
				int32 cnt;				//每个速度片段运行时间计数器
				_iq OutPut;				//输出的当前速度指令值
				void (*init)();
				void (*rst)();
				void (*calc)();
				} MULTISPD;
typedef MULTISPD *MULTISPD_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the MULTISPD object.
-----------------------------------------------------------------------------*/
#define MULTISPD_DEFAULTS { \
/*Segment              */0, \
/*cmpflg               */0, \
/*Time                 */0, \
/*cnt                  */0, \
/*OutPut               */0, \
/*void (*init)()       */(void (*)(long))MultiSpd_init,\
/*void (*rst)()        */(void (*)(long))MultiSpd_rst,\
/*void (*calc)()       */(void (*)(long))MultiSpd_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions
------------------------------------------------------------------------------*/
void MultiSpd_init(MULTISPD_handle);
void MultiSpd_rst(MULTISPD_handle);
void MultiSpd_calc(MULTISPD_handle);
extern void MultiSpd_Calc(void);
extern MULTISPD MultiSpd;
#endif /* MULTISPD_H_ */
