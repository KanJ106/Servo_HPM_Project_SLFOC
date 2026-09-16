/*
 * FtVfTest.h
 *
 *  Created on: 2016-4-12
 *      Author: rd0213
 *///

#ifndef FTVFTEST_H_
#define FTVFTEST_H_
#include "IQmathLib.h" 

typedef struct 	{
					_iq frqref;//目标频率
					_iq deltafrq;//加速度
					_iq fout;//输出频率
					_iq foutpre;//输出频率128times
					_iq vout;//输出电压
					_iq v_fcoeff;//频率电压系数
                    Uint8 FirstFlag;
				} FTVFTEST;

typedef FTVFTEST *FTVFTEST_handle;
/*-----------------------------------------------------------------------------
Default initalizer
-----------------------------------------------------------------------------*/
#define FTVFTEST_DEFAULTS {     \
/*frqref                   */0, \
/*deltafrq                 */0, \
/*fout                     */0, \
/*foutpre                  */0, \
/*vout                     */0, \
/*v_fcoeff                 */0, \
/*FirstFlag                */0};

extern FTVFTEST Ftvftst;
extern void ftvftst_init(void);
extern void ftvftst_rst(void);
extern void ftvftst_update(void);
extern void ftvftst_calc(void);
extern FTVFTEST Ftvftst;
#endif /* FTVFTEST_H_ */
