/*
 * Cur_PiGainAdpt.h
 *
 *  Created on:
 *      Author:
 *///
#ifndef CUR_PIGAINADPT_H_
#define CUR_PIGAINADPT_H_
#include "IQmathLib.h" 
typedef struct 	{
				  _iq Ud_z;		//直流实验输出电压
				  _iq Udc_z;	//直流实验母线电压
				  _iq Ud_z1;	//直流实验输出电压1
				  _iq Udc_z1;	//直流实验母线电压1
				  _iq Udc_ac;	//交流实验母线电压
				  _iq Id_z;		//直流实验输出电流
				  _iq Id_z1;	//直流实验输出电流1
				  _iq Rud;		//电压dFT实部
				  _iq Iud;		//电压dFT实部
				  _iq Rid;		//电流dFT实部
				  _iq Iid;		//电流dFT实部
				  float RsF;
				  _iq Rs;		//定子电阻pu
				  _iq Ld;		//d电感pu
				  _iq Lq;		//q电感pu
				  _iq kp;		//电流调节器比例增益原始输入
				  _iq ki;		//电流调节器积分增益原始输入
				  _iq frqpu;	//交流时间频率pu
				  Uint16 N;		//DFT点数
				  void (*RsLscalc)();
				  void (*PiGainAdptcalc)();	    /* */
				} CUR_PIGAINADPT;

typedef CUR_PIGAINADPT *CUR_PIGAINADPT_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_ACCFB object.
-----------------------------------------------------------------------------*/
#define CUR_PIGAINADPT_DEFAULTS { \
/*Ud_z                       */0, \
/*Udc_z                      */0, \
/*Ud_z1                      */0, \
/*Udc_z1                     */0, \
/*Udc_ac                     */0, \
/*Id_z                       */0, \
/*Id_z1                      */0, \
/*Rud                        */0, \
/*Iud                        */0, \
/*Rid                        */0, \
/*Iid                        */0, \
/*RsF                        */0, \
/*Rs                         */0, \
/*Ld                         */0, \
/*Lq                         */0, \
/*kp                         */0, \
/*ki                         */0, \
/*frqpu                      */0, \
/*N                          */0, \
							(void (*)(long)) RsLscalc, \
							(void (*)(long)) PiGainAdptcalc}

void RsLscalc(CUR_PIGAINADPT_handle);
void PiGainAdptcalc(CUR_PIGAINADPT_handle);
extern CUR_PIGAINADPT CurPiGainAdpt;
#endif /* CUR_PIGAINADPT_H_ */
