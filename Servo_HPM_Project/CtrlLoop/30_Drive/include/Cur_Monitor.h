/*
 * Cur_Monitor.h
 *
 *  Created on: 2016-1-11
 *      Author: rd0213
 *///

#ifndef CUR_MONITOR_H_
#define CUR_MONITOR_H_
#include "IQmathLib.h" 
typedef struct 	{
				  _iq IeeValuPu;						//电机电流Imag值
				  _iq IeeValuDispPu;					//经过滤波器后的电机电流Imag值
                  int16 CoeffIQ16;
				  Uint16 IeeValuDisp;					//电机电流有效值单位0.01A
				  int16 TorqRatsDisp;					//电机转矩百分比，单位0.1% 虑波后显示用
				  int16 TorqRatsDispS;					//电机转矩百分比，单位0.1% 虑波前反馈主站
                  int16 TorqRatsRef;					//电机目标转矩百分比，单位0.1%有符号
                  int16 TorqFbId;                       //电机D轴电流
				  Uint16 cnt;							//软件过流判断计数器
                  Uint16 rsv;							//rst
				  void (*init)();
				  void (*rst)();
				  void (*calc)();
				  void (*monit)();
				} CUR_MONITOR;

typedef CUR_MONITOR *CUR_MONITOR_handle;

#define CUR_MONITOR_DEFAULTS { \
/*IeeValuPu               */0, \
/*IeeValuDispPu           */0, \
/*CoeffIQ30               */0, \
/*IeeValuDisp             */0, \
/*TorqRatsDisp            */0, \
/*TorqRatsDispS           */0, \
/*TorqRatsRef             */0, \
/*TorqFbId                */0, \
/*cnt                     */0, \
/*rst                     */0, \
        (void (*)(long)) Cur_Monitor_init ,\
		(void (*)(long)) Cur_Monitor_rst ,\
		(void (*)(long)) Cur_Monitor_calc ,\
		(void (*)(long)) Cur_Monitor1ms_calc}

void Cur_Monitor_init(CUR_MONITOR_handle);
void Cur_Monitor_rst(CUR_MONITOR_handle);
void Cur_Monitor_calc(CUR_MONITOR_handle);
void Cur_Monitor1ms_calc(CUR_MONITOR_handle);
extern void Monitor_Calc(void);
extern CUR_MONITOR CurMonitor;
#endif /* CUR_MONITOR_H_ */
