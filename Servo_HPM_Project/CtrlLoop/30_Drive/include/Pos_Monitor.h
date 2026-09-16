/*
 * Pos_Monitor.h
 *
 *  Created on: 2016-1-18
 *      Author: rd0213
 *///

#ifndef POS_MONITOR_H_
#define POS_MONITOR_H_
#include "IQmathLib.h" 
typedef struct 	{
				  Uint16 frqcnt;			//脉冲频率测量使用的计数器
				  Uint16 frqmax;			//脉冲频率测量计数器最大值
                  
                  int16  PosCmd;
                  int16  PosCmdFilter;
                  
                  int16  PosFwdcmd;
                  int16  rsd;
                  
				  Uint32 frqvalue;			//脉冲频率，单位0.01Khz
				  int32 PosOld;				//上次的位置指令值
				  void (*init)();
				  void (*rst)();
				  void (*monit)();	    	/* */
				  void (*monit1ms)();	    /* */
				} POS_MONITOR;

typedef POS_MONITOR *POS_MONITOR_handle;

#define POS_MONITOR_DEFAULTS {  \
/*frqcnt                   */0, \
/*frqmax                   */0, \
/*PosCmd                   */0, \
/*PosCmdFilter             */0, \
/*PosFwdcmd                */0, \
/*rsd                      */0, \
/*frqvalue                 */0, \
/*PosOld                   */0, \
/*(*init)()                */(void (*)(long)) Pos_Monitor_init ,\
/*(*rst)()                 */(void (*)(long)) Pos_Monitor_rst ,\
/*(*monit)()               */(void (*)(long)) Pos_Monitor_calc ,\
/*(*monit1ms)()            */(void (*)(long)) Pos_Monitor1ms_calc}

void Pos_Monitor_init(POS_MONITOR_handle);
void Pos_Monitor_rst(POS_MONITOR_handle);
void Pos_Monitor_calc(POS_MONITOR_handle);
void Pos_Monitor1ms_calc(POS_MONITOR_handle);
void Pos_Monitor_updata(POS_MONITOR_handle);

void Pos_Monitor_Calc(void);//位置环调度
extern void Pos_Monitor1ms_Calc(void);
extern POS_MONITOR PosMonitor;


#endif /* POS_MONITOR_H_ */
