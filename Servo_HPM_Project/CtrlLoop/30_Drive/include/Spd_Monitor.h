/*
 * Spd_Monitor.h
 *
 *  Created on: 2016-1-11
 *      Author: rd0213
 *///

#ifndef SPD_MONITOR_H_
#define SPD_MONITOR_H_
#include "IQmathLib.h" 
typedef struct 	{
				  int16 SpdArry[64];		//滑动平均滤波内存空间
				  Uint16 cnt;				//FIR滤波计数器
				  int32 Sum;				//FIR滤波总和寄存器
				  int16 SpdAvrg;			//速度平均值单位1rpm
				  int16 SpdAvrgOld;			//速度平均旧值单位1rpm
                  int16 SpdToTorCmd;
                  int16 SpdPiOut;
                  int16 TorqueRef;
                  int16 TorCoeffIQ16;
				  Uint16 OvSpdCnt;			//超速报警计数器
				  void (*init)();
				  void (*rst)();
				  void (*calc)();	    	/* */
				  void (*monit)();	    	/* */
				} SPD_MONITOR;

typedef SPD_MONITOR *SPD_MONITOR_handle;

#define SPD_MONITOR_DEFAULTS { \
/*SpdArry[64]             */{0}, \
/*cnt                     */0, \
/*Sum                     */0, \
/*SpdAvrg                 */0, \
/*SpdAvrgOld              */0, \
/*SpdToTorCmd             */0, \
/*SpdPiOut                */0, \
/*TorqueRef                */0, \
/*TorCoeffIQ16            */0, \
/*OvSpdCnt                */0, \
/*(*init)()               */(void (*)(long)) Spd_Monitor_init ,\
/*(*rst)()                */(void (*)(long)) Spd_Monitor_rst ,\
/*(*calc)()               */(void (*)(long)) Spd_Monitor_calc ,\
/*(*monit)()              */(void (*)(long)) Spd_Monitor1ms_calc}

void Spd_Monitor_init(SPD_MONITOR_handle);
void Spd_Monitor_rst(SPD_MONITOR_handle);
void Spd_Monitor_calc(SPD_MONITOR_handle);
void Spd_Monitor1ms_calc(SPD_MONITOR_handle);
extern void Spd_Monitor_Calc(void);
extern void Spd_Monitor1ms_Cacl(void);//1ms调度
extern SPD_MONITOR SpdMonitor;
#endif /* SPD_MONITOR_H_ */
