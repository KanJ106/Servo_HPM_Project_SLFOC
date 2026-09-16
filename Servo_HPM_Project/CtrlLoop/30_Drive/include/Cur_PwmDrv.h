/*
 * Cur_PwmDrv.h
 *
 *  Created on: 2016-1-5
 *      Author: rd0213
 *///

#ifndef CUR_PWMDRV_H_
#define CUR_PWMDRV_H_
#include "IQmathLib.h" 
typedef struct 	{
				  _iq svg_swTa;				//u相注入的电压占空比
				  _iq svg_swTb;				//v相注入的电压占空比
				  _iq svg_swTc;				//w相注入的电压占空比
                  void (*init)();
				  void (*rst)();
				  void (*calc)();	    /* */
				} CUR_PWMDRV;

typedef CUR_PWMDRV *CUR_PWMDRV_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_PWMDRV object.
-----------------------------------------------------------------------------*/
#define CUR_PWMDRV_DEFAULTS { \
/*svg_swTa               */0, \
/*svg_swTb               */0, \
/*svg_swTc               */0, \
  (void (*)(long)) Cur_PwmDrv_init ,\
						(void (*)(long)) Cur_PwmDrv_rst ,\
						(void (*)(long)) Cur_PwmDrv_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in Cur_PwmDrv.c
------------------------------------------------------------------------------*/
void Cur_PwmDrv_init(CUR_PWMDRV_handle);
void Cur_PwmDrv_rst(CUR_PWMDRV_handle);
void Cur_PwmDrv_calc(CUR_PWMDRV_handle);
extern void PwmDrv_Calc(void);
extern CUR_PWMDRV PwmDrv;
#endif /* CUR_PWMDRV_H_ */
