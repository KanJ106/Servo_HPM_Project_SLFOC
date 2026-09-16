/*
 * ToqLoop.h
 *
 *  Created on: 2015-12-24
 *      Author: w
 *///
#ifndef TOQLOOP_H_
#define TOQLOOP_H_
#include "IQmathLib.h" 
typedef struct 	{
					_iq tq;
					_iq iqfb;
					_iq idfb;
				} TOQLOOP;

typedef TOQLOOP *TOQLOOP_handle;
#define TOQLOOP_TYPE_DEFAULTS {0,0,0}
/*------------------------------------------------------------------------------
Prototypes for the functions in SpdLoop.c
------------------------------------------------------------------------------*/

extern void ToqLoop_Calc(void);
extern TOQLOOP ToqLoop;
#endif /* TOQLOOP_H_ */
