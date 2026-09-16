/*
 * PosLoop.h
 *
 *  Created on: 2015-12-24
 *      Author: w
 *///

#ifndef POSLOOP_H_
#define POSLOOP_H_
#include "IQmathLib.h"

struct POSLOOPFLG_BITS {
	Uint16 Clr:2;
	Uint16 RegClr:1;
	Uint16 PosModeFlg:1;
    Uint16 Rsvd1:12;
};

union POSLOOPFLG {
   Uint16                 all;
   struct POSLOOPFLG_BITS    bit;
};
typedef struct 	{
				  union POSLOOPFLG PosFlg;
				  _iq pos;	    /* */
				} POSLOOP;

typedef POSLOOP *POSLOOP_handle;
#define POSLOOP_TYPE_DEFAULTS {0,0}
/*------------------------------------------------------------------------------
Prototypes for the functions in PosLoop.c
------------------------------------------------------------------------------*/
extern void PosLoop_Calc(void);
extern POSLOOP PosLoop;
#endif /* POSLOOP_H_ */
