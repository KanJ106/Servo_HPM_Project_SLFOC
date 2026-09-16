/*
 * SpdLoop.h
 *
 *  Created on: 2015-12-24
 *      Author: w
 *///

#ifndef SPDLOOP_H_
#define SPDLOOP_H_
#include "IQmathLib.h" 
typedef struct 	{
				  Uint16 SpdFbSw;		//����ѡ����0ʱΪ��ͨ���٣�1ʱʹ���ٶȹ۲�������
				  //void (*calc)();	    /* */
				} SPDLOOP;

typedef SPDLOOP *SPDLOOP_handle;
#define SPDLOOP_TYPE_DEFAULTS {0}
/*------------------------------------------------------------------------------
Prototypes for the functions in SpdLoop.c
------------------------------------------------------------------------------*/
extern void SpdLoop_Calc(void);
extern SPDLOOP SpdLoop;
#endif /* SPDLOOP_H_ */
