/*
 * SV_IncEncode.h
 *
 *  Created on: 2016-11-5
 *      Author: yyl
 */
#ifndef __SV_INCENCODE_H__
#define __SV_INCENCODE_H__
#include "IQmathLib.h"

////////////////////////////////////////////////////////////////////////////
typedef struct
{
    Uint16 State;
    Uint16 OffsetAngle;
    int32  UVWOffsetPulse;
} CUR_UVWETHETA;

extern CUR_UVWETHETA Hall;

#define Get_UvwState()   (Hall.State = GET_HALLSTATUS()) 

void Hall_Encoder1_Init(void);
uint8_t GET_HALLSTATUS(void);
#endif
