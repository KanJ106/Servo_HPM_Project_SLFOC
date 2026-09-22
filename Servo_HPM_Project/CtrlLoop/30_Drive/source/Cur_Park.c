/*
 * Cur_Park.c
 *
 *  Created on: 2015-12-16//
 *      Author: w
 */
#include "Drive.h"//
/******************************************************************************
 **函 数 名：
 **描    述：两相静止坐标系-两相旋转坐标系变换
 **调    用：
 **输    入：
 **输    出：
 **返    回：无
 **其    它：无
 **日    期：2015-12-17
 *******************************************************************************/
void Cur_Park_rst(CUR_PARK *v)
{
	v->Alpha = 0;
	v->Beta = 0;
	v->Ds = 0;
	v->Qs = 0;
}


void Cur_Park_calc(CUR_PARK *v)
{
#if FUNCTYPEN
    _iq Cosine,Sine;

// Using look-up IQ sine table
     Sine = _IQsinPU(v->Angle);
     Cosine = _IQcosPU(v->Angle);

     v->Ds = _IQmpy(v->Alpha,Cosine) + _IQmpy(v->Beta,Sine);
     v->Qs = _IQmpy(v->Beta,Cosine) - _IQmpy(v->Alpha,Sine);
#endif
}


void park(_iq Cosine,_iq Sine)
{
    Park.Ds = _IQmpy(Park.Alpha,Cosine) + _IQmpy(Park.Beta,Sine);
    Park.Qs = _IQmpy(Park.Beta,Cosine)  - _IQmpy(Park.Alpha,Sine);
}
