/*
 * Cur_Clark.c
 *
 *  Created on: 2015-12-16//
 *      Author: w
 */


#include "Drive.h"
#define SQRT3 (9686330) //1/SQRT3//
/******************************************************************************
 **函 数 名：
 **描    述：三相静止坐标系到两相静止坐标系变换
 **调    用：
 **输    入：
 **输    出：
 **返    回：无
 **其    它：无
 **日    期：2015-12-17
 *******************************************************************************/
void Cur_clarke_rst(CUR_CLARKE *v)
{
	v->As = 0;
	v->Bs = 0;
	v->Alpha = 0;
	v->Beta = 0; //
}

void Cur_clarke_calc(CUR_CLARKE *v)
{
#if FUNCTYPEN
    v->Alpha = v->As;
    v->Beta = _IQmpy(v->As + (v->Bs<<1),_IQ(0.57735026918963)); //
#endif
}

void clark(void)
{
	Clark.Alpha = Clark.As;
	//Clark.Beta = _IQmpy(Clark.As + (Clark.Bs<<1),_IQ(0.57735026918963)); //
    Clark.Beta = _IQmpy(Clark.As + (Clark.Bs << 1),9686330L); //
}




