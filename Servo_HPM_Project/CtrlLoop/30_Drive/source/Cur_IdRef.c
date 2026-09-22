/*
 * Cur_IdRefCal.c
 *
 *  Created on: 2015-12-16//
 *      Author: w
 */
#include "IQmathLib.h"         /* Include header for IQmath library */
/* Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file */
#include "dataType.h"
#include "Drive.h"//
/******************************************************************************
 **函 数 名：
 **描    述：D轴电流给定计算
 **调    用：
 **输    入：
 **输    出：
 **返    回：无
 **其    它：无
 **日    期：2015-12-17
 *******************************************************************************/
void Cur_IdRef_init(CUR_IDREF *v)
{
	v->Ref = 0;
	v->IdRefGet = 0;
	v->IdRefInner = 0;
	v->IdComRef = _IQmpy((int16)DPI_IDRef*_IQ(0.001),DrvCoeff.MotIe);//d轴通用给定电流
}
void Cur_IdRef_rst(CUR_IDREF *v)
{
	v->Ref = 0;
	v->IdRefGet = 0;
	v->IdRefInner = 0;
	v->IdComRef = _IQmpy((int16)DPI_IDRef*_IQ(0.001),DrvCoeff.MotIe);
}


void Cur_IdRef_calc(CUR_IDREF *v)
{
#if FUNCTYPEN
	v->Ref = 0;
	if(v->IdRefGet == 1)//内部强制给定
		v->Ref = v->IdRefInner;
#endif
}


void IdRef_Calc(void)
{
	if(IdRef.IdRefGet == 0)
		IdRef.Ref = IdRef.IdComRef;
	else//内部强制给定
		IdRef.Ref = IdRef.IdRefInner;
}
