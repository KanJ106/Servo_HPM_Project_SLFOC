/*
 * Cur_Ipark.c
 *
 *  Created on: 2015-12-16//
 *      Author: w
 */
#include "IQmathLib.h"         // Include header for IQmath library
// Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file
#include "dataType.h"
#include "Cur_Ipark.h"
#include "Drive.h"//
/******************************************************************************
 **函 数 名：
 **描    述：反park变换
 **调    用：
 **输    入：
 **输    出：
 **返    回：无
 **其    它：无
 **日    期：2015-12-17
 *******************************************************************************/
void Cur_Ipark_rst(CUR_IPARK *v)
{
	v->Alpha = 0;
	v->Beta = 0;
	v->Ds = 0;
	v->Qs = 0;
}

void Cur_Ipark_calc(CUR_IPARK *v)
{
#if FUNCTYPEN
   _iq Cosine,Sine;

// Using look-up IQ sine table
     Sine = _IQsinPU(v->Angle);
     Cosine = _IQcosPU(v->Angle);

     v->Alpha = _IQmpy(v->Ds,Cosine) - _IQmpy(v->Qs,Sine);
     v->Beta = _IQmpy(v->Qs,Cosine) + _IQmpy(v->Ds,Sine);
#endif
}

_iq IparkqRefLim(CUR_IQPIREG *vq,CUR_UDFWD   *ud)
{
    _iq qs,out;
    
    qs = vq->Out + ud->qOut;
    
    if(qs < vq->OutMin)
    {
        out = vq->OutMin;
        vq->PiSatFlg = 2;//负向饱和
    }
    else if(qs > vq->OutMax)
    {
        out = vq->OutMax;
        vq->PiSatFlg = 1;//正向饱和
    }
    else
    {
        out = qs;
        vq->PiSatFlg = 0;//正向饱和
    }
    
    return out;
}

_iq IparkdRefLim(CUR_IDPIREG *vd,CUR_UDFWD   *ud)
{
    _iq ds,out;
    
    ds = vd->Out + ud->dOut;
    
    if(ds < vd->OutMin)
    {
        out = vd->OutMin;
        vd->PiSatFlg = 2;//负向饱和
    }
    else if(ds > vd->OutMax)
    {
        out = vd->OutMax;
        vd->PiSatFlg = 1;//正向饱和
    }
    else
    {
        out = ds;
        vd->PiSatFlg = 0;//正向饱和
    }
    
    return out;
}

void ipark_Calc(_iq Cosine,_iq Sine)
{
	Ipark.Alpha = _IQmpy(Ipark.Ds,Cosine) - _IQmpy(Ipark.Qs,Sine);
	Ipark.Beta = _IQmpy(Ipark.Qs,Cosine) + _IQmpy(Ipark.Ds,Sine);
}
