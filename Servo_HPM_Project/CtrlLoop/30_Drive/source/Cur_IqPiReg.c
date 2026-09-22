/*
 * Cur_IdPiReg.c
 *
 *  Created on: 2015-12-16//
 *      Author: w
 */
#include "drive.h"

#define PCHGPOINT _IQmpy(_IQ(0.10),DrvCoeff.MotIe)//增益补偿点
#define PCHGPOINTSQ _IQmpy(PCHGPOINT,PCHGPOINT)
#define KPMINRATS _IQ(1.0)//kP补偿最小倍数
#define KPMAXRATS _IQ(1.3)//kP补偿最大倍数
#define QKPCHGEN 0         //Kp分段使能,参考台达伺服做法//

//#define PCHGPONT1 _IQmpy(_IQ(0.25),DrvCoeff.MotIe)		//25%
//#define PCHGPONT2 _IQmpy(_IQ(0.5),DrvCoeff.MotIe)		//50%
//#define PCHGPONT3 _IQmpy(_IQ(0.75),DrvCoeff.MotIe)		//75%
//#define PCHGPONT4 _IQmpy(_IQ(1.00),DrvCoeff.MotIe)		//100%
//#define PCHGPONT5 _IQmpy(_IQ(1.25),DrvCoeff.MotIe)		//125%
//#define PCHGPONT6 _IQmpy(_IQ(1.5),DrvCoeff.MotIe)		//150%
//#define PCHGPONT7 _IQmpy(_IQ(1.75),DrvCoeff.MotIe)		//175%
//#define PCHGPONT8 _IQmpy(_IQ(2.00),DrvCoeff.MotIe)		//200%
//#define PCHGPONT9 _IQmpy(_IQ(2.25),DrvCoeff.MotIe)		//225%
//#define PCHGPONT10 _IQmpy(_IQ(2.5),DrvCoeff.MotIe)		//250%
//#define PCHGPONT11 _IQmpy(_IQ(2.75),DrvCoeff.MotIe)		//275%
//#define PCHGPONT12 _IQmpy(_IQ(3.00),DrvCoeff.MotIe)		//300%

//#define PCHGPONT1 _IQmpy(4194304L,DrvCoeff.MotIe)		//25%
//#define PCHGPONT2 _IQmpy(8388608L,DrvCoeff.MotIe)		//50%
//#define PCHGPONT3 _IQmpy(12582912L,DrvCoeff.MotIe)		//75%
//#define PCHGPONT4 _IQmpy(16777216L,DrvCoeff.MotIe)		//100%
//#define PCHGPONT5 _IQmpy(20971520L,DrvCoeff.MotIe)		//125%
//#define PCHGPONT6 _IQmpy(25165824L,DrvCoeff.MotIe)		//150%
//#define PCHGPONT7 _IQmpy(29360128L,DrvCoeff.MotIe)		//175%
//#define PCHGPONT8 _IQmpy(33554432L,DrvCoeff.MotIe)		//200%
//#define PCHGPONT9 _IQmpy(37748736L,DrvCoeff.MotIe)		//225%
//#define PCHGPONT10 _IQmpy(41943040L,DrvCoeff.MotIe)		//250%
//#define PCHGPONT11 _IQmpy(46137344L,DrvCoeff.MotIe)		//275%
//#define PCHGPONT12 _IQmpy(50331648L,DrvCoeff.MotIe)		//300%

_iq QPCHGPONT1 = 0;
_iq QPCHGPONT2 = 0;
_iq QPCHGPONT3 = 0;
_iq QPCHGPONT4 = 0;
_iq QPCHGPONT5 = 0;
_iq QPCHGPONT6 = 0;
_iq QPCHGPONT7 = 0;
_iq QPCHGPONT8 = 0;
_iq QPCHGPONT9 = 0;
_iq QPCHGPONT10 = 0;
_iq QPCHGPONT11 = 0;
_iq QPCHGPONT12 = 0;

uint8_t testperr = 0;
#define QKPCHG1EN 1//Kp分段使能,参考台达伺服做法

#define ICHGPONT1 _IQmpy(_IQ(0.25),DrvCoeff.MotIe)		//25%
#define ICHGPONT2 _IQmpy(_IQ(0.5),DrvCoeff.MotIe)		//50%
#define ICHGPONT3 _IQmpy(_IQ(0.75),DrvCoeff.MotIe)		//75%
#define ICHGPONT4 _IQmpy(_IQ(1.00),DrvCoeff.MotIe)		//100%
#define ICHGPONT5 _IQmpy(_IQ(1.25),DrvCoeff.MotIe)		//125%
#define ICHGPONT6 _IQmpy(_IQ(1.5),DrvCoeff.MotIe)		//150%
#define ICHGPONT7 _IQmpy(_IQ(1.75),DrvCoeff.MotIe)		//175%
#define ICHGPONT8 _IQmpy(_IQ(2.00),DrvCoeff.MotIe)		//200%
#define ICHGPONT9 _IQmpy(_IQ(2.25),DrvCoeff.MotIe)		//225%
#define ICHGPONT10 _IQmpy(_IQ(2.5),DrvCoeff.MotIe)		//250%
#define ICHGPONT11 _IQmpy(_IQ(2.75),DrvCoeff.MotIe)		//275%
#define ICHGPONT12 _IQmpy(_IQ(3.00),DrvCoeff.MotIe)		//300%
#define QKICHGEN 0//Ki分段使能

#define KCOFFE _IQmpyI32(_IQ(0.01),DPI_ACR_AntiwindupTi)
/******************************************************************************
 **函 数 名：
 **描    述：Q轴电流调节器设计
 **调    用：
 **输    入：
 **输    出：
 **返    回：无
 **其    它：无
 **日    期：2015-12-17
 *******************************************************************************/
void Cur_IqPiReg_init(CUR_IQPIREG *v)
{
	v->SatErr = 0;
	v->GainSw = 0;

	if(v->GainSw == 0)
	{
//		if(DrvCoeff.Lq != 0 && DrvCoeff.Rs != 0)
//		{
//			CurPiGainAdpt.Lq = DrvCoeff.Lq >> 1;
//			CurPiGainAdpt.Rs = DrvCoeff.Rs >> 1;
//			CurPiGainAdpt.PiGainAdptcalc(&CurPiGainAdpt);
//			v->Kp = _IQmpy(CurPiGainAdpt.kp,_IQ(0.5));
//			v->Ki = _IQmpy(CurPiGainAdpt.ki,_IQ(0.5));
//		}
//		else
//		{
//			v->Kp = _IQmpy(_IQ11toIQ(DPI_ACR_Pd),_IQ(0.5));
//			v->Ki = _IQmpy(_IQ16toIQ(DPI_ACR_Id),_IQ(0.5));
//		}
         
        if(DPI_ACR_Pd > 0)
        {
            v->Kp = _IQmpy(_IQ11toIQ(DPI_ACR_Pd),_IQ(0.5));
        }
        else
        {
            v->Kp = _IQ(0.1);
        }
        
        if(DPI_ACR_Id > 0)
        {
            v->Ki = _IQmpy(_IQ16toIQ(DPI_ACR_Id),_IQ(0.5));   
        }
        else
        {
            v->Ki = _IQ(0.01);
        }

		//v->PChgGain = _IQdiv(_IQmpy(v->Kp,_IQ(0.8)),_IQmpy(_IQ(2.0),DrvCoeff.MotIe));
		v->PChgOffSet = _IQmpy(v->Kp,KPMAXRATS);
		v->PChgGain = _IQdiv(_IQmpy(v->Kp,KPMINRATS)-v->PChgOffSet,PCHGPOINTSQ);
	}
	else
	{
		v->Kp = _IQ(0.9);
		v->Ki = _IQ(0.07);
	}
	//v->Kc = KCOFFE;
    v->Kc = _IQmpyI32(_IQ(0.01),DPI_ACR_AntiwindupTi);
	v->PWMDuty = _IQ(0.5);
    
    QPCHGPONT1 = _IQmpy(_IQ(0.25),DrvCoeff.MotIe);		//25%
    QPCHGPONT2 = _IQmpy(_IQ(0.5),DrvCoeff.MotIe);		//50%
    QPCHGPONT3 = _IQmpy(_IQ(0.75),DrvCoeff.MotIe);		//75%
    QPCHGPONT4 = _IQmpy(_IQ(1.00),DrvCoeff.MotIe);		//100%
    QPCHGPONT5 = _IQmpy(_IQ(1.25),DrvCoeff.MotIe);		//125%
    QPCHGPONT6 = _IQmpy(_IQ(1.5),DrvCoeff.MotIe);		//150%
    QPCHGPONT7 =  _IQmpy(_IQ(1.75),DrvCoeff.MotIe);		//175%
    QPCHGPONT8 =  _IQmpy(_IQ(2.00),DrvCoeff.MotIe);		//200%
    QPCHGPONT9 = _IQmpy(_IQ(2.25),DrvCoeff.MotIe);		//225%
    QPCHGPONT10 =  _IQmpy(_IQ(2.5),DrvCoeff.MotIe);		//250%
    QPCHGPONT11 = _IQmpy(_IQ(2.75),DrvCoeff.MotIe);		//275%
    QPCHGPONT12 = _IQmpy(_IQ(3.00),DrvCoeff.MotIe);		//300%
    
    v->OutMax = _IQ(MAXPIOUT);
    v->OutMin = _IQ(-MAXPIOUT);
}

void Cur_IqPiReg_update(CUR_IQPIREG *v)
{
	if(v->GainSw == 0)
	{
//		if(DrvCoeff.Lq != 0 && DrvCoeff.Rs != 0)
//		{
//			//CurPiGainAdpt.Lq = DrvCoeff.Lq>>1;
//			//CurPiGainAdpt.Rs = DrvCoeff.Rs>>1;
//			//CurPiGainAdpt.PiGainAdptcalc(&CurPiGainAdpt);
//			//v->Kp = _IQmpy(CurPiGainAdpt.kp,_IQ(0.5));
//			//v->Ki = _IQmpy(CurPiGainAdpt.ki,_IQ(0.5));
//		}
//		else
//		{
//			v->Kp = _IQmpy(_IQ11toIQ(DPI_ACR_Pd),_IQ(0.5));
//			v->Ki = _IQmpy(_IQ16toIQ(DPI_ACR_Id),_IQ(0.5));
//		}
        
        if(DPI_ACR_Pd > 0)
        {
            v->Kp = _IQmpy(_IQ11toIQ(DPI_ACR_Pd),_IQ(0.5));
        }
        else
        {
            v->Kp = _IQ(0.1);
        }
        
        if(DPI_ACR_Id > 0)
        {
            v->Ki = _IQmpy(_IQ16toIQ(DPI_ACR_Id),_IQ(0.5));   
        }
        else
        {
            v->Ki = _IQ(0.01);
        }

        if(RamMotor->ACR_Pq2 > 0)
        {
            v->Kp2 = _IQdiv(RamMotor->ACR_Pq2,1000); 
        }
        else
        {
            v->Kp2 = 0;//_IQ(0.1);
        }
        
		//v->PChgGain = _IQdiv(_IQmpy(v->Kp,_IQ(0.8)),_IQmpy(_IQ(2.0),DrvCoeff.MotIe));
		v->PChgOffSet = _IQmpy(v->Kp,KPMAXRATS);
		v->PChgGain = _IQdiv(_IQmpy(v->Kp,KPMINRATS)-v->PChgOffSet,	PCHGPOINTSQ);
		//v->IChgGain = _IQdiv(_IQmpy(v->Ki,_IQ(0.8)),_IQ(1.0));
		//v->IChgOffSet = _IQmpy(v->Ki,_IQ(1.8));
	}
	else
	{
		v->Kp = _IQ(0.9);
		v->Ki = _IQ(0.07);
	}
    
    v->Kc = _IQmpyI32(_IQ(0.01),DPI_ACR_AntiwindupTi);
}

void Cur_IqPiReg_rst(CUR_IQPIREG *v)
{
	v->Err = 0;
	v->Fdb = 0;
	v->Out = 0;
	v->OutPreSat = 0;
	v->Ref = 0;
	v->RefOld = 0;
	v->Ud = 0;
	v->Ui = 0;
	v->Up = 0;
	v->Up1 = 0;
	v->SatErr = 0;
	v->GainSw = 0;
	v->PWMDuty = _IQ(0.5);
	v->halfpflg = 0;
	v->PiSatFlg = 0;
    testperr = 0;
}

void IqPiReg_Calc(void)
{
	_iq Kp,Ki,Kp2;
	_iq Ref;//RefSq;
	_iq KpCoeff,KiCoeff;//Err,;

    // Compute the error
	IqPiReg.Err = IqPiReg.Ref - IqPiReg.Fdb;

    if(IqPiReg.GainSw == 0)
	{
        //Ref = _IQabs(IqPiReg.Ref);
        Ref = _IQabs(IqPiReg.Err);
        
		Kp = IqPiReg.Kp;
		Ki = IqPiReg.Ki;
        KpCoeff = _IQ24_P;
        
        #if QKPCHG1EN
        if(Ref < QPCHGPONT1)					//25%
			KpCoeff = 16777216L;//_IQ(1.00);
		else if(Ref < QPCHGPONT2)				//50%
			KpCoeff = 15099494L;//_IQ(0.9);
		else if(Ref < QPCHGPONT3)				//75%
			KpCoeff = 14260634L;//_IQ(0.85);
		else if(Ref < QPCHGPONT4)				//100%
			KpCoeff = 13421772L;//_IQ(0.8);
		else if(Ref < QPCHGPONT5)				//125%
			KpCoeff = 12582912L;//_IQ(0.75);
		else if(Ref < QPCHGPONT6)				//150%
			KpCoeff = 11744051L;//_IQ(0.7);
		else if(Ref < QPCHGPONT7)				//175%
			KpCoeff = 10905190L;//_IQ(0.65);
		else if(Ref < QPCHGPONT8)				//200%
			KpCoeff = 10066330L;//_IQ(0.6);
		else if(Ref < QPCHGPONT9)				//225%
			KpCoeff =  8388608L;//_IQ(0.5);
		else if(Ref < QPCHGPONT10)				//250%
			KpCoeff =  6710886L;//_IQ(0.4);
		else if(Ref < QPCHGPONT11)				//275%
			KpCoeff =  5033164L;//_IQ(0.3);
		else if(Ref < QPCHGPONT12)				//300%
			KpCoeff =  3355443L;//_IQ(0.2);
		else
			KpCoeff =  1677721L;//_IQ(0.1);
        #endif
        
		Kp = _IQmpy(Kp,KpCoeff);
		if(DPI_ACR_RES2 == 0)
		{
			if(IqPiReg.halfpflg > 0)
			{
				Kp >>= 1;
				IqPiReg.halfpflg = 0;
			}
		}

        #if 1   //DKICHGEN
		if(Ref < QPCHGPONT1)						//25%
			KiCoeff = 16777216L;                // 1
		else if(Ref < QPCHGPONT2)				//50%
			KiCoeff = 15099494L;                // 0.9
		else if(Ref < QPCHGPONT3)				//75%
			KiCoeff = 13421772L;                // 0.8
		else if(Ref < QPCHGPONT4)				//100%
			KiCoeff = 11744051L;                // 0.7
		else if(Ref < QPCHGPONT5)				//125%
			KiCoeff = 10066330L;                // 0.6
		else if(Ref < QPCHGPONT6)				//150%
			KiCoeff = 8388608L;                 // 0.5
		else
			KiCoeff = 8388608L;
		Ki = _IQmpy(Ki,KiCoeff);
        #endif

         Kp2 = IqPiReg.Kp2;
	}
	else
	{
		Kp = IqPiReg.Kp;
		Ki = IqPiReg.Ki;

         Kp2 = IqPiReg.Kp2;
	}

//    Ref = _IQabs(IqPiReg.Err);
//    
//    Kp = IqPiReg.Kp;
//    Ki = IqPiReg.Ki;
    
//    if(Ref <= QPCHGPONT1)      //25
//    {
//        Kp = IqPiReg.Kp;
//        Ki = IqPiReg.Ki;
//    }
//    else if(Ref <= QPCHGPONT4) //100
//    {
//        Kp = _IQmpy(IqPiReg.Kp,20132659L);
//        Ki = IqPiReg.Ki;
//    }
//    else if(Ref <= QPCHGPONT6) //150
//    {
//        Kp = _IQmpy(IqPiReg.Kp,23488102L);
//        Ki = IqPiReg.Ki;
//    }
//    else
//    {
//        Kp = _IQmpy(IqPiReg.Kp,23488102L);
//        Ki = IqPiReg.Ki;
//    }

    // Compute the proportional output
	IqPiReg.Up = _IQmpy(Kp,IqPiReg.Err);
	IqPiReg.Up = _IQsat(IqPiReg.Up,IqPiReg.OutMax,IqPiReg.OutMin);

    // Compute the proportional output---电流指令前馈
	IqPiReg.Up2 = _IQmpy(Kp2,IqPiReg.Ref);
	IqPiReg.Up2 = _IQsat(IqPiReg.Up2,IqPiReg.OutMax,IqPiReg.OutMin);

    if(IqPiReg.PiSatFlg == 0)//说明之前调节器未饱和
    {
        IqPiReg.Ui = IqPiReg.Ui + _IQmpy(Ki,IqPiReg.Err);
    }
    else if(IqPiReg.PiSatFlg == 1)//说明之前调节器正向饱和
    {
        if(IqPiReg.Err < 0)//误差小于0才积分
            IqPiReg.Ui = IqPiReg.Ui + _IQmpy(Ki,IqPiReg.Err);
    }
    else//说明之前调节器负向饱和
    {
        if(IqPiReg.Err > 0)//误差大于0才积分
            IqPiReg.Ui = IqPiReg.Ui + _IQmpy(Ki,IqPiReg.Err);
    }
    
    IqPiReg.OutPreSat = IqPiReg.Up + IqPiReg.Ui + IqPiReg.Up2;
    
    if(IqPiReg.OutPreSat < IqPiReg.OutMin)
    {
        IqPiReg.Out = IqPiReg.OutMin;
        IqPiReg.PiSatFlg = 2;//负向饱和
    }
    else if(IqPiReg.OutPreSat > IqPiReg.OutMax)
    {
        IqPiReg.Out = IqPiReg.OutMax;
        IqPiReg.PiSatFlg = 1;//正向饱和
    }
    else
    {
        IqPiReg.Out = IqPiReg.OutPreSat;
        IqPiReg.PiSatFlg = 0;
    }

}

void Cur_IqPiReg_calc(CUR_IQPIREG *v)
{
#if FUNCTYPEN
	_iq Kp,Ki;
	_iq Ref;
	Ref = (v->Ref+v->RefOld)>>1;
	v->RefOld = v->Ref;

	//Kp = _IQmpy(v->PChgGain,v->PWMDuty)+v->PChgOffSet;
	//Ki = _IQmpy(v->IChgGain,v->PWMDuty)+v->IChgOffSet;

	Kp = -_IQmpy(v->PChgGain,Ref)+v->PChgOffSet;
	//Kp = v->Kp;
	Ki = v->Ki;
    // Compute the error
	v->Err = v->Ref - v->Fdb;


    // Compute the proportional output
	v->Up = _IQmpy(Kp,v->Err);

    // Compute the integral output
	v->Ui = v->Ui + _IQmpy(Ki,v->Err) + _IQmpy(v->Kc,v->SatErr);
    //v->Ui = v->Ui + _IQmpy(Ki,v->Err);

    // Compute the derivative output
	v->Ud = _IQmpy(v->Kd,(v->Up - v->Up1));

    // Compute the pre-saturated output
    //v->OutPreSat = v->Up + v->Ui + v->Ud;
	v->OutPreSat = v->Up + v->Ui;

    // Saturate the output
	v->Out  = _IQsat(v->OutPreSat,v->OutMax,v->OutMin);//

    // Compute the saturate difference
	v->SatErr = v->Out - v->OutPreSat;

    // Update the previous proportional output
	v->Up1 = v->Up;
#endif
}

