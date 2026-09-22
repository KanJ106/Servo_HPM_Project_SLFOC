/*
 * Cur_IqRefCal.c
 *
 *  Created on: 2015-12-16//
 *      Author: w
 */
#include "IQmathLib.h"         /* Include header for IQmath library */
/* Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file */
#include "dataType.h"
#include "Drive.h"
#include "SV_OLProtect.h"
#include "SV_UdcCtrl.h"
#include "SV_OTProtect.h"
//#include "SV_CanbusCtrl.h"
#include "TorClosedCtrLoop.h"

#define ADPTFLTEN 1

int32 CurInerCmd = 0;
int32 CurCmdToIq = 0;
/******************************************************************************
 **函 数 名：
 **描    述：Q轴给定电流计算，转矩限制，摩擦力补偿，共振抑制
 **调    用：
 **输    入：
 **输    出：
 **返    回：无
 **其    它：无
 **日    期：2015-12-17
 *******************************************************************************/
void Cur_TorqLimt_calc(CUR_IQREF *v);
void Cur_IqRef_init(CUR_IQREF *v)
{
	Uint32 temp;
    Uint32 temp1;
    
	v->IqRef = 0;
	v->IqRefGet = 0;
	v->IqRefInner = 0;
	temp = 1000L*SpdGainAdpt.TorqCmdFiltTimes;
	v->fK = _IQdiv(temp,(Uint32)v->Tsamp+temp);
	v->MaxCnt = (Uint16)(100000L/v->Tsamp);//理论应该是ECAT通信周期
    
    temp1 = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_SysMaxTorq),DrvCoeff.MotIe);
    
    temp  = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_CCW_TorqMaxLmt),DrvCoeff.MotIe);
    
    if(temp > temp1)
        v->TlmtsysCCW = temp1;
    else
        v->TlmtsysCCW = temp;
    
    temp  = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_CW_TorqMaxLmt),DrvCoeff.MotIe);
    
    if(temp > temp1)
        v->TlmtsysCW = temp1;
    else
        v->TlmtsysCW = temp;
    
    v->TlmtEcatCCW = v->TlmtsysCCW;
    v->TlmtEcatCW  = v->TlmtsysCW;
}

void Cur_IqRef_rst(CUR_IQREF *v)
{
    Uint32 temp;
    Uint32 temp1;
    
	v->IqRef = 0;
	v->Ref = 0;
	v->IqRefGet = 0;
    v->Refilter = 0;
	v->IqRefInner = 0;
    v->TorqueFlag = 1;

    CurInerCmd = 0;
	TorqGen.rst(&TorqGen);
	v->Cnt = 0;
    
    temp1 = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_SysMaxTorq),DrvCoeff.MotIe);
    
    temp  = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_CCW_TorqMaxLmt),DrvCoeff.MotIe);
    
    if(temp > temp1)
        v->TlmtsysCCW = temp1;
    else
        v->TlmtsysCCW = temp; 
    
    temp  = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_CW_TorqMaxLmt),DrvCoeff.MotIe);
    
    if(temp > temp1)
        v->TlmtsysCW = temp1;
    else
        v->TlmtsysCW = temp; 
}

void Cur_IqRef_update(CUR_IQREF *v)
{
	Uint32 temp;
	temp = 1000L*SpdGainAdpt.TorqCmdFiltTimes;
	v->fK = _IQdiv(temp,(Uint32)v->Tsamp+temp);//一阶低通滤波时间常数单位0.01ms 

    v->TlmtEcatCCW = _IQmpy(_IQmpyI32(_IQ(0.001),v->EcatMax_P),DrvCoeff.MotIe);
    v->TlmtEcatCW  = _IQmpy(_IQmpyI32(_IQ(0.001),v->EcatMax_N),DrvCoeff.MotIe);
}

void Cur_TorqLimt_calc(CUR_IQREF *v)//转矩限幅计算
{

}

void Cur_IqRef_calc(CUR_IQREF *v)
{

}

//#define MaxTorqLmt _IQmpy(_IQ(6.0),DrvCoeff.MotIe)//最大的转矩限制
#define MaxTorqLmt _IQmpy(100663296L,DrvCoeff.MotIe)//最大的转矩限制

uint16_t TorqueToSpd_Flag(uint16_t flag)
{
    if(flag == 1)  //力矩模式
    {
        if(_IQabs(Spd_Ref.Ref) > _IQabs(SpdFb.Fb))
        {
            //if(Spd_PiReg.Torout > 0)
            //    IqRef.Ref = Spd_PiReg.OutMax;
            //else if(Spd_PiReg.Torout < 0)
            //    IqRef.Ref = Spd_PiReg.OutMin;
            //else
            //    IqRef.Ref = 0;
        }
        else
        {
            flag = 0;
        }
    }
    else 
    {
        if(RamPC->TorFullSel == 2)   //力矩闭环模式
        {
            int32_t torque = _IQabs(_IQmpy(_IQmpyI32(16777L,(int16)Spd_PiReg.ToroutRef),DrvCoeff.MotIe));
            if(torque != 0)
            {
                if(_IQabs(Spd_PiReg.Out) > torque)
                {
                    flag = 1;
                    CurFullReg.Ui = CurMonitor.TorqRatsDispS;
                    CurFullReg.PiFlag = 0;
                }
                //判断那个参数切换到力控
            }
            else
            {
                flag = 1; 
                CurFullReg.Ui = CurMonitor.TorqRatsDispS;
                CurFullReg.PiFlag = 0;
            }   
        }
        else
        {
            if(IqRef.TorqModCCWCWLimt != 0)
            {
                if(_IQabs(Spd_PiReg.Out) > IqRef.TorqModCCWCWLimt)
                {
                    flag = 1;
                }
                //判断那个参数切换到力控
            }
            else
            {
                flag = 1; 
            }   
        }
    } 
    
    return flag;
}

_iq TorqLimt_Calc(_iq Ref)//转矩限幅计算
{
    _iq Out;
	_iq TlmtCCW,TlmtCW;	//正反限制最小值
    
	_iq TorqModDigtCCW;	//力矩模式键盘正力矩给定限制
	_iq TorqModDigtCW;	//力矩模式键盘反力矩给定限制
  
	_iq TorqEcatCCW;	//ECAT正力矩给定限制
	_iq TorqEcatCW;  	//ECAT反力矩给定限制

	_iq TorqDigtCCW;	//键盘给定正力矩给定限制
	_iq TorqDigtCW;		//键盘给定反力矩给定限制
    
//	_iq TorqStopCCW;	//堵转正限制
//	_iq TorqStopCW;		//堵转反限制
    
	_iq TorqPLRTCCW;	//瞬停不停正限制
	_iq TorqPLRTCW;		//瞬停不停反限制
	_iq TorqHomCCW;		//原点回归中的转矩正限制
	_iq TorqHomCW;		//原点回归中的转矩反限制
	Uint16 Flg = 0;
    Uint16 Modeflag;
    
//转矩限幅来源于下面条件：
//1.客户通过功能码设置的数字转矩限制
//2.客户通过外部模拟量设定的转矩限制
//3.根据不同电机内部设定的转矩限制
//4.通过DI端子切换选择的两段转矩限制
//5.力矩控制时通过数字给定的转矩限制
//6.力矩控制时通过模拟量给定的转矩限制
//7.离线惯量辨识时的转矩限制
//8.瞬停不停
    Glo_DO_FUNC.Flag_Func0.bit.V_LT = 0;//转速限制中解除

//*********************************1step*************************************************************
	if(StateMachine.BrakeStatus >= SM_BRAKESTATUS_LOOSE)//电机松闸
	{
		if(StateMachine.CmdIn.bit.JOG == 0)//非jog
		{
			if(StateMachine.OffZSpdStopFlg == 0 && \
               StateMachine.HaltStopFlg    == 0 && \
               OTP.ZeroPosLockFlg == 0 && \
               OTP.ZeroSpdStopFlg == 0    )//非零速停车标志置位,非超程锁轴
			{
                if(CtrlMode.ActualMode == ACTUALMODE_TOR)                      
                {
                    IqRef.TorqModCCWCWLimt = _IQabs(Spd_PiReg.Torout);                                                
                    Flg = 1;

                    TorqModDigtCCW = IqRef.TorqModCCWCWLimt; //记录力矩模式正限制
                    TorqModDigtCW = IqRef.TorqModCCWCWLimt;  //记录力矩模式反限制
                    
                    Modeflag = IqRef.TorqueFlag;
                    if(IqRef.TorqueFlag == 1) Ref = Spd_PiReg.Torout;  
                    IqRef.TorqueFlag = TorqueToSpd_Flag(Modeflag);   
                                       

                    //转速限制判断
                    if(IqRef.TorqModCCWCWLimt > _IQabs(Spd_PiReg.Out))
                        Glo_DO_FUNC.Flag_Func0.bit.V_LT = 1;//转速限制中,力矩模式转成速度控制               
                }
                else
                {
                    TorqGen.Output = 0;
                    IqRef.TorqueFlag = 1;

                    if(CtrlMode.ECATCtrlMode == ETHERCATSTATE_CSP || CtrlMode.ECATCtrlMode == ETHERCATSTATE_CSV)          
                    {
                        Ref += _IQmpy(_IQmpyI32(16777L,Cia402_PP.PPBuffer0.TorqueOffset60B2),DrvCoeff.MotIe);
                    }
                }
			}
            else
            {
                TorqGen.Output = 0;
                IqRef.TorqueFlag = 1;
            }
		}
	}
	/***********2023.5.25确定*************************/
	else
	{
	    if(CtrlMode.ActualMode == ACTUALMODE_TOR)
	    {
            TorqModDigtCCW = 0;   //这样置零，在速度为0时，关闭使能会出现，电流的阶跃
            TorqModDigtCW = 0;    //是否改成，电流目标值不变，直接等待RegulFlg = 0，关断管子
            Flg = 1;
	    }

	}
	/***********2023.5.25确定*************************/
    
	if(Flg == 0)//说明不是力矩控制输出的力矩限幅
	{
        TorqModDigtCCW = IqRef.TlmtsysCCW;	//记录力矩模式正限制
		TorqModDigtCW  = TorqModDigtCCW;	//记录力矩模式反限制
	}
	Flg = 0;
//*********************************1step*************************************************************
    
//*********************************2step*************************************************************

    TorqEcatCCW = IqRef.TlmtEcatCCW;//_IQmpyI32(LocalAxes.Objects.objPosTqLimitValue,DrvCoeff.MotIe);
    TorqEcatCW  = IqRef.TlmtEcatCW; //_IQmpyI32(LocalAxes.Objects.objNegTqLimitValue,DrvCoeff.MotIe);

//*********************************3step*************************************************************
//	TorqDigtCCW = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_CCW_TorqMaxLmt),DrvCoeff.MotIe);
//	TorqDigtCW = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_CW_TorqMaxLmt),DrvCoeff.MotIe);
//	TorqStopCCW = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_SysMaxTorq),DrvCoeff.MotIe);
    
    TorqDigtCCW = IqRef.TlmtsysCCW;
    TorqDigtCW  = IqRef.TlmtsysCW;    //用户能够设置的转矩限制  
    
//*********************************4step*************************************************************    
	if(UdcCtrl.PLRTFlag)//瞬停不停
	{
		//TorqPLRTCCW = _IQmpy(UDC_PLRT_TORQLIMIT, DrvCoeff.MotIe);
        TorqPLRTCCW = _IQmpy(13421773L, DrvCoeff.MotIe); // 0.8*DrvCoeff.MotIe
		TorqPLRTCW = TorqPLRTCCW;
	}
	else
	{
        TorqPLRTCCW = IqRef.TlmtsysCCW;
		TorqPLRTCW  = IqRef.TlmtsysCW;
	}

//*********************************5step*************************************************************    
	if(PosRef.Homing.Step == 0)
	{
		//TorqHomCCW = MaxTorqLmt;
		//TorqHomCW = TorqHomCCW;
        TorqHomCCW = IqRef.TlmtsysCCW;
		TorqHomCW  = IqRef.TlmtsysCW;
	}
	else//原点回归中的转矩限制
	{
		//TorqHomCCW = _IQmpy(_IQ(2.0),DrvCoeff.MotIe);
        TorqHomCCW = _IQmpy(33554432L,DrvCoeff.MotIe);
		TorqHomCW = TorqHomCCW;
	}


//分别求出正限制和反限制的最小值！
//TorqModDigtCCW	|	TorqModDigtCW
    
//TorqAgCCW			|	TorqAgCW
//TorqAg1CCW		|	TorqAg1CW
    
//TorqDigtCCW		|	TorqDigtCW
//TorqPLRTCCW 		|	TorqPLRTCW
//TorqHomCCW		|	TorqHomCCW
//正限制最小值
	TlmtCCW = TorqModDigtCCW;
     
    if(TlmtCCW > TorqEcatCCW)
        TlmtCCW = TorqEcatCCW; 
	if(TlmtCCW > TorqDigtCCW)
		TlmtCCW = TorqDigtCCW;
	if(TlmtCCW > TorqPLRTCCW)
		TlmtCCW = TorqPLRTCCW;
	if(TlmtCCW > TorqHomCCW)
		TlmtCCW = TorqHomCCW;
    
//反限制最小值
	TlmtCW = TorqModDigtCW;

    if(TlmtCW > TorqEcatCW)
        TlmtCW = TorqEcatCW; 
	if(TlmtCW > TorqDigtCW)
		TlmtCW = TorqDigtCW;
	if(TlmtCW > TorqPLRTCW)
		TlmtCW = TorqPLRTCW;
	if(TlmtCW > TorqHomCW)
		TlmtCW = TorqHomCW;
    
	IqRef.TlmtCCW = TlmtCCW;//可以将此值换算为百分比监控当前实际正力矩限制值
	IqRef.TlmtCW  = TlmtCW; //可以将此值换算为百分比监控当前实际反力矩限制值
//*********************************6step*************************************************************

    //实际限制
	if(Ref < 0)//力为负则取反力矩限制
	{
		if(-Ref > TlmtCW)
		{
			Out = -TlmtCW;
			Glo_DO_FUNC.Flag_Func0.bit.C_LT = 1;//力矩限制中
		}
		else
		{
            Out = Ref;
			Glo_DO_FUNC.Flag_Func0.bit.C_LT = 0;//力矩限制中解除
		}
	}
	else//力为正则取正力矩限制
	{
		if(Ref > TlmtCCW)
		{
			Out = TlmtCCW;
			Glo_DO_FUNC.Flag_Func0.bit.C_LT = 1;//力矩限制中
		}
		else
		{
            Out = Ref;
			Glo_DO_FUNC.Flag_Func0.bit.C_LT = 0;//力矩限制中解除
		}
	}
    
    return Out;
}

_iq TorqLimt_Calc_PV(_iq Ref)//转矩限幅计算
{
    _iq Out;
	_iq TlmtCCW,TlmtCW;	//正反限制最小值
  
	_iq TorqEcatCCW;	//ECAT正力矩给定限制
	_iq TorqEcatCW;  	//ECAT反力矩给定限制

	_iq TorqDigtCCW;	//键盘给定正力矩给定限制
	_iq TorqDigtCW;		//键盘给定反力矩给定限制
    
//	_iq TorqStopCCW;	//堵转正限制
//	_iq TorqStopCW;		//堵转反限制
    
	_iq TorqPLRTCCW;	//瞬停不停正限制
	_iq TorqPLRTCW;		//瞬停不停反限制
	_iq TorqHomCCW;		//原点回归中的转矩正限制
	_iq TorqHomCW;		//原点回归中的转矩反限制
    
//转矩限幅来源于下面条件：
//1.客户通过功能码设置的数字转矩限制
//2.客户通过外部模拟量设定的转矩限制
//3.根据不同电机内部设定的转矩限制
//4.通过DI端子切换选择的两段转矩限制
//5.力矩控制时通过数字给定的转矩限制
//6.力矩控制时通过模拟量给定的转矩限制
//7.离线惯量辨识时的转矩限制
//8.瞬停不停

	//Glo_DO_FUNC.Flag_Func0.bit.V_LT = 0;//转速限制中解除
//*********************************1step*************************************************************
	if(StateMachine.BrakeStatus >= SM_BRAKESTATUS_LOOSE)//电机松闸
	{
		if(StateMachine.CmdIn.bit.JOG == 0)//非jog
		{
			if(StateMachine.OffZSpdStopFlg == 0 && \
               StateMachine.HaltStopFlg    == 0 && \
               OTP.ZeroPosLockFlg == 0 && \
               OTP.ZeroSpdStopFlg == 0    )//非零速停车标志置位,非超程锁轴
			{
                if(CtrlMode.ActualMode == ACTUALMODE_TOR)                      
                {
                  
                }
                else
                {
                    if(CtrlMode.ECATCtrlMode == ETHERCATSTATE_CSP || CtrlMode.ECATCtrlMode == ETHERCATSTATE_CSV)          
                    {
                        Ref += _IQmpy(_IQmpyI32(16777L,Cia402_PP.PPBuffer0.TorqueOffset60B2),DrvCoeff.MotIe);
                    }
                }
			}
            else
            {

            }
		}
	}
	/***********2023.5.25确定*************************/
	else
	{

	}
	/***********2023.5.25确定*************************/

//*********************************2step*************************************************************
    TorqEcatCCW = IqRef.TlmtEcatCCW;//_IQmpyI32(LocalAxes.Objects.objPosTqLimitValue,DrvCoeff.MotIe);
    TorqEcatCW  = IqRef.TlmtEcatCW; //_IQmpyI32(LocalAxes.Objects.objNegTqLimitValue,DrvCoeff.MotIe);

//*********************************3step*************************************************************
//	TorqDigtCCW = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_CCW_TorqMaxLmt),DrvCoeff.MotIe);
//	TorqDigtCW = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_CW_TorqMaxLmt),DrvCoeff.MotIe);
//	TorqStopCCW = _IQmpy(_IQmpyI32(_IQ(0.001),DPI_SysMaxTorq),DrvCoeff.MotIe);
    
    TorqDigtCCW = IqRef.TlmtsysCCW;
    TorqDigtCW  = IqRef.TlmtsysCW;    //用户能够设置的转矩限制  
    
//*********************************4step*************************************************************    
	if(UdcCtrl.PLRTFlag)//瞬停不停
	{
		//TorqPLRTCCW = _IQmpy(UDC_PLRT_TORQLIMIT, DrvCoeff.MotIe);
        TorqPLRTCCW = _IQmpy(13421773L, DrvCoeff.MotIe); // 0.8*DrvCoeff.MotIe
		TorqPLRTCW = TorqPLRTCCW;
	}
	else
	{
        TorqPLRTCCW = IqRef.TlmtsysCCW;
		TorqPLRTCW  = IqRef.TlmtsysCW;
	}

//*********************************5step*************************************************************    
	if(PosRef.Homing.Step == 0)
	{
		//TorqHomCCW = MaxTorqLmt;
		//TorqHomCW = TorqHomCCW;
        TorqHomCCW = IqRef.TlmtsysCCW;
		TorqHomCW  = IqRef.TlmtsysCW;
	}
	else//原点回归中的转矩限制
	{
		//TorqHomCCW = _IQmpy(_IQ(2.0),DrvCoeff.MotIe);
        TorqHomCCW = _IQmpy(33554432L,DrvCoeff.MotIe);
		TorqHomCW = TorqHomCCW;
	}

//分别求出正限制和反限制的最小值！
//TorqModDigtCCW	|	TorqModDigtCW
    
//TorqAgCCW			|	TorqAgCW
//TorqAg1CCW		|	TorqAg1CW
    
//TorqDigtCCW		|	TorqDigtCW
//TorqPLRTCCW 		|	TorqPLRTCW
//TorqHomCCW		|	TorqHomCCW
//正限制最小值
	TlmtCCW = TorqEcatCCW;
     
	if(TlmtCCW > TorqDigtCCW)
		TlmtCCW = TorqDigtCCW;
	if(TlmtCCW > TorqPLRTCCW)
		TlmtCCW = TorqPLRTCCW;
	if(TlmtCCW > TorqHomCCW)
		TlmtCCW = TorqHomCCW;
    
//反限制最小值
	TlmtCW = TorqEcatCW;
 
	if(TlmtCW > TorqDigtCW)
		TlmtCW = TorqDigtCW;
	if(TlmtCW > TorqPLRTCW)
		TlmtCW = TorqPLRTCW;
	if(TlmtCW > TorqHomCW)
		TlmtCW = TorqHomCW;
//*********************************6step*************************************************************

    //实际限制
	if(Ref < 0)//力为负则取反力矩限制
	{
		if(-Ref > TlmtCW)
		{
			Out = -TlmtCW;
		}
		else
		{
            Out = Ref;
		}
	}
	else//力为正则取正力矩限制
	{
		if(Ref > TlmtCCW)
		{
			Out = TlmtCCW;
		}
		else
		{
            Out = Ref;
		}
	}
    
    return Out;
}

_iq TorqLimt_Calc_T(_iq Ref)//转矩限幅计算
{
    _iq Out;
	_iq TlmtCCW,TlmtCW;	//正反限制最小值
    
	_iq TorqModDigtCCW;	//力矩模式键盘正力矩给定限制
	_iq TorqModDigtCW;	//力矩模式键盘反力矩给定限制
  
	Uint16 Flg = 0;
    Uint16 Modeflag;

	Glo_DO_FUNC.Flag_Func0.bit.V_LT = 0;//转速限制中解除
//*********************************1step*************************************************************
	if(StateMachine.BrakeStatus >= SM_BRAKESTATUS_LOOSE)//电机松闸
	{
		if(StateMachine.CmdIn.bit.JOG == 0)//非jog
		{
			if(StateMachine.OffZSpdStopFlg == 0 && \
               StateMachine.HaltStopFlg    == 0 && \
               OTP.ZeroPosLockFlg == 0 && \
               OTP.ZeroSpdStopFlg == 0    )//非零速停车标志置位,非超程锁轴
			{
                if(CtrlMode.ActualMode == ACTUALMODE_TOR)                      
                {
                    IqRef.TorqModCCWCWLimt = _IQabs(Spd_PiReg.Torout);                                                
                    Flg = 1;

                    TorqModDigtCCW = IqRef.TorqModCCWCWLimt; //记录力矩模式正限制
                    TorqModDigtCW = IqRef.TorqModCCWCWLimt;  //记录力矩模式反限制
                    
                    Modeflag = IqRef.TorqueFlag;
                    IqRef.TorqueFlag = TorqueToSpd_Flag(Modeflag);   
                    if(IqRef.TorqueFlag == 1) Ref = Spd_PiReg.Torout;                     

                    //转速限制判断
                    if(IqRef.TorqModCCWCWLimt > _IQabs(Spd_PiReg.Out))
                        Glo_DO_FUNC.Flag_Func0.bit.V_LT = 1;//转速限制中                   
                }
                else
                {
                    TorqGen.Output = 0;
                    IqRef.TorqueFlag = 1;
                }
			}
            else
            {
                TorqGen.Output = 0;
                IqRef.TorqueFlag = 1;
            }
		}
	}
	/***********2023.5.25确定*************************/
	else
	{
	    if(CtrlMode.ActualMode == ACTUALMODE_TOR)
	    {
            TorqModDigtCCW = 0;   //这样置零，在速度为0时，关闭使能会出现，电流的阶跃
            TorqModDigtCW = 0;    //是否改成，电流目标值不变，直接等待RegulFlg = 0，关断管子
            Flg = 1;
	    }

	}
	/***********2023.5.25确定*************************/
    
	if(Flg == 0)//说明不是力矩控制输出的力矩限幅
	{
        TorqModDigtCCW = IqRef.TlmtsysCCW;	//记录力矩模式正限制
		TorqModDigtCW  = TorqModDigtCCW;	//记录力矩模式反限制
	}

//正限制最小值
	TlmtCCW = TorqModDigtCCW;

//反限制最小值
	TlmtCW = TorqModDigtCW;
//*********************************6step*************************************************************

	IqRef.TlmtCCW = TlmtCCW;//可以将此值换算为百分比监控当前实际正力矩限制值
	IqRef.TlmtCW  = TlmtCW; //可以将此值换算为百分比监控当前实际反力矩限制值

    //实际限制
	if(Ref < 0)//力为负则取反力矩限制
	{
		if(-Ref > TlmtCW)
		{
			Out = -TlmtCW;
			Glo_DO_FUNC.Flag_Func0.bit.C_LT = 1;//力矩限制中
		}
		else
		{
            Out = Ref;
			Glo_DO_FUNC.Flag_Func0.bit.C_LT = 0;//力矩限制中解除
		}
	}
	else//力为正则取正力矩限制
	{
		if(Ref > TlmtCCW)
		{
			Out = TlmtCCW;
			Glo_DO_FUNC.Flag_Func0.bit.C_LT = 1;//力矩限制中
		}
		else
		{
            Out = Ref;
			Glo_DO_FUNC.Flag_Func0.bit.C_LT = 0;//力矩限制中解除
		}
	}
    
    return Out;
}


void SpdToIq_Cacl(void)
{
    _iq Ref;

    Ref = TorqLimt_Calc(Spd_PiReg.Out);//转矩限制计算

    NotchFilter1.in = Ref;
    NotchFilter1.calc(&NotchFilter1);
    Ref = NotchFilter1.out;
    
    IqRef.Ref = Ref + _IQmpy(IqRef.fK,IqRef.Ref - Ref);//一阶低通滤波

    //IqRef.Refilter = Ref;
    //IqRef.Ref = TorqLimt_Calc(Ref);//转矩限制计算
}


void IqRef_Calc(void)
{
	if(IqRef.IqRefGet == 1)//内部强制给定
    {
		IqRef.IqRef = IqRef.IqRefInner;
    }
	else
	{  
        IqRef.IqRef = IqRef.Ref;   //来自速度环
    }
}



                        /*
                        if(Spd_PiReg.Flg.bit.TorqueFlag == 1)
                        {
                            if(_IQabs(Spd_Ref.Ref) > _IQabs(SpdFb.Fb))
                            {
                                if(Spd_PiReg.Torout > 0)
                                    IqRef.Ref = Spd_PiReg.OutMax;
                                else if(Spd_PiReg.Torout < 0)
                                    IqRef.Ref = Spd_PiReg.OutMin;
                                else
                                    IqRef.Ref = 0;
                            }
                            else
                            {
                                Spd_PiReg.Flg.bit.TorqueFlag = 0;
                            }
                        }
                        else 
                        {
                            if(RamPD->TorFullSel == 0)
                            {
                                if(IqRef.TorqModCCWCWLimt != 0)
                                {
                                    if(_IQabs(Spd_PiReg.Out) > IqRef.TorqModCCWCWLimt)
                                    {
                                        Spd_PiReg.Flg.bit.TorqueFlag = 1;
                                    }
                                    //判断那个参数切换到力控
                                }
                                else
                                {
                                    Spd_PiReg.Flg.bit.TorqueFlag = 1; 
                                }
                            }
                            else   //力矩闭环模式
                            {
                                int32_t torque = _IQabs(_IQmpy(_IQmpyI32(16777L,(int16)Spd_PiReg.ToroutRef),DrvCoeff.MotIe));
                                if(torque != 0)
                                {
                                    if(_IQabs(Spd_PiReg.Out) > torque)
                                    {
                                        Spd_PiReg.Flg.bit.TorqueFlag = 1;
                                        CurFullReg.Ui = CurMonitor.TorqRatsDispS;
                                        CurFullReg.PiFlag = 0;
                                    }
                                    //判断那个参数切换到力控
                                }
                                else
                                {
                                    Spd_PiReg.Flg.bit.TorqueFlag = 1; 
                                    CurFullReg.Ui = CurMonitor.TorqRatsDispS;
                                    CurFullReg.PiFlag = 0;
                                }   
                            }
                        }
                        */

                        //_iq SpdTorFwd(void)
//{
//    _iq Ref = 0;   
    
//    if(CtrlMode.CtrlSoure == CMD_REMOTE)
//    {
//        if(StateMachine.BrakeStatus >= SM_BRAKESTATUS_LOOSE)//电机松闸
//        {
//            if(StateMachine.CmdIn.bit.JOG == 0)//非jog
//            {
//                if(StateMachine.OffZSpdStopFlg == 0)//非零速停车标志置位
//                {
//                    if(OTP.ZeroPosLockFlg == 0 && OTP.ZeroSpdStopFlg == 0)//非超程锁轴
//                    {
//                        if(CtrlMode.ECATCtrlMode == ETHERCATSTATE_CSP \
//                        || CtrlMode.ECATCtrlMode == ETHERCATSTATE_CSV)
//                        {
//                            //Ref += _IQmpy(_IQmpyI32(_IQ(0.001),Cia402_PP.PPBuffer0.TorqueOffset60B2),DrvCoeff.MotIe);
//                            Ref = _IQmpy(_IQmpyI32(16777L,Cia402_PP.PPBuffer0.TorqueOffset60B2),DrvCoeff.MotIe);
//                        }
//                    }
//                }
//            }
//        }
//        /***********2023.5.25确定*************************/
//        else
//        {
//            if((CtrlMode.ECATCtrlMode == ETHERCATSTATE_CST) || (CtrlMode.ECATCtrlMode == ETHERCATSTATE_TQ))
//                Ref = 0;
//        }
//    }
//    /***********2023.5.25确定*************************/
    
//    return Ref;
//}

