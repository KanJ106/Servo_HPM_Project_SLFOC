/*
 * SV_Fault.c
 *
 *  Created on: 2015-12-19
 *      Author: zhangwei//
 */

#include "userdefine.h"
#include "r_cg_Project.h"
#include "Common_Lib.h"
#include "SV_FuncVar.h"
#include "SV_FaultProtect.h"
#include "Drive.h"
#include "SV_Fault.h"
#include "SV_StateMachine.h"
#include "SV_Servocode.h" //


TYPE_FAULTJUDGE FaultJudge = {	\
/*	ILP	*/	\
/*	ILPTimer,ILPTimerPrd*/					0,10,	\
/*	ILPHighCnt,ILPLowCnt,ILPYesCnt;*/		0,0,0,	\
/*	ILPFlg,PowOffCnt,PowOffFlag;*/			0,0,0,	\
};
#if FJ_OLP_CHECK
/*	OLP	*/	\
/*	ImagLp,IuAbsLp,IvAbsLp,IwAbsLp*/		0,0,0,0,\
/*	OLPFlg*/								0,\
};
#endif

/*-------------------------------------------------------------------------------------------------
函数：void FaultJudge_ILPCheck(void);
描述 ：输入电源监测
-------------------------------------------------------------------------------------------------*/
Uint16 ILP_SIGNAL  = 0;
static void FaultJudge_ILPCheck(void)
{
	TYPE_FAULTJUDGE *fj = &FaultJudge;

	//周期定时器计数
	fj->ILPTimer ++;
	if(fj->ILPTimer >= fj->ILPTimerPrd) fj->ILPTimer = 0; //从此处往后ILPTImer取0~ILPTimerPrd-1
	//读取ILP端口电平
    
    #if SERVO_HARDWARE == HARDWARE_AC0
	ILP_SIGNAL = FJ_ILP_SIGNAL;
    #endif

	//if(FJ_ILP_SIGNAL == FJ_ILP_LOW)
    if(ILP_SIGNAL == FJ_ILP_LOW) //2021.10.25
	    fj->ILPLowCnt++;
	else
	    fj->ILPHighCnt++;

	//每周期分析一次采样数据
	if(fj->ILPTimer == 0)
	{
		if(fj->ILPLowCnt == fj->ILPTimerPrd) //全低电平,电源正常
		{
			fj->ILPYesCnt     = 0;
			fj->PowOffCnt  = 0;
			fj->ILPFlg    = 0; //清除ILP标志
			fj->PowOffFlag = 0; //清除掉电标志
		}
		else if(fj->ILPHighCnt == fj->ILPTimerPrd) //全高电平
		{
			fj->PowOffCnt ++;
			//TODO:抱闸模块需要30ms有效，220V 1kw以下接单相220V，也需有
			if(fj->PowOffCnt > 2) //3次全高认为电源掉电(约30ms检出)
			{
				fj->PowOffCnt  = 0;
				fj->PowOffFlag = 1;
			}
		}
		else if(fj->ILPLowCnt > (fj->ILPTimerPrd >> 2) && fj->ILPHighCnt > (fj->ILPTimerPrd >> 3)) //脉冲性质
		{
			fj->PowOffCnt  = 0;
			fj->PowOffFlag = 0;
			fj->ILPYesCnt ++;
			if(fj->ILPYesCnt > 2) //3个脉冲周期认为输入缺相(约30ms检出)
			{
				fj->ILPYesCnt  = 0;
				fj->ILPFlg = 1;
			}
		}
		//分析结束，高低电平计数器清零
		fj->ILPHighCnt = 0;
		fj->ILPLowCnt  = 0;
	}
}

#if FJ_OLP_CHECK		//伺服闭环系统不需要做OLP，TC200老不转，三菱（实际）装的不顺畅，电流异常，过载预报警...
/*-------------------------------------------------------------------------------------------------
函数：void FaultJudge_OLPCheck(void);
描述 ：输出电源监测
-------------------------------------------------------------------------------------------------*/
static void FaultJudge_OLPCheck(void)
{
	TYPE_FAULTJUDGE *fj = &FaultJudge;
	_iq	IuAbs,IvAbs,IwAbs,Iabs;

	IuAbs = _IQ24abs(FJ_OLP_IU);	//求按电机标幺的A相电流的绝对值
	IvAbs = _IQ24abs(FJ_OLP_IV);	//求按电机标幺的B相电流的绝对值
	IwAbs = _IQ24abs(FJ_OLP_IW);	//求按电机标幺的C相电流的绝对值
	
	//求A、B、C三相中绝对值的最大值，即三相电流绝对值的包络线
	Iabs = (IuAbs > IvAbs) ? IuAbs : IvAbs;
	Iabs = (Iabs > IwAbs) ? Iabs : IwAbs;
	
	fj->ImagLp  += _IQ24mpy(FJ_OLP_Kx, Iabs  - fj->ImagLp);		//对三相电流绝对值的包络线进行滤波
	fj->IuAbsLp += _IQ24mpy(FJ_OLP_Kx, IuAbs - fj->IuAbsLp);		//对A相电流的绝对值进行滤波
	fj->IvAbsLp += _IQ24mpy(FJ_OLP_Kx, IvAbs - fj->IvAbsLp);		//对B相电流的绝对值进行滤波
	fj->IwAbsLp += _IQ24mpy(FJ_OLP_Kx, IwAbs - fj->IwAbsLp);		//对C相电流的绝对值进行滤波
	
	//驱动状态为运行，且同步频率大于电机额定频率的10%时开始检测输出缺相OLP
	if((FJ_DRVSTATUS == FJ_RUNSTATUS) && (_IQabs(FJ_OLP_We) > FJ_OLP_WE_THR))
	{
		//三相电流绝对值最大值的滤波值大于0.5倍额定值，且三相中任意一相电流绝对值小于0.05倍额定值，则认为满足缺相条件
		if((fj->ImagLp > FJ_OLP_CURR_THR1) && (fj->IuAbsLp < FJ_OLP_CURR_THR2 || fj->IvAbsLp < FJ_OLP_CURR_THR2 || fj->IwAbsLp < FJ_OLP_CURR_THR2))
		{						
			fj->OLPFlg = 1;
		}
	}
	else
	{
		//不满足检测条件时清零
		fj->ImagLp = 0;
		fj->IuAbsLp = 0;
		fj->IvAbsLp = 0;
		fj->IwAbsLp = 0;
	}
}
#endif

void Fault_Check(void)
{
	TYPE_FAULTJUDGE *fj = &FaultJudge;

	FaultJudge_ILPCheck();
	if((fj->ILPFlg) && (FJ_ILP_MASK == 0))
	{
		FaultPrtt_FaultInterface(UdcLPErr);
	}
#if 1
//	fj->PowOffFlag = 0;
	if((fj->PowOffFlag) && (FJ_LOSS_MASK==0))
	{
		if(StateMachine.RegulFlg)
		{
			FaultPrtt_FaultInterface(PowLoseErr);
		}
	}
#endif
}

/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
