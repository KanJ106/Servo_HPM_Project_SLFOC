/*
 * SV_Fault.h
 *
 *  Created on: 2016-1-19
 *      Author: zhangwei
 */

#ifndef SV_FAULT_H_
#define SV_FAULT_H_
#include "userdefine.h"//
#include "Datatype.h"
#include "IQmathLib.h"         /* Include header for IQmath library */

#define FJ_DRVSTATUS				(StateMachine.RegulFlg)
#define	FJ_RUNSTATUS				(1)

#define FJ_ILP_MASK					(RamServo->ILPMaskFlag)
#define FJ_LOSS_MASK                (RamServo->ILPLossFlag)

#define FJ_ILP_HIGH					(1)
#define FJ_ILP_LOW					(0)
#define FJ_ILP_YES					(FJ_ILP_HIGH)

#define FJ_OLP_CHECK				(0)
#if FJ_OLP_CHECK
#define FJ_OLP_MASK					(0)
#define	FJ_OLP_Kx					(_IQ24(0.00049975))	//ABC三相电流绝对值滤波系数，1s，Kx=(1ms/(1ms+2000ms))，此值根据调度周期计算
#define FJ_OLP_IU					(AdRead.Iu)
#define FJ_OLP_IV					(AdRead.Iv)
#define FJ_OLP_IW					(AdRead.Iw)
#define	FJ_OLP_We					(cv.We)		//同步角频率
#define FJ_OLP_WE_THR				(_IQ24(0.05))//OLP故障判断的同步角频率阈值,大于此值为满足条件
#define FJ_OLP_CURR_THR1			(_IQ24(0.5))//OLP故障判断的输出电流阈值,大于此值为满足条件
#define FJ_OLP_CURR_THR2			(_IQ24(0.05))//OLP故障判断的各相电流阈值，小于此值为满足条件
#endif

typedef struct{
	//ILP信号检测相关
	Uint16				ILPTimer;			//ILP采样周期计数
	Uint16				ILPTimerPrd;		//ILP采样周期
	Uint16 				ILPHighCnt;			//ILP周期内高电平计数
	Uint16				ILPLowCnt;			//ILP周期内低电平计数
	Uint16				ILPYesCnt;			//ILP判定计数
	Uint16				ILPFlg;				//ILP标志
	Uint16				PowOffCnt;			//掉电计数
	Uint16				PowOffFlag;			//掉电标志
#if FJ_OLP_CHECK
	_iq					ImagLp;				//变量：滤波后的Iabs
	_iq					IuAbsLp;			//变量：滤波
	_iq					IvAbsLp;			//变量：滤波
	_iq					IwAbsLp;			//变量：滤波
	Uint16				OLPFlg;				//OLP标志
#endif

}TYPE_FAULTJUDGE;

extern TYPE_FAULTJUDGE FaultJudge;
extern void Fault_Check(void);
#endif /* SV_FAULT_H_ */
/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
