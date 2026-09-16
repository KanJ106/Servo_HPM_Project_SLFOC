/*
 * cia402ControlMode.c
 *
 *  Created on: 2016-1-27//
 *      Author: sunzhixin
 */

#include "cia402Ctrl.h"
#include "SV_CtrlMode.h"
#include "Common_Lib.h"
#include "Drive.h"
#include "SV_FaultProtect.h"
#include "SV_OTProtect.h"
#include "Pos_FullClosedLoop.h"//
#include "Mit_CtrLoop.h"
#include "Canopenappl.h"
#include "CtrLoop.h"

Cia402PP  Cia402_PP = DefCia402PPValue;//{0};/*位置控制模块*/
extern float TargetTorqueClose;
extern void Updata_EcatFb(void);

#if SERVOTYPE == SERVO_CANOPEN

/**
 * PP模块给定
 * 程序执行时间(以下时间均是在PWM中断打开的情况下测出)：140us左右
 */
void PP_DataGetHandle(Cia402PP_handle p,Uint16 SaveToBuf)
{
	int32 P_user = 0;                         //用户单位位置给定
	int32 P_pluse = 0;                        //脉冲单位位置给定
	Uint32 Acc_user = 0;

	INT32 Acc_GivetoDrive = 0;                //标幺值单位
	Uint32 Dec_user = 0;

	INT32 Dec_GivetoDrive = 0;                //标幺值单位
	Uint32 V_user = 0;                        //用户单位速度给定

	INT32 V_GivetoDrive = 0;                  //标幺值单位
	TCiA402Axis *pLocalAxes = &LocalAxes;
    TYPE_STATEMACHINE *sm = &StateMachine;

    INT32 Min_PositionLimit,Max_PositionLimit;
    INT32 PosTarget,PosTarget607A;
    static UINT8 Limflag = 0;
    
    p->StatusConbit.Bit.GiveLimit = 0;

    P_user = pLocalAxes->Objects.objTargetPosition;
    
    //单位换算
    P_pluse = (INT32)(((INT64)P_user * PosCmxCdv.PosCmxNum) / PosCmxCdv.PosCdv) - p->PosFbInitValue;                         
    P_pluse = P_pluse - p->PosOffset;
    //速度限定
	if(pLocalAxes->Objects.objMaxProfileVelocity > pLocalAxes->Objects.objProfileVelocity)
	{
		V_user = pLocalAxes->Objects.objProfileVelocity;
	}
	else
	{
		V_user =pLocalAxes->Objects.objMaxProfileVelocity;
		p->StatusConbit.Bit.GiveLimit = 1;
	}
	//速度单位转换
	//V_pluse = ((float64)V_user*PosCmxCdv.PosCmxNum)/PosCmxCdv.PosCdv;
	V_GivetoDrive = (INT32)(pLocalAxes->PosPU * (float64)V_user);//脉冲单位转换成标幺值
    
	/************************加速度限定***********************/
	if(pLocalAxes->Objects.objMaxAcceleration > pLocalAxes->Objects.objProfileAcceleration)
	{
		Acc_user =  pLocalAxes->Objects.objProfileAcceleration;
	}
	else
	{
		Acc_user = pLocalAxes->Objects.objMaxAcceleration;
		p->StatusConbit.Bit.GiveLimit = 1;
	}

//	tmpdb = (float64)PosCmxCdv.PosCmxNum / PosCmxCdv.PosCdv;
//	Acc_given = (float64)Acc_user * tmpdb;
    
	Acc_GivetoDrive = (INT32)(LocalAxes.PosAccDecPu * Acc_user);
	if(!Acc_GivetoDrive)
	{
		Acc_GivetoDrive = 1;
	}
    /************************加速度限定***********************/
    
	/************************减速度设定***********************/
	if(pLocalAxes->Objects.ojbMaxDeceleration > pLocalAxes->Objects.objProfileDeceleration)
	{
		Dec_user = pLocalAxes->Objects.objProfileDeceleration;
	}
	else
	{
		Dec_user = pLocalAxes->Objects.ojbMaxDeceleration;
		p->StatusConbit.Bit.GiveLimit = 1;
	}
    
//	tmpdb = ((float64)Dec_user*PosCmxCdv.PosCmxNum)/PosCmxCdv.PosCdv;
//	Dec_given = (float64)Dec_user * tmpdb;
	Dec_GivetoDrive = (INT32)(LocalAxes.PosAccDecPu * Dec_user);
	if(!Dec_GivetoDrive)
	{
		Dec_GivetoDrive = 1;
	}
    /************************减速度设定***********************/
    
    if(SaveToBuf == 0)//要写入第一个缓冲区
    {
        p->PPBuffer0.ControlWord.All = pLocalAxes->Objects.objControlWord;
        //相对位置与绝对位置判断
        if(p->PPBuffer0.ControlWord.PPBit.absrel)//1：相对位置
        {
            PosTarget607A = p->PPBuffer0.TargetPos607A + P_user;
            PosTarget     = p->PPBuffer0.TargetPostion + P_pluse;
        }
        else	                                //0：绝对位置
        {
            PosTarget607A = P_user;
            PosTarget = P_pluse;
        }
    }
    else if(SaveToBuf == 1)//要写入第二个缓冲区
    {
        p->PPBuffer1.ControlWord.All = pLocalAxes->Objects.objControlWord;
        //相对位置与绝对位置判断
        if(p->PPBuffer1.ControlWord.PPBit.absrel)//1：相对位置
        {
            PosTarget607A = p->PPBuffer1.TargetPos607A + P_user;
            PosTarget     = p->PPBuffer1.TargetPostion + P_pluse;
        }
        else	                                //0：绝对位置
        {
            PosTarget607A = P_user;
            PosTarget = P_pluse;
        }        
    }
    
    Min_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit - pLocalAxes->Objects.objHomeOffset;
	Max_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MaxLimit - pLocalAxes->Objects.objHomeOffset;

	if((DPI_SoftOTSel == 1) && (PosTarget > Max_PositionLimit) && sm->SrvOnStatus)
    {
		//位置正向给定极限
        FaultPrtt_FaultInterface(POTWarn);
        p->PPBuffer0.TargetPos607A = pLocalAxes->Objects.objPositionActualValue;
        p->PPBuffer0.TargetPostion = pLocalAxes->Objects.objPositionActualInterValue;
        Limflag = 1;
        return;
	}
    else if((DPI_SoftOTSel == 1) && (PosTarget < Min_PositionLimit) && sm->SrvOnStatus)
    {
        //位置负向给定极限
        FaultPrtt_FaultInterface(NOTWarn);
        p->PPBuffer0.TargetPos607A = pLocalAxes->Objects.objPositionActualValue;
        p->PPBuffer0.TargetPostion = pLocalAxes->Objects.objPositionActualInterValue;
        Limflag = 1;
        return;
    }
    else
    {
        if(Limflag == 1)
        {
            FaultPrtt_WarnInterfaceClr(POTWarn);
            FaultPrtt_WarnInterfaceClr(NOTWarn); 
            Limflag = 0;
        }                  
    } 

	if(SaveToBuf == 0)//要写入第一个缓冲区
	{
		//p->PPBuffer0.ControlWord.All = pLocalAxes->Objects.objControlWord;
        
        p->PPBuffer0.TargetPos607A = PosTarget607A;
        p->PPBuffer0.TargetPostion = PosTarget;
        
		p->PPBuffer0.TargetVelo = V_GivetoDrive;
		p->PPBuffer0.TargetAcc  = Acc_GivetoDrive;
		p->PPBuffer0.TargetDec  = Dec_GivetoDrive;
		p->PPBuffer0.ConBit.Bit.GiveLimit = p->StatusConbit.Bit.GiveLimit;
		p->PPBuffer0.ConBit.Bit.BufferFull = 1;
	}
	else if(SaveToBuf == 1)//要写入第二个缓冲区
	{
		//p->PPBuffer1.ControlWord.All = pLocalAxes->Objects.objControlWord;
        
        /*p->PPBuffer0.TargetPos607A = PosTarget607A;
        p->PPBuffer0.TargetPostion = PosTarget;*/

        p->PPBuffer1.TargetPos607A = PosTarget607A;
        p->PPBuffer1.TargetPostion = PosTarget;
        
		p->PPBuffer1.TargetVelo = V_GivetoDrive;
		p->PPBuffer1.TargetAcc  = Acc_GivetoDrive;
		p->PPBuffer1.TargetDec  = Dec_GivetoDrive;
		p->PPBuffer1.ConBit.Bit.GiveLimit = p->StatusConbit.Bit.GiveLimit;
		p->PPBuffer1.ConBit.Bit.BufferFull = 1;
	}
}
/**
 * 函数功能：IP模块给定
 * 说明：IP模式处理同CSP模式，给定量为目标位置，插补周期为同步周期。
 * 程序执行时间(以下时间均是在PWM中断打开的情况下测出)：
 */
void IP_DataGetHandle(Cia402PP_handle p)
{
	//目标位置限定
	int32 Max_PositionLimit;
	int32 Min_PositionLimit;
	int32 P_user;//位置给定中间变量
//	int64 tmpPos = 0;
	TYPE_STATEMACHINE *sm = &StateMachine;
	p->StatusConbit.Bit.GiveLimit = 0;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	P_user = pLocalAxes->Objects.objInterpolationDataRecord.u32Interpolationdatarecord;//插补数据

//	Min_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit[0] - pLocalAxes->Objects.objHomeOffset;
//	Max_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit[1] - pLocalAxes->Objects.objHomeOffset;
	Min_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit - pLocalAxes->Objects.objHomeOffset;
	Max_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MaxLimit - pLocalAxes->Objects.objHomeOffset;
//	if(P_user > Max_PositionLimit && sm->SrvOnStatus && pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit[1] != 0x7FFFFFFF)
	if(P_user > Max_PositionLimit && sm->SrvOnStatus && pLocalAxes->Objects.objSoftwarePositionLimit.i32MaxLimit != 0x7FFFFFFF)
	{
		//位置正向给定极限
		FaultPrtt_FaultInterface(POSLIMIT_P);
		p->PPBuffer0.TargetPos607A = 0;
	}
//	if(P_user<Min_PositionLimit && sm->SrvOnStatus && pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit[0] != 0x80000000)
	if(P_user<Min_PositionLimit && sm->SrvOnStatus && pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit != 0x80000000)
	{
		//位置负向给定极限
		FaultPrtt_FaultInterface(POSLIMIT_N);
		p->PPBuffer0.TargetPos607A = 0;
	}

	//位置给定
	p->PPBuffer0.TargetPos607A = P_user;
    
//	p->PPBuffer0.TargetPostion = (P_user*\
//			pLocalAxes->Objects.objPositionFactor.numerator)/pLocalAxes->Objects.objPositionFactor.divisor;//经过单位转换
	//p->PPBuffer0.TargetPostion = (INT32)(((INT64)P_user * PosCmxCdv.PosCmxNum) / PosCmxCdv.PosCdv);
    
    p->PPBuffer0.TargetPostion = (INT32)(((INT64)P_user * PosCmxCdv.PosCmxNum) / PosCmxCdv.PosCdv) - \
                                 p->PosFbInitValue;
    
    
    //插补时间
	p->PPBuffer0.InterTime60C2.i8InterpolationIndex = pLocalAxes->Objects.objInterpolationTimePeriod.i8InterpolationIndex;
	switch(p->PPBuffer0.InterTime60C2.i8InterpolationIndex)
	{//程序内部0.01us
	case 0://给定单位是s
		p->PPBuffer0.InterTime60C2.u8InterpolationPeriod = pLocalAxes->Objects.objInterpolationTimePeriod.u8InterpolationPeriod * 100000000;
		break;
	case -1://给定单位是100ms
		p->PPBuffer0.InterTime60C2.u8InterpolationPeriod = pLocalAxes->Objects.objInterpolationTimePeriod.u8InterpolationPeriod * 10000000;
		break;
	case -2://给定单位是10ms
		p->PPBuffer0.InterTime60C2.u8InterpolationPeriod = pLocalAxes->Objects.objInterpolationTimePeriod.u8InterpolationPeriod * 1000000;
		break;
	//TODO:default呢？
	default:
	case -3://给定单位是ms
		p->PPBuffer0.InterTime60C2.u8InterpolationPeriod = pLocalAxes->Objects.objInterpolationTimePeriod.u8InterpolationPeriod * 100000;
		break;
	case -4://给定单位是100us
		p->PPBuffer0.InterTime60C2.u8InterpolationPeriod = pLocalAxes->Objects.objInterpolationTimePeriod.u8InterpolationPeriod * 10000;
		break;
	case -5://给定单位是10us
		p->PPBuffer0.InterTime60C2.u8InterpolationPeriod = pLocalAxes->Objects.objInterpolationTimePeriod.u8InterpolationPeriod * 1000;
		break;
	case -6://给定单位是us
		p->PPBuffer0.InterTime60C2.u8InterpolationPeriod = pLocalAxes->Objects.objInterpolationTimePeriod.u8InterpolationPeriod*100;
		break;
	}
	/*
	 * 杨玉亮处理边界条件和插补数据
	p->PosrefDeltaMax = _IQmpyI32int(_IQdiv(p->PPBuffer0.InterTime60C2.u8InterpolationPeriod,500000L),DrvCoeff.MotEncSglRevLns);//位置环路内脉冲个数的最大值，按照12000rpm限制
	p->PosrefDeltaMin = -p->PosrefDeltaMax;
	tmpPos = (int64)p->PPBuffer0.TargetPostion - (int64)p->PPBuffer0.LTargetPostion;
	if(tmpPos > p->PosrefDeltaMax)//溢出处理
	{
		tmpPos = (int32)((int64)tmpPos-(int64)4294967296);
	}
	else if(tmpPos < p->PosrefDeltaMin)
	{
		tmpPos = (int32)((int64)tmpPos+(int64)4294967296);
	}
	p->PPBuffer0.Posdeta = (int32)((tmpPos * DrvCoeff.PosTsamp )/ ((int64)p->PPBuffer0.InterTime60C2.u8InterpolationPeriod));
 	p->PPBuffer0.Posdetarem = (int32)((tmpPos * DrvCoeff.PosTsamp ) % ((int64)p->PPBuffer0.InterTime60C2.u8InterpolationPeriod));
	*/
	//最大力矩
	p->PPBuffer0.MaxTorque = pLocalAxes->Objects.objMaxTorque;//最大扭矩0x6072
	p->PPBuffer0.ConBit.Bit.GiveLimit = p->StatusConbit.Bit.GiveLimit;
	PosRef.PosNow = p->PPBuffer0.LTargetPostion;//传输的是上一次的位置值
	if(OTP.ZeroPosLockFlg > 0)
	{
		PosRef.PosOld = PosRef.PosNow;
	}
	p->PPBuffer0.LTargetPostion = p->PPBuffer0.TargetPostion;
	//PosRef.InterpdataRem = 0;//余数清零 2023.10.30删除

}
/**
 * CSP模块给定
 */
void CSP_DataGetHandle(Cia402PP_handle p)
{
	int32 Max_PositionLimit;
	int32 Min_PositionLimit;
	int32 P_user;//位置给定中间变量
	int32 v_user;//2018.8.20    xiugaiqian uint32 v_user;
	INT32 v_givetodrive;

	TCiA402Axis *pLocalAxes = &LocalAxes;
    
	TYPE_STATEMACHINE *sm = &StateMachine;
    
	p->StatusConbit.Bit.GiveLimit = 0;
    
	P_user = pLocalAxes->Objects.objTargetPosition + pLocalAxes->Objects.objPositionoffset;

	Min_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit - pLocalAxes->Objects.objHomeOffset;
	Max_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MaxLimit - pLocalAxes->Objects.objHomeOffset;

	if(P_user > Max_PositionLimit && sm->SrvOnStatus && pLocalAxes->Objects.objSoftwarePositionLimit.i32MaxLimit != 0x7FFFFFFF)
    {
		//位置正向给定极限
		//FaultPrtt_FaultInterface(POSLIMIT_F);
        OTP.Ccw_CwFlg = 1;
	}
    else if(P_user < Min_PositionLimit && sm->SrvOnStatus && pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit != 0x80000000)
    {
        //位置负向给定极限
		//FaultPrtt_FaultInterface(POSLIMIT_N);
      
        OTP.Ccw_CwFlg = 2;
    }
    else
    {
        //OTP.Ccw_CwFlg = 0; //在超程保护哪里清除
    }

	//位置给定
	p->PPBuffer0.TargetPos607A = P_user;
    P_user = (INT32)(((INT64)P_user * PosCmxCdv.PosCmxNum) / PosCmxCdv.PosCdv) - p->PosFbInitValue;  //p->PPBuffer0.TargetPostion = (INT32)(((INT64)P_user * PosCmxCdv.PosCmxNum) / PosCmxCdv.PosCdv) - p->PosFbInitValue;
    p->PPBuffer0.TargetPostion = P_user - p->PosOffset;
    
	//速度前馈
	v_user = pLocalAxes->Objects.objVelocityOffset;		
	v_givetodrive = (INT32)(pLocalAxes->SpdCmdPU * (float64)v_user);    //脉冲单位转换成标幺值
	p->PPBuffer0.VelOffset60B1 = v_givetodrive;                      //速度偏移0x60B1

	//力矩前馈
	p->PPBuffer0.TorqueOffset60B2 = pLocalAxes->Objects.objTorqueOffset;            //力矩偏移0x60C2
    
	//最大力矩
	p->PPBuffer0.MaxTorque = pLocalAxes->Objects.objMaxTorque;//最大扭矩0x6072
    
	p->PPBuffer0.ConBit.Bit.GiveLimit = p->StatusConbit.Bit.GiveLimit;
    
	//PosRef.PosNow = p->PPBuffer0.LTargetPostion;//传输的是上一次的位置值
    
//	if(OTP.ZeroPosLockFlg > 0)移入 1ms中断处理
//	{
//        if(FullCloseLoop.FullFlag > 0)
//        {
//            PosRef.PosOld = (INT32)((int64)ExtPosFb.PosFbTotle * FullCloseLoop.InnerRev/FullCloseLoop.ExtRev);
//        }
//        else
//        {
//            PosRef.PosOld = PosFb.PosFbTotle;//2022.12.24修复超程位置不准确 
//        }
//	}
    
	p->PPBuffer0.LTargetPostion = p->PPBuffer0.TargetPostion;
	//PosRef.InterpdataRem = 0;//余数清零2023.10.30删除
}
/**
 * HM模块给定
 */
void HM_DataGetHandle(Cia402PP_handle p)
{
	TCiA402Axis *pLocalAxes = &LocalAxes;
	//更新给定变量
	INT32  P_pluse = 0;                        //脉冲单位位置给定
	UINT32 Dec_User = 0;
	UINT32 Dec_GivetoDrive = 0;               //标幺值单位
	UINT32 V_pluse = 0;                       //脉冲单位速度给定
	UINT32 V_GivetoDrive = 0;                 //标幺值单位
	p->StatusConbit.Bit.GiveLimit = 0;
	p->PPBuffer0.HomeMethod = pLocalAxes->Objects.objHomeMethod;            //原点回归方法0x6098
	
    #if SERVOPOWER == SERVO_DC
    if(p->PPBuffer0.HomeMethod < 33)
        p->PPBuffer0.HomeMethod = 33;
    #endif
    
    //速度和加减速为限制
    /*******************SPD switch**********************/
//	V_pluse = (INT64)pLocalAxes->Objects.objHomingSpeeds.i32Speedforswitch*\
//			 PosCmxCdv.PosCmxNum/PosCmxCdv.PosCdv;//用户单位转换成脉冲单位
    
    V_pluse = pLocalAxes->Objects.objHomingSpeeds.i32Speedforswitch;
	V_GivetoDrive =  (INT32)(pLocalAxes->SpdCmdPU * (float64)V_pluse);
	p->PPBuffer0.HomingSpeeds.i32Speedforswitch = V_GivetoDrive;//原点回归速度0x6099.1
    /*******************SPD switch**********************/
    
    /*******************SPD zero**********************/
//	V_pluse = (INT64)pLocalAxes->Objects.objHomingSpeeds.i32Speedforzero*\
//	           PosCmxCdv.PosCmxNum/PosCmxCdv.PosCdv;//用户单位转换成脉冲单位
    
    V_pluse = pLocalAxes->Objects.objHomingSpeeds.i32Speedforzero;
	V_GivetoDrive =  (int32)(pLocalAxes->SpdCmdPU * (float64)V_pluse);
	p->PPBuffer0.HomingSpeeds.i32Speedforzero = V_GivetoDrive;//原点回归速度0x6099.2
    /*******************SPD zero**********************/
    
    /*******************ACC DEC**********************/
//	Dec_User = (INT64)pLocalAxes->Objects.objHomingAcceleration*\
//			PosCmxCdv.PosCmxNum/PosCmxCdv.PosCdv;
    
    Dec_User = pLocalAxes->Objects.objHomingAcceleration;
	Dec_GivetoDrive = (UINT32)(LocalAxes.PosAccDecPu * (float64)Dec_User);
	p->PPBuffer0.HomingAcc = Dec_GivetoDrive;//原点回归加速度0x609A
    /*******************ACC DEC**********************/
    
	P_pluse = (INT32)((INT64)pLocalAxes->Objects.objHomeOffset*\
			PosCmxCdv.PosCmxNum / PosCmxCdv.PosCdv);/*采用分子/分母*/
	p->PPBuffer0.HomeOffset = P_pluse;//原点偏移0x607C
    
	p->PPBuffer0.ConBit.Bit.GiveLimit = p->StatusConbit.Bit.GiveLimit;
	p->PPBuffer0.ControlWord.All = pLocalAxes->Objects.objControlWord;//获取当前的控制字
}
/********************PV模块给定*******************/
void PV_DataGetHandle(Cia402PP_handle p)
{
//	TOBJ607E polatity ={0};
//	float64 tmpdb = 0;                         //速度，加速度因子 2023.4.2删除
	UINT32 Acc_user = 0;					   //对象字典传递
//	float64 Acc_given = 0;                     //加速度给定       2023.4.2删除
	INT32 Acc_GivetoDrive = 0;                 //标幺值单位
	Uint32 Dec_user = 0;                       //对象字典传递
//	float64 Dec_given = 0;                     //减速度给定       2023.4.2删除
	INT32 Dec_GivetoDrive = 0;                 //标幺值单位
	INT32 V_user = 0;                          //用户单位速度给定
	INT32 V_pluse = 0;                         //脉冲单位速度给定
	INT32 V_GivetoDrive = 0;                   //标幺值单位
	TCiA402Axis *pLocalAxes = &LocalAxes;
//	polatity.All = pLocalAxes->Objects.objPolarity;
	p->StatusConbit.Bit.GiveLimit = 0;
    
	if(pLocalAxes->Objects.objMaxProfileVelocity > Common_Abs(pLocalAxes->Objects.objTargetVelocity))
	{
		V_user = pLocalAxes->Objects.objTargetVelocity;
	}
	else
	{
		if(pLocalAxes->Objects.objTargetVelocity < 0)
		{
			V_user = -pLocalAxes->Objects.objMaxProfileVelocity;
		}
		else
		{
			V_user = pLocalAxes->Objects.objMaxProfileVelocity;
		}
		p->StatusConbit.Bit.GiveLimit = 1;
	}
    
	//极性转换
#if 0
	if(polatity.Bit.VelPol)
	{
		V_user = -V_user;
	}
#endif
    
    //tmpdb = (float64)PosCmxCdv.PosCmxNum / PosCmxCdv.PosCdv;  2023.4.2删除
    V_pluse = V_user;
	//速度单位转换
	V_GivetoDrive = (INT32)(pLocalAxes->SpdCmdPU * (float64)V_user);         //脉冲单位转换成标幺值
    
	//加速度限定
	if(pLocalAxes->Objects.objMaxAcceleration > pLocalAxes->Objects.objProfileAcceleration)
	{
		Acc_user =  pLocalAxes->Objects.objProfileAcceleration;
	}
	else
	{
		Acc_user = pLocalAxes->Objects.objMaxAcceleration;
		p->StatusConbit.Bit.GiveLimit = 1;
	}
	
	Acc_GivetoDrive = (INT32)(LocalAxes.SpdAccDecPu * (float64)Acc_user);
	if(!Acc_GivetoDrive)
	{
		Acc_GivetoDrive = 1;
	}
    
	//减速度设定
	if(pLocalAxes->Objects.ojbMaxDeceleration > pLocalAxes->Objects.objProfileDeceleration)
	{
		Dec_user = pLocalAxes->Objects.objProfileDeceleration;
	}
	else
	{
		Dec_user = pLocalAxes->Objects.ojbMaxDeceleration;
		p->StatusConbit.Bit.GiveLimit = 1;
	}
	Dec_GivetoDrive = (INT32)(LocalAxes.SpdAccDecPu * (float64)Dec_user);
	if(!Dec_GivetoDrive)
	{
		Dec_GivetoDrive = 1;
	}
    
	p->PPBuffer0.TargetSpd = V_pluse;         //目标速度，用户单位
	p->PPBuffer0.TargetVelo = V_GivetoDrive;  //目标速度,标要值
	p->PPBuffer0.TargetAcc = Acc_GivetoDrive; //目标加速度，标要值
	p->PPBuffer0.TargetDec = Dec_GivetoDrive; //目标减速度，标要值
	p->PPBuffer0.ConBit.Bit.GiveLimit = p->StatusConbit.Bit.GiveLimit;
}
/**
 * CSV模块给定
 */
void CSV_DataGetHandle(Cia402PP_handle p)
{
	INT32 v_user;
	INT32 v_givetodrive;
	//TOBJ607E polatity ={0};
	TCiA402Axis *pLocalAxes = &LocalAxes;
//	polatity.All = pLocalAxes->Objects.objPolarity;
	p->StatusConbit.Bit.GiveLimit = 0;
    
	v_user = pLocalAxes->Objects.objTargetVelocity + pLocalAxes->Objects.objVelocityOffset;
    
    if(pLocalAxes->Objects.objMaxProfileVelocity > Common_Abs(v_user))
	{
		v_user = v_user;
	}
	else
	{
		if(v_user < 0)
		{
			v_user = -pLocalAxes->Objects.objMaxProfileVelocity;
		}
		else
		{
			v_user = pLocalAxes->Objects.objMaxProfileVelocity;
		}
		p->StatusConbit.Bit.GiveLimit = 1;
	}
    
    
#if 0
	if(polatity.Bit.VelPol)
	{
		v_user = -v_user;
	}
#endif
    
	//v_user = (INT32)((INT64)v_user * PosCmxCdv.PosCmxNum / PosCmxCdv.PosCdv); 2023.4.2删除
	v_givetodrive =  (INT32)(pLocalAxes->SpdCmdPU * (float64)v_user);
	p->PPBuffer0.TargetVelo = v_givetodrive;
    
	p->PPBuffer0.TorqueOffset60B2 = pLocalAxes->Objects.objTorqueOffset;        //力矩偏移0x60C2
	
	//最大力矩
	p->PPBuffer0.MaxTorque = pLocalAxes->Objects.objMaxTorque; //最大扭矩0x6072
	p->PPBuffer0.ConBit.Bit.GiveLimit = p->StatusConbit.Bit.GiveLimit;
}
/**
 * TQ模块给定
 */
void TQ_DataGetHandle(Cia402PP_handle p)
{
	TCiA402Axis *pLocalAxes = &LocalAxes;
	p->StatusConbit.Bit.GiveLimit = 0;
	if(pLocalAxes->Objects.objTargetTorque >= 0)//正向力矩
	{
		//正向力矩下找出最大限制值
		p->PPBuffer0.MaxTorque = Common_Min2(pLocalAxes->Objects.objMaxTorque,pLocalAxes->Objects.objPosTqLimitValue);
		if(pLocalAxes->Objects.objTargetTorque > p->PPBuffer0.MaxTorque)//给定大于限制
		{
			p->PPBuffer0.TargetTorque = p->PPBuffer0.MaxTorque;//选择最大力矩
			p->StatusConbit.Bit.GiveLimit = 1;//力矩限制标志位置1
		}
		else
		{
			p->PPBuffer0.TargetTorque = pLocalAxes->Objects.objTargetTorque;
		}
	}
	else//反向力矩
	{
		//反向力矩下找出最大限制值
		p->PPBuffer0.MaxTorque = Common_Min2(pLocalAxes->Objects.objMaxTorque,pLocalAxes->Objects.objNegTqLimitValue);
		if((-pLocalAxes->Objects.objTargetTorque) > p->PPBuffer0.MaxTorque)//给定大于限制
		{
			p->PPBuffer0.TargetTorque = -p->PPBuffer0.MaxTorque;//选择最大力矩
			p->StatusConbit.Bit.GiveLimit = 1;//力矩限制标志位置1
		}
		else
		{
			p->PPBuffer0.TargetTorque = pLocalAxes->Objects.objTargetTorque;
		}
	}

    if(pLocalAxes->Objects.objMaxProfileVelocity > pLocalAxes->Objects.objMaxMotorSpeed)
    {
        pLocalAxes->Objects.objMaxProfileVelocity = pLocalAxes->Objects.objMaxMotorSpeed;
    }

    p->PPBuffer0.TorqueSpdLim = (INT32)(pLocalAxes->SpdCmdPU * (float64)pLocalAxes->Objects.objMaxProfileVelocity);
	p->PPBuffer0.TargetSlop = (INT32)((float)pLocalAxes->Objects.objTorqueSlop * pLocalAxes->TorquePU); //目标加速度，标要值
	p->PPBuffer0.ConBit.Bit.GiveLimit = p->StatusConbit.Bit.GiveLimit;
}
/**
 * CST模块给定
 */
void CST_DataGetHandle(Cia402PP_handle p)
{
	INT32 Torque = 0;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	p->StatusConbit.Bit.GiveLimit = 0;
	Torque = pLocalAxes->Objects.objTargetTorque + pLocalAxes->Objects.objTorqueOffset;//力矩偏移0x60C2
	
    if(Torque >= 0)//正向力矩
	{
		//正向力矩下找出最大限制值
		p->PPBuffer0.MaxTorque = Common_Min2(pLocalAxes->Objects.objMaxTorque,pLocalAxes->Objects.objPosTqLimitValue);
		if(Torque > p->PPBuffer0.MaxTorque)//给定大于限制
		{
			p->PPBuffer0.TargetTorque = p->PPBuffer0.MaxTorque;//选择最大力矩
			p->StatusConbit.Bit.GiveLimit = 1;//力矩限制标志位置1
		}
		else
		{
			p->PPBuffer0.TargetTorque = Torque;
		}
	}
	else//反向力矩
	{
		//反向力矩下找出最大限制值
		p->PPBuffer0.MaxTorque = Common_Min2(pLocalAxes->Objects.objMaxTorque,pLocalAxes->Objects.objNegTqLimitValue);
		if((-Torque) > p->PPBuffer0.MaxTorque)//给定大于限制
		{
			p->PPBuffer0.TargetTorque = -p->PPBuffer0.MaxTorque;//选择最大力矩
			p->StatusConbit.Bit.GiveLimit = 1;//力矩限制标志位置1
		}
		else
		{
			p->PPBuffer0.TargetTorque = Torque;
		}
	}

    if(pLocalAxes->Objects.objMaxProfileVelocity > pLocalAxes->Objects.objMaxMotorSpeed)
    {
        pLocalAxes->Objects.objMaxProfileVelocity = pLocalAxes->Objects.objMaxMotorSpeed;
    }
    p->PPBuffer0.TorqueSpdLim = (INT32)(pLocalAxes->SpdCmdPU * (float64)pLocalAxes->Objects.objMaxProfileVelocity);
    
	//插补时间
	//p->PPBuffer0.InterTime60C2.u8InterpolationPeriod = pLocalAxes->Objects.objInterpolationTimePeriod.u8InterpolationPeriod;
	//p->PPBuffer0.InterTime60C2.i8InterpolationIndex = pLocalAxes->Objects.objInterpolationTimePeriod.i8InterpolationIndex;//插补时间0x60C2
	p->PPBuffer0.ConBit.Bit.GiveLimit = p->StatusConbit.Bit.GiveLimit;
}
/******************位置到达模块处理*****************************/
/**
 * 程序执行时间（在PWM中断打开的情况下测试）：2us
 */
void cia402_PosReachHanle(Cia402PP_handle p)             //目标位置限制与到达判断
{
	int32 tmp = 0;
	TCiA402Axis *pLocalAxes = &LocalAxes;
    
	//更新给定参数
	if(p->PPBuffer0.TargetPostion == p->PosDemVal60FC )
	{
		p->PPBuffer0.ConBit.Bit.BufferFull = 0;
	}

	tmp = p->PPBuffer0.TargetPos607A - (pLocalAxes->Objects.objPositionActualValue - p->PosFbInitValue_user);// YH

	if(Common_Abs(tmp) <= pLocalAxes->Objects.objPositionWindow)//本段已经到达目标范围，注意滤波处理
	{
		p->ReachedTime += p->CycleTime;
	}
	else
	{
		p->ReachedTime = 0;
	}
    
	//位置到达判断
	if(p->ReachedTime >= pLocalAxes->Objects.objPositionWindowTime && p->ReachedTime)
	{
		p->StatusConbit.Bit.ThisPosFinished = 1;
	}
	else
	{
		p->StatusConbit.Bit.ThisPosFinished = 0;
	}
    
	//完成路程判断
	if(p->StatusConbit.Bit.ThisPosFinished && !p->PPBuffer1.ConBit.Bit.BufferFull)
	{
		pLocalAxes->Objects.objStatusWord |= 1<<10;
		Glo_DO_FUNC.Flag_Func0.bit.COIN = 1;//位置到达
		//DO位置到达输出
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<10);
		Glo_DO_FUNC.Flag_Func0.bit.COIN = 0;//位置未到达
		//DO位置到达输出无效
	}
}
/************************速度到达模块*******************/
void cia402_VelReachHanle(Cia402PP_handle p)
{
	TCiA402Axis *pLocalAxes = &LocalAxes;
	INT64 tmpvel = 0;
    INT32 P_ver = 0;
    INT16 vel = 0;
	tmpvel = (INT64)p->PPBuffer0.TargetSpd;//非标幺值，用户单位
    tmpvel = tmpvel * PosCmxCdv.PosCmxNum / PosCmxCdv.PosCdv;
	vel = (INT16)(tmpvel * LocalAxes.Encoderres);
	vel = vel - RamMonitor->RotatingSpd;
	vel = Common_Abs(vel);
    
	//速度到达滤波
	if(vel <= pLocalAxes->Objects.objVelocityWindow)//本段已经到达目标范围，注意滤波处理
	{
		p->VelReachedTime += p->CycleTime;
	}
	else
	{
		p->VelReachedTime = 0;
	}
	//速度到达
	if(p->VelReachedTime >= pLocalAxes->Objects.objVelocityWindowTime && p->VelReachedTime)
	{
		p->VelReachedTime = pLocalAxes->Objects.objVelocityWindowTime;
		pLocalAxes->Objects.objStatusWord |= 1<<10;
		Glo_DO_FUNC.Flag_Func0.bit.V_CMP = 1;//速度到达
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<10);
		Glo_DO_FUNC.Flag_Func0.bit.V_CMP = 0;//速度未到达
	}
    
	//零速判断,滤波
	P_ver = Common_Abs(pLocalAxes->Objects.objVelocityActualValue);
	vel = (INT16)((float64)P_ver * LocalAxes.Encoderres);

	if(vel <= pLocalAxes->Objects.objVelocitythreshold)//本段已经到达目标范围，注意滤波处理
	{
		p->VelZeroTime += p->CycleTime;
	}
	else
	{
		p->VelZeroTime = 0;
	}
    
	//零速判断，到达,DO输出在驱动里面做
	if(p->VelZeroTime >= pLocalAxes->Objects.objVelocitythresholdTime && p->VelZeroTime)
	{
		pLocalAxes->Objects.objStatusWord |= (1<<12);
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
	}
}
/************************力矩到达模块*******************/
void cia402_TqReachdeHandle(Cia402PP_handle p)
{
	int32 tmp = 0;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	tmp = p->PPBuffer0.TargetTorque;
	tmp = tmp - pLocalAxes->Objects.objTorqueActualValue;
	if(Common_Abs(tmp) <= RamBasePara->TrqArriveLmt)//本段已经到达目标范围，注意滤波处理
	{
		p->TqReachedTime += p->CycleTime;
	}
	else
	{
		p->TqReachedTime = 0;
	}
	if(p->TqReachedTime>10)
	{
		p->TqReachedTime = 11;
		pLocalAxes->Objects.objStatusWord |= 1<<10;
		Glo_DO_FUNC.Flag_Func0.bit.T_CMP = 1;//转矩到达
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<10);
		Glo_DO_FUNC.Flag_Func0.bit.T_CMP = 0;//转矩到达
	}
}

/************************跟随误差判断*******************/
void cia402_FloowErr(Cia402PP_handle p)
{
	INT32 tmp1 = 0;
	UINT32 tmp = 0;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	TYPE_STATEMACHINE *sm = &StateMachine;
	TYPE_OTPROTECT *ot = &OTP;
    
	if(!sm->OffZSpdStopFlg)//在执行其他停车方式时不动作
	{
		//6062 - 6064
		//tmp1 = pLocalAxes->Objects.objPositionDemandValue - pLocalAxes->Objects.objPositionActualValue;
        tmp1 = (INT32)(((INT64)PosCmxCdv.PosCdv * (INT64)PosReg.Err) / PosCmxCdv.PosCmxNum);
		
        pLocalAxes->Objects.objFollowingErrorActualValue = tmp1;

		tmp = Common_Abs(tmp1);
        //if(tmp >= pLocalAxes->Objects.objFollowingErrorWindow && sm->RegulFlg && 0 == ot->DirOT)//本段已经到达目标范围，注意滤波处理
		if(tmp >= pLocalAxes->Objects.objFollowingErrorWindow && sm->RegulFlg && 0 == ot->POTNOTFlg)//DirOT 会清零，如果中断打断可能会误报位置偏差
		{
			p->FollowErrtime += p->CycleTime;
		}
		else
		{
			p->FollowErrtime = 0;
		}
        
		//跟随误差过大
		if(p->FollowErrtime >= (pLocalAxes->Objects.objFollowErrorTimeOut + 1) && p->FollowErrtime)
		{
			pLocalAxes->Objects.objStatusWord |= 1<<13;
			FaultPrtt_FaultInterface(OvPosErr);//位置偏差过大
		}
	}
}

/*************************反馈模块*****************************/
/**
 * 执行时间为
 */
void cia402_FeedbackHanle(Cia402PP_handle p)
{
	TCiA402Axis *pLocalAxes = &LocalAxes;

    /*位置单位转换单位转换*/
    pLocalAxes->Objects.objPositionDemandInterValue = p->PosDemVal60FC + Cia402_PP.PosOffset;//YH         60FC  内部单位
    pLocalAxes->Objects.objPositionActualInterValue = p->GetPos6063    + Cia402_PP.PosOffset;//内部脉冲单位  6063  内部单位
  
    pLocalAxes->Objects.objPositionDemandValue = (INT32)(((INT64)PosCmxCdv.PosCdv * (INT64)pLocalAxes->Objects.objPositionDemandInterValue) / PosCmxCdv.PosCmxNum);
    pLocalAxes->Objects.objPositionActualValue = (INT32)(((INT64)PosCmxCdv.PosCdv * (INT64)pLocalAxes->Objects.objPositionActualInterValue) / PosCmxCdv.PosCmxNum);      
 
    /*反馈速度*/
    pLocalAxes->Objects.objVelocityActualValue = (INT32)(pLocalAxes->SpdFbPu * (double)p->GetVel606C);
    pLocalAxes->Objects.objVelocityDemandValue = (INT32)(pLocalAxes->SpdFbPu * (double)p->VelDemVal606B);
    
    /*反馈转矩*/
    pLocalAxes->Objects.objTorqueActualValue = p->GetTor6077;
    pLocalAxes->Objects.objTorqueDemandValue = p->TorDemVal6074;
    RamPC->ActualTorClose = p->GetTorqueClose;

    if(LocalAxes.Objects.objModesOfOperation == (int8)ETHERCATSTATE_MIT)
    {
        Mit_WriteEsc();
    }

    //0x603F的处理  0x1001处理放在CANopen应用层
	if(FaultP.FaultStatus & 0x3)//有警告或者故障发生
	{
		//CumObj.u16ErrorRegister = 1;
		LocalAxes.Objects.objErrorCode = FaultP.FaultDisp;
	}
	else
	{
		//CumObj.u16ErrorRegister = 0;
		LocalAxes.Objects.objErrorCode = 0;
	}
}
/***************** 位置控制模式******************/
/**
 * 函数执行时间为130us
 */
void cia402_PP(void)
{
	Cia402PP*pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	TOBJ6040 ControlWord;
	ControlWord.All = pLocalAxes->Objects.objControlWord;//获取当前的控制字
	cia402_FloowErr(pCia402_PP);//跟随误差
	if(ControlWord.PPBit.ChangeSetImmd)//立即改变
	{
		if(!pCia402_PP->LastControlWord.PPBit.NewSetPoint && ControlWord.PPBit.NewSetPoint )//Con.bit4上升沿到来
		{
			pLocalAxes->Objects.objStatusWord |= 1<<12;
			PP_DataGetHandle(pCia402_PP,0);//给定处理
		}
	}
	else//带一级缓存
	{
		if(!pCia402_PP->PPBuffer0.ConBit.Bit.BufferFull && pCia402_PP->PPBuffer1.ConBit.Bit.BufferFull)//如果Buffer0是空的，Buffer1是满的
		{
			 HMEMCPY(&pCia402_PP->PPBuffer0,&pCia402_PP->PPBuffer1,SIZEOF(Control_PPBuffer));
			 pCia402_PP->PPBuffer1.ConBit.Bit.BufferFull = 0;
		}
		if((!pCia402_PP->LastControlWord.PPBit.NewSetPoint) && ControlWord.PPBit.NewSetPoint )//Con.bit4上升沿到来
		{
			pLocalAxes->Objects.objStatusWord |= 1<<12;
			if(!pCia402_PP->PPBuffer0.ConBit.Bit.BufferFull && !pCia402_PP->PPBuffer1.ConBit.Bit.BufferFull)//两个都空
			{
				//将新的数据给Buffer0
				PP_DataGetHandle(pCia402_PP,0);//给定处理
			}
			else if(pCia402_PP->PPBuffer0.ConBit.Bit.BufferFull && !pCia402_PP->PPBuffer1.ConBit.Bit.BufferFull)//BUFFER1 正在处理
			{
				//将新的数据给Buffer1
				PP_DataGetHandle(pCia402_PP,1);//给定处理
			}
		}
	}
	if(!ControlWord.PPBit.NewSetPoint&&!pCia402_PP->PPBuffer1.ConBit.Bit.BufferFull)
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
	}

	if(ControlWord.PPBit.Halt && pLocalAxes->i16State == STATE_OPERATION_ENABLED)
	{
        pLocalAxes->OffStop_HaltStop = CiA402_HaltStopAction(pLocalAxes->Objects.objHaltOptionCode);//发出停机指令
	}
	else
	{
		cia402_PosReachHanle(pCia402_PP);//位置到达判断
		pLocalAxes->OffStop_HaltStop = 0;
	}
    
	//更新极性
    OTP.Ccw_CwFlg = 0;
	pCia402_PP->StatusConbit.Bit.Polar = pLocalAxes->Objects.objPolarity&0x80;//更新极性，只跟bit7有关
	pCia402_PP->LastControlWord.All = ControlWord.All;
	//DO控制
	//位置到达、速度到达
}
/**
 *
 */
void cia402_IP(void)
{
	Cia402PP*pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	TOBJ6040 ControlWord;
	ControlWord.All = pLocalAxes->Objects.objControlWord;//获取当前的控制字
	cia402_FloowErr(pCia402_PP);//跟随误差
	if(ControlWord.IPBit.CSPstart)//使能插补模式
	{
		IP_DataGetHandle(pCia402_PP);
		pLocalAxes->Objects.objStatusWord |= 1<<12;
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
	}

	//Halt操作
	if(ControlWord.IPBit.Halt && pLocalAxes->i16State == STATE_OPERATION_ENABLED)
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
		pLocalAxes->OffStop_HaltStop = CiA402_HaltStopAction(pLocalAxes->Objects.objHaltOptionCode);//发出停机指令
	}
	else
	{
		cia402_PosReachHanle(pCia402_PP);//位置到达判断
		pLocalAxes->OffStop_HaltStop = 0;
	}
	pCia402_PP->StatusConbit.Bit.Polar = (pLocalAxes->Objects.objPolarity&0x80)>>7;//更新极性，只跟bit7有关
}
/**
 * 同步位置模式给定模块
 */
void cia402_CSP(void)
{
	Cia402PP *pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	pLocalAxes->Objects.objStatusWord &= ~(9<<10);//bit10,bit13清0
	cia402_FloowErr(pCia402_PP);//跟随误差

	if(pLocalAxes->bAxisFunctionEnabled)
	{
		pLocalAxes->Objects.objStatusWord |= 1<<12;
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
	}

	CSP_DataGetHandle(pCia402_PP);//给定模块
    
    pLocalAxes->OffStop_HaltStop = 0;
	pCia402_PP->StatusConbit.Bit.Polar = (pLocalAxes->Objects.objPolarity&0x80) >> 7;//更新极性，只跟bit7有关
}
/**
 * 原点回归模式处理模块
 */
void cia402_HM(void)
{
	Cia402PP*pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	TOBJ6040 ControlWord;
	HM_DataGetHandle(pCia402_PP);
	//Halt操作
	ControlWord.All = pLocalAxes->Objects.objControlWord;//获取当前的控制字
	if(ControlWord.HMBit.Halt && pLocalAxes->i16State == STATE_OPERATION_ENABLED)
	{
		pLocalAxes->OffStop_HaltStop = CiA402_HaltStopAction(pLocalAxes->Objects.objHaltOptionCode);//发出停机指令
	}
	else
	{
		pLocalAxes->OffStop_HaltStop = 0;
	}
	//状态字更新
	if(Cia402_PP.HMstatusWord.Bit.HomingAttained)
	{
		pLocalAxes->Objects.objStatusWord |= 1<<12;
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
	}
	if(Cia402_PP.HMstatusWord.Bit.TargetReached)
	{
		pLocalAxes->Objects.objStatusWord |= 1<<10;
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<10);
	}
	pCia402_PP->StatusConbit.Bit.Polar = 0;
}
/**
 * PV模式处理模块
 */
void cia402_PV(void)
{
	Cia402PP*pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	TOBJ6040 ControlWord;
	ControlWord.All = pLocalAxes->Objects.objControlWord;//获取当前的控制字
	PV_DataGetHandle(pCia402_PP);//更新参数
	if(ControlWord.PVBit.Halt && pLocalAxes->i16State == STATE_OPERATION_ENABLED)
	{
		pLocalAxes->OffStop_HaltStop = CiA402_HaltStopAction(pLocalAxes->Objects.objHaltOptionCode);//发出停机指令
	}
	else
	{
		cia402_VelReachHanle(pCia402_PP);//速度到达判断
		pLocalAxes->OffStop_HaltStop = 0;
	}
	pCia402_PP->StatusConbit.Bit.Polar = (pLocalAxes->Objects.objPolarity&0x40)>>6;//更新极性，只跟bit7有关
}
/**
 *CSV处理模块
 */
void cia402_CSV(void)
{
	Cia402PP*pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	pLocalAxes->Objects.objStatusWord &= ~(9<<10);//bit10,bit13清0
	if(pLocalAxes->bAxisFunctionEnabled)
	{
		pLocalAxes->Objects.objStatusWord |= 1<<12;
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
	}
	CSV_DataGetHandle(pCia402_PP);
    
    pLocalAxes->OffStop_HaltStop = 0;
	pCia402_PP->StatusConbit.Bit.Polar = (pLocalAxes->Objects.objPolarity&0x40)>>6;//更新极性，只跟bit7有关
}
/**
 * TQ处理模块
 */
void cia402_TQ(void)
{
	Cia402PP*pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	TOBJ6040 ControlWord;
    
    if(pLocalAxes->bAxisFunctionEnabled)
	{
		pLocalAxes->Objects.objStatusWord |= 1<<12;
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
	}
    
	TQ_DataGetHandle(pCia402_PP);
	ControlWord.All = pLocalAxes->Objects.objControlWord;//获取当前的控制字
	if(ControlWord.TQBit.Halt && pLocalAxes->i16State == STATE_OPERATION_ENABLED)
	{
		pLocalAxes->OffStop_HaltStop = CiA402_HaltStopAction(pLocalAxes->Objects.objHaltOptionCode);//发出停机指令
	}
	else
	{
		cia402_TqReachdeHandle(pCia402_PP);
		pLocalAxes->OffStop_HaltStop = 0;
        pLocalAxes->EcatStopMode = 0;
	}
	pCia402_PP->StatusConbit.Bit.Polar = 0;
}
/**
 * CST处理模块
 */
void cia402_CST(void)
{
	Cia402PP*pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	pLocalAxes->Objects.objStatusWord &= ~(9<<10);//bit10,bit13清0
	if(pLocalAxes->bAxisFunctionEnabled)
	{
		pLocalAxes->Objects.objStatusWord |= 1<<12;
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
	}
	CST_DataGetHandle(pCia402_PP);
    
    pLocalAxes->OffStop_HaltStop = 0;
	pCia402_PP->StatusConbit.Bit.Polar = 0;
}

void cia402LimCacl(void)
{
    INT32 Min_PositionLimit,Max_PositionLimit,PosFb;

	TCiA402Axis *pLocalAxes = &LocalAxes;
    TYPE_STATEMACHINE *sm = &StateMachine;    
    
    Min_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit - pLocalAxes->Objects.objHomeOffset;
	Max_PositionLimit = pLocalAxes->Objects.objSoftwarePositionLimit.i32MaxLimit - pLocalAxes->Objects.objHomeOffset;

    PosFb = pLocalAxes->Objects.objPositionActualValue;
	if(PosFb > Max_PositionLimit && sm->SrvOnStatus && pLocalAxes->Objects.objSoftwarePositionLimit.i32MaxLimit != 0x7FFFFFFF)
    {
		//位置正向给定极限
        OTP.Ccw_CwFlg = 1;
	}
    else if(PosFb < Min_PositionLimit && sm->SrvOnStatus && pLocalAxes->Objects.objSoftwarePositionLimit.i32MinLimit != 0x80000000)
    {
        //位置负向给定极限
      
        OTP.Ccw_CwFlg = 2;
    }
    else
    {
        //OTP.Ccw_CwFlg = 0; //在超程保护哪里清除
    }    
}

void cia402_TorQueClose(void)
{
	Cia402PP*pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	pLocalAxes->Objects.objStatusWord &= ~(9<<10);//bit10,bit13清0
	if(pLocalAxes->bAxisFunctionEnabled)
	{
		pLocalAxes->Objects.objStatusWord |= 1<<12;
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~(1<<12);
	}

    pCia402_PP->PPBuffer0.TargetTorqueClose = RamPC->TargetTorClose;
    pCia402_PP->PPBuffer0.TorqueSpdLim = (INT32)(pLocalAxes->SpdCmdPU * (float64)pLocalAxes->Objects.objMaxProfileVelocity);
    
    pLocalAxes->OffStop_HaltStop = 0;
}

void Cia402TorLimCacl(void)  
{
    UINT16  Torque = 0;
    UINT16  Torque_N = 0;
    UINT16  Torque_P = 0;

    Torque = LocalAxes.Objects.objMaxTorque;

    //正向力矩下找出最大限制值
    if(Torque > LocalAxes.Objects.objPosTqLimitValue)//给定大于限制
    {
        Torque_P = LocalAxes.Objects.objPosTqLimitValue;
    }
    else
    {
        Torque_P = LocalAxes.Objects.objMaxTorque;
    }

    //反向力矩下找出最大限制值
    if(Torque > LocalAxes.Objects.objNegTqLimitValue)//给定大于限制
    {
        Torque_N = LocalAxes.Objects.objNegTqLimitValue;
    }
    else
    {
        Torque_N = LocalAxes.Objects.objMaxTorque;
    }

    Cia402_PP.PPBuffer0.MaxPosTorque = Torque_P;            //最大正扭矩
    Cia402_PP.PPBuffer0.MaxNegTorque = Torque_N;
}

void cia402_MIT(void)
{
	TCiA402Axis *pLocalAxes = &LocalAxes;
    Cia402PP*pCia402_PP = &Cia402_PP;

	LocalAxes.Objects.objStatusWord &= ~(9<<10);

	if(LocalAxes.bAxisFunctionEnabled)
	{
		LocalAxes.Objects.objStatusWord |= 1<<12;
	}
	else
	{
		LocalAxes.Objects.objStatusWord &= ~(1<<12);
	}
    
    Mit_ReadEsc();

    pCia402_PP->PPBuffer0.TorqueSpdLim = (INT32)(pLocalAxes->SpdCmdPU * (float64)pLocalAxes->Objects.objMaxProfileVelocity);
}

/**
 * 402控制模式
 */

void Control_Mode(void)
{
	TYPE_CtrlMode *cm = &CtrlMode;
	Cia402PP *pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
	switch(cm->ECATCtrlMode)
	{
        case ETHERCATSTATE_PP:/*位置控制模式*/
            cia402_PP();
            Cia402TorLimCacl();
            break;
        case ETHERCATSTATE_PV:/*速度控制模式*/
            pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;//其他模式下速度实时更新
            cia402_PV();
            cia402LimCacl();
            Cia402TorLimCacl();
            break;
        case ETHERCATSTATE_TQ:/*转矩控制模式*/
            pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
            cia402_TQ();
            cia402LimCacl();
            break;
        case ETHERCATSTATE_HM:/*原点回归模式*/
            pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
            cia402_HM();
            Cia402TorLimCacl();
            break;
        case ETHERCATSTATE_IP:/*插补模式*/
            cia402_IP();
            Cia402TorLimCacl();
            break;
        case ETHERCATSTATE_CSP:/*同步位置控制模式*/
            //Location_TargetLimit();
            cia402_CSP();
            Cia402TorLimCacl();
            break;
        case ETHERCATSTATE_CSV:/*同步速度模式*/
            pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
            cia402_CSV();
            cia402LimCacl();
            Cia402TorLimCacl();
            break;
        case ETHERCATSTATE_CST:/*同步力矩模式*/
            pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
            cia402_CST();
            cia402LimCacl();
            break;
        case ETHERCATSTATE_TORCLOSE:
            pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
            cia402_TorQueClose();
            break;
        case ETHERCATSTATE_MIT:
            pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
            cia402_MIT();
            break;
        default:
            pCia402_PP->StatusConbit.Bit.Polar = 0;
            break;
	}
    
	if(pCia402_PP->PPBuffer0.ConBit.Bit.GiveLimit)
	{
		pLocalAxes->Objects.objStatusWord |= STATUSWORD_INTERNAL_LIMIT;
	}
	else
	{
		pLocalAxes->Objects.objStatusWord &= ~STATUSWORD_INTERNAL_LIMIT;
	}
    
	cm->LastECATCtrlMode = cm->ECATCtrlMode;
    
    Updata_EcatFb();
	cia402_FeedbackHanle(pCia402_PP);//反馈接收

    
/*DI/DO信号处理*/
//DI信号包括正限位、反限位和Home
	pLocalAxes->Objects.objDigitalInputs = ((Glo_DI_FUNC.Flag_Func0.bit.N_OT&0x01)<<0)|\
			                               ((Glo_DI_FUNC.Flag_Func0.bit.P_OT&0x01)<<1)|\
			                               ((Glo_DI_FUNC.Flag_Func1.bit.ORGP&0x01)<<2)|\
			                               ((Uint32)(Glo_DI_FUNC.Flag_Func0.bit.ALM_RST&0x01)<<16)|\
										   ((Uint32)(Glo_DI_FUNC.Flag_Func0.bit.INHIBIT&0x01)<<17)|\
										   ((Uint32)(Glo_DI_FUNC.Flag_Func0.bit.GAIN_SEL&0x01)<<18)|\
										   ((Uint32)(Glo_DI_FUNC.Flag_Func1.bit.J_SEL&0x01)<<19)|\
										   ((Uint32)(Glo_DI_FUNC.Flag_Func1.bit.GNUM0&0x01)<<20)|\
										   ((Uint32)(Glo_DI_FUNC.Flag_Func1.bit.GNUM1&0x01)<<21);

//DO信号处理
//IO信号清0处理
	if(cm->ECATCtrlMode != ETHERCATSTATE_PP)
	{
		Glo_DO_FUNC.Flag_Func0.bit.COIN = 0;//位置到达
	}
	if(cm->ECATCtrlMode != ETHERCATSTATE_PV)
	{
		Glo_DO_FUNC.Flag_Func0.bit.V_CMP = 0;//速度到达
	}
	if(cm->ECATCtrlMode != ETHERCATSTATE_TQ)
	{
		Glo_DO_FUNC.Flag_Func0.bit.T_CMP = 0;//转矩到达
	}
/*********************************************************************/
}



void Location_TargetLimitInit(void)
{
//    float temp=0.0;
//    temp=((float64)DrvCoeff.MotEncSglRevLns*DPI_Nmax)/60000;
//    LocalAxes.LocationTarget_limit=(Uint32)(temp*(pSyncManOutPar->u32CycleTime/1000000));//最大转速时1ms对应的目标值增量
}
void Location_TargetLimit(void)
{
    int32 temp=0;
    Uint32 temp1=0;
    static Uint16 i=0;
    TCiA402Axis *pLocalAxes = &LocalAxes;
    TYPE_STATEMACHINE *sm = &StateMachine;

    temp=pLocalAxes->Objects.objTargetPosition-pLocalAxes->objTargetLastPosition;
    temp1 = Common_Abs(temp);

    if((sm->RegulFlg == 1)&&(temp1>pLocalAxes->LocationTarget_limit))//
    {
        i++;
        pLocalAxes->Objects.objTargetPosition=pLocalAxes->objTargetLastPosition;
        if(i>=10)
        {
            FaultPrtt_FaultInterface(SeetPointErr);
        }
    }
    else
    {
        i=0;
        pLocalAxes->objTargetLastPosition=pLocalAxes->Objects.objTargetPosition;
    }
}
#endif
