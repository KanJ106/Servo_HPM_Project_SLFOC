/*
 * SV_FunCodeAttrLit.c
 *
 *  Created on: 2016-5-4
 *      Author: zhangwei//
 */

#include "SV_FuncCode.h"
#include "SV_FuncVar.h"
#include "SV_FunCodeAttrLit.h"
#include "SV_Servocode.h"

TYPE_PreFunc	PreFunc;
void FuncLitProc_init(void);//初始化调用一次//

/******************************************************************************
 *函数名：	FuncLitInit()
 *函数描述：	参数属性限制结构初始化
 *功能：		初始化结构体，值为实际值+1，这样第一次调用时即可每个功能码限制函数都执行
 *输入：		功能码实际值
 *输出：
 *****************************************************************************/
void FuncLitInit(void)
{
	PreFunc.Ne = RamMotor->Ne + 1;				//PE-06电机额定转速
	PreFunc.Nmax = RamMotor->Nmax + 1;			//PE-07电机最大转速
    PreFunc.Tmax = RamMotor->TrqConst + 1;      //PE-12电机最大过载倍数
	PreFunc.ServoCode = RamServo->ServoCode;	//PF-06驱动器代码

	FuncLitProc_init();			                //参数限定-初始化部分
}

/******************************************************************************
**函 数 名：属性最小值限定函数
**描      述：修改功能码属性的最小值
*******************************************************************************/
void LitAttrMin(Uint16 *RamFuncAddr, Uint16 MinVal)
{
	Uint16 AddrCnt;
	TYPE_Attribute *FuncAttrAddr;  //属性结构体
	TYPE_AttrSign *RamVal = (TYPE_AttrSign *)RamFuncAddr,\
			       *Min = (TYPE_AttrSign *)&MinVal;

	AddrCnt = RamFuncAddr - RamFuncStartAddr;
	FuncAttrAddr = FuncAttrStartAddr + AddrCnt;//计算出功能码属性区的地址

	if(FuncAttrAddr->Sign == USIGN)
	{
		FuncAttrAddr->Min.U = Min->U;
		if(RamVal->U < FuncAttrAddr->Min.U)//如果该功能码大于属性的最大值，则等于最大值
		{
			RamVal->U = Min->U;
		}
        
        if(*RamFuncAddr < Min->U) *RamFuncAddr = Min->U;
	}
	else// if(FuncAttrAddr->Sign == SIGN)//为啥不行了，汇编代码也是对的
	{
		FuncAttrAddr->Min.S = Min->S;
		if(RamVal->S < FuncAttrAddr->Min.S)//如果该功能码大于属性的最大值，则等于最大值
		{
			RamVal->S = Min->S;
		}
        
        if(*RamFuncAddr < Min->S) *RamFuncAddr = Min->S;
	}
}

/******************************************************************************
**函 数 名：属性最大值限定函数
**描    述：修改功能码属性的最大值
*******************************************************************************/
void LitAttrMax(Uint16 *RamFuncAddr, Uint16 MaxVal)
{
	Uint16 AddrCnt;
	TYPE_Attribute *FuncAttrAddr;  //属性结构体
	TYPE_AttrSign *RamVal = (TYPE_AttrSign *)(RamFuncAddr),\
					*Max = (TYPE_AttrSign *)&MaxVal;

	AddrCnt = RamFuncAddr - RamFuncStartAddr;
	FuncAttrAddr = FuncAttrStartAddr + AddrCnt;//计算出功能码属性区的地址

	if(FuncAttrAddr->Sign == USIGN)
	{
		FuncAttrAddr->Max.U = Max->U;
		if(RamVal->U > FuncAttrAddr->Max.U)//如果该功能码大于属性的最大值，则等于最大值
		{
			RamVal->U = Max->U;
		}
        
        if(*RamFuncAddr > Max->U) *RamFuncAddr = Max->U;
	}
	else// if(FuncAttrAddr->Sign == SIGN)//为啥不行了，汇编代码也是对的
	{
		FuncAttrAddr->Max.S = Max->S;
		if(RamVal->S> FuncAttrAddr->Max.S)//如果该功能码大于属性的最大值，则等于最大值
		{
			RamVal->S= Max->S;
		}
        
        if(*RamFuncAddr > Max->S) *RamFuncAddr = Max->S;
	}
}

/******************************************************************************
**函 数 名：void LitAttrDot(Uint16 *RamFuncAddr,Uint16 Dotbits)
**描    述：修改功能码小数点函数
*******************************************************************************/
void LitAttrDot(Uint16 *RamFuncAddr,Uint16 Dotbits)
{
	Uint16 AddrCnt;
	TYPE_Attribute *FuncAttrAddr;  //属性结构体

	AddrCnt = RamFuncAddr - RamFuncStartAddr;
	FuncAttrAddr = FuncAttrStartAddr + AddrCnt;//计算出功能码属性区的地址

    FuncAttrAddr->Dotbits= (ENUM_Dot)Dotbits;//将属性的功能码的最大值改为修改后的最大值
}

/**************************************************************************************************
函数：LitDiFunc(void)
描述：DI功能码设置冲突限定
**************************************************************************************************/
void LitDiFunc(void)
{
#if 0
	Uint16 i;
	Uint16 CntInDi_Cnt = 0, LgthInDi_Cnt = 0;
	//1、实际端子功能码设置是否冲突判断
	for(i = 0 ; i < DI_NUM_XI+DI_NUM_ANA ; i++)
	{
		//检查设置为“34	计数输入（≤250Hz）”端子的个数
		if(RomFuncCode.F02[i] == CntInDi)
		{
			CntInDi_Cnt ++;
		}
		//检查设置为“37	长度计数输入（≤250Hz）”端子的个数
		if(RomFuncCode.F02[i] == LgthInDi)
		{
			LgthInDi_Cnt ++;
		}

		//如果是X7端子，不进行之后程序，直接跳出此次判断(注意程序先后顺序)
		if(i == X7)
		{
			continue;
		}

		//普通端子不能设置为高速功能，否者“E11 参数设置冲突 ”
		if(RomFuncCode.F02[i] == CntInHdi || \
		   RomFuncCode.F02[i] == LgthInHdi ||\
		   RomFuncCode.F02[i] == PulseInHdi)
		{
			FaultProc.FaultFlag.bit.PARACONFLICT = 1;
		}
	}
	//2、虚拟端子功能码设置是否冲突判断
	for(i = 0 ; i < DI_NUM_VIRT ; i++)
	{
		//检查设置为“34	计数输入（≤250Hz）”端子的个数
		if(RomFuncCode.F17[i] == CntInDi)
		{
			CntInDi_Cnt ++;
		}
		//检查设置为“37	长度计数输入（≤250Hz）”端子的个数
		if(RomFuncCode.F17[i] == LgthInDi)
		{
			LgthInDi_Cnt ++;
		}

		//普通端子不能设置为高速功能，否者“E11 参数设置冲突 ”
		if(RomFuncCode.F17[i] == CntInHdi || \
		   RomFuncCode.F17[i] == LgthInHdi ||\
		   RomFuncCode.F17[i] == PulseInHdi)
		{
			FaultProc.FaultFlag.bit.PARACONFLICT = 1;
		}
	}
	//3、“34	计数输入（≤250Hz）”和“37	长度计数输入（≤250Hz）”最多只能有1个端子设置
	if(CntInDi_Cnt > 1)
	{
		FaultProc.FaultFlag.bit.PARACONFLICT = 1;
	}
	if(LgthInDi_Cnt > 1)
	{
		FaultProc.FaultFlag.bit.PARACONFLICT = 1;
	}
#endif
}

void FuncLitProc_init(void)//功能码限定-初始化调用一次
{
    int16_t spdlim;
    
	//PE-07电机最大转速
	if(PreFunc.Nmax != RamMotor->Nmax)
	{
		PreFunc.Nmax = RamMotor->Nmax;

		//P1-39最高转速限定
		LitAttrMax(&RamBasePara->SpdMaxLmt, RamMotor->Nmax);
        
        //P4-01转矩控制时速度限幅
		LitAttrMax(&RamTrqCtrl->TrqSpdLmt, RamBasePara->SpdMaxLmt);
	}    
    
	//PE-06电机额定转速
	if(PreFunc.Ne != RamMotor->Ne)
	{
		PreFunc.Ne = RamMotor->Ne;
        
        //spdlim = RamMotor->Ne;
        spdlim = RamBasePara->SpdMaxLmt;

		//P1-42速度接近门限
		LitAttrMax(&RamBasePara->SpdApproachLmt, spdlim);
		//P1-43速度到达门限
		LitAttrMax(&RamBasePara->SpdArriveLmt, spdlim);

		//P2-02内部位置指令1移动速度
		LitAttrMax(&RamMultiPos->MultiPos1Spd, spdlim);
		//P2-06内部位置指令2移动速度
		LitAttrMax(&RamMultiPos->MultiPos2Spd, spdlim);
		//P2-10内部位置指令3移动速度
		LitAttrMax(&RamMultiPos->MultiPos3Spd, spdlim);
		//P2-14内部位置指令4移动速度
		LitAttrMax(&RamMultiPos->MultiPos4Spd, spdlim);
		//P2-18内部位置指令5移动速度
		LitAttrMax(&RamMultiPos->MultiPos5Spd, spdlim);
		//P2-22内部位置指令6移动速度
		LitAttrMax(&RamMultiPos->MultiPos6Spd, spdlim);
		//P2-26内部位置指令7移动速度
		LitAttrMax(&RamMultiPos->MultiPos7Spd, spdlim);
		//P2-30内部位置指令8移动速度
		LitAttrMax(&RamMultiPos->MultiPos8Spd, spdlim);
		//P2-34内部位置指令9移动速度
		LitAttrMax(&RamMultiPos->MultiPos9Spd, spdlim);
		//P2-38内部位置指令10移动速度
		LitAttrMax(&RamMultiPos->MultiPos10Spd, spdlim);
		//P2-42内部位置指令11移动速度
		LitAttrMax(&RamMultiPos->MultiPos11Spd, spdlim);
		//P2-46内部位置指令12移动速度
		LitAttrMax(&RamMultiPos->MultiPos12Spd, spdlim);
		//P2-50内部位置指令13移动速度
		LitAttrMax(&RamMultiPos->MultiPos13Spd, spdlim);
		//P2-54内部位置指令14移动速度
		LitAttrMax(&RamMultiPos->MultiPos14Spd, spdlim);
		//P2-58内部位置指令15移动速度
		LitAttrMax(&RamMultiPos->MultiPos15Spd, spdlim);
		//P2-62内部位置指令16移动速度
		LitAttrMax(&RamMultiPos->MultiPos16Spd, spdlim);

		//P3-00内部速度指令1
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd1, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd1, (Uint16)(-spdlim) );
		//P3-02内部速度指令2
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd2, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd2, (Uint16)(-spdlim) );
		//P3-04内部速度指令3
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd3, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd3, (Uint16)(-spdlim) );
		//P3-06内部速度指令4
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd4, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd4, (Uint16)(-spdlim) );
		//P3-08内部速度指令5
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd5, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd5, (Uint16)(-spdlim) );
		//P3-10内部速度指令6
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd6, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd6, (Uint16)(-spdlim) );
		//P3-12内部速度指令7
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd7, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd7, (Uint16)(-spdlim) );
		//P3-14内部速度指令8
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd8, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd8, (Uint16)(-spdlim) );
		//P3-16内部速度指令9
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd9, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd9, (Uint16)(-spdlim) );
		//P3-18内部速度指令10
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd10, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd10, (Uint16)(-spdlim) );
		//P3-20内部速度指令11
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd11, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd11, (Uint16)(-spdlim) );
		//P3-22内部速度指令12
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd12, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd12, (Uint16)(-spdlim) );
		//P3-24内部速度指令13
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd13, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd13, (Uint16)(-spdlim) );
		//P3-26内部速度指令14
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd14, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd14, (Uint16)(-spdlim) );
		//P3-28内部速度指令15
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd15, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd15, (Uint16)(-spdlim) );
		//P3-30内部速度指令16
		LitAttrMax((Uint16 *)&RamMultiSpeed->MultiSpd16, spdlim);
		LitAttrMin((Uint16 *)&RamMultiSpeed->MultiSpd16, (Uint16)(-spdlim) );

		//P4-03转矩速度限制指令增益
		LitAttrMax(&RamTrqCtrl->TrqSpdLmtGain, spdlim);

		//P8-03点动速度
		LitAttrMax(&RamAid->SpdJog, spdlim);
        
		//P8-17SON有效速度设定
		LitAttrMax(&RamAid->SpdSON, spdlim);

		//PB-04回零第一段高速设定
		LitAttrMax(&RamZeroReturn->HighSpeedZero, spdlim);
        
		//PB-05回零第二段低速设定
		LitAttrMax(&RamZeroReturn->LowSpeedZero, spdlim);
	}

    //PE-12电机最大力矩
    if(PreFunc.Tmax != RamMotor->TrqConst)
    {
        PreFunc.Tmax = RamMotor->TrqConst;
        
        //P1-46最高力矩
		LitAttrMax(&RamBasePara->CCW_TorqMaxLmt, RamMotor->TrqConst);
        
        //P1-47最高力矩
		LitAttrMax(&RamBasePara->CW_TorqMaxLmt, RamMotor->TrqConst);
    }
}

void FuncLitProc_sch(void)//功能码限定-调度处理
{
	/* 格式范例
	//PE-07电机最大转速
	if(PreFunc.Nmax != RamMotor->Nmax)
	{
		PreFunc.Nmax = RamMotor->Nmax;

		//P1-39最高转速限定
		LitAttrMax(&RamBasePara->SpdMaxLmt, RamMotor->Nmax);
	}
	*/
    
	//PF-06驱动器代码
	if(PreFunc.ServoCode != RamServo->ServoCode)
	{
		PreFunc.ServoCode = RamServo->ServoCode;
		ServoparaInit_Adjust();	//伺服code改变后，相关参数刷新
	}

}
/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
