/*
 * SV_Do.c
 *
 *  Created on: 2016-1-6
 *      Author: rd0217//
 */

#include "SV_Do.h"
#include "userdefine.h"
#include "r_cg_Project.h"
#include "string.h"
#include "SV_FuncVar.h"

TYPE_DO_YiData	DoFlag_Last;
TYPE_DO_Yi	Glo_DO_Yi[DO_NUM_PORT];
TYPE_DO_FUNC	Glo_DO_FUNC; //

TYPE_DO_YiData DogetData={0};
void DO_Init(void)
{
	Glo_DO_FUNC.Flag_Func0.all = 0;
	Glo_DO_FUNC.Flag_Func1.all = 0;
	DoFlag_Last.all = 0;

	memset(Glo_DO_Yi , 0 , sizeof(Glo_DO_Yi)/sizeof(Uint16));

	Glo_DO_Yi[Y1].FuncNum = &RamIO->DO1Fun;
	Glo_DO_Yi[Y2].FuncNum = &RamIO->DO2Fun;
#if (SERVO_HARDWARE == HARDWARE_AC0 ||SERVO_HARDWARE == HARDWARE_AC1)
	Glo_DO_Yi[Y3].FuncNum = &RamIO->DO3Fun;
	Glo_DO_Yi[Y4].FuncNum = &RamIO->DO4Fun;
#endif

#if DO_DELAY
	Glo_DO_Yi[Y1].EnDelaySet = &RamIO->reserve46;
	Glo_DO_Yi[Y2].EnDelaySet = &RamIO->reserve47;
	Glo_DO_Yi[Y3].EnDelaySet = &RamIO->reserve48;
	Glo_DO_Yi[Y4].EnDelaySet = &RamIO->reserve48;

	Glo_DO_Yi[Y1].DisDelaySet = &RamIO->reserve46;
	Glo_DO_Yi[Y2].DisDelaySet = &RamIO->reserve47;
	Glo_DO_Yi[Y3].DisDelaySet = &RamIO->reserve48;
	Glo_DO_Yi[Y4].DisDelaySet = &RamIO->reserve48;
#endif
}
static Uint16 GetOutputFlag(Uint16 *func)
{
	switch(*func)
	{
		case	DODisable:				return	DO_INAVALID;	//根据设置功能号，返回对应状态
		case	S_RDY:					return	Glo_DO_FUNC.Flag_Func0.bit.S_RDY;
		case	BK:						return	Glo_DO_FUNC.Flag_Func0.bit.BK;
		case	TGON:					return	Glo_DO_FUNC.Flag_Func0.bit.TGON;
		case	ZERO:					return	Glo_DO_FUNC.Flag_Func0.bit.ZERO;
		case	V_CLS:					return	Glo_DO_FUNC.Flag_Func0.bit.V_CLS;
		case	V_CMP:					return	Glo_DO_FUNC.Flag_Func0.bit.V_CMP;
		case	PNEAR:					return	Glo_DO_FUNC.Flag_Func0.bit.PNEAR;
		case	COIN:					return	Glo_DO_FUNC.Flag_Func0.bit.COIN;
		case	C_LT:					return	Glo_DO_FUNC.Flag_Func0.bit.C_LT;
		case	V_LT:					return	Glo_DO_FUNC.Flag_Func0.bit.V_LT;
		case	WARN:					return	Glo_DO_FUNC.Flag_Func0.bit.WARN;
		case	ALM:					return	Glo_DO_FUNC.Flag_Func0.bit.ALM;
		case	T_CMP:					return	Glo_DO_FUNC.Flag_Func0.bit.T_CMP;
		case	Home:					return	Glo_DO_FUNC.Flag_Func0.bit.Home;
		case	S_RUN:					return	Glo_DO_FUNC.Flag_Func0.bit.S_RUN;
		case    PWM_OUT:                return  Glo_DO_FUNC.Flag_Func0.bit.PWMOUT;
		case    CMDIN:                  return  Glo_DO_FUNC.Flag_Func0.bit.CMD;
		default	:						return	DO_INAVALID;
	}
}
static void YiFunction(TYPE_DO_Yi *p)
{
//#if SERVO_HARDWARE == HARDWARE_AC0
    Uint16 i;
	for(i = 0 ; i < DO_NUM_PORT ; i++)
	{
		p->Value = GetOutputFlag(p->FuncNum);//调用多功能函数，得到对应状态
		p++;
	}
//#endif

//#if SERVO_HARDWARE == HARDWARE_DC0///2018.4.16
//    p->Value =DogetData.bit.y1;
//    p++;
//    p->Value =DogetData.bit.y2;
//#endif
}
#if DO_DELAY
static void YiDelay(TYPE_DO_Yi *p)
{
	Uint16 i;
	for(i = 0 ; i <DO_NUM_PORT; i++)
	{
		//当前端口有效
		if(DO_AVALID == p->Value)
		{
			p->DisDelayCnt = 0;	//无效延时清0
			//有效延时到
			if(p->EnDelayCnt >= *(p->EnDelaySet))
			{
				p->Value = DO_AVALID;
			}
			//有效延时未到
			else
			{
				p->EnDelayCnt ++;
				p->Value = p->DelayPreValue;
			}
		}
		//当前端口无效
		else
		{
			p->EnDelayCnt = 0;	//有效延时清0
			//无效延时到
			if(p->DisDelayCnt >= *(p->DisDelaySet))
			{
				p->Value = DO_INAVALID;
			}
			//无效延时未到
			else
			{
				p->DisDelayCnt ++;
				p->Value = p->DelayPreValue;
			}
		}
		p->DelayPreValue = p->Value;
		p++;
	}
}
#endif
static void YiLogic(TYPE_DO_Yi *p)
{
	Uint16 i;
	TYPE_DO_YiData Tmp ;
	Tmp.all = 0;
	//1.端子最终状态确定
	for(i = 0 ; i <DO_NUM_PORT; i++)
	{
		Tmp.all |= (p->Value << i);
		p++;

	}
	//2.正反逻辑处理
	DoFlag_Last.all= Tmp.all ^ RamIO->DOLogicLevel;
}
static void YiOutPut(void)
{
	/*	Y1	*/
	if(DoFlag_Last.bit.y1 == DO_AVALID)
	{
		SetY1_Invaild;					//若端子状态有效，置对应I/O口为有效电平
	}
	else
	{
		SetY1_Valid;					//若端子状态无效，置对应I/O口为无效电平
	}
	/*	Y2	*/
	if(DoFlag_Last.bit.y2 == DO_AVALID)
	{
		SetY2_Invaild ;
	}
	else
	{
		SetY2_Valid;
	}
#if (SERVO_HARDWARE == HARDWARE_AC0 ||SERVO_HARDWARE == HARDWARE_AC1)
	/*	R1	*/
	if(DoFlag_Last.bit.y3 == DO_AVALID)
	{
		SetY3_Invaild ;
	}
	else
	{
		SetY3_Valid;
	}
	/*	R2	*/
	if(DoFlag_Last.bit.y4 == DO_AVALID)
	{
		SetY4_Invaild ;
	}
	else
	{
		SetY4_Valid;
	}
#endif
}
void DO_Handle(void)
{
	YiFunction(Glo_DO_Yi);//
#if DO_DELAY
	YiDelay(Glo_DO_Yi);
#endif
	YiLogic(Glo_DO_Yi);
	YiOutPut();
}
