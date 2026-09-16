/*
 * SV_Di.c
 *
 *  Created on: 2016-1-6
 *      Author: rd0217//
 */
#include "SV_Di.h"
#include "userdefine.h"
#include "r_cg_Project.h"
#include "string.h"
#include "SV_FuncVar.h"

TYPE_DI_XI Glo_DI_Xi[DI_NUM_PORT];  //DI模块结构体 //
TYPE_XI_Logic Xi_Logic;				//IO口实际采样值
TYPE_DI_FUNC  Glo_DI_FUNC;			//电平有效/无效
//TYPE_XI_Logic DiValue={0};
void DI_Init(void)
{

	Xi_Logic.XILog_All = 0;
	Glo_DI_FUNC.Flag_Func0.all = 0;
	Glo_DI_FUNC.Flag_Func1.all = 0;

	memset(Glo_DI_Xi,0,sizeof(Glo_DI_Xi)/sizeof(Uint16));

	Glo_DI_Xi[X1].FuncAddr = &RamIO->DI1Fun ;			//多功能编程功能码赋值
	Glo_DI_Xi[X2].FuncAddr = &RamIO->DI2Fun ;
	Glo_DI_Xi[X3].FuncAddr = &RamIO->DI3Fun ;
	Glo_DI_Xi[X4].FuncAddr = &RamIO->DI4Fun ;
	Glo_DI_Xi[X5].FuncAddr = &RamIO->DI5Fun ;
#if (SERVO_HARDWARE == HARDWARE_AC0 ||SERVO_HARDWARE == HARDWARE_AC1)
	Glo_DI_Xi[X6].FuncAddr = &RamIO->DI6Fun ;
	Glo_DI_Xi[X7].FuncAddr = &RamIO->DI7Fun ;
	Glo_DI_Xi[X8].FuncAddr = &RamIO->DI8Fun ;
#endif
}

static void GetXiValue(void)
{
	Xi_Logic.XILog_Bit.X1 = GetX1_Pin;
	Xi_Logic.XILog_Bit.X2 = GetX2_Pin;
	Xi_Logic.XILog_Bit.X3 = GetX3_Pin;
	Xi_Logic.XILog_Bit.X4 = GetX4_Pin;
	Xi_Logic.XILog_Bit.X5 = GetX5_Pin;
#if (SERVO_HARDWARE == HARDWARE_AC0 ||SERVO_HARDWARE == HARDWARE_AC1)
	Xi_Logic.XILog_Bit.X6 = GetX6_Pin;
	Xi_Logic.XILog_Bit.X7 = GetX7_Pin;
	Xi_Logic.XILog_Bit.X8 = GetX8_Pin;
#endif

	//端子的逻辑处理，正逻辑，闭合有效，断开无效，，反逻辑，闭合无效，断开有效
	Xi_Logic.XILog_All  = ~(Xi_Logic.XILog_All ^ RamIO->DILogicLevel);

	Glo_DI_Xi[X1].Value = Xi_Logic.XILog_Bit.X1;
	Glo_DI_Xi[X2].Value = Xi_Logic.XILog_Bit.X2;
	Glo_DI_Xi[X3].Value = Xi_Logic.XILog_Bit.X3;
	Glo_DI_Xi[X4].Value = Xi_Logic.XILog_Bit.X4;
	Glo_DI_Xi[X5].Value = Xi_Logic.XILog_Bit.X5;
#if (SERVO_HARDWARE == HARDWARE_AC0 ||SERVO_HARDWARE == HARDWARE_AC1)
	Glo_DI_Xi[X6].Value = Xi_Logic.XILog_Bit.X6;
	Glo_DI_Xi[X7].Value = Xi_Logic.XILog_Bit.X7;
	Glo_DI_Xi[X8].Value = Xi_Logic.XILog_Bit.X8;
#endif
}

static void XiFilerProc(TYPE_DI_XI *p)
{
	Uint16 i =0;

	for(i=0; i<DI_NUM_PORT; i++)
	{
		if(DI_AVALID == p->Value)		//当前端口有效
		{
			p->AntSkDisCnt = 0;		//无效计数清0
			if(p->AntSkEnCnt < RamIO->T_DILpf)
			{
				p->AntSkEnCnt++;
				p->Value = p->FilerPreValue ;
			}
		}
		else							//当前端口无效
		{
			p->AntSkEnCnt = 0;			//有效计时清零
			if(p->AntSkDisCnt < RamIO->T_DILpf)
			{
				p->AntSkDisCnt++;
				p->Value = p->FilerPreValue ;
			}
		}
		p->FilerPreValue = p->Value;
		p++;
	}
}
static void XiFuncProc(TYPE_DI_XI *p)
{
	Uint16  i = 0, tmp;
	Uint16	funcNum = 0;
	Uint32  DIFunc0 = (Glo_DI_FUNC.Flag_Func0.all << 1) & 0xAAAAAAAA, \
			DIFunc1 = (Glo_DI_FUNC.Flag_Func1.all << 1) & 0xAAAAAAAA;

	for(i=0; i<DI_NUM_PORT; i++)
	{
		funcNum = *(p->FuncAddr);
		p->Result = p->Value;

		if((DI_AVALID == p->Result) && (DIDisable != funcNum))
		{
			tmp = (funcNum & 0x0f) << 1;
			switch(funcNum >> 4)				//÷16
			{
				case 0:
					DIFunc0 |= (1UL<<tmp);
					break;
				case 1:
					DIFunc1 |= (1UL<<tmp);
					break;
				default: break;
			}
		}
		p++;
	}
	Glo_DI_FUNC.Flag_Func0.all = DIFunc0;
	Glo_DI_FUNC.Flag_Func1.all = DIFunc1;
}
void DI_Handle(void)		//
{
	GetXiValue();				//采样IO口
	XiFilerProc(Glo_DI_Xi);		//防抖处理
	XiFuncProc(Glo_DI_Xi);		//多功能编程处理
//----------code-end------------//
}

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
