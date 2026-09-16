/*
 * SV_Di.h
 *
 *  Created on: 2013-7-9
 *      Author: zhangwei
 */

#ifndef SV_DI_H_
#define SV_DI_H_
//
#include "Datatype.h"
#include "userdefine.h"//

//实际输入端子枚举
typedef enum
{
	X1 = 0,			//DI1
	X2 = 1,			//DI2
	X3 = 2,			//DI3
	X4 = 3,			//DI4
	X5 = 4,			//DI5
	X6 = 5,			//DI6
	X7 = 6,			//DI7
	X8 = 7,			//DI8
	X9 = 8,			//保留
	X10 = 9,		//保留
	X11 = 10		//保留
}ENUM_DI_XI;

typedef enum
{
	DIDisable = 0,			//0		无功能
	S_ON = 1,				//1		伺服使能
	ALM_RST= 2,				//2		报警复位信号
	P_CLR = 3,				//3		位置控制脉冲偏差计数器清除
	DIR_SEL = 4,			//4		速度指令方向选择
	CMD0 = 5,				//5		内部指令bit0
	CMD1 = 6,				//6		内部指令bit1
	CMD2 = 7,				//7		内部指令bit2

	CMD3 = 8,				//8		内部指令bit3
	CTRG = 9,		        //9		内部指令触发
	MSEL = 10,				//10	控制模式切换
	ZCLMP = 11,				//11	模拟速度指令零位固定使能
	INHIBIT = 12,			//12	脉冲禁止
	P_OT = 13,				//13	禁止正向驱动
	N_OT = 14,				//14	禁止反向驱动
	GAIN_SEL = 15,			//15	增益切换

	J_SEL = 16,				//16	惯量比切换
	JOG_P = 17,				//17	正向点动
	JOG_N = 18,				//18	反向点动
	TDIR_SEL = 19,			//19	转矩指令方向选择
	GNUM0 = 20,				//20	电子齿轮比分子选择0
	GNUM1 = 21,				//21	电子齿轮比分子选择1
	ORGP = 22,		     	//22	原点回归检测信号
	SHOM = 23,			    //23	原点回归启动信号

	EMGS = 24,			    //24	急停
	RESERV25 = 25,			//25	保留
	RESERV26 = 26,			//26	保留
	RESERV27 = 27,			//27	保留
	RESERV28 = 28,			//28	保留
	RESERV29 = 29,			//29	保留
	RESERV30 = 30,			//30	保留
	RESERV31 = 31			//31	保留
}ENUM_DI_FUNC0;

typedef enum{
	FuncState_Invalid = 0,			//无效00
	FuncState_UpTrig = 1,			//上升沿01
	FuncState_DnTrig = 2,			//下降沿10
	FuncState_Valid = 3				//有效11
}ENUM_FUNC_STATE;

typedef struct
{                                       //
	Uint32	 DIDisable:2 ;				//0		无功能
	Uint32	 S_ON:2 ;					//1		伺服使能			              脉冲√
	Uint32	 ALM_RST:2 ;				//2		报警复位信号   	                          全√
	Uint32	 P_CLR:2 ;					//3		位置控制脉冲偏差计数器清除	  全√
	Uint32	 DIR_SEL:2 ;				//4		速度指令方向选择	                         脉冲 √
	Uint32	 CMD0:2 ;					//5		内部指令bit0			 脉冲 √
	Uint32	 CMD1:2 ;					//6		内部指令bit1			 脉冲 √
	Uint32	 CMD2:2 ;					//7		内部指令bit2			 脉冲 √
	Uint32	 CMD3:2 ;					//8		内部指令bit3			 脉冲 √
	Uint32	 CTRG:2 ;					//9		内部指令触发			 脉冲 √
	Uint32	 MSEL:2 ;					//10	控制模式切换			 脉冲 √
	Uint32	 ZCLMP:2 ;					//11	模拟速度指令零位固定使能	 脉冲 √
	Uint32	 INHIBIT:2 ;				//12	脉冲禁止		    	 全√
	Uint32	 P_OT:2 ;					//13	禁止正向驱动			 全√
	Uint32	 N_OT:2 ;					//14	禁止反向驱动			 全√
	Uint32	 GAIN_SEL:2 ;				//15	增益切换				 全√
}TYPE_DI_FUNC0DWORDS;
typedef struct
{
	Uint32	 J_SEL:2 ;					//16	惯量比切换				 全√
	Uint32	 JOG_P:2 ;					//17	正向点动				脉冲 √
	Uint32	 JOG_N:2 ;					//18	反向点动				脉冲 √
	Uint32	 TDIR_SEL:2 ;				//19	转矩指令方向选择			脉冲 √
	Uint32	 GNUM0:2 ;					//20	电子齿轮比分子选择0		全√
	Uint32	 GNUM1:2 ;					//21	电子齿轮比分子选择1		全√
	Uint32	 ORGP:2 ;					//22	原点检测信号			全√
	Uint32	 HOM:2 ;					//23	原点回归启动信号			脉冲 √
	Uint32	 EMGS:2 ;					//24           急停					全 ×
	Uint32	 Func25:2 ;					//25           高电平电机停止，低电平正常处理 2019.3.6
	Uint32	 Func26:2 ;					//26
	Uint32	 Func27:2 ;					//27
	Uint32	 Func28:2 ;					//28
	Uint32	 Func29:2 ;					//29
	Uint32	 Func30:2 ;					//30
	Uint32	 Func31:2 ;					//31
}TYPE_DI_FUNC1DWORDS;

typedef union
{
	Uint32 all;
	TYPE_STRUCT_DWORD word;
	TYPE_DI_FUNC0DWORDS bit;
}TYPE_DIFUNC0;

typedef union
{
	Uint32 all;
	TYPE_STRUCT_DWORD word;
	TYPE_DI_FUNC1DWORDS bit;
}TYPE_DIFUNC1;

typedef struct
{
	TYPE_DIFUNC0	Flag_Func0;		//功能有无效标志寄存器0~15
	TYPE_DIFUNC1	Flag_Func1;		//功能有无效标志寄存器16~31
}TYPE_DI_FUNC;	  //DI模块功能结构体
#define Glo_DIFUNC_Default	{0,0}

typedef struct
{
	Uint16 X1:1;
	Uint16 X2:1;
	Uint16 X3:1;
	Uint16 X4:1;
	Uint16 X5:1;
	Uint16 X6:1;
	Uint16 X7:1;
	Uint16 X8:1;

	Uint16 X9:1;
	Uint16 X10:1;
	Uint16 X11:1;
	Uint16 X12:1;
	Uint16 X13:1;
	Uint16 X14:1;
	Uint16 X15:1;
	Uint16 X16:1;
}TYPE_XI_LOGIC_Bit;
typedef union
{
	Uint16 XILog_All;
	TYPE_XI_LOGIC_Bit XILog_Bit;
}TYPE_XI_Logic;
typedef struct
{
	Uint16 Value;					//IO口当前采样值
	Uint16 FilerPreValue;			//上次防抖值
	Uint16 DelayPreValue;			//上次延时值
	Uint16 Result;					//此次最终值
	Uint16 *FuncAddr;				//输入端子的多功能编程功能吗映射

	Uint16	AntSkEnCnt;				//防抖有效次数计数
	Uint16	AntSkDisCnt;			//防抖无效次数计数
}TYPE_DI_XI;

//--------------------------------------------------------------------------//
//DI模块接口函数
extern void DI_Init(void);			//初始化函数
extern void DI_Handle(void);		//处理函数
//DI模块全局变量
extern TYPE_DI_XI Glo_DI_Xi[DI_NUM_PORT];		//实际端子结构体
extern TYPE_DI_FUNC   Glo_DI_FUNC;	//功能结构体
extern TYPE_XI_Logic Xi_Logic;

//------------------------------------END-----------------------------------//
#endif /* SV_DI_H_ */
