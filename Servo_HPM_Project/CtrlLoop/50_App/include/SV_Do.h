/*
 * SV_Do.h
 *
 *  Created on: 2013-7-9
 *      Author: zhangwei
 */

#ifndef SV_DO_H_
#define SV_DO_H_
//
#include "Datatype.h"
#include "userdefine.h"//

//输出端子枚举
typedef enum
{
	Y1 = 0,			//0
	Y2 = 1,			//1
	Y3 = 2,			//2
	Y4 = 3			//3
}ENUM_DO_Yi;

//端子输出类型枚举
typedef enum
{
	DO_LEVEL= 0,			//0，电平
	DO_SINGLEPULSE = 1		//1，单脉冲
}ENUM_DO_OUTTYPE;

//---------------------------位操作-------------------------------------------
typedef enum
{
	DODisable = 0,			//0		无功能
	S_RDY = 1,				//1		伺服准备好	            全√
	BK = 2,					//2		制动器控制	            全√
	TGON = 3,				//3		电机旋转		全√
	ZERO = 4,				//4		电机零速		全√
	V_CLS = 5,				//5		速度接近		脉冲√
	V_CMP = 6,				//6		速度到达		全√
	PNEAR = 7,				//7		位置接近		脉冲√

	COIN = 8,				//8		位置到达		全√
	C_LT = 9,				//9		转矩限制		全√
	V_LT = 10,				//10	转速限制		全√
	WARN = 11,				//11	警告输出		全√
	ALM = 12,				//12	故障输出		全√
	T_CMP = 13,				//13	转矩到达		全√
	Home = 14,		    	//14            原点回归完成      	 全√
	S_RUN = 15,				//15	伺服使能                   测试√

	PWM_OUT = 16,			//16    PWM有输出              测试√
	CMDIN = 17,		    	//17           输入命令有效
	OFunc18 = 18,			//18
	OFunc19 = 19,			//19
	OFunc20 = 20,			//20
	OFunc21 = 21,			//21
	OFunc22 = 22,			//22
	OFunc23 = 23,			//23
	OFunc24 = 24,			//24

	OFunc25 = 25,			//25
	OFunc26 = 26,			//26
	OFunc27 = 27,			//27
	OFunc28 = 28,			//28
	OFunc29 = 29,			//29
	OFunc30 = 30,			//30
	OFunc31 = 31,			//31

	OFunc32 = 32,			//0
	OFunc33 = 33,			//1
	OFunc34 = 34,			//2
	OFunc35 = 35,			//3
	OFunc36 = 36,			//4
	OFunc37 = 37,			//5
	OFunc38 = 38,			//6
	OFunc39 = 39,			//7

	OFunc40 = 40,			//8
	OFunc41 = 41,			//9
	OFunc42 = 42,			//10
	OFunc43 = 43,			//11
	OFunc44 = 44,			//12
	OFunc45 = 45,			//13
	OFunc46 = 46,			//14
	OFunc47 = 47,			//15

	OFunc48 = 48,			//16
	OFunc49 = 49,			//17
	OFunc50 = 50,			//18
	OFunc51 = 51,			//19
	OFunc52 = 52,			//20
	OFunc53 = 53,			//21
	OFunc54 = 54,			//22
	OFunc55 = 55,			//23

	OFunc56 = 56,			//24
	OFunc57 = 57,			//25
	OFunc58 = 58,			//26
	OFunc59 = 59,			//27
	OFunc60 = 60,			//28
	OFunc61 = 61,			//29
	OFunc62 = 62,			//30
	OFunc63 = 63			//31
}ENUM_DO_FUNC0;

typedef struct
{                               //                    	  脉冲                           总线
	Uint32	 DODisable:1 ;		//0		无功能                                Y             Y
	Uint32	 S_RDY:1 ;			//1		伺服准备好                        Y             Y
	Uint32	 BK:1 ;				//2		制动器控制                        Y             Y
	Uint32	 TGON:1 ;			//3		电机旋转                            Y             Y(同脉冲)
	Uint32	 ZERO:1 ;			//4		电机零速                            Y             Y(同脉冲)
	Uint32	 V_CLS:1 ;			//5		速度接近                            Y             Y(同脉冲)  为啥只有脉冲
	Uint32	 V_CMP:1 ;			//6		速度到达                            Y             Y(402)
	Uint32	 PNEAR:1 ;			//7		位置接近                            Y             Y(同脉冲)  为啥只有脉冲

	Uint32	 COIN:1 ;			//8		位置到达                            Y             Y(402)
	Uint32	 C_LT:1 ;			//9		转矩限制                            Y             Y(同脉冲)
	Uint32	 V_LT:1 ;			//10	力矩模式转速限制            Y             Y(同脉冲)
	Uint32	 WARN:1 ;			//11	警告输出                            Y             Y
	Uint32	 ALM:1 ;			//12	故障输出                            Y             Y
	Uint32	 T_CMP:1 ;			//13	转矩到达                            Y             Y
	Uint32	 Home:1 ;		    //14         原点回归完成                    Y             Y			不知
	Uint32	 S_RUN:1 ;			//15         伺服使能                            N             Y(测试版本做)

	Uint32	 PWMOUT:1 ;		    //16    PWM使能                             N				 Y(测试版本用)
	Uint32	 CMD:1 ;	    	//17          输入命令有效
	Uint32	 OFunc18:1 ;		//18
	Uint32	 OFunc19:1 ;		//19
	Uint32	 OFunc20:1 ;		//20
	Uint32	 OFunc21:1 ;		//21
	Uint32	 OFunc22:1 ;		//22
	Uint32	 OFunc23:1 ;		//23
	Uint32	 OFunc24:1 ;		//24

	Uint32	 OFunc25:1 ;		//25
	Uint32	 OFunc26:1 ;		//26
	Uint32	 OFunc27:1 ;		//27
	Uint32	 OFunc28:1 ;		//28
	Uint32	 OFunc29:1 ;		//29
	Uint32	 OFunc30:1 ;		//30
	Uint32	 OFunc31:1 ;		//31
}TYPE_DO_FUNC0DWORDS;
typedef union
{
	Uint32 all;
	TYPE_STRUCT_DWORD word;
	TYPE_DO_FUNC0DWORDS bit;
}TYPE_DO_FUNC0;

typedef struct
{
	Uint32	 Func32:1 ;			//0
	Uint32	 Func33:1 ;			//1
	Uint32	 Func34:1 ;			//2
	Uint32	 Func35:1 ;			//3
	Uint32	 Func36:1 ;			//4
	Uint32	 Func37:1 ;			//5
	Uint32	 Func38:1 ;			//6
	Uint32	 Func39:1 ;			//7

	Uint32	 Func40:1 ;			//8
	Uint32	 Func41:1 ;			//9
	Uint32	 Func42:1 ;			//10
	Uint32	 Func43:1 ;			//11
	Uint32	 Func44:1 ;			//12
	Uint32	 Func45:1 ;			//13
	Uint32	 Func46:1 ;			//14
	Uint32	 Func47:1 ;			//15

	Uint32	 Func48:1 ;			//16
	Uint32	 Func49:1 ;			//17
	Uint32	 Func50:1 ;			//18
	Uint32	 Func51:1 ;			//19
	Uint32	 Func52:1 ;			//20
	Uint32	 Func53:1 ;			//21
	Uint32	 Func54:1 ;			//22
	Uint32	 Func55:1 ;			//23

	Uint32	 Func56:1 ;			//24
	Uint32	 Func57:1 ;			//25
	Uint32	 Func58:1 ;			//26
	Uint32	 Func59:1 ;			//27
	Uint32	 Func60:1 ;			//28
	Uint32	 Func61:1 ;			//29
	Uint32	 Func62:1 ;			//30
	Uint32	 Func63:1 ;			//31
}TYPE_DO_FUNC1DWORDS;
typedef union
{
	Uint32 all;
	TYPE_STRUCT_DWORD word;
	TYPE_DO_FUNC1DWORDS bit;
}TYPE_DO_FUNC1;

typedef struct
{
	TYPE_DO_FUNC0	Flag_Func0;		//DO输出功能有无效标志寄存器0，功能码0~31
	TYPE_DO_FUNC1	Flag_Func1;		//DO输出功能有无效标志寄存器1，功能码32~63
}TYPE_DO_FUNC;	  //DO模块变量结构体

#define Glo_DO_FUNC_Default	{{0},{0}}

typedef struct
{
	Uint16	 y1:1 ;				//0
	Uint16	 y2:1 ;				//1
	Uint16	 y3:1 ;				//2
	Uint16	 y4:1 ;				//3
	Uint16	 rsvd1:12 ;			//15~4
}TYPE_DO_YiBits;
typedef union
{
	Uint16 all;
	TYPE_DO_YiBits bit;
}TYPE_DO_YiData;

typedef struct
{
	Uint16		Value;				//中间变量
#if DO_DELAY
	Uint16		EnDelayCnt;			//有效延时计数
	Uint16		DisDelayCnt;		//无效延时计数
	Uint16		DelayPreValue;		//上一次延时处理后的值
#endif
	Uint16		Pulsing;			//单脉冲输出标志 0:无输出；1：正在输出
	Uint16		PrPulse;			//上次状态，仅用于单脉冲
	Uint16		PulseCnt;			//单脉冲输出计数

	Uint16		*FuncNum;			//功能设置
#if DO_DELAY
	Uint16		*EnDelaySet;		//有效延时设置
	Uint16		*DisDelaySet;		//无效延时设置
#endif
//	Uint16		*PulseSet;			//单脉冲时间设置

//	TYPE_DO_YiData	Flag_Last;		//端子最终有无效标志寄存器
}TYPE_DO_Yi;
//--------------------------------------------------------------------------//
//DO模块接口函数
extern void DO_Handle(void);
extern void DO_Init(void);

//DO模块全局变量
extern TYPE_DO_YiData	DoFlag_Last;
extern TYPE_DO_Yi	Glo_DO_Yi[DO_NUM_PORT];
extern TYPE_DO_FUNC	Glo_DO_FUNC;
//------------------------------------END-----------------------------------//
#endif /* SV_DO_H_ */
