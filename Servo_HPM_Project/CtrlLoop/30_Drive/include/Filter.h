/**************************************************************************************
文   件   名：Filter.h (IQ version)
创   建   人：
描        述：//
最后修改时间：
**************************************************************************************/
#ifndef __FILTER_H__
#define __FILTER_H__
#include "IQmathLib.h" 

#define LPF1Ord(Out,In,Kx)	Out += (_IQmpy(Kx,In-Out))  //Out,In,Kx均为GLOBAL_Q格式
/*-------------------------------------------------------------------------------------
结构体类型定义：一阶低通滤波器
-------------------------------------------------------------------------------------*/
typedef struct{
	_iq  In;		//输入
	_iq  Out;		//输出
	_iq  Kx;		//参数：滤波系数Ts/(Ts+Tc)
	void (*calc)(); //函数指针
}LPF1ORD;
#define  LPF1ORD_DEFAULTS { \
/*input                    */0, \
/*output                   */0, \
/*Kx                       */_IQ(0.5), \
/*void (*calc)()           */LPF1OrdCalc }

extern void LPF1OrdCalc(LPF1ORD *);
/*-------------------------------------------------------------------------------------
结构体类型定义：二阶低通滤波器
-------------------------------------------------------------------------------------*/
typedef struct{
	_iq  In;			//输入
	_iq  Out;			//输出
	_iq  In1,In2,Out2; 	//变量
	_iq  TsDivTc;		//参数：采样周期与滤波时间的比值
    #if FLOAT_TYPE_USE
    float a1,a2,b0;
    #else
	_iq  a1,a2,b0;		//参数：滤波器参数（由TsDivTc计算得出）
    #endif
	void (*calc)(); 	//函数指针
	void (*ParaCalc)(); //函数指针
	void (*reset)(); 	//复位
}LPF2ORD;

#define  LPF2ORD_DEFAULTS { \
/*In                       */0, \
/*Out                      */0, \
/*In1                      */0, \
/*In2                      */0, \
/*Out2                     */0, \
/*TsDivTc                  */_IQ(0.5), \
/*a1                       */_IQ(-1.324103), \
/*a2                       */_IQ(0.50065), \
/*b0                       */_IQ(0.04414), \
/*void (*calc)()           */LPF2OrdCalc, \
/*void (*ParaCalc)()       */LPF2OrdParaCalc, \
/*void (*reset)()          */LPF2OrdReset }

extern void LPF2OrdCalc(LPF2ORD *);
extern void LPF2OrdParaCalc(LPF2ORD *p);
extern void LPF2OrdReset(LPF2ORD *);
/*-----------------------------------------------------------------------------------
-----------------------------------------------------------------------------------*/


#endif //__FILTER_H__
