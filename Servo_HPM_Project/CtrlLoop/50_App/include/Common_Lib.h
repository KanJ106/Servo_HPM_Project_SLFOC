/*
 * Common_Lib.h
 *
 *  Created on: 2015-9-24
 *      Author: zhangwei
 *///
 
#ifndef COMMON_LIB_H_
#define COMMON_LIB_H_

#include "Datatype.h"//

#define	Common_Abs(in)				((in) >= 0 ? (in) : (-(in)))
#define	Common_Sat(in,max,min)		(in>max?max:(in<min?min:in))
#define	Common_Max2(a,b)			(a>b?a:b)
#define	Common_Max3(a,b,c)			(a>b?(a>c?a:c):(b>c?b:c))
#define	Common_Min2(a,b)			(a<b?a:b)
#define	Common_Min3(a,b,c)			(a<b?(a<c?a:c):(b<c?b:c))

#define Common_Scheduling			(1000)                   	//1000us??è°?åº¦å?¨æ??
#define Common_Base_1ms				(1000/Common_Scheduling)		//1mså®??¶å?ºå??
#define Common_Base_10ms			(10*Common_Base_1ms)				//10ms?ºå??
#define Common_Base_100ms			(100*Common_Base_1ms)				//100ms?ºå??
#define Common_Base_1000ms   		(1000*Common_Base_1ms)			//1000ms?ºå??
#define Common_Base_Dot1min			(6000*Common_Base_1ms)			//0.1min?ºå??
#define Common_Base_1min			(Uint32)(60000*Common_Base_1ms)	//1min?ºå??
#define	Common_Base_1hour			(Uint32)(3600000*Common_Base_1ms)	//1h?ºå??
#define Common_VALID				(1)
#define Common_INVALID				(0)

#define Common_GetU32(H,L)			(((Uint32)H)<<16+(Uint32)L)
#define Common_GetI32(H,L)			(((int32)H)<<16+(int32)L)
#define Common_GetU32H(N)			((Uint16)(((N)&0xFFFF0000)>>16))
#define Common_GetU32L(N)			((Uint16)(((N)&0x0000FFFF)>>0))
#define Common_GetI32L(N)			((int16)(((N)&0x0000FFFF)>>0))

#define Common_BIT0					(0)
#define Common_BIT1					(1)
#define Common_BIT2					(2)
#define Common_BIT3					(3)
#define Common_BIT4					(4)
#define Common_BIT5					(5)
#define Common_BIT6					(6)
#define Common_BIT7					(7)
#define Common_BIT8					(8)
#define Common_BIT9					(9)
#define Common_BIT10				(10)
#define Common_BIT11				(11)
#define Common_BIT12				(12)
#define Common_BIT13				(13)
#define Common_BIT14				(14)
#define Common_BIT15				(15)
#define Common_Check16Bit(x,n)		((x & (0x0001<<(n))) >> (n))	//ä¸?-Faultï¼?1  -Ture
#define Common_Check16Bit2(x,n)		((x & (0x0001<<(n)))       )	//ä¸?-Faultï¼???0-Ture
#define Common_Set16Bit(x,n)		((x |= (0x0001<<(n)))       )	//å¯¹å?ä½?ç½?
#define Common_Clr16Bit(x,n)		((x &= (~(0x0001<<(n))))    )	//å¯¹å?ä½?æ¸?0

/**********å®??¶å???½å?½æ??************/
typedef struct
{
	Uint32	CntBase;
	Uint32	FunBase;
}TYPE_COMMON_CNT;
#define COMMON_DEFAULTS	{0,	0}

/****************************************************************************
 * ??è¿°ï?å®??¶å??½æ?°ï???????ms
 * å®??¶å?½æ?°å?ä¹?  MsU32Cntä¸ºè®¡?°å?¨ï???å®?ä¹?ä¸ºé????????ï¼?
 * Valueä¸ºå??¶å?¼ï?,å®??¶æ?¶é?´å?°ï?è¿???1ï¼??¦å??è¿???0ï¼????½ç??????
 * Common_Base:??ä½??ºå??æ¯?å¦?å®???.1sï¼?Common_Base=Common_Base_100ms
 * ??ä½??ºå??æ¯?å¦?å®???.01sï¼?Common_Base=Common_Base_10ms
****************************************************************************/
#define Common_DelayShort(U32Cnt,Value,Common_Base)	\
	((U32Cnt)<(Uint32)(Value*Common_Base) ? (U32Cnt++,Common_INVALID):Common_VALID)	//0.776us
extern void Common_DelayClear(TYPE_COMMON_CNT *CntStruct);//è®¡æ?°å?¨ç???ä½???å§???è°??¨æ?¶é??.38us
extern Uint16  Common_DelayLong(TYPE_COMMON_CNT *CntStruct,Uint16 DelayTime,Uint32 CntBase);//min/hå»¶æ?¶å?½æ??è°??¨æ?¶é??.79us
/*****************************************************************************
 *?½æ?°æ??è¿?  ï¼?ä¸??¶æ?¯æ?§æ»¤æ³¢å?½æ?°ï?Uint16/Int16 ??Uint32/Int32
 *????      ï¼???ç¬¦å?·æ»¤æ³¢å?½æ?°è?è¡?æ»¤æ³¢
 *è¾???      ï¼? in	è¾??¥å??
 *	      	old	ä¸?ä¸?æ¬¡è??ºå??
 *			*remä¸?ä¸?æ¬¡ä??°å?°å??
 *			Tæ»¤æ³¢?¶é??0.001s,Int32Tf=T/??ï¼???ä¸ºè?åº¦å?¨æ??ms;T??ä½?ms)
 *è¾???      ï¼? out	è¾??ºå??
*****************************************************************************/
extern int16 Common_FliterInt16(int16 Int16in , int16 Int16old , int16 *Int16rem , Uint16 Int16Tf);//?§è?è°??¨æ?¶é??.24us
extern Uint16 Common_FliterUint16(Uint16 U16in , Uint16 U16old , Uint16 *U16rem , Uint16 U16Tf);//?§è??¶é??.54us
extern int32 Common_InertiaFilterInt32(int32 Int32in , int32 Int32old , int32 *Int32rem , Uint16 Int32Tf);//?§è?è°??¨æ?¶é??.50usï¼????¿æ?¶é??.35us
extern Uint32 Common_InertiaFilterUint32(Uint32 Uint32in , Uint32 Uint32old , Uint32 *Uint32rem , Uint16 Uint32Tf);//????.56us,????.47us

extern void Common_SeprateDataU16(Uint16 in, Uint16 *out, Uint16 num);	//????16?°ç??ä¸ªå???¾å??ä¸?ä½?

#endif /* COMMON_LIB_H_ */
//============================================================================
// End of file.
//============================================================================
