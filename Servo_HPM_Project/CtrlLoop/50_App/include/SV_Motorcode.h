/*
 * SV_Motorcode.h
 *
 *  Created on: 2016-1-12
 *      Author: rd0217
 */

#ifndef SV_MOTORCODE_H_
#define SV_MOTORCODE_H_
#include "Datatype.h"
/***********电机参数组*PE****************/
typedef struct
{
	Uint16 Pe;									//PE-02电机额定功率
	Uint16 Ie;									//PE-03电机额定电流
	Uint16 Te;									//PE-04电机额定转矩

	Uint16 Ue;									//PE-05电机额定电压
	Uint16 Ne;									//PE-06电机额定转速
	Uint16 Nmax;								//PE-07电机最大转速
	Uint16 PoleNum;								//PE-08电机极对数
	Uint16 Lq;									//PE-09Q轴电感

	Uint16 Ld;									//PE-10D轴电感
	Uint16 Rs;									//PE-11线间电阻
	Uint16 TrqConst;							//PE-12 转矩参数
	Uint16 Es;									//PE-13 反电动势
	Uint16 Jmot;								//PE-14电机转子惯量

	Uint16 EncType;								//PE-15编码器类型
	Uint16 EncLineL;							//PE-16编码器线数低位
	Uint16 EncLineH;							//PE-17编码器线数高位
	Uint16 ZeroElecAngle;						//PE-18编码器原点电角度
	Uint16 UElecAngle;							//PE-19编码器U相上升沿电角度

	Uint16 TuningType;							//PE-20电机参数自学习方式
	Uint16 ABSEnc;								//PE-21绝对值编码器使用方法
	Uint16 ABPhaseSequ;							//PE-22增量式编码器AB相序关系
	Uint16 ACR_Pq;								//PE-23电流调节器Q轴比例增益
	Uint16 ACR_Pd;								//PE-24电流调节器D轴比例增益

	Uint16 ACR_Iq;								//PE-25电流调节器Q轴积分增益
	Uint16 ACR_Id;								//PE-26电流调节器D轴积分增益
    
    Uint16 ACR_KeqGain;						    //PE-27 Q轴反电动势补偿增益
	Uint16 ACR_KedGain;							//PE-28 D轴电压补偿增益
	Uint16 ACR_KeqGain1;						//PE-29 Q轴电压补偿增益

	Uint16 Reduction;                           //PE-30 减速比
}TYPE_MOTOPARA;
extern void MotoParaInit (Uint16 code);
#define CODE10	1
#define CODE11	1
#define CODE12	1
#define CODE13	0
#define CODE14	1
#define CODE15	1
#define CODE16	1
#define CODE17	1
#define CODE18	1
#define CODE19	0

#define CODE20	1
#define CODE21	1
#define CODE22	0
#define CODE23	0
#define CODE24	0
#define CODE25	0
#define CODE26	0
#define CODE27	0
#define CODE28	0
#define CODE29	0

#define CODE30	1
#define CODE31	1
#define CODE32	1
#define CODE33	0
#define CODE34	0
#define CODE35	0
#define CODE36	0
#define CODE37	0
#define CODE38	0
#define CODE39	0

#define CODE40	1
#define CODE41	1
#define CODE42	0
#define CODE43	0
#define CODE44	0
#define CODE45	0
#define CODE46	0
#define CODE47	0
#define CODE48	0
#define CODE49	0

#define CODE50	1
#define CODE51	1
#define CODE52	0
#define CODE53	0
#define CODE54	0
#define CODE55	0
#define CODE56	0
#define CODE57	0
#define CODE58	0
#define CODE59	0

#define CODE60	1
#define CODE61	1
#define CODE62	0
#define CODE63	0
#define CODE64	0
#define CODE65	0
#define CODE66	0
#define CODE67	0
#define CODE68	0
#define CODE69	0

#define CODE70	0
#define CODE71	0
#define CODE72	0
#define CODE73	0
#define CODE74	0
#define CODE75	0
#define CODE76	0
#define CODE77	0
#define CODE78	0
#define CODE79	0

#define CODE80	0
#define CODE81	0
#define CODE82	0
#define CODE83	0
#define CODE84	0
#define CODE85	0
#define CODE86	0
#define CODE87	0
#define CODE88	0
#define CODE89	0

#define CODE90	0
#define CODE91	0
#define CODE92	0
#define CODE93	0
#define CODE94	0
#define CODE95	0
#define CODE96	0
#define CODE97	0
#define CODE98	0
#define CODE99	0
#endif /* SV_MOTORCODE_H_ */
