/*
 * Cur_MotPaLearn.h
 *
 *  Created on:
 *      Author:
 *///

#ifndef CUR_MOTPALEARN_H_
#define CUR_MOTPALEARN_H_
#include "IQmathLib.h" 
typedef enum
{
	STEP0 =0,
	STEP1 =1,
	STEP2 =2,
	STEP3 =3,
	STEP4 =4,
	STEP5 =5,
	STEP6 =6,
	STEP7 =7,
	STEP8 =8,
	STEP9 =9,
	STEP10 =10,
	STEP11 =11,
	STEP12 =12,
	STEP13 =13,
	STEP14 =14,
	STEP15 =15
}LEARNSTEP;

typedef enum
{
	NOLearn = 0,
	ALLLEARN = 1,
	RsLdLq = 2,
	ENCOFFSET = 3,
	Es = 4
}LMODE;
struct LEARNMODE_BITS {
    Uint16 Mode:3;     //0 连续  1：触发
    Uint16 CmpFlg:1;
    Uint16 FirstFlg:1;
    Uint16 Rsvd1:11;
};

union LEARNMODE {
   Uint16                 all;
   struct LEARNMODE_BITS  bit;
};

typedef struct
{
	union LEARNMODE LearnMod;				//自学习模式选择
	Uint16 EncLnStep;						//编码器自学习步骤记录寄存器
	Uint16 OutStep;							//外层自学习步骤记录寄存器
	Uint32 ZcomeCnt;						//编码器自学习Z信号出现次数计数器，如果为串行编码器则为超正方向旋转时间计数器
	Uint32 DelayCnt;						//学习过程使用的延时计数器
	_iq frqpu;								//1/(2pi*f)，f为注入的正弦波频率
	_iq DeltaTheta;							//电感自学习注入的正弦波频率，每个调度周期内步进的角度
	Uint16 FftCnt;							//离散傅里叶变换计数器
	_iq UdSum1;								//直流实验1时D轴电压累计和
	_iq UdSum2;								//直流实验2时D轴电压累计和
	_iq UdcSum1;							//直流实验1时母线电压累计和
	_iq UdcSum2;							//直流实验2时母线电压累计和
	_iq UdcSum3;							//交流时母线电压累计和
	_iq IdSum1;								//直流实验1时D轴电流累计和
	_iq IdSum2;								//直流实验2时D轴电流累计和
	_iq Rud;								//交流实验实轴电压
	_iq Iud;								//交流实验虚轴电压
	_iq Rid;								//交流实验实轴电流
	_iq Iid;								//交流实验虚轴电流
	_iq Rs;
	_iq Rs1;
	_iq Ld;
	_iq Ld1;
	Uint16 RLtimes;							//阻感学习次数
	void (*init)();
	void (*rst)();
	void (*calc)();	    /* */
}CUR_MOTPALEARN;
typedef CUR_MOTPALEARN *CUR_MOTPALEARN_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_MOTPALEARN object.
-----------------------------------------------------------------------------*/
#define CUR_MOTPALEARN_DEFAULTS { \
/*LearnMod                    */0,\
/*EncLnStep                   */0,\
/*OutStep                     */0,\
/*ZcomeCnt                    */0,\
/*DelayCnt                    */0,\
/*frqpu                       */0,\
/*DeltaTheta                  */0,\
/*FftCnt                      */0,\
/*UdSum1                      */0,\
/*UdSum2                      */0,\
/*UdcSum1                     */0,\
/*UdcSum2                     */0,\
/*UdcSum3                     */0,\
/*IdSum1                      */0,\
/*IdSum2                      */0,\
/*Rud                         */0,\
/*Iud                         */0,\
/*Rid                         */0,\
/*Iid                         */0,\
/*Rs                          */0,\
/*Rs1                         */0,\
/*Ld                          */0,\
/*Ld1                         */0,\
/*RLtimes                     */0,\
		(void (*)(long)) Cur_MotPaLearn_init,\
		(void (*)(long)) Cur_MotPaLearn_rst,\
		(void (*)(long)) Cur_MotPaLearn_calc }
void Cur_MotPaLearn_init(CUR_MOTPALEARN_handle);
void Cur_MotPaLearn_rst(CUR_MOTPALEARN_handle);
void Cur_MotPaLearn_calc(CUR_MOTPALEARN_handle);

extern Uint8 Cur_BandwidthTest(void);

extern CUR_MOTPALEARN MotPaLearn;
#endif /* CUR_MOTPALEARN_H_ */
