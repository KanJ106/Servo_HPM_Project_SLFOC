/*
 * AdRead.h
 *
 *  Created on: 2015-12-24
 *      Author: w//
 */

#ifndef ADREAD_H_
#define ADREAD_H_
#include "IQmathLib.h" 

typedef struct 	{
				_iq Iv;					//电机v相电流 原始值
				_iq Iu;					//电机u相电流 原始值
				_iq Iw;					//电机w相电流 原始值
				_iq Iv2;				//电机v相电流 经过下桥采样窗口处理
				_iq Iu2;				//电机u相电流 经过下桥采样窗口处理
				_iq Iw2;				//电机w相电流 经过下桥采样窗口处理
				_iq Iv3;		        //电机v相电流 经过U，W反向处理，CLARK使用
				_iq Iu3;		        //电机u相电流 经过U，W反向处理，CLARK使用
				_iq Iv4;		        //电机v相电流 
				_iq Iu4;		        //电机u相电流 
				_iq Udc;				//母线电压
				_iq IGBTemper;			//模块温度
                _iq MortorTemper;		//电机温度
				_iq Ref1v5;				//1.5v基准
				_iq Gnd;				//Gnd
				int16 IvZero;			//v相电流零点偏移
				int16 IuZero;			//u相电流零点偏移
                int16 IwZero;			//u相电流零点偏移
				_iq   IuZeroq;			//v相电流零点偏移2
				_iq   IvZeroq;			//u相电流零点偏移2
				_iq   IwZeroq;			//w相电流零点偏移2
				Uint16 Cnt;				//零点偏移计算计数器
				Uint16 Cnt1;			//AD校正计算计数器
				_iq SumIu;				//零点偏移计算u相电流累计和
				_iq SumIv;				//零点偏移计算v相电流累计和
				_iq SumIw;				//零点偏移计算w相电流累计和
				_iq SumIv2;				//零点偏移计算v相电流累计和2
				_iq SumRef1v5;			//AD校正1.5v累计
				_iq SumGnd;				//AD校正Gnd累计
				_iq Ref1v5Avg;			//AD校正1.5v平均值
				_iq GndAvg;				//AD校正gnd平均值
				_iq AdcGain;			//AD校正增益
				float AdcGainf;			//AD校正增益浮点型，便于观察
				_iq AdcOffSet;			//AD校正偏移量
				Uint16 Udcf;			//母线电压真实值，便于观察
				float Ref1v5f;			//1.5v参考电压值，便于观察
				Uint16 SamWinMode;		//采样窗口判断
                Uint16 ClarkMode;       //clark变换相序
				Uint16 Ref1V5cnt;		//1.5v检测
				Uint16 Iucnt;		    //iu检测
				Uint16 Ivcnt;		    //iv检测
				Uint16 dycnt;			//延时计数器
				int16 iu;				//原始iu
				int16 iv;				//原始iv
                int16 iw;				//原始iv
                Uint16 bus;             //母线电流
				void (*Iuvcalc)(void);	    /* */
				void (*Udcalc)(void);	    /* */
				void (*IGBTempercalc)(void);/* */
				void (*MorTorTemperCalc)(void);	/* */
				void (*AdAdjcalc)(void);	/* */
				} ADREAD;

typedef ADREAD *ADREAD_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the ADREAD object.
-----------------------------------------------------------------------------*/
#define ADREAD_DEFAULTS { \
/*Iv,Iu,Iw,Iv2,Iu2,Iw2					*/0,0,0,0,0,0, \
/*Iv1AdAvg,Iu1AdAvg,Iv2AdAvg,Iu2AdAvg	*/0,0,0,0, \
/*Udc,IGBTemper,MortorTemper,Ref1v5,Gnd */0,0,0,0,0, \
/*IvZero,IuZero,Iv2Zero,Iu2Zero			*/0,0,0,0,0, \
/*ServOn,Cnt,Cnt1						*/0,0,0, \
/*SumIu,SumIv,SumIu2,SumIv2				*/0,0,0,0, \
/*SumRef1v5,SumGnd						*/0,0, \
/*Ref1v5Avg,GndAvg						*/0,0, \
/*AdcGain,AdcGainf,AdcOffSet			*/_IQ(1.0),0,0, \
/*Udcf,Ref1v5f,SamWinMode,ClarkMode		*/0,0,0,0, \
/*Ref1V5cnt,Iucnt,Ivcnt,dycnt			*/0,0,0,0, \
/*iu,iv									*/0,0,0,0, \
		(void (*)(long)) IuvRead_Calc,\
		(void (*)(long)) UdcRead_Calc,\
		(void (*)(long)) IGBTemperRead_Calc,\
		(void (*)(long)) MorTorTemperRead_Calc,\
        (void (*)(long)) AdAdjCal_Calc}

extern void IuvRead_Calc(void);
extern void UdcRead_Calc(void);
extern void IGBTemperRead_Calc(void);
extern void MorTorTemperRead_Calc(void);
extern void AdAdjCal_Calc(void);

extern ADREAD AdRead;
#endif /* ADREAD_H_ */





