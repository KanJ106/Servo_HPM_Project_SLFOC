/*
 * Spd_GainAdpt.h
 *
 *  Created on:
 *      Author:
 *///

#ifndef SPD_GAINADPT_H_
#define SPD_GAINADPT_H_
#include "IQmathLib.h" 
struct GAINSW_BITS {
	Uint16 GainSwOption:4;
	Uint16 GainSwMod:4;
    Uint16 rsd1:8;
};

union GAINSW {
   Uint16                 all;
   struct GAINSW_BITS    bit;
};
typedef struct 	{
				  union GAINSW GainSw;
				  _iq Coeff1;		//内部系数一
				  _iq Coeff2;		//内部系数二
				  _iq Coeff3;		//内部系数三
				  Uint16 fwc;		//新速度截止频率
				  Uint16 fwcOld;	//旧速度截止频率
				  Uint16 GainChg;	//增益切换标志
				  _iq  GainLmt;		//阈值
				  _iq Kp1;			//第一增益
				  _iq Ki1;			//第一积分
				  _iq Kp2;			//第二增益
				  _iq Ki2;			//第二积分
				  _iq Kp;			//最终增益
				  _iq Ki;			//最终增益
				  _iq J;			//实时惯量比
				  Uint16 PpGain1;	//位置第一增益
				  Uint16 PpGain2;	//位置第二增益
				  Uint16 PpGain;	//最终位置增益
				  Uint16 SfwdGain;	//速度前馈增益
				  Uint16 TorqCmdFiltTimes;//转矩指令滤波时间常数前馈
				  Uint16 RgdtyTab[32][6]; //刚性表：位置增益0.1/s-速度前馈/1000-速度增益0.1Hz-速度积分0.1ms-转矩滤波0.01ms-速度检测滤波-0.01ms
				  _iq    RampTpu;	//增益切换时间
				  void (*init)();
				  void (*rst)();
				  void (*update)();
				  void (*calc)();	/* */
				} SPD_GAINADPT;

typedef SPD_GAINADPT *SPD_GAINADPT_handle;

#define SPD_GAINADPT_DEFAULTS { \
/*GainSw                   */0, \
/*Coeff1                   */0, \
/*Coeff2                   */0, \
/*Coeff3                   */0, \
/*fwc                      */0, \
/*fwcOld                   */0, \
/*GainChg                  */0, \
/*GainLmt                  */0, \
/*Kp1                      */0, \
/*Ki1                      */0, \
/*Kp2                      */0, \
/*Ki2                      */0, \
/*Kp                       */0, \
/*Ki                       */0, \
/*J                        */0, \
/*PpGain1                  */0, \
/*PpGain2                  */0, \
/*PpGain                   */0, \
/*SfwdGain                 */0, \
/*TorqCmdFiltTimes         */0, \
/*RgdtyTab[32][6]             */{/*0*/{20,300,15,3700,1500,0},\
								 /*1*/{25,300,20,2800,1100,0},\
								 /*2*/{30,300,25,2200,900,0},\
								 /*3*/{40,300,30,1900,800,0},\
								 /*4*/{45,300,35,1600,600,0},\
								 /*5*/{55,300,45,1200,500,0},\
								 /*6*/{75,300,60,900,400,0},\
								 /*7*/{95,300,75,700,300,0},\
								 /*8*/{115,300,90,600,300,0},\
								 /*9*/{140,300,110,500,200,0},\
								 /*APR_P|*|ASR_P|ASR_Ti|T_TrqCmdLpf|*/\
								 /*10*/{175,300,140,400,200,0},\
								 /*11*/{320,300,180,310,126,0},\
								 /*12*/{390,300,220,250,103,0},\
								 /*13*/{480,300,270,210,84,0},\
								 /*14*/{630,300,350,160,65,0},\
								 /*15*/{720,300,400,140,57,0},\
								 /*16*/{900,300,500,120,45,0},\
								 /*17*/{1080,300,600,110,38,0},\
								 /*18*/{1350,300,750,90,30,0},\
								 /*19*/{1620,300,900,80,25,0},\
	 	 	 	 	 	 	 	 /*APR_P|*|ASR_P|ASR_Ti|T_TrqCmdLpf|*/\
								 /*20*/{2060,300,1150,70,20,0},\
								 /*21*/{2510,300,1400,60,16,0},\
								 /*22*/{3050,300,1700,50,13,0},\
								 /*23*/{3770,300,2100,40,11,0},\
								 /*24*/{4490,300,2500,40,9,0},\
								 /*25*/{5000,300,2800,35,8,0},\
								 /*26*/{5600,300,3100,30,7,0},\
								 /*27*/{6100,300,3400,30,7,0},\
								 /*28*/{6600,300,3700,25,6,0},\
								 /*29*/{7200,300,4000,25,6,0},\
								 /*30*/{8100,300,4500,20,5,0},\
								 /*31*/{9000,300,5000,20,5,0}},\
/*RampTpu                  */0, \
		(void (*)(long)) Spd_GainAdpt_init ,\
		(void (*)(long)) Spd_GainAdpt_rst ,\
		(void (*)(long)) Spd_GainAdpt_update ,\
		(void (*)(long)) Spd_GainAdpt_calc }

void Spd_GainAdpt_init(SPD_GAINADPT_handle);
void Spd_GainAdpt_rst(SPD_GAINADPT_handle);
void Spd_GainAdpt_update(SPD_GAINADPT_handle);
void Spd_GainAdpt_calc(SPD_GAINADPT_handle);
extern SPD_GAINADPT SpdGainAdpt;
#endif /* SPD_GAINADPT_H_ */
