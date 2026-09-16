/*
 * ScurveAccDec.h
 *
 *  Created on: 2016-1-13
 *      Author: rd0213
 *///

#ifndef SCURVEACCDEC_H_
#define SCURVEACCDEC_H_
#include "IQmathLib.h" 
typedef struct 	{
				  _iq Target;			//目标值
				  _iq Output;			//输出值
				  _iq AccDelta;			//加速度
				  _iq DecDelta;			//减速度
				  _iq ZstopDecDelta;	//零速停车减速度
				  _iq NPOTDecDelta;		//超程停车减速度
                  _iq SFaultDecDelta;	//二级故障零速停车
				  _iq JogDecDelta;		//jog减速度
                  _iq ScopeJogDecDelta;	//jog减速度
				  _iq Delta;			//加速度给定
				  _iq DeltaOut;			//加速度输出
				  _iq Err;				//速度误差
				  _iq Remaind;			//剩余速度
				  _iq ArcAccHeight;		//圆弧加高度
				  _iq ArcDecHeight;		//圆弧减高度
				  _iq DeltaAccSquare;	//加加速
				  _iq DeltaDecSquare;	//减减速

				  _iq in;				//s曲线输入标幺值
				  _iq out;				//s曲线输出标幺值
				  _iq Rem;				//内部余数

				  Uint16 TSpdAcc;		//减加速度时间单位ms
				  Uint16 TSpdDec;		//减速度时间单位ms
				  Uint16 TZstop;		//零速停车减速度时间单位ms
				  Uint16 TNPot;			//超程速停车减速度时间单位ms
				  Uint16 Tjog;			//jog停车减速度时间单位ms
				  Uint16 TSpdAccDecLpf;	//圆弧时间ms
				  Uint16 Tsamp;			//计算周期单位0.01us

				  Uint16 ArcTimes;		//最终圆弧时间
				  Uint32 ArcSteps;		//圆弧步进数
				  Uint16 SuvType;		//s类型
				  Uint16 CmpFlg;			//完成标志
				  Uint16 States;			//所处状态
				  void (*init)();
				  void (*rst)();
				  void (*update)();
				  void (*calc)();	    /* */
				} SCURVEACCDEC;

typedef SCURVEACCDEC *SCURVEACCDEC_handle;

#define SCURVEACCDEC_DEFAULTS { \
/*Target                   */0, \
/*Output                   */0, \
/*AccDelta                 */0, \
/*DecDelta                 */0, \
/*ZstopDecDelta            */0, \
/*NPOTDecDelta             */0, \
/*SFaultDecDelta             */0, \
/*JogDecDelta              */0,0, \
/*Delta                    */0, \
/*DeltaOut                 */0, \
/*Err                      */0, \
/*Remaind                  */0, \
/*ArcAccHeight             */0, \
/*ArcDecHeight             */0, \
/*DeltaAccSquare           */0, \
/*DeltaDecSquare           */0, \
/*in                       */0, \
/*out                      */0, \
/*Rem                      */0, \
/*TSpdAcc                  */0, \
/*TSpdDec                  */0, \
/*TZstop                   */0, \
/*TNPot                    */0, \
/*Tjog                     */0, \
/*TSpdAccDecLpf            */0, \
/*Tsamp                    */0, \
/*ArcTimes                 */0, \
/*ArcSteps                 */0, \
/*SuvType                  */0, \
/*CmpFlg                   */0, \
/*States                   */0, \
/*(*init)()                */(void (*)(long)) Scurve_init ,\
/*(*rst)()                 */(void (*)(long)) Scurve_rst ,\
/*(*update)()              */(void (*)(long)) Scurve_update ,\
/*(*calc)()                */(void (*)(long)) Scurve_calc }

void Scurve_init(SCURVEACCDEC_handle);
void Scurve_rst(SCURVEACCDEC_handle);
void Scurve_update(SCURVEACCDEC_handle);
void Scurve_calc(SCURVEACCDEC_handle);


typedef struct 	{
	  	  	  	  _iq Target;
	  	  	  	  _iq Output;
				  _iq AccDelta;
				  _iq DecDelta;
				  _iq Err;
				  void (*init)();
				  void (*rst)();
				  void (*update)();
				  void (*calc)();	    /* */
				} RAMPGENERATOR;

typedef RAMPGENERATOR *RAMPGENERATOR_handle;

#define RAMPGENERATOR_DEFAULTS { \
/*Target                    */0, \
/*Output                    */0, \
/*AccDelta                  */0, \
/*DecDelta                  */0, \
/*Err                       */0, \
/*(*init)()                 */(void (*)(long)) RampGenerator_init ,\
/*(*rst)()                  */(void (*)(long)) RampGenerator_rst ,\
/*(*update)()               */(void (*)(long)) RampGenerator_update ,\
/*(*calc)()                 */(void (*)(long)) RampGenerator_calc }

void RampGenerator_init(RAMPGENERATOR_handle);
void RampGenerator_rst(RAMPGENERATOR_handle);
void RampGenerator_update(RAMPGENERATOR_handle);
void RampGenerator_calc(RAMPGENERATOR_handle);
extern SCURVEACCDEC SpdScurve;
extern SCURVEACCDEC PosScuv;
extern RAMPGENERATOR RampKp;
extern RAMPGENERATOR RampKi;
extern RAMPGENERATOR RampKPp;
extern RAMPGENERATOR TorqGen;
#endif /* SCURVEACCDEC_H_ */
