/*
 * Cur_IqRef.h
 *
 *  Created on: 2015-12-30
 *      Author: w
 *///
#ifndef CUR_IQREF_H_
#define CUR_IQREF_H_
#include "IQmathLib.h" 
typedef struct 	{
                Uint16 IqRefGet;			//Q轴电流给定来源选择，为1时为内部强制给定
				_iq IqRef;					//Q轴电流给定
				_iq IqRefInner;				//Q轴电流内部强制给定时的值
                _iq IqEcatOffset;           //ECAT偏移值
				_iq TorqModCCWCWLimt;		//力矩模式时转矩限制值
				_iq TlmtCCW;				//最终正转力矩限制值
				_iq TlmtCW;					//最终反转转矩限制值
 				_iq TlmtEcatCCW;			//最终正转力矩限制值
				_iq TlmtEcatCW;				//最终反转转矩限制值               
                _iq TlmtsysCCW;             //来自用户的转矩限制
                _iq TlmtsysCW;              //来自用户的转矩限制
				_iq Ref;					//Q轴给定过渡变量滤波前
                _iq Refilter;				//Q轴给定过渡变量滤波后
				_iq fK;						//转矩给定一阶低通滤波系数
                int16 IqEcatCmd;             //ECAT给定
				Uint16  Tsamp;				//电流环调度周期
				Uint16  Tao;				//一阶低通滤波时间常数
				Uint16  Cnt;				//402协议力矩模式时，斜坡给定时1S钟计数器
				Uint16  MaxCnt;				//1s计数器最大值
                Uint16  TorqueFlag;
                Uint16  EcatMax_P;
                Uint16  EcatMax_N;
				void (*init)();
				void (*rst)();
				void (*update)();
				void (*calc)();
				} CUR_IQREF;
typedef CUR_IQREF *CUR_IQREF_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_IQREF object.
-----------------------------------------------------------------------------*/
#define CUR_IQREF_DEFAULTS {\
/*IqRefGet				*/0,\
/*IqRef					*/0,\
/*IqRefInner			*/0,\
/*IqEcatOffset			*/0,\
/*TorqModCCWCWLimt  	*/0,\
/*TlmtCCW				*/0,\
/*TlmtCW				*/0,\
/*TlmtEcatCCW			*/0,\
/*TlmtEcatCW			*/0,\
/*TlmtsysCCW			*/0,\
/*TlmtsysCW				*/0,\
/*Ref					*/0,0,\
/*fK					*/0,\
/*IqEcatCmd				*/0,\
/*Tsamp					*/0,\
/*Tao					*/0,\
/*Cnt					*/0,\
/*MaxCnt				*/0,\
/*TorqueFlag	        */0,\
/*EcatMax_P	            */0,\
/*EcatMax_N	            */0,\
                        (void (*)(long))Cur_IqRef_init,\
						(void (*)(long))Cur_IqRef_rst,\
						(void (*)(long))Cur_IqRef_update,\
              			(void (*)(long))Cur_IqRef_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in Cur_IqRef.C
------------------------------------------------------------------------------*/
void Cur_IqRef_init(CUR_IQREF_handle);
void Cur_IqRef_rst(CUR_IQREF_handle);
void Cur_IqRef_update(CUR_IQREF_handle);
void Cur_IqRef_calc(CUR_IQREF_handle);

extern void SpdToIq_Cacl(void);
extern _iq SpdTorFwd(void);
extern void IqRef_Calc(void);

extern CUR_IQREF IqRef;

#endif /* CUR_IQREF_H_ */


