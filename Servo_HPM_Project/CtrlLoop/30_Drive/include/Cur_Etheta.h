/*
 * Cur_Etheta.h
 *
 *  Created on: 2015-12-17
 *      Author: w//
 */

#ifndef CUR_ETHETA_H_
#define CUR_ETHETA_H_
#include "IQmathLib.h" 

typedef struct {
				 Uint32  MagEncoffsetAngle;				//低压伺服复合型编码器磁编得到的偏移电角度Unit 0.1°
				 int32  ZLatch;  						//Z信号处计数器的值
				 int32  ZLatchOld; 						//Z信号处计数器上次的值
				 int32  ZPos;	    					//Z信号处偏移电角度对应的脉冲个数
				 int32  UVWPos;							//UVW信号U信号处偏移电角度对应的脉冲个数
				 int32  MagPos;							//磁编推导得到的偏移电角度对应的脉冲个数
				 int32  Pos;							//编码器返回的单圈位置值
				 int32  PerAngleLines;					//每个极对数下的脉冲个数
				 _iq    EtaInerGet;						//电角度强制给定值
				 _iq    Spd; 						    //resvd
                 
				 _iq    Etheta;							//计算的原始电角度
				 _iq    EthetaOld;						//计算的原始电角度旧值
				 _iq    Ethetapk;						//Park变换需要的电角度
				 _iq    Ethetaipk;						//Ipark变换需要的电角度
				 _iq    EthetaCps;						//电角度补偿量
				 _iq   Coeff;							//
				 _iq   Coeff1;
                 _iq   Coeff2;
                 
                 Uint16  ZoffsetAngle;					//Z信号处或点击原点处的偏移电角度相对于电机U相Unit 0.1°
				 Uint16  UVWoffsetAngle;				//普通带UVW信号增量型编码器U的偏移电角度Unit 0.1°
				 Uint16 Nmax; 							//resvd
				 Uint16 MotPoles; 						//resvd
				 Uint16 EtaGetSwt; 						//电角度给定方式，为1时为内部强制给定
				 Uint16 CpsEtaFlg; 						//电角度是否补偿开关为1时不补偿
				 Uint16 ZcomeFlg; 						//普通ABZ型编码器Z信号是否到来标志
				 Uint16 ZEncAlmFlg;						// Output:
				 Uint16 ABEncAlmFlg;					// Output:
				 Uint16 Zcnt;							// Output:
				 Uint16 ABcnt;							// Output:
				 Uint16 EtheataDecm;
                 
				 void  (*init)();						// Pointer to calculation function
				 void  (*rst)();
				 void  (*calc)();						// Pointer to calculation function
				 } CUR_ETHETA;

typedef CUR_ETHETA *CUR_ETHETA_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_ETHETA object.
-----------------------------------------------------------------------------*/
#define CUR_ETHETA_DEFAULTS { 	 \
/*MagEncoffsetAngle			*/0, \
/*ZLatch					*/0, \
/*ZLatchOld					*/0, \
/*ZPos						*/0, \
/*UVWPos					*/0, \
/*MagPos					*/0, \
/*Pos						*/0, \
/*PerAngleLines				*/0, \
/*EtaInerGet				*/0, \
/*Spd						*/0, \
/*Etheta					*/0, \
/*EthetaOld					*/0, \
/*Ethetapk					*/0, \
/*Ethetaipk					*/0, \
/*EthetaCps					*/0, \
/*Coeff						*/0, \
/*Coeff1					*/0, \
/*Coeff2					*/0, \
/*ZoffsetAngle				*/0, \
/*UVWoffsetAngle			*/0, \
/*Nmax						*/0, \
/*MotPoles					*/0, \
/*EtaGetSwt					*/0, \
/*CpsEtaFlg					*/0, \
/*ZcomeFlg					*/0, \
/*ZEncAlmFlg				*/0, \
/*ABEncAlmFlg				*/0, \
/*Zcnt						*/0, \
/*ABcnt						*/0, \
/*EtheataDecm				*/0, \
              			  (void (*)(long))Cur_Etheta_init,\
              			  (void (*)(long))Cur_Etheta_rst,\
              			  (void (*)(long))Cur_Etheta_calc}
/*------------------------------------------------------------------------------
Prototypes for the functions in Cur_Etheta.C
------------------------------------------------------------------------------*/
void Cur_Etheta_init(CUR_ETHETA_handle);
void Cur_Etheta_rst(CUR_ETHETA_handle);
void Cur_Etheta_calc(CUR_ETHETA_handle);

extern CUR_ETHETA Etheta;
extern void Cur_Ethetacps_calc(CUR_ETHETA *v);
#endif /* CUR_ETHETA_H_ */
