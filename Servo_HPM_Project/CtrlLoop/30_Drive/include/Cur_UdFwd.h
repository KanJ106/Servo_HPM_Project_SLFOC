/*
 * Cur_UdFwd.h
 *
 *  Created on: 2015-12-18
 *      Author: w
 *///

#ifndef CUR_UDFWD_H_
#define CUR_UDFWD_H_
#include "IQmathLib.h" 
typedef struct {
				 _iq  iqFb;//q轴电流反馈
				 _iq  idFb;//d轴电流反馈
				 _iq  Faif;//磁链系数
				 _iq  Faid;//磁链系数
				 _iq  UqidFwd;//q轴前馈电压
				 _iq  UqFaifFwd;//q轴磁链系数
				 _iq  qOut;//最终q轴前馈电压
				 _iq  Faiq;//磁链系数
				 _iq  dOut;//最终d轴前馈电压
				 _iq Rate;//补偿系数
				 _iq Coeff1;//内部过程系数1
				 _iq Coeff2;//内部过程系数2
			     Uint16 Sw;//电压前馈开关
				 void  (*init)();
				 void  (*rst)();
				 void  (*calc)();	// Pointer to calculation function
				 } CUR_UDFWD;

typedef CUR_UDFWD *CUR_UDFWD_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_UDFWD object.
-----------------------------------------------------------------------------*/
#define CUR_UDFWD_DEFAULTS {   \
/*iqFb                    */0, \
/*idFb                    */0, \
/*Faif                    */0, \
/*Faid                    */0, \
/*UqidFwd                 */0, \
/*UqFaifFwd               */0, \
/*qOut                    */0, \
/*Faiq                    */0, \
/*dOut                    */0, \
/*Rate                    */0, \
/*Coeff1                  */0, \
/*Coeff2                  */0, \
/*Sw                      */0, \
                          (void (*)(long))Cur_UdFwd_init, \
                          (void (*)(long))Cur_UdFwd_rst, \
              			  (void (*)(long))Cur_UdFwd_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in CUR_UDFWD.C
------------------------------------------------------------------------------*/
void Cur_UdFwd_init(CUR_UDFWD_handle);
void Cur_UdFwd_rst(CUR_UDFWD_handle);
void Cur_UdFwd_calc(CUR_UDFWD_handle);
void UdUqFwd_Calc(void);
extern CUR_UDFWD UFwd;
#endif /* CUR_UDFWD_H_ */




