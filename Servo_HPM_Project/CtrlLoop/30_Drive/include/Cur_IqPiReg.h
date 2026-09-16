/*
 * Cur_IqPiReg.h
 *
 *  Created on: 2015-12-18
 *      Author: w
 *///
#ifndef CUR_IQPIREG_H_
#define CUR_IQPIREG_H_
#include "IQmathLib.h" 
typedef struct {
                Uint16  PiSatFlg;		//���ͱ�־
				  _iq  	Ref;   			// Input: Reference input
				  _iq  	RefOld;   		// Input: Reference input
				  _iq  	Fdb;   			// Input: Feedback input
				  _iq  	Err;			// Variable: Error
				  _iq  	Kp;				// Parameter: Proportional gain
				  _iq  	Up;				// Variable: Proportional output
				  _iq  	Ui;				// Variable: Integral output
				  _iq  	Ud;				// Variable: Derivative output
				  _iq  	OutPreSat; 		// Variable: Pre-saturated output
				  _iq  	OutMax;		    // Parameter: Maximum output
				  _iq  	OutMin;	    	// Parameter: Minimum output
				  _iq  	Out;   			// Output: PID output
				  _iq  	SatErr;			// Variable: Saturated difference
				  _iq  	Ki;			    // Parameter: Integral gain
				  _iq  	Kc;		     	// Parameter: Integral correction gain
				  _iq  	Kd; 		    // Parameter: Derivative gain
				  _iq  	Up1;		   	// History: Previous proportional output
                  _iq  	Kp2;			// Parameter: Proportional gain
				  _iq  	Up2;			// Variable: Proportional output
				 Uint16 GainSw;
				  _iq   PChgGain;		//�����л�ϵ��
				  _iq   PChgOffSet;		//�����л�ƫ��
				  _iq   IChgGain;		//�����л�ϵ��
				  _iq   IChgOffSet;		//�����л�ƫ��
				  _iq   PWMDuty;		//��ʱռ�ձ�
				 Uint16 halfpflg;		//1/2p��־
                 Uint16   Gain_M;         //多段增益开关
				  void  (*init)();
				  void  (*update)();
				  void  (*rst)();
				  void  (*calc)();	  	// Pointer to calculation function
				 } CUR_IQPIREG;

typedef CUR_IQPIREG *CUR_IQPIREG_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_IQPIREG object.
-----------------------------------------------------------------------------*/
#define CUR_IQPIREG_DEFAULTS { 	 \
/*PiSatFlg					*/0, \
/*Ref						*/0, \
/*RefOld					*/0, \
/*Fdb						*/0, \
/*Err						*/0, \
/*Kp						*/_IQ(1.3), \
/*Up						*/0, \
/*Ui						*/0, \
/*Ud						*/0, \
/*OutPreSat					*/0, \
/*OutMax					*/_IQ(1), \
/*OutMin					*/_IQ(-1), \
/*Out						*/0, \
/*SatErr					*/0, \
/*Ki						*/_IQ(0.02), \
/*Kc						*/_IQ(0.2), \
/*Kd						*/_IQ(0.0), \
/*Up1						*/0,0,0, \
/*GainSw					*/0, \
/*PChgGain					*/_IQ(1.0), \
/*PChgOffSet				*/_IQ(0.0), \
/*IChgGain					*/_IQ(1.0), \
/*IChgOffSet				*/_IQ(0.0), \
/*PWMDuty					*/_IQ(0.5), \
/*halfpflg					*/0,0,\
                          (void (*)(Uint32))Cur_IqPiReg_init, \
                          (void (*)(Uint32))Cur_IqPiReg_update, \
                          (void (*)(Uint32))Cur_IqPiReg_rst, \
              			  (void (*)(Uint32))Cur_IqPiReg_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in Cur_IqPiReg.c
------------------------------------------------------------------------------*/
void Cur_IqPiReg_init(CUR_IQPIREG_handle);
void Cur_IqPiReg_update(CUR_IQPIREG_handle);
void Cur_IqPiReg_rst(CUR_IQPIREG_handle);
void Cur_IqPiReg_calc(CUR_IQPIREG_handle);
void IqPiReg_Calc(void);
extern CUR_IQPIREG IqPiReg;
#endif /* CUR_IQPIREG_H_ */
