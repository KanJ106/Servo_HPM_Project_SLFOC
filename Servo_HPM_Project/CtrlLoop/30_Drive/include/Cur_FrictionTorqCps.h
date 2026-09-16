/*
 * Cur_FrictionTorqCps.h
 *
 *  Created on: 2015-12-18
 *      Author: yhj//
 */

#ifndef CUR_FRICTIONTORQCPS_H_
#define CUR_FRICTIONTORQCPS_H_
#include "IQmathLib.h" 
typedef struct 	{
				  _iq TorqCmdCpsRates;		//ת�ز�����
				  _iq FwdDirCmdCpsRates;	//����ת�ز�����
				  _iq RevsDirCmdCpsRates;	//����ת�ز�����
				  _iq Fk;					//�˲�ϵ��
				  _iq TorqCmdCpsPre;		//ת�ز���ֵ
				  _iq TorqCmdCps;			//����ת�ز���ֵ
				  void (*init)();
				  void (*rst)();
				  void (*calc)();	    	/* */
				} CUR_FRICTIONTORQCPS;

typedef CUR_FRICTIONTORQCPS *CUR_FRICTIONTORQCPS_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_FRICTIONTORQCPS object.
-----------------------------------------------------------------------------*/
#define CUR_FRICTIONTORQCPS_DEFAULTS { \
/*TorqCmdCpsRates                 */0, \
/*FwdDirCmdCpsRates               */0, \
/*RevsDirCmdCpsRates              */0, \
/*Fk                              */0, \
/*TorqCmdCpsPre                   */0, \
/*TorqCmdCps                      */0, \
		(void (*)(long)) Cur_FrictionTorqCps_init ,\
		(void (*)(long)) Cur_FrictionTorqCps_rst ,\
		(void (*)(long)) Cur_FrictionTorqCps_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in Cur_AccFb.c
------------------------------------------------------------------------------*/
void Cur_FrictionTorqCps_init(CUR_FRICTIONTORQCPS_handle);
void Cur_FrictionTorqCps_rst(CUR_FRICTIONTORQCPS_handle);
void Cur_FrictionTorqCps_calc(CUR_FRICTIONTORQCPS_handle);
extern void FrictionTorqCps_Calc(void);
extern CUR_FRICTIONTORQCPS Frctoqcps;
#endif /* CUR_FRICTIONTORQCPS_H_ */
