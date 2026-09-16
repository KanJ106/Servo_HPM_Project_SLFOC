/*
 * Cur_Ipark.h
 *
 *  Created on: 2015-12-18
 *      Author: w
 *///

#ifndef CUR_IPARK_H_
#define CUR_IPARK_H_
#include "Cur_IdPiReg.h"//
#include "Cur_IqPiReg.h"
#include "Cur_UdFwd.h"

typedef struct {  _iq  Alpha;  		// Output: stationary d-axis stator variable
				  _iq  Beta;		// Output: stationary q-axis stator variable
				  _iq  Ds;			// Input: rotating d-axis stator variable
				  _iq  Qs;			// Input: rotating q-axis stator variable
				  void  (*rst)();
		 	 	  void  (*calc)();	// Pointer to calculation function
				 } CUR_IPARK;

typedef CUR_IPARK *CUR_IPARK_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_IPARK object.
-----------------------------------------------------------------------------*/
#define CUR_IPARK_DEFAULTS { \
/*Alpha					*/0, \
/*Beta					*/0, \
/*Ds					*/0, \
/*Qs					*/0, \
                         (void (*)(Uint32))Cur_Ipark_rst, \
              			 (void (*)(Uint32))Cur_Ipark_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in CUR_IPARK.C
------------------------------------------------------------------------------*/
void Cur_Ipark_rst(CUR_IPARK_handle);
void Cur_Ipark_calc(CUR_IPARK_handle);
extern void ipark_Calc(_iq Cosine,_iq Sine);
extern _iq IparkqRefLim(CUR_IQPIREG *vq,CUR_UDFWD   *ud);
extern _iq IparkdRefLim(CUR_IDPIREG *vd,CUR_UDFWD   *ud);
extern CUR_IPARK Ipark;
#endif /* CUR_IPARK_H_ */
