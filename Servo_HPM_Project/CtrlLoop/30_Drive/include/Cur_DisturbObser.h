/*
 * Cur_DisturbObser.h
 *
 *  Created on: 2015-12-17
 *      Author: w//
 */

#ifndef CUR_DISTURBOBSER_H_
#define CUR_DISTURBOBSER_H_
#include "IQmathLib.h" 
typedef struct {
				 _iq  temp1;  		// Input:
				 _iq  temp2;	    // Input:
				 _iq  temp3;		// Output:
				 _iq  temp4;		// Output:
				 void  (*calc)();	// Pointer to calculation function
				 } CUR_DISTURBOBSER;

typedef CUR_DISTURBOBSER *CUR_DISTURBOBSER_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_DISTURBOBSER object.
-----------------------------------------------------------------------------*/
#define CUR_DISTURBOBSER_DEFAULTS { \
/*temp1						   */0, \
/*temp2                        */0, \
/*temp3                        */0, \
/*temp4                        */0, \
              			  (void (*)(long))Cur_DisturbObser_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in CLARKE.C
------------------------------------------------------------------------------*/
void Cur_DisturbObser_calc(CUR_DISTURBOBSER_handle);
#endif /* CUR_DISTURBOBSER_H_ */
