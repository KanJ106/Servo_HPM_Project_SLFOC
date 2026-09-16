/*
 * Cur_Park.h
 *
 *  Created on: 2015-12-19
 *      Author: w
 *///

#ifndef CUR_PARK_H_
#define CUR_PARK_H_
#include "IQmathLib.h" 
typedef struct {  _iq  Alpha;  		// Input: stationary d-axis stator variable
				  _iq  Beta;	 	// Input: stationary q-axis stator variable
				  _iq  Ds;			// Output: rotating d-axis stator variable
				  _iq  Qs;			// Output: rotating q-axis stator variable
				  void  (*rst)();
		 	 	  void  (*calc)();	// Pointer to calculation function
				 } CUR_PARK;

typedef CUR_PARK *CUR_PARK_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_PARK object.
-----------------------------------------------------------------------------*/
#define CUR_PARK_DEFAULTS {	  \
/*Alpha                  */0, \
/*Beta                   */0, \
/*Ds                     */0, \
/*Qs                     */0, \
                          (void (*)(Uint32))Cur_Park_rst, \
                          (void (*)(Uint32))Cur_Park_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in CUR_PARK.C
------------------------------------------------------------------------------*/
void Cur_Park_rst(CUR_PARK_handle);
void Cur_Park_calc(CUR_PARK_handle);

extern void park(_iq Cosine,_iq Sine);
extern CUR_PARK Park;
#endif /* CUR_PARK_H_ */
