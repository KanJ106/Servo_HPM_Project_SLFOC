/*
 * Cur_Clark.h
 *
 *  Created on: 2015-12-17
 *      Author: w//
 */

#ifndef CUR_CLARK_H_
#define CUR_CLARK_H_
#include "IQmathLib.h" 
typedef struct {
				 _iq  As;  			// Input: phase-a stator variable
				 _iq  Bs;			// Input: phase-b stator variable
				 _iq  Alpha;		// Output: stationary d-axis stator variable
				 _iq  Beta;			// Output: stationary q-axis stator variable
                 void  (*rst)();
				 void  (*calc)();	// Pointer to calculation function
				 } CUR_CLARKE;

typedef CUR_CLARKE *CUR_CLARKE_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_CLARKE object.
-----------------------------------------------------------------------------*/
#define CUR_CLARKE_DEFAULTS { \
/*As                     */0, \
/*Bs                     */0, \
/*Alpha                  */0, \
/*Alpha                  */0, \
                          (void (*)(long))Cur_clarke_rst, \
              			  (void (*)(long))Cur_clarke_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in CLARKE.C
------------------------------------------------------------------------------*/
void Cur_clarke_rst(CUR_CLARKE_handle);
void Cur_clarke_calc(CUR_CLARKE_handle);
extern void clark(void);
extern CUR_CLARKE Clark;
#endif /* CUR_CLARK_H_ */
