/*
 * Cur_DeadCps.h
 *
 *  Created on: 2015-12-17//
 *      Author: w
 */

#ifndef CUR_DEADCPS_H_
#define CUR_DEADCPS_H_
#include "IQmathLib.h" 
typedef struct 	{
				  _iq Etheta;			/* Input:    */
				  _iq Ia;			    /* Input:    */
				  _iq Ib; 				/* Input:    */
				  _iq Ic; 				/* Input:    */
				  _iq Uain;			    /* Input:    */
				  _iq Ubin; 			/* Input:    */
				  _iq Ucin; 			/* Input:    */
				  Uint16 DeadCpsSwch;	/* Input:    */
				  _iq Uaout; 			/* Output:   */
				  _iq Ubout; 			/* Output:   */
				  _iq Ucout;			/* Output:   */
				  void (*init)();
				  void (*rst)();
				  void (*calc)();	    /* */
				} CUR_DEADCPS;

typedef CUR_DEADCPS *CUR_DEADCPS_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_DEADCPS object.
-----------------------------------------------------------------------------*/
#define CUR_DEADCPS_DEFAULTS { \
/*Etheta                   */0,\
/*Ia                       */0,\
/*Ib                       */0,\
/*Ic                       */0,\
/*Uain                     */0,\
/*Ubin                     */0,\
/*Ucin                     */0,\
/*DeadCpsSwch              */0,\
/*Uaout                    */0,\
/*Ubout                    */0,\
/*Ucout                    */0,\
		(void (*)(long)) Cur_DeadCps_init ,\
		(void (*)(long)) Cur_DeadCps_rst ,\
		(void (*)(long)) Cur_DeadCps_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in Cur_AccFb.c
------------------------------------------------------------------------------*/
void Cur_DeadCps_init(CUR_DEADCPS_handle);
void Cur_DeadCps_rst(CUR_DEADCPS_handle);
void Cur_DeadCps_calc(CUR_DEADCPS_handle);


extern CUR_DEADCPS             DeadCps;

#endif /* CUR_DEADCPS_H_ */
