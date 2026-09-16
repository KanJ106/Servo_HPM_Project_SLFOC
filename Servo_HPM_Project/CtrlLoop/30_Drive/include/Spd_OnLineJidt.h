/*
 * Spd_OnLineJidt.h
 *
 *  Created on: 2016-2-23
 *      Author: rd0213
 *///

#ifndef SPD_ONLINEJIDT_H_
#define SPD_ONLINEJIDT_H_
#include "IQmathLib.h" 
typedef struct
{
	Uint16 method;
	Uint16 StFlg;
	Uint16 RegSatFlg;

	Uint16 Ib;
	Uint16 Kt;
	Uint16 nb;
	_iq ts;
	_iq OneIQ;
	_iq Coff;
	_iq w_new;
	_iq w_old_1;
	_iq w_old_2;
	_iq delta_w;
	_iq estw;
	_iq te_new;
	_iq tor_d_old_1;
	_iq tor_d_old_2;
	_iq delta_t;
	_iq b_new;
	_iq b_old;
	_iq J;
	_iq gama;
	Uint16 AccLimt;
	int16 Spd;
	int16 SpdOld;
	void (*init)();
	void (*rst)();
	void (*calc)();	    /* */
}SPD_ONLINEJIDT;
typedef SPD_ONLINEJIDT *SPD_ONLINEJIDT_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SPD_ONLINEJIDT object.
-----------------------------------------------------------------------------*/
#define SPD_ONLINEJIDT_DEFAULTS { \
/*method                     */0, \
/*StFlg                      */0, \
/*RegSatFlg                  */0, \
/*Ib                         */0, \
/*Kt                         */0, \
/*nb                         */0, \
/*ts                         */0, \
/*OneIQ                      */0, \
/*Coff                       */0, \
/*w_new                      */0, \
/*w_old_1                    */0, \
/*w_old_2                    */0, \
/*delta_w                    */0, \
/*estw                       */0, \
/*te_new                     */0, \
/*tor_d_old_1                */0, \
/*tor_d_old_2                */0, \
/*delta_t                    */0, \
/*b_new                      */0, \
/*b_old                      */0, \
/*J                          */0, \
/*gama                       */0, \
/*AccLimt                    */0, \
/*Spd                        */0, \
/*SpdOld                     */0, \
/*(*init)()                  */(void (*)(long)) OnLineJidt_init,\
/*(*rst)()                   */(void (*)(long)) OnLineJidt_rst,\
/*(*calc)()                  */(void (*)(long)) OnLineJidt_calc }
void OnLineJidt_init(SPD_ONLINEJIDT_handle);
void OnLineJidt_rst(SPD_ONLINEJIDT_handle);
void OnLineJidt_calc(SPD_ONLINEJIDT_handle);
extern SPD_ONLINEJIDT OnLineJidt;
#endif /* SPD_ONLINEJIDT_H_ */
