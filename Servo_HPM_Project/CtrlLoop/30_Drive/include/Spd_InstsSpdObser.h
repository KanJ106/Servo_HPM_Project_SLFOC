/*
 * Spd_InstsSpdObser.h
 *
 *  Created on: 2016-3-29
 *      Author: rd0213
 *///

#ifndef SPD_INSTSSPDOBSER_H_
#define SPD_INSTSSPDOBSER_H_
#include "IQmathLib.h" 
typedef struct 	{
				_iq win;					//input spd
                _iq spdF;				
                _iq wout;					//output spd
				_iq Err;					//obs err
				_iq tcmd;					//input torq
                _iq tcmdF;				
                _iq Pout;					//pout
				_iq Iout;
				_iq Tout;
				_iq OutPre;
				_iq OutMax;
				_iq OutMin;
				_iq pk1;					//obs gain1
				_iq pk2;					//obs gain2
                _iq pk3;                    //obs coeff
				_iq Tsamp;
				_iq coeff;
                float SpdFbFilter;
                float TcmdFilter;				
                Uint16 mask;					//obs calc option
				Uint16 Patable[14][2];		//������
				void (*init)();
				void (*rst)();
				void (*update)();
				void (*calc)();
				} SPD_INSTSSPDOBSER;
typedef SPD_INSTSSPDOBSER *SPD_INSTSSPDOBSER_handle;
/*-----------------------------------------------------------------------------
Default initalizer
-----------------------------------------------------------------------------*/
#define SPD_INSTSSPDOBSER_DEFAULTS { \
/*win                           */0, \
0, \
/*wout                          */0, \
/*Err                           */0, \
/*tcmd                          */0, \
0, \
/*Pout                          */0, \
/*Iout                          */0, \
/*Tout                          */0, \
/*OutPre                        */0, \
/*OutMax                        */0, \
/*OutMin                        */0, \
/*pk1                           */0, \
/*pk2                           */0, \
/*pk3                           */0, \
/*Tsamp                         */0, \
/*coeff                         */0, \
/*SpdFbFilter                   */0, \
/*TcmdFilter                    */0, \
/*mask                          */0, \
/*Patable[14][2]          */{{44,40},\
							 {89,158},\
							 {133,355},\
							 {178,632},\
							 {222,987},\
							 {267,1421},\
							 {311,1934},\
							 {444,3947},\
							 {666,8882},\
							 {889,15791},\
							 {1111,24674},\
							 {1333,35531},\
							 {1555,48361},\
							 {1777,63165}},\
							(void (*)(long))Spd_InstSpdObser_init,\
							(void (*)(long))Spd_InstSpdObser_rst,\
							(void (*)(long))Spd_InstSpdObser_update,\
              				(void (*)(long))Spd_InstSpdObser_calc }
/*------------------------------------------------------------------------------
Prototypes
------------------------------------------------------------------------------*/
void Spd_InstSpdObser_init(SPD_INSTSSPDOBSER_handle);
void Spd_InstSpdObser_rst(SPD_INSTSSPDOBSER_handle);
void Spd_InstSpdObser_update(SPD_INSTSSPDOBSER_handle);
void Spd_InstSpdObser_calc(SPD_INSTSSPDOBSER_handle);
void ObserInit(void);
extern SPD_INSTSSPDOBSER InstSpdObser;
#endif /* SPD_INSTSSPDOBSER_H_ */
