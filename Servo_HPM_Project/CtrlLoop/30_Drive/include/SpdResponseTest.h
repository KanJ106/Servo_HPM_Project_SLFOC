/*
 * SpdResponseTest.h
 *
 *  Created on: 2016-3-8
 *      Author: rd0213
 *///
#ifndef SPDRESPONSETEST_H_
#define SPDRESPONSETEST_H_
#include "IQmathLib.h" 
typedef struct {
				  _iq  StartFrq;
				  _iq  EndFrq;
				  _iq  DeltaFrq;
				  _iq  AmpRates;
				  _iq  Re;
				  _iq  Im;
				  _iq  Imag;
				  _iq  Phase;
				  Uint16 Cnt;
				  Uint16 CntMax;
				  Uint16 DelayCnt;
				  _iq  Out;
				 } SPDRESPONSETEST;
#define SPDRESPONSETEST_DEFAULTS {  \
/*StartFrq                     */0, \
/*EndFrq                       */0, \
/*DeltaFrq                     */0, \
/*AmpRates                     */0, \
/*Re                           */0, \
/*Im                           */0, \
/*Imag                         */0, \
/*Phase                        */0, \
/*Cnt                          */0, \
/*CntMax                       */0, \
/*DelayCnt                     */0, \
/*Out                          */0}
extern void SpdResponseTestUpdateInit(void);
extern void SpdResponseTest(void);
extern SPDRESPONSETEST SpdRespTest;
#endif /* SPDRESPONSETEST_H_ */
