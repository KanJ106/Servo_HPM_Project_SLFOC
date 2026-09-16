/*
 * ReferenceTest.h
 *
 *  Created on: 2016-1-5
 *      Author: rd0213
 *///

#ifndef REFERENCETEST_H_
#define REFERENCETEST_H_
#include "IQmathLib.h" 
typedef enum
{
	TrigSingle = 0,
	TrigDulal = 1
}TrigMode;

struct TRIGMODE_BITS {
	Uint16 TrigMode:1;
	Uint16 Rsvd1:15;
};

union TRIGMODE {
   Uint16                 all;
   struct TRIGMODE_BITS    bit;
};
typedef struct 	{
				  union TRIGMODE TrigMod;
				  _iq TriangleRef;
				  _iq StepRef;
				  _iq SinRef;
				  _iq CosRef;
				  _iq ConstRef;
				  _iq TriangleRefDelta;
				  _iq TriangleRefMax;
				  _iq AngleDelta;
				  _iq Angle;
				  _iq Amp;
				  _iq ConstGive;
				  _iq Ref;
				  _iq StepRefLow;
				  _iq StepRefHigh;
				  Uint32 StepCnt;
				  Uint32 StepTimes;
				  void (*init)();
				  void (*rst)();
				  void (*calc)();	    /* */
				} REFERENCETEST;

typedef REFERENCETEST *REFERENCETEST_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the REFERENCETEST object.
-----------------------------------------------------------------------------*/
#define REFERENCETEST_DEFAULTS { \
/*TrigMod                   */0, \
/*TriangleRef               */0, \
/*StepRef                   */0, \
/*SinRef                    */0, \
/*CosRef                    */0, \
/*ConstRef                  */0, \
/*TriangleRefDelta          */0, \
/*TriangleRefMax            */0, \
/*AngleDelta                */0, \
/*Angle                     */0, \
/*Amp                       */_IQ(0.5), \
/*ConstGive                 */0, \
/*Ref                       */0, \
/*StepRefLow                */0, \
/*StepRefHigh               */0, \
/*StepCnt                   */0, \
/*StepTimes                 */0, \
/*(*init)()                 */(void (*)(long)) ReferenceTest_init ,\
/*(*rst)()                  */(void (*)(long)) ReferenceTest_rst ,\
/*(*calc)()                 */(void (*)(long)) ReferenceTest_calc}

/*------------------------------------------------------------------------------
Prototypes for the functions in ReferenceTest.c
------------------------------------------------------------------------------*/
void ReferenceTest_init(REFERENCETEST_handle);
void ReferenceTest_rst(REFERENCETEST_handle);
void ReferenceTest_calc(REFERENCETEST_handle);


typedef struct 	{
				  _iq SinRef;
				  _iq CosRef;
				  _iq ConstRef;
				  _iq Angle;
				  _iq AngleDelta;
				  _iq Amp;
				  void (*init)();
				  void (*rst)();
				  void (*calc)();	    /* */
				} SINGEN;

typedef SINGEN *SINGEN_handle;


#define SINGEN_DEFAULTS {   \
/*SinRef               */0, \
/*CosRef               */0, \
/*ConstRef             */0, \
/*Angle                */0, \
/*AngleDelta           */0, \
/*Amp                  */0, \
/*(*init)()            */(void (*)(long)) SinGen_init,\
/*(*rst)()             */(void (*)(long)) SinGen_rst,\
/*(*calc)()            */(void (*)(long)) SinGen_calc}
void SinGen_init(SINGEN_handle);
void SinGen_rst(SINGEN_handle);
void SinGen_calc(SINGEN_handle);




typedef struct 	{
				  _iq Cnt;
				  _iq Times;
				  _iq Ref;
				  _iq RefLow;
				  _iq RefHigh;
				  _iq Const;
				  void (*init)();
				  void (*rst)();
				  void (*calc)();	    /* */
				} STEPGEN;

typedef STEPGEN *STEPGEN_handle;


#define STEPGEN_DEFAULTS { \
/*Cnt                 */0, \
/*Times               */0, \
/*Ref                 */0, \
/*RefLow              */0, \
/*RefHigh             */0, \
/*Const               */0, \
/*(*init)()           */(void (*)(long)) StepGen_init,\
/*(*rst)()            */(void (*)(long)) StepGen_rst,\
/*(*calc)()           */(void (*)(long)) StepGen_calc}
void StepGen_init(STEPGEN_handle);
void StepGen_rst(STEPGEN_handle);
void StepGen_calc(STEPGEN_handle);
extern REFERENCETEST RefTest;
extern SINGEN SinGen;
extern SINGEN SinGen1;
extern SINGEN SinGen2;
extern SINGEN SinGentst;
extern SINGEN SinGentst1;
extern STEPGEN StepGen;
#endif /* REFERENCETEST_H_ */
