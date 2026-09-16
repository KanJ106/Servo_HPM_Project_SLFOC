/*
 * PosGenerator.h
 *
 *  Created on: 2016-1-18
 *      Author: rd0213
 *///

#ifndef POSGENERATOR_H_
#define POSGENERATOR_H_
#include "IQmathLib.h" 
struct PGEN_BITS {
	Uint16 PcmpFlg:1;
	Uint16 Posarrvsts:1;
	Uint16 State:4;
	Uint16 ClrFlg:1;
	Uint16 DirLogic:1;//反逻辑
    Uint16 rsd1:8;
};

union PGEN {
   Uint16              all;
   struct PGEN_BITS    bit;
};
typedef struct 	{
				  union PGEN Pgen;
				  int32 Pt;
				  int32 Po;
				  int64 Potemp;
				  int32 Rem;
                  int32 SpdRem;
//				  int32 PErrLimt;
//				 _iq SErrLimt;
				 int32 Remaind;
				 int32 SpdInput;
				 int32 Target;
				 int32 OutPut;
				 int32 Out;
				 int32 Delta;
				 int32 DeltaDec;
				 int32 DeltaDecRes;//实际减速用的减速度
				 int32 Srem;

				  void (*init)();
				  void (*rst)();
				  void (*update)();
				  void (*calc)();	    /* */
                  void (*deltacalc)();	    /* */
				} POSGENERATOR;

typedef POSGENERATOR *POSGENERATOR_handle;

#define POSGENERATOR_DEFAULTS { \
/*Pgen                     */0, \
/*Pt                       */0, \
/*Po                       */0, \
/*Potemp                   */0, \
/*Rem                      */0, \
/*SpdRem                   */0, \
/*Remaind                  */0, \
/*SpdInput                 */0, \
/*Target                   */0, \
/*OutPut                   */0, \
/*Out                      */0, \
/*Delta                    */0, \
/*DeltaDec                 */0, \
/*DeltaDecRes              */0, \
/*Srem                     */0, \
/*(*init)()                */(void (*)(long)) PosGenerator_init ,\
/*(*rst)()                 */(void (*)(long)) PosGenerator_rst ,\
/*(*update)()              */(void (*)(long)) PosGenerator_update ,\
/*(*calc)()                */(void (*)(long)) PosGenerator_calc,\
/*(*deltacalc)()           */(void (*)(long)) PosDeltaGen_calc }

void PosGenerator_init(POSGENERATOR_handle);
void PosGenerator_rst(POSGENERATOR_handle);
void PosGenerator_update(POSGENERATOR_handle);
void PosGenerator_calc(POSGENERATOR_handle);
void PosDeltaGen_calc(POSGENERATOR_handle);

extern POSGENERATOR PosGenerator;
extern POSGENERATOR Poscia402Gen;
extern POSGENERATOR HomingGen;
extern POSGENERATOR SysAdjustGen;
extern POSGENERATOR TestPosGen;
#endif /* POSGENERATOR_H_ */
