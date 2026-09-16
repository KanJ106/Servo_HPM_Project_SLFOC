/*
 * EncPosFb.h
 *
 *  Created on:
 *      Author:
 */
//
#ifndef ENCPOSFB_H_
#define ENCPOSFB_H_
#include "IQmathLib.h" 
typedef enum
{
	PosLg =0,
	NegLg =1
}PosLgFlg;
struct ENCPOSFBFLG_BITS {
	Uint16 PosLgFlg:1;
	Uint16 Rsvd1:7;
    Uint16 Rsvd2:8;
};

union ENCPOSFBFLG {
   Uint16                 all;
   struct ENCPOSFBFLG_BITS    bit;
};
typedef struct {
				 union ENCPOSFBFLG EncPosFlg;			//rsvd
                 Uint16 RxFlag;
				 int32 EncSinglePos;					//当前单圈位置值
                 int32 EncSingleErr;
                 
				 int16 MultiCircle;						//当前多圈值，只对多圈绝对值编码器有用
                 int32 OutEncSinglePos;					//输出端单圈位置值
                 int16 OutMultiCircle;                  //输出端多圈值              
				 void (*init)();
				 } ENCPOSFB;

typedef ENCPOSFB *ENCPOSFB_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the ENCPOSFB object.
-----------------------------------------------------------------------------*/
#define ENCPOSFB_DEFAULTS { \
/*EncPosFlg            */0, 0,\
/*EncSinglePos         */0, \
/*EncSingleErr         */0, \
/*MultiCircle          */0, \
/*OutEncSinglePos      */0, \              		  
/*OutMultiCircle       */0, \                      
                      (void (*)(long))EncPosFb_init}
/*------------------------------------------------------------------------------
Prototypes for the functions in EncPosFb.C
------------------------------------------------------------------------------*/
void EncPosFb_init(ENCPOSFB_handle);
extern void EncPosFb_Calc(void);
extern ENCPOSFB	EncPosFb;
#endif /* ENCPOSFB_H_ */
