/*
 * Spd_Ref.h
 *
 *  Created on: 2015-12-30
 *      Author: w
 *///

#ifndef SPD_REF_H_
#define SPD_REF_H_
#include "IQmathLib.h" 
typedef struct
{
		_iq Ref;			//最终速度给定
		int16 SpdRefDecm;	//速度给定对应的十进制
		_iq Spdinput;		//S曲线前的的速度给定
        _iq EcatIn;         //ECAT CSV给定速度
		Uint16 SurvRcdFlg;	//速度曲线规划前的速度值来源(1:速度模式；0：位置/力矩)
        Uint16 SpdCmdSoure;
		void (*init)();
		void (*rst)();
		void (*calc)();	    /* */
} SPD_REF;
typedef SPD_REF *SPD_REF_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SPD_REF object.
-----------------------------------------------------------------------------*/
#define SPD_REF_DEFAULTS {  \
/*Ref                  */0, \
/*SpdRefDecm           */0, \
/*Spdinput             */0, \
/*SpdEcatIn            */0, \
/*SurvRcdFlg           */0, \
/*SpdCmdSoure          */0, \
/*(*init)()            */(void (*)(long)) Spd_Ref_init ,\
/*(*rst)()             */(void (*)(long)) Spd_Ref_rst ,\
/*(*calc)()            */(void (*)(long)) Spd_Ref_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in Spd_Ref.c
------------------------------------------------------------------------------*/
void Spd_Ref_init(SPD_REF_handle);
void Spd_Ref_rst(SPD_REF_handle);
void Spd_Ref_calc(SPD_REF_handle);
extern void SpdRef_Calc(void);
extern SPD_REF Spd_Ref;
#endif /* SPD_REF_H_ */
