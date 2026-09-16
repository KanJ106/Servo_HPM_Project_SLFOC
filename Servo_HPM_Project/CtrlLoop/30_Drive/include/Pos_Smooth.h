/*
 * Pos_Smooth.h
 *
 *  Created on: 2016-1-27
 *      Author: rd0213
 *///

#ifndef POS_SMOOTH_H_
#define POS_SMOOTH_H_
#include "IQmathLib.h" 
typedef struct 	{
				  int32 Ref;				//位置平滑输入指令值
				  int32 Rem;				//位置平滑余数
				  int32 Out;				//位置平滑输出指令值
				  int32 Cmx;
				  int32 Cdv;
				  _iq   CdvInv;				//分母倒数
				  int32 intotle;			//rsvd
				  int32 outotle;			//rsvd
				  int32 Detotle;			//累积误差
				  int16 fK;					//滤波器系数
				  Uint16 Mask;				//位置指令平滑旁路选择，为1时旁路
				  void (*init)();
				  void (*rst)();
                  void (*updata)();
				  void (*calc)();	    	/* */
				} POS_SMOOTH;

typedef POS_SMOOTH *POS_SMOOTH_handle;

#define POS_SMOOTH_DEFAULTS { \
/*Ref                    */0, \
/*Rem                    */0, \
/*Out                    */0, \
/*Cmx                    */0, \
/*Cdv                    */0, \
/*CdvInv                 */0, \
/*intotle                */0, \
/*outotle                */0, \
/*Detotle                */0, \
/*fK                     */0, \
/*Mask                   */0, \
/*(*init)()              */(void (*)(long)) Pos_Smooth_init ,\
/*(*rst)()               */(void (*)(long)) Pos_Smooth_rst ,\
/*(*updata)()            */(void (*)(long)) Pos_Smooth_updata ,\
/*(*calc)()              */(void (*)(long)) Pos_Smooth_calc}

void Pos_Smooth_init(POS_SMOOTH_handle);
void Pos_Smooth_rst(POS_SMOOTH_handle);
void Pos_Smooth_calc(POS_SMOOTH_handle);
void Pos_Smooth_updata(POS_SMOOTH_handle);
extern POS_SMOOTH PosSmooth;


#endif /* POS_SMOOTH_H_ */
