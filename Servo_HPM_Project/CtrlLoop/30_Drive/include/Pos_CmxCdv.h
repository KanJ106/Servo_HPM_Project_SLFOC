/*
 * Pos_CmxCdv.h
 *
 *  Created on: 2016-1-27
 *      Author: rd0213
 *///

#ifndef POS_CMXCDV_H_
#define POS_CMXCDV_H_
#include "IQmathLib.h" 
typedef struct 	{
				  int32 Ref;						//输入的脉冲数
				  int32 Out;						//经过电子齿轮后的脉冲数
				  int32 PosCmxNum;					//电子齿轮分子
				  int32 PosCdv;						//电子齿轮分母
				  int32 CmxCdvRem;					//电子齿轮计算后的余数
				  Uint16 Mask;						//电子齿轮计算屏蔽标志位，为1时旁路电子齿轮计算
				  void (*init)();
				  void (*rst)();
				  void (*calc)();	    /* */
				} POS_CMXCDV;

typedef POS_CMXCDV *POS_CMXCDV_handle;

#define POS_CMXCDV_DEFAULTS {  \
/*Ref                     */0, \
/*Out                     */0, \
/*PosCmxNum               */0, \
/*PosCdv                  */0, \
/*CmxCdvRem               */0, \
/*Mask                    */0, \
/*void (*init)()          */(void (*)(long)) Pos_CmxCdv_init ,\
/*void (*rst)()           */(void (*)(long)) Pos_CmxCdv_rst ,\
/*void (*calc)()          */(void (*)(long)) Pos_CmxCdv_calc}

void Pos_CmxCdv_init(POS_CMXCDV_handle);
void Pos_CmxCdv_rst(POS_CMXCDV_handle);
void Pos_CmxCdv_calc(POS_CMXCDV_handle);
extern POS_CMXCDV PosCmxCdv;
#endif /* POS_CMXCDV_H_ */
