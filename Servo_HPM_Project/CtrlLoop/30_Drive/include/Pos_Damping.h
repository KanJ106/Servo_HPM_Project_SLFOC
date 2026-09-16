/*
 * Pos_Damping.h
 *
 *  Created on: 2016-1-27
 *      Author: rd0213
 *///

#ifndef POS_DAMPING_H_
#define POS_DAMPING_H_
#include "IQmathLib.h" 
//位置指令陷波器结构体
typedef struct 	{
    int32   in;
    int32   out;
	int32   fb2;                 //
	int32   fb3;	             //
    int64   rem1;                //DenCoeff[0]的倒数
    int64   rem2;                //倒数的截尾误差
    int64   rem3;                //采样频率
    int32   PosErrSum;
    Uint32  K1Q24;               //带阻滤波中心频率
	Uint32  K2Q24;               //分母部分中心频率
    Uint32  K3Q24;		         //陷波器分子阻尼比
    
    Uint16  Mask;
    void (*init)();
    void (*rst)();
    void (*calc)();	    /* */
} POS_DAMPING;
typedef POS_DAMPING *POS_DAMPING_handle;

#define POS_DAMPING_DEFAULTS { \
/*in                         */0, \
/*out                        */0, \
/*fb2                        */0, \
/*fb3                        */0, \
/*rem1                       */0, \
/*rem2                       */0,\
/*rem3                       */0,\
/*PosErrSum                  */0,\
/*K1Q24                      */0,\
/*K2Q24                      */0,\
/*K3Q24                      */0,\
/*Mask                       */0,\
/*void (*init)()          */(void (*)(long)) Pos_Damping_init ,\
/*void (*rst)()           */(void (*)(long)) Pos_Damping_rst ,\
/*void (*calc)()          */(void (*)(long)) Pos_Damping_calc}


void Pos_Damping_init(POS_DAMPING_handle);
void Pos_Damping_rst(POS_DAMPING_handle);
void Pos_Damping_calc(POS_DAMPING_handle);
extern POS_DAMPING PosDamping;
#endif /* POS_DAMPING_H_ */
