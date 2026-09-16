/*
 * Cur_AdaptiveFilter.h
 *
 *  Created on: 2016-3-31
 *      Author: rd0213//
 */

#ifndef CUR_ADAPTIVEFILTER_H_
#define CUR_ADAPTIVEFILTER_H_
#include "IQmathLib.h" 
//陷波滤波器结构体类型定义
typedef struct
{
    int32   in;
    int32   NotchInput[2];	  //保存的输入参数
	int32   NotchOutput[2];   //保存的输出参数    
    int32   out;
    
	Uint16  k1;               //数字陷波器分子系数
	Uint16  k2;               //数字陷波器分子系数
    Uint16  k3;               //数字陷波器分子系数
    Uint16  k4;               //数字陷波器分子系数
    Uint16  Mask;
    
    void (*init)();
    void (*rst)();
    void (*calc)();	    /* */    
}NOTCHFILTER ;
typedef NOTCHFILTER *NOTCHFILTER_handle;

#define NOTCHFILTER_DEFAULTS { \
/*in                         */0, \
/*NotchInput                 */{0,0}, \
/*NotchOutput                */{0,0}, \
/*out                        */0,\
/*k1                         */0,\
/*k2                         */0,\
/*k3                         */0,\
/*k4                         */0,\
/*Mask                       */1,\
/*void (*init)()          */(void (*)(long)) NotchFilter_init ,\
/*void (*rst)()           */(void (*)(long)) NotchFilter_rst ,\
/*void (*calc)()          */(void (*)(long)) NotchFilter_cacl}

void NotchFilter_init(NOTCHFILTER_handle);
void NotchFilter_rst(NOTCHFILTER_handle);
void NotchFilter_cacl(NOTCHFILTER_handle);

extern NOTCHFILTER NotchFilter1;
#endif /* CUR_ADAPTIVEFILTER_H_ */
