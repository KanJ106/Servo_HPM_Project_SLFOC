/*
 * r_cg_math.h
 *
 *  Created on: 2018?3?14?
 *      Author: Administrator
 */

#ifndef _R_CG_MATH_H_
#define _R_CG_MATH_H_
#include <stdint.h>
#include <math.h>//

#define  PI_MPY_2  (6.283185307f)
#define  DIV_PI_MPY_2  ((double)(0.000000374507))
#define  _IQ24_P      (16777216L) 
#define  _IQ24_N      (-16777216L)
#define  _IQ24_DIV2   (8388608L) 
#define  _IQ28_P      (268435456L) 

#define  FLOAT_TYPE_USE   1

#define  MAXPIOUT     (0.5773503f)
#define  MAXPIOUT_IQ  (9686330L)

typedef int8_t          int8;
typedef uint8_t         Uint8;

typedef int16_t         int16;
typedef uint16_t        Uint16;

typedef int32_t         int32;
typedef uint32_t        Uint32;

typedef int64_t         int64;
typedef uint64_t        Uint64;
typedef float           float32;
typedef long double     float64;

typedef   long    _iq;
typedef   long    _iq30;
typedef   long    _iq29;
typedef   long    _iq28;
typedef   long    _iq27;
typedef   long    _iq26;
typedef   long    _iq25;
typedef   long    _iq24;
typedef   long    _iq23;
typedef   long    _iq22;
typedef   long    _iq21;
typedef   long    _iq20;
typedef   long    _iq19;
typedef   long    _iq18;
typedef   long    _iq17;
typedef   long    _iq16;
typedef   long    _iq15;
typedef   long    _iq14;
typedef   long    _iq13;
typedef   long    _iq12;
typedef   long    _iq11;
typedef   long    _iq10;
typedef   long    _iq9;
typedef   long    _iq8;
typedef   long    _iq7;
typedef   long    _iq6;
typedef   long    _iq5;
typedef   long    _iq4;
typedef   long    _iq3;
typedef   long    _iq2;
typedef   long    _iq1;


//80ns   HPM float 85ns    double 112ns
#define _IQ(A)          ((long)((A)*16777216.0f))                      // A ??float ???                  ?????_iq24 ???
//#define _IQ11(A)        ((long)((A)*32768))                         // A ??float ???                  ?????_iq11 ???
#define _IQ15(A)        ((long)((A)*32768.0f))                         // A ??float ???                  ?????_iq15 ???
#define _IQ22(A)        ((long)((A)*4194304.0f))                       // A ??float ???                  ?????_iq22 ???
#define _IQ30(A)        ((long)((A)*1073741824.0f))                    // A ??float ???                  ?????_iq30 ???

//175ns   HPM 180~215ns
#define _IQdiv2(A)      ((long)((A)>>1))                                // A ??iq24 ???                   ?????_iq24 ???
#define _IQ10div(A,B)   ((long)(((long long)(A)*1024)/(long)(B)))          // A B ?????????                 ?????_iq24 ???        
#define _IQ15div(A,B)   ((long)(((long long)(A)*32768)/(long)(B)))         // A B ?????????                 ?????_iq24 ???        
#define _IQ20div(A,B)   ((long)(((long long)(A)*1048576)/(long)(B)))       // A B ?????????                 ?????_iq24 ???
#define _IQ22div(A,B)   ((long)(((long long)(A)*4194304)/(long)(B)))       // A B ?????????                 ?????_iq24 ???
#define _IQdiv(A,B)     ((long)(((long long)(A)*16777216)/(long)(B)))
#define _IQ24div(A,B)   ((long)(((long long)(A)*16777216)/(long)(B)))      // A B ?????????                 ?????_iq24 ???
#define _IQ28div(A,B)   ((long)(((long long)(A)*268435456)/(long)(B)))     // A B ?????????                 ?????_iq24 ???
#define _IQ30div(A,B)   ((long)(((long long)(A)*1073741824)/(long)(B)))    // A B ?????????                 ?????_iq24 ???

//115ns  HPM  max52ns
#define _IQmpy2(A)      ((long)((A)<<1))                                // A ??iq24 ???                   ?????_iq24 ??? 
//#define _IQmpy(A,B)     ((long)((long long)(A)*(B)>>24))     // A B ?????? _iq24               ?????_iq24 ???
#define _IQ15mpy(A,B)   ((long)((long long)(A)*(B)>>15))     // A B ?????? _iq15               ?????_iq15 ???
#define _IQ16mpy(A,B)   ((long)((long long)(A)*(B)>>16))     // A B ?????? _iq16               ?????_iq16 ???
#define _IQ18mpy(A,B)   ((long)((long long)(A)*(B)>>18))     // A B ?????? _iq18               ?????_iq18 ???
#define _IQ22mpy(A,B)   ((long)((long long)(A)*(B)>>22))     // A B ?????? _iq22               ?????_iq22 ???
#define _IQ24mpy(A,B)   ((long)((long long)(A)*(B)>>24))     // A B ?????? _iq24               ?????_iq24 ???
#define _IQ28mpy(A,B)   ((long)((long long)(A)*(B)>>28))     // A B ?????? _iq28               ?????_iq28 ???
#define _IQ29mpy(A,B)   ((long)((long long)(A)*(B)>>29))     // A B ?????? _iq29               ?????_iq29 ???
#define _IQ30mpy(A,B)   ((long)((long long)(A)*(B)>>30))     // A B ?????? _iq30               ?????_iq30 ???

//60ns
#define _IQ11toIQ(A)    ((long)((A)<<13))                             // A ??iq11 ???                   ?????_iq24 ??? 
#define _IQ12toIQ(A)    ((long)((A)<<12))                             // A ??iq12 ???                   ?????_iq24 ??? 
#define _IQ15toIQ(A)    ((long)((A)<<9))                              // A ??iq15 ???                   ?????_iq24 ??? 
#define _IQ16toIQ(A)    ((long)((A)<<8))                              // A ??iq16 ???                   ?????_iq24 ??? 

//60ns
#define _IQtoIQ11(A)    ((long)((A)>>13))                             // A ??iq24 ???                   ?????_iq11 ??? 
#define _IQtoIQ12(A)    ((long)((A)>>12))                             // A ??iq24 ???                   ?????_iq12 ??? 
#define _IQtoIQ15(A)    ((long)((A)>>9))                              // A ??iq24 ???                   ?????_iq15 ??? 
#define _IQtoIQ16(A)    ((long)((A)>>8))                              // A ??iq24 ???                   ?????_iq16 ??? 
#define _IQtoIQ18(A)    ((long)((A)>>6))                              // A ??iq24 ???                   ?????_iq18 ??? 
#define _IQtoIQ20(A)    ((long)((A)>>4))                              // A ??iq24 ???                   ?????_iq20 ??? 
#define _IQtoIQ30(A)    ((long)((A)<<6))                              // A ??iq24 ???                   ?????_iq30 ??? 

//115ns  HPM 150ns
#define _IQtoF(A)       (((float)(A)/(float)16777216))                       // A ??iq24 ???                   ?????float 
 
//110ns  HPM 50ns
#define _IQmpyI32int(A,B)    (((long long)(A)*(long long)(B))>>24)    // A ??iq24 ??? B ????????      ?????????????????????????
#define _IQ22mpyI32int(A,B)  (((long long)(A)*(long long)(B))>>22)    // A ??iq22 ??? B ????????      ?????????????????????????

//65ns
#define _IQmpyI32(A,B)       ((long long)(A)*(long long)(B))           // A ??iq24 ??? B ????????      ?????_iq24
#define _IQ22mpyI32(A,B)     ((long long)(A)*(long long)(B))           // A ??iq24 ??? B ????????      ?????_iq22

#define _IQsign(A)      (((A) == (0)) ? (0) : (((A) > (0)) ? (1) : (-1)))//-1 0 1

//100ns
#define _IQsat(A,B,C)   (((A) > (B)) ? (B) :(((A) < (C)) ? (C) : (A))) //A,B,C ?????????

//73ns
#define _IQabs(A)       (((A) >= 0) ? (A) : (-(A)))                    //

//150ns   HPM 800ns
#define _IQsqrt(A)      (((long)sqrtf((float)(A)))<<12)                  // A ??iq24        ?????_iq24
//170ns
#define _IQ15sqrt(A)    (((long)sqrtf((float)(2*(A))))<<7)               // A ??iq24        ?????_iq24

//95ns   HPM henkuai
#define _IQfrac(A)         ((long)((A)%16777216L))                                          // A ??iq24 ???                    ?????_iq24
#define _IQ30frac(A)       ((long)((A)%1073741824L))   
                                     // A ??iq30 ???                    ?????_iq30
//188ns  HPM 110ns
#define _IQmpyI32frac(A,B) ((long)(((long long)(A)*(long long)(B))%16777216L))              // A ??iq24 ??? B ????????      ?????_iq24

//1.34us HPM 1.3us
#define _IQsinPU(A)     (long)(sinf((float)(A)*PI_MPY_2/_IQ24_P)*_IQ24_P)          
#define _IQ30sinPU(A)   (long)(sinf((float)(A)*PI_MPY_2/1073741824L)*1073741824L)        

//1.34us HPM 1.65us
#define _IQcosPU(A)     (long)(cosf((float)(A)*PI_MPY_2/_IQ24_P)*_IQ24_P)            
#define _IQ30cosPU(A)   (long)(cosf((float)(A)*PI_MPY_2/1073741824L)*1073741824L)       

//1.75us HPM 2us
//自学习使用
#define _IQ20atan2PU(A,B) (long)(atan2f((float)(A)*PI_MPY_2/1048576L,(float)(B)*PI_MPY_2/1048576L)*166886L)  
//速度环响应分析使用
#define _IQatan2PU(A,B)   (long)(atan2f((float)(A)*PI_MPY_2/_IQ24_P,(float)(B)*PI_MPY_2/_IQ24_P)*2670177L) 

//定点Q24乘法，-16777216 < X < 0,右移24等于0
static inline long _IQmpy_HP(long A, long B)
{
    unsigned long temp[2];
    
    *(unsigned long long*)temp = (unsigned long long )A * (unsigned long)B;
    
    if(temp[1] == 0xFFFFFFFF && temp[0] > 0xFF000000)
    {
        return 0;
    }
    else
    {
        return ((long)((*(unsigned long long*)temp) >> 24));
    }
}

//  HPM 55ns
static inline long _IQmpy(long A, long B)
{
    long long temp;
    
    temp = (long long)A * B;
    
    if(temp < 0)
    {
        return (-((-temp) >> 24));
    }
    else
    {
        return (temp >> 24);
    }   
}

//    小数 * 小数  float  55ns   double 85ns

#endif /* _R_CG_MATH */
