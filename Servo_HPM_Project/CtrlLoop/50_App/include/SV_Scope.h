/*
 * FreeCars.h
 *
 *  Created on: 2016年11月19日
 *      Author: Administrator
 */
//
#ifndef SVFREECARS_H_
#define SVFREECARS_H_
#include "userdefine.h"
#include "IQmathLib.h"

#define OSCSTARTINDEX    6

#define OSCIBUFFNUM           (4096u)  //采样4096个Dword
#define OSCIBUFFFAULTNUM      (4096u)  //采样4096个Dword

#define SERIESFIRSTINDEX 0       //Dword地址偏移  
#define SERIESCONEINDEX  8192    //Dword地址偏移  


#define SEROSCIBUFFNUM_HALF   2000    //缓冲数组的一半
#define SEROSCIMAXSENDNUM     1000    //单通道连续最大数据量，根据请求和发送时间决定

//示波器位结构体类型定义
typedef struct{
    Uint16   Mode:3;                //示波器模式
    Uint16   TringStatus:1;         //0:关闭 1：开启
    Uint16   SampleStutas:4;        //采样状态 0:未采样 1:触发前准备 2:触发前采样 3:触发后采样 4:单次采样完成 5:连续采样
    Uint16   TrigEdgeSel_A:3;       //触发沿选择A  0:上升沿 1:下降沿 2:沿变化 3:水平之上 4:水平之下
    Uint16   TrigEdgeSel_B:3;       //触发沿选择B  0:上升沿 1:下降沿 2:沿变化 3:水平之上 4:水平之下
    Uint16   TrigCtrl:2;            //触发控制     0:条件A 1:条件A或条件B 2:条件A与条件B
}STR_OSCILLOSCOPECTRL_BIT;

typedef union{
    volatile Uint16                      all;
    volatile STR_OSCILLOSCOPECTRL_BIT    bit;
}UNI_OSCILLOSCOPECTRL_REG;
//

//示波器变量结构体类型定义  
typedef struct{
    volatile UNI_OSCILLOSCOPECTRL_REG   OsciCtrl;    //示波器控制结构体

    Uint8   SectionSel;                     //连续采样时扇区选择

    Uint16  SampleTime;                     //采样间隔时间(单位:电流环调度周期)
    Uint16  SampleTimeMin;                  //连续采样时,采样间隔时间最小值(单位:电流环调度周期)
    Uint16  SampleTxCnt;                    //采样数据长度
    Uint16  SampleCnt;                      //采样数据计数器

    int32 * pTrigTarget_A;                   //触发对象A地址指针
    int32 * pTrigTarget_B;                   //触发对象B地址指针
    int32   TrigTargetLatch_A;               //触发锁存值A
    int32   TrigTargetLatch_B;               //触发锁存值B
    uint8_t   TrigTargetCoeff_A;              //触发对象系数A
    uint8_t   TrigTargetCoeff_B;              //触发对象系数B
    uint8_t   TypeTringA;
    uint8_t   TypeTringB;

    Uint16  TrigTargetBit_A;                //触发对象位控制A
    Uint16  TrigTargetBit_B;                //触发对象位控制B
    int32   TrigLevel_A;                    //触发水平A
    int32   TrigLevel_B;                    //触发水平B
    Uint16  DataLenAfterTrig;               //触发有效后的数据长度
    Uint16  StartIndex;
    
    Uint16  TringEndIndex;
    Uint16  SeriesStartIndex;
    Uint16  SeriesTxReq;
    Uint16  SeriesTxNum;
    Uint16  SeriesStatus;
    Uint16  SeriesTxEndFlag;

    //注意下面变量的顺序不能改变
    int32 * pCH1Addr;                        //通道1变量地址指针
    int32 * pCH2Addr;                        //通道2变量地址指针
    int32 * pCH3Addr;                        //通道3变量地址指针
    int32 * pCH4Addr;                        //通道4变量地址指针

    //注意下面变量的顺序不能改变
    Uint32   CH1Coeff;                   //通道1系数
    Uint32   CH2Coeff;                   //通道2系数
    Uint32   CH3Coeff;                   //通道3系数
    Uint32   CH4Coeff;                   //通道4系数
    
    uint8_t   Ch1Type;
    uint8_t   Ch2Type;
    uint8_t   Ch3Type;
    uint8_t   Ch4Type;

    uint8_t   ScopeTxFlag;
    uint8_t   FaultFlag;
}STR_OSCILLOSCOPE;

typedef union
{
    int8_t   all_8bit[4];
    int16_t  all_16bit[2];
    int32_t  all_32bit;  
}UNI_32TYPE;

/* 结构体变量类型定义 枚举变量类型定义 */ 
typedef union{
/*
    Uint8                   all_8Bits[24576];
    Uint16                  all_16Bits[12288];
    Uint32                  all_32Bits[6144];
*/
  
    int8                   all_8Bits[65536];
    int16                  all_16Bits[32768];
    int32                  all_32Bits[16384];  //全部按32位有符数据，上传
}UNI_OSCILLOSCOPEBUFFER;

extern UNI_OSCILLOSCOPEBUFFER   UNI_OsciBuffer;
extern STR_OSCILLOSCOPE         STR_Osci;

//void VofaSendData(void);
//void Scope_Init(STR_OSCILLOSCOPE * p);

void OscilloscopeSampling_us(void);
void OscilloscopeSampling_Fault(void);
void Scope_Process(void);
#endif /* FREECARS_H_ */
