#ifndef POS_FULLCLOSEDLOOP_H_
#define POS_FULLCLOSEDLOOP_H_

#include "IQmathLib.h" 

typedef struct 
{
    Uint32  ExtEncSglRevLns;
    Uint32  EncSinglePosOld;			//编码器位置旧值
    int32  PosFbTotle;					//编码器转动的总位置
    int32  PosFbDelta;					//此次调度周期内的位置变化值
    int32  PosFbDeltaMax;				//位置变化值正限幅
    int32  PosFbDeltaMin;				//位置变化值负限幅
    Uint16 Cnt;
    Uint16 FirstFlg;
    Uint16 ExtRotPolar;
    Uint16 cnt;
    int32  FbDelta[66];
    int32  Sum;
    float  SpdCoffe;
    int32  SpdFbOut;
    float  viewCoffe;
    int16  SpdFbView;
    Uint16  SpdMode;
    Uint16 cntnum;
} EXTPOSFB;

typedef struct{  
    int32 Ref;
    int32 Fb;
    
    int32 Err;//编码器偏差
    
    int32 InnerRev;
    int32 ExtRev;
    int32 RefRemainder;
    int32 FbRemainder;
    
    int32 LowPassInput;
    int32 LowPassOutput;
    int32 LowPassOutputLatch;
    int32 LowPassRemainder;
    
    int32 LowPassKa;
    int32 MaxSpd;
    
    Uint16 FullFlag;
    Uint16 rsd;
}FULL_POSCONTROL;

void ExPosFb_Init(void);
void ExPosFb_rst(void);
void ExtPosFb_Cacl(void);
int32 PosFullCloseRef_Cacl(int32 Ref);
int32 PosFullCloseErr_Cacl(int32 ExPosRef,int32 ExPosFb,int32 *Err);
void PosFullClose_Init(void);
void PosFullClose_Rst(void);

extern EXTPOSFB        ExtPosFb;
extern FULL_POSCONTROL FullCloseLoop;
#endif
