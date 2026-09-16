#ifndef TORCLOSEDCTRLOOP_H_
#define TORCLOSEDCTRLOOP_H_
#include "IQmathLib.h" 

typedef struct 	{
    int16_t   TorValue;
    float     Coeff;
    float     Coeff2;
    float     OutMax;
    float     OutMin;
    float     Ref;
    float     Fdb;
    int16_t   Err;
    float     Err_pre;
    float     Kp;
    float     Kp2;
    float     Ki;
    float     Kd;
    float     Up;
    float     Up2;
    float     Ui;
    float     Ud;
    float     OutPreSat;  
    int16_t   Out;
    uint16_t  PiFlag;
    uint16_t  PdFlag;
    uint16_t  cnt1;
    uint16_t  Time1;
    float     TorKsf;//力矩环低通滤波系数
    float     TorRefPre;//力矩环低通滤波前一次输出
    float     ErrValLmt;//力矩环到位误差值
    uint16_t  ErrTimLmt;//力矩环到位误差时间
    uint16_t  TorDonFlag;//力矩环力矩到位标志
    uint16_t  TorRefRev;//力矩命令方向取反
    float     RampTarget;//力矩命令斜坡目标
    float     RampOutput;//力矩命令斜坡输出
    float     RampErr;//力矩命令斜坡偏差
    float     RampAccDelta;//力矩命令斜坡加速度
    float     RampDecDelta;//力矩命令斜坡减速度
    int32  Testcnt1;//测试计数变量1
    int32  Testcnt2;//测试计数变量2
}CURFULLCLOSE_PIREG;


extern CURFULLCLOSE_PIREG CurFullReg;
extern void CurFullPiReg_init(CURFULLCLOSE_PIREG *v);
extern void CurFullPiReg_rst(CURFULLCLOSE_PIREG *v);
extern void CurFullPiReg_updatat(CURFULLCLOSE_PIREG *v);
extern int16_t CurFullPiReg_Calc(CURFULLCLOSE_PIREG *v);
//extern int16_t CurFullPdReg_Calc(CURFULLCLOSE_PIREG *v);
extern void TorRampGenerator_calc(CURFULLCLOSE_PIREG *v);
extern int16_t TorCtrLoop_Calc(CURFULLCLOSE_PIREG *v);

#endif

