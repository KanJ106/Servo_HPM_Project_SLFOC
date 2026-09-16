/*
 * SV_Scope.c
 *
 *  Created on: 2016年11月19日
 *      Author: Administrator
 */

#include "SV_Scope.h"//
#include "userdefine.h"
#include "r_cg_Project.h"
#include "Drive.h"
#include "SV_UdcCtrl.h"
#include "main.h"
#include "SV_MagEncode.h"
#include "Pos_FullClosedLoop.h"
#include "SV_OTProtect.h"
#include "SV_CanbusCtrl.h"//
#include "SV_ModbusConTrolVar.h"
#include "SV_RtT036.h"
#include "SV_Sci.h"

#define CHNUMBER  80

extern int16_t  EctDcOffsetime;

uint32_t testch0;
uint16_t testch1;
uint16_t testch2;

extern uint16_t PwmTestU;
extern uint16_t PwmTestV;
extern uint16_t PwmTestW;
extern uint16_t testpwmcnt;
extern int32_t  TorQueData;
extern uint16_t SmPeriodtime;
extern uint16_t SmDcOffsettime;

UNI_OSCILLOSCOPEBUFFER   UNI_OsciBuffer;
uint8_t FirstSendData[6] = {0x01,0x06,0x76,0x0C,0x7D,0x00};
uint8_t HarmoCore_FirstSendData[10] = {0x3C, 0x3C, 0x01, 0x64, 0x76, 0x0C, 0x00, 0x00, 0x3E, 0x3E}; 
uint8_t HarmoCore_SendEndFrame[2] = {0x3E, 0x3E};
int32  NullScope = 0;

int32*  pOscTargetAddr[CHNUMBER] =
{
    (int32*)&PosRef.PosRefTotle,             // 0       目标位置
    (int32*)&PosFb.ScopeFbTotle,             // 1       反馈位置
    (int32*)&PosMonitor.PosCmd,              // 2       位置速度指令(rpm)-滤波前
    (int32*)&PosMonitor.PosCmdFilter,        // 3       位置速度指令(rpm)-滤波后
    (int32*)&PosRef.Delta,                   // 4       位置指令(Pulse)-滤波前
    (int32*)&PosReg.Ref,                     // 5       位置指令(Pulse)-滤波后
    (int32*)&PosReg.Err,                     // 6       位置偏差
    (int32*)&PosReg.TotalErr,                // 7       位置混合偏差
    (int32*)&PosReg.Pout,                    // 8       位置环P输出
    (int32*)&PosReg.FwdOut,                  // 9       位置内部前馈输出
    (int32*)&PosRef.VelOffset,               // 10      位置ECAT前馈输出
    (int32*)&PosRef.PosOld,                  // 11      位置指令锁存
    (int32*)&PosReg.Out,                     // 12      位置环输出
    (int32*)&ExtPosFb.PosFbTotle,            // 13      输出端位置反馈
    (int32*)&PosFb.PosFbTotle,               // 14      电机端位置反馈
    (int32*)&PosRef.PosNow,                  // 15
      
    (int32*)&Spd_Ref.SpdRefDecm,             // 16      速度指令
    (int32*)&SpdFb.SpdFbDecm,                // 17      速度反馈滤波后
    (int32*)&SpdFb.FbPre,                    // 18      速度反馈滤波前
    (int32*)&AccFwd.Out,                     // 19      速度环前馈输出
    (int32*)&SpdMonitor.SpdPiOut,            // 20      速度环调节器PI输出
    (int32*)&SpdMonitor.SpdToTorCmd,         // 21      速度环总输出
    (int32*)&SpdMonitor.TorqueRef,           // 22      电流环指令滤波前
    (int32*)&NotchFilter1.out,               // 23      
    (int32*)&InstSpdObser.wout,              // 24
    (int32*)&ExtPosFb.SpdFbView,             // 25      输出端转速
    (int32*)&NullScope,                      // 26
    (int32*)&NullScope,                      // 27
    (int32*)&NullScope,                      // 28
    (int32*)&NullScope,                      // 29
    (int32*)&NullScope,                      // 30
    (int32*)&NullScope,                      // 31
      
    (int32*)&CurMonitor.TorqRatsRef,         // 32
    (int32*)&CurMonitor.TorqRatsDispS,       // 33   
    (int32*)&CurMonitor.TorqFbId,            // 34
    (int32*)&Ipark.Qs,                       // 35
    (int32*)&TorQueData,                     // 36
    (int32*)&NullScope,                      // 37
    (int32*)&NullScope,                      // 38
    (int32*)&NullScope,                      // 39
    
    #if SERVOTYPE == SERVO_CAN
    (int32*)&CM_BusCtrl.ScopePos,            // 40
    (int32*)&CM_BusCtrl.ScopeVel,            // 41
    (int32*)&CM_BusCtrl.ScopeTor,            // 42
    (int32*)&NullScope,                      // 43
    (int32*)&NullScope,                      // 44  
    (int32*)&CM_BusVar.CtrlWord,             // 45
    (int32*)&CM_BusVar.Status.Flag,          // 46
    (int32*)&CM_BusVar.ActualValue.Pos,      // 47
    (int32*)&CM_BusVar.ActualValue.Vel,      // 48
    (int32*)&CM_BusVar.ActualValue.Torque,   // 49
    (int32*)&SyncOffsetime,                  // 50 
    (int32*)&NullScope,                      // 51 
    (int32*)&NullScope,                      // 52 
    (int32*)&NullScope,                      // 53
    (int32*)&SmPeriodtime,                   // 54 
    (int32*)&SmDcOffsettime,                 // 55 
    #endif
    
    #if SERVOTYPE == SERVO_MODBUS
    (int32*)&CM_BusVar.TargetPos1,           // 40
    (int32*)&CM_BusVar.TargetSpd_S,          // 41
    (int32*)&CM_BusVar.TargetTor_M,          // 42
    (int32*)&NullScope,                      // 43
    (int32*)&NullScope,                      // 44  
    (int32*)&NullScope,                      // 45
    (int32*)&CM_BusVar.Status,               // 46
    (int32*)&CM_BusVar.PosFbAct,             // 47
    (int32*)&CM_BusVar.VelFbAct,             // 48
    (int32*)&CM_BusVar.TorFbAct,             // 49
    (int32*)&NullScope,                      // 50 
    (int32*)&CM_BusVar.TargetSpd_p,          // 51 
    (int32*)&CM_BusVar.TargetPos2,           // 52 
    (int32*)&NullScope,                      // 53
    (int32*)&SmPeriodtime,                      // 54 
    (int32*)&SmDcOffsettime,                      // 55 
    #endif
 
    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
    (int32*)&LocalAxes.Objects.objTargetPosition,                      // 40
    (int32*)&LocalAxes.Objects.objTargetVelocity,                      // 41
    (int32*)&LocalAxes.Objects.objTargetTorque,                        // 42
    (int32*)&LocalAxes.Objects.objVelocityOffset,                      // 43
    (int32*)&LocalAxes.Objects.objTorqueOffset,                        // 44
    (int32*)&LocalAxes.Objects.objControlWord,                         // 45
    (int32*)&LocalAxes.Objects.objStatusWord,                          // 46
    (int32*)&LocalAxes.Objects.objPositionActualValue,                 // 47 
    (int32*)&LocalAxes.Objects.objVelocityActualValue,                 // 48
    (int32*)&LocalAxes.Objects.objTorqueActualValue,                   // 49
    (int32*)&EctDcOffsetime,                                           // 50
    (int32*)&NullScope,                                                // 51
    (int32*)&NullScope,                                                // 52 
    (int32*)&NullScope,                                                // 53 
    (int32*)&SmPeriodtime,                                             // 54 
    (int32*)&SmDcOffsettime,                                           // 55
    #endif      
    
    (int32*)&UdcCtrl.Udc,                      // 56 
    (int32*)&NullScope,                        // 57 
    (int32*)&NullScope,                        // 58 
    (int32*)&NullScope,                        // 59 
    (int32*)&NullScope,                        // 60 
    (int32*)&NullScope,                        // 61 
    (int32*)&NullScope,                        // 62 
    (int32*)&NullScope,                        // 63
      
    (int32*)&NullScope,                        // 64 
    (int32*)&NullScope,                        // 65 
    (int32*)&NullScope,                        // 66 
    (int32*)&NullScope,                        // 67 
    (int32*)&NullScope,                        // 68 
    (int32*)&NullScope,                        // 69 
    (int32*)&NullScope,                        // 70 
    (int32*)&NullScope,                        // 71
      
    (int32*)&RTData.CalMotorAglErr3,           // 72 
    (int32*)&RTData.CalReducerAglErr3,         // 73 
    (int32*)&Etheta.EtheataDecm,               // 74 
    (int32*)&IqRef.TorqueFlag,                 // 75 
    (int32*)&AdRead.iw,                        // 76 
    (int32*)&AdRead.iv,                        // 77 
    (int32*)&AdRead.iu,                        // 78 
    (int32*)&FaultP.FaultDisp                  // 79                                                                                           
};

uint8_t OscTargetCoeff[CHNUMBER] =
{
     0,                                      // 0    
     0,                                      // 1  
     0,                                      // 2
     0,                                      // 3
     0,                                      // 4
     0,                                      // 5     
     0,                                      // 6 
     0,                                      // 7 
     11,                                     // 8 
     11,                                     // 9 
     11,                                     // 10 
     0,                                      // 11 
     11,                                     // 12 
     0,                                      // 13 
     0,                                      // 14 
     0,                                      // 15 

     0,                                      // 16
     0,                                      // 17
     11,                                     // 18
     0,                                      // 19
     0,                                      // 20
     0,                                      // 21
     0,                                      // 22
     0,                                      // 23     
     11,                                     // 24
     0,                                      // 25
     0,                                      // 26
     0,                                      // 27
     0,                                      // 28
     0,                                      // 29
     0,                                      // 30
     0,                                      // 31
     
     0,                                      // 32
     0,                                      // 33
     0,                                      // 34
     11,                                     // 35
     0,                                      // 36
     0,                                      // 37
     0,                                      // 38
     0,                                      // 39 
     
     #if SERVOTYPE == SERVO_CAN
     0,                                      // 40   
     0,                                      // 41
     0,                                      // 42
     0,                                      // 43
     0,                                      // 44
     0,                                      // 45
     0,                                      // 46
     0,                                      // 47
     
     0,                                      // 48    
     0,                                      // 49
     0,                                      // 50
     0,                                      // 51
     0,                                      // 52
     0,                                      // 53
     0,                                      // 54
     0,                                      // 55    
     #endif
     
     #if SERVOTYPE == SERVO_MODBUS
     0,                                      // 40   
     0,                                      // 41
     0,                                      // 42
     0,                                      // 43
     0,                                      // 44
     0,                                      // 45
     0,                                      // 46
     0,                                      // 47
     
     0,                                      // 48    
     0,                                      // 49
     0,                                      // 50
     0,                                      // 51
     0,                                      // 52
     0,                                      // 53
     0,                                      // 54
     0,                                      // 55     
     #endif
     
     #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
     0,                                      // 40   
     0,                                      // 41
     0,                                      // 42
     0,                                      // 43
     0,                                      // 44
     0,                                      // 45
     0,                                      // 46
     0,                                      // 47
     
     0,                                      // 48    
     0,                                      // 49
     0,                                      // 50
     0,                                      // 51
     0,                                      // 52
     0,                                      // 53
     0,                                      // 54
     0,                                      // 55     
     #endif
     
     0,                                      // 56       
     0,                                      // 57   
     0,                                      // 58
     0,                                      // 59
     0,                                      // 60
     0,                                      // 61
     0,                                      // 62
     0,                                      // 63
     
     0,                                      // 64    
     0,                                      // 65  
     0,                                      // 66
     0,                                      // 67
     0,                                      // 68
     0,                                      // 69
     0,                                      // 70
     0,                                      // 71
     
     0,                                      // 72
     0,                                      // 73  
     0,                                      // 74   
     0,                                      // 75
     0,                                      // 76
     0,                                      // 77
     0,                                      // 78
     0,                                      // 79
};

uint8_t ChType[CHNUMBER] =
{
     0,                                      // 0    
     0,                                      // 1  
     1,                                      // 2
     1,                                      // 3
     0,                                      // 4
     0,                                      // 5     
     0,                                      // 6 
     0,                                      // 7 
     0,                                      // 8 
     0,                                      // 9 
     0,                                      // 10 
     0,                                      // 11 
     0,                                      // 12 
     0,                                      // 13 
     0,                                      // 14 
     0,                                      // 15 

     1,                                      // 16
     1,                                      // 17
     0,                                      // 18
     0,                                      // 19
     1,                                      // 20
     1,                                      // 21
     1,                                      // 22
     0,                                      // 23 
     
     0,                                      // 24
     1,                                      // 25
     0,                                      // 26
     0,                                      // 27
     0,                                      // 28
     0,                                      // 29
     0,                                      // 30
     0,                                      // 31
     
     1,                                      // 32
     1,                                      // 33
     1,                                      // 34
     0,                                      // 35
     0,                                      // 36
     0,                                      // 37
     0,                                      // 38
     0,                                      // 39  
     
     #if SERVOTYPE == SERVO_CAN
     0,                                      // 40   
     1,                                      // 41
     1,                                      // 42
     0,                                      // 43
     0,                                      // 44
     1,                                      // 45
     1,                                      // 46
     0,                                      // 47
     
     1,                                      // 48    
     1,                                      // 49
     1,                                      // 50
     1,                                      // 51
     0,                                      // 52
     0,                                      // 53
     1,                                      // 54
     1,                                      // 55    
     #endif
     
     #if SERVOTYPE == SERVO_MODBUS
     0,                                      // 40   
     0,                                      // 41
     1,                                      // 42
     0,                                      // 43
     0,                                      // 44
     0,                                      // 45
     1,                                      // 46
     0,                                      // 47  
     
     0,                                      // 48 
     1,                                      // 49
     0,                                      // 50
     0,                                      // 51
     0,                                      // 52
     0,                                      // 53
     1,                                      // 54
     1,                                      // 55   
     #endif
     
     #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
     0,                                      // 40   
     0,                                      // 41
     1,                                      // 42
     0,                                      // 43
     1,                                      // 44
     1,                                      // 45
     1,                                      // 46
     0,                                      // 47  
     
     0,                                      // 48 
     1,                                      // 49
     1,                                      // 50
     0,                                      // 51
     0,                                      // 52
     0,                                      // 53
     1,                                      // 54
     1,                                      // 55
     #endif
     
     1,                                      // 56
     0,                                      // 57
     0,                                      // 58
     0,                                      // 59
     0,                                      // 60
     0,                                      // 61
     0,                                      // 62
     0,                                      // 63
     
     0,                                      // 64   
     0,                                      // 65
     0,                                      // 66
     0,                                      // 67
     0,                                      // 68
     0,                                      // 69
     0,                                      // 70
     0,                                      // 71
     
     1,                                      // 72
     1,                                      // 73
     1,                                      // 74
     1,                                      // 75
     1,                                      // 76
     1,                                      // 77
     1,                                      // 78
     1,                                      // 79
};

//示波器内部结构体
STR_OSCILLOSCOPE    STR_Osci;

void ScopeCh_Init(STR_OSCILLOSCOPE * p)
{
    int16 temp;
    
    if(ScopePara->Enable == 0)
    {
        STR_Osci.OsciCtrl.bit.SampleStutas = 0;
        ScopePara->SeriesEnable = 0;
        
        if(ScopePara->ChSel1 >= CHNUMBER) ScopePara->ChSel1 = CHNUMBER -1;
        p->pCH1Addr = pOscTargetAddr[ScopePara->ChSel1];
        p->CH1Coeff = OscTargetCoeff[ScopePara->ChSel1];
        p->Ch1Type  = ChType[ScopePara->ChSel1];

        if(ScopePara->ChSel2 >= CHNUMBER) ScopePara->ChSel2 = CHNUMBER - 1;
        p->pCH2Addr = pOscTargetAddr[ScopePara->ChSel2];
        p->CH2Coeff = OscTargetCoeff[ScopePara->ChSel2];
        p->Ch2Type  = ChType[ScopePara->ChSel2];

        if(ScopePara->ChSel3 >= CHNUMBER) ScopePara->ChSel3 = CHNUMBER - 1;
        p->pCH3Addr  = pOscTargetAddr[ScopePara->ChSel3];
        p->CH3Coeff = OscTargetCoeff[ScopePara->ChSel3];
        p->Ch3Type  = ChType[ScopePara->ChSel3];

        if(ScopePara->ChSel4 >= CHNUMBER) ScopePara->ChSel4 = CHNUMBER - 1;
        p->pCH4Addr = pOscTargetAddr[ScopePara->ChSel4];
        p->CH4Coeff = OscTargetCoeff[ScopePara->ChSel4];
        p->Ch4Type  = ChType[ScopePara->ChSel4];
        
        if(ScopePara->ChSelTrigA >= CHNUMBER) ScopePara->ChSelTrigA = CHNUMBER - 1;
        p->pTrigTarget_A     =  pOscTargetAddr[ScopePara->ChSelTrigA];
        p->TrigTargetCoeff_A = OscTargetCoeff[ScopePara->ChSelTrigA];
        p->TypeTringA        = ChType[ScopePara->ChSelTrigA];
        
        p->TrigLevel_A = ScopePara->TrigLevel;
        
        if(ScopePara->ChSelTrigA == (CHNUMBER - 1))
        {
            p->TrigTargetBit_A = 0xFFFE;
            p->TrigTargetBit_B = 0xFFFE;          
        }
        else
        {
            p->TrigTargetBit_A = 0xFFFF;
            p->TrigTargetBit_B = 0xFFFF;
        }
        
        p->OsciCtrl.bit.Mode = ScopePara->AcqMode;
        
        p->SampleTime = (ScopePara->SampleRate == 0) ? 1 : ScopePara->SampleRate;            

        p->OsciCtrl.bit.TrigEdgeSel_A = ScopePara->ChSelTrigEdge;
        
        //触发对象系数增益转换
        if(p->TypeTringA == 0x01)  //16bit
        {
            temp = (int16)((*p->pTrigTarget_A) & 0x0000FFFF);

            p->TrigTargetLatch_A = (int32_t)temp;
        }
        else 
        {
            p->TrigTargetLatch_A = ((*p->pTrigTarget_A) >> p->TrigTargetCoeff_A);
        }

        if(ScopePara->BufSize > 50) ScopePara->BufSize = 50;
        p->DataLenAfterTrig = ScopePara->BufSize * OSCIBUFFNUM / 100;
        
        p->SampleCnt = 0;
        p->SeriesStartIndex = 0;
        p->SeriesTxNum = 0;
        
        p->OsciCtrl.bit.TringStatus = 0;

        p->ScopeTxFlag = 0;
        p->SeriesTxEndFlag = 0;

        if(ScopePara->FaultTriEnable == 1)
        {
            p->DataLenAfterTrig = OSCIBUFFFAULTNUM >> 1;
            ScopePara->AcqMode = 3;
            p->OsciCtrl.bit.Mode = 3;
            p->SampleTime = (ScopePara->SampleRate == 0) ? 1 : ScopePara->SampleRate;
            p->TrigTargetBit_A = 0xFFFE;
            p->OsciCtrl.bit.TringStatus = 1;
            
            STR_Osci.OsciCtrl.bit.SampleStutas = 1; 
            ScopePara->Enable = 1;
            STR_Osci.FaultFlag = 1;
        }
    }
    else if((ScopePara->Enable == 1) && (ScopePara->AcqMode == 0))
    {
        p->OsciCtrl.bit.Mode = ScopePara->AcqMode;
    }
    else if(STR_Osci.FaultFlag == 0x01)
    {
        if(ScopePara->FaultTriEnable == 0x00)
        {
            ScopePara->Enable = 0;
            STR_Osci.FaultFlag = 0;
        }
    }
    else
    {
        if((p->OsciCtrl.bit.Mode == 1) && (p->OsciCtrl.bit.TringStatus == 0) && (ScopePara->SeriesEnable == 1))     //电流环触发捕捉
        {
            STR_Osci.OsciCtrl.bit.SampleStutas = 1;  
            p->OsciCtrl.bit.TringStatus = 1;
            //ScopePara->SeriesEnable = 0;
        }
    }
}

/*******************************************************************************
  函数名: 
  输入:   无 
  输出:   无 
  子函数: 无
    1.
    2.
********************************************************************************/

static Uint8 TrigDeal(Uint8 EdgeSel, int32 Level, int32 TargetLatch, int32 Target)
{
    switch(EdgeSel)
    {
        case 0:                 //上升沿
            if((Target > Level) && (TargetLatch <= Level)) return 1;
            else return 0;

        case 1:                 //下升沿
            if((Target < Level) && (TargetLatch >= Level)) return 1;
            else return 0;

        case 2:                 //沿变化
            if( ((Target > Level) && (TargetLatch <= Level)) ||
                ((Target < Level) && (TargetLatch >= Level)) ) return 1;
            else return 0;

        case 3:                 //水平之上
            if(Target > Level) return 1;
            else return 0;

        case 4:                 //水平之下
            if(Target < Level) return 1;
            else return 0;

       default:                 //无触发
            return 0;
    }
}

/*******************************************************************************
  函数名: 
  输入:   无 
  输出:   无 
  子函数: 无
    1.
    2.
********************************************************************************/
/*
static uint8_t BitTrigDeal(Uint8 EdgeSel,Uint16 TargetLatch, Uint16 Target)
{
    switch(EdgeSel)
    {
        case 0:                 //上升沿
            if(Target > TargetLatch) return 1;
            else return 0;

        case 1:                 //下升沿
            if(Target < TargetLatch) return 1;
            else return 0;

        case 2:                 //沿变化
            if(Target != TargetLatch) return 1;
            else return 0;

        case 3:                 //水平之上
            if(Target > (int32)0) return 1;
            else return 0;

        case 4:                 //水平之下
            if(Target < (int32)1) return 1;
            else return 0;

       default:                 //无触发
            return 1;
    }
}
*/

uint8_t TrigDeal_A(STR_OSCILLOSCOPE * p)
{
    int32 Target = 0;
    int32 TargetLatch = 0;
    uint8_t flag = 0;
    int16   temp;

    //无触发返回1
    //if(p->pTrigTarget_A == &NullScope) return 1;

    if(p->TrigTargetBit_A == 0xFFFF)
    {
        //触发对象系数增益转换
        if(p->TypeTringA == 0x01)  //16bit
        {
            temp = (int16)((*p->pTrigTarget_A) & 0x0000FFFF);
            Target =  (int32)temp;
            TargetLatch = (int32)p->TrigTargetLatch_A;
        }
        else 
        {
            Target = (*p->pTrigTarget_A) >> p->TrigTargetCoeff_A;
            TargetLatch = (int32)p->TrigTargetLatch_A;
        }
        
        flag = TrigDeal(p->OsciCtrl.bit.TrigEdgeSel_A, (int32)p->TrigLevel_A, (int32)TargetLatch, (int32)Target);

        p->TrigTargetLatch_A = Target; 
        
        return( flag );
    }
    else //故障触发
    {
        if(FaultP.FaultStatus > 0)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}


static void SingleModeSampling(STR_OSCILLOSCOPE * p)
{
    uint16_t    ScpoeNum;
    uint8_t     startflag;
    UNI_32TYPE  DataBuff;
    uint8_t *pp;
    
    ScpoeNum = (p->SampleCnt << 2);                             //Dword地址索引
    pp = (uint8_t *)&UNI_OsciBuffer.all_8Bits[ScpoeNum << 2];   //Byte地址索引
    
    if(p->Ch1Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH1Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH1Addr) >> p->CH1Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    
    if(p->Ch2Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH2Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH2Addr) >> p->CH2Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }   
    
    if(p->Ch3Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH3Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH3Addr) >> p->CH3Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    
    if(p->Ch4Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH4Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH4Addr) >> p->CH4Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
    }
  
    p->SampleCnt++;
    if(p->SampleCnt == OSCIBUFFNUM)
    {
        p->SampleCnt = 0;
    }
     
    if(p->OsciCtrl.bit.SampleStutas == 1)       //预采样 
    {
        if(p->SampleCnt > p->DataLenAfterTrig) 
        {
            p->OsciCtrl.bit.SampleStutas = 2;   
        }
    }
    else if(p->OsciCtrl.bit.SampleStutas == 2)  //等待触发采样 
    {
        startflag = TrigDeal_A(p);
        if(startflag == 0x1)
        {
            p->OsciCtrl.bit.SampleStutas = 3; 
            
            if(p->SampleCnt < p->DataLenAfterTrig) 
            {
               p->StartIndex = OSCIBUFFNUM - (p->DataLenAfterTrig - p->SampleCnt);
            }
            else
            {
                p->StartIndex = p->SampleCnt - p->DataLenAfterTrig;
            }
        }
    }
    else if(p->OsciCtrl.bit.SampleStutas == 3) //采样中
    {
        if(p->SampleCnt == p->StartIndex) 
        {
            p->OsciCtrl.bit.SampleStutas = 4; //采样结束
            p->TringEndIndex = p->StartIndex;
            p->SampleTxCnt = 0;
            p->SampleCnt = 0; 
        }
    }
    else
    {
      
    }
}

void OscilloscopeSampling_us(void)
{
    static Uint16   Cnt = 0;
    
    //如果没有使能采样或者采样完成,计数器清零并退出
    if((STR_Osci.OsciCtrl.bit.Mode != 1) || ((STR_Osci.OsciCtrl.bit.SampleStutas == 0) || (STR_Osci.OsciCtrl.bit.SampleStutas > 3)))
    {
        Cnt = 0;
        return;
    }

    //采样间隔时间判断,如果STR_Osci.SampleTime = 0,表示每次调度电流环时采样
    Cnt ++;
    if(Cnt >= STR_Osci.SampleTime) 
    {
        Cnt = 0;
    }
    else
    {
        return;
    }  

    SingleModeSampling(&STR_Osci);
}

static void FaultModeSampling(STR_OSCILLOSCOPE * p)
{
    uint16_t    ScpoeNum;
    uint8_t     startflag;
    UNI_32TYPE  DataBuff;
    uint8_t     *pp;
    
    ScpoeNum = (p->SampleCnt << 2);                             //Dword地址索引
    pp = (uint8_t *)&UNI_OsciBuffer.all_8Bits[ScpoeNum << 2];   //Byte地址索引
    if(p->Ch1Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH1Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH1Addr) >> p->CH1Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    
    if(p->Ch2Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH2Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH2Addr) >> p->CH2Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }   
    
    if(p->Ch3Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH3Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH3Addr) >> p->CH3Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    
    if(p->Ch4Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH4Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH4Addr) >> p->CH4Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
    }

    p->SampleCnt++;
    if(p->SampleCnt == OSCIBUFFFAULTNUM)
    {
        p->SampleCnt = 0;
    }
     
    if(p->OsciCtrl.bit.SampleStutas == 1)       //预采样 
    {
        if(p->SampleCnt > p->DataLenAfterTrig) 
        {
            p->OsciCtrl.bit.SampleStutas = 2;   
        }
    }
    else if(p->OsciCtrl.bit.SampleStutas == 2)  //等待触发采样 
    {
        startflag = TrigDeal_A(p);
        if((startflag == 0x1) && ((p->SampleCnt % 4) == 0))
        {
            p->OsciCtrl.bit.SampleStutas = 3; 
            
            if(p->SampleCnt < p->DataLenAfterTrig) 
            {
                p->StartIndex = OSCIBUFFFAULTNUM - (p->DataLenAfterTrig - p->SampleCnt);//0~4096
            }
            else
            {
                p->StartIndex = p->SampleCnt - p->DataLenAfterTrig;
            }
        }
    }
    else if(p->OsciCtrl.bit.SampleStutas == 3) //采样中
    {
        if(p->SampleCnt == p->StartIndex) 
        {
            p->OsciCtrl.bit.SampleStutas = 4; //采样结束
            p->TringEndIndex = p->StartIndex;
            p->SampleTxCnt = 0;
            p->SampleCnt = 0; 
            ScopePara->FaultTriFlag = 1;
        }
    }
    else
    {
      
    }
}

void OscilloscopeSampling_Fault(void)
{
    static Uint16   Cnt = 0;
    
    //如果没有使能采样或者采样完成,计数器清零并退出
    if((STR_Osci.OsciCtrl.bit.Mode != 3) || ((STR_Osci.OsciCtrl.bit.SampleStutas == 0) || (STR_Osci.OsciCtrl.bit.SampleStutas > 3)))
    {
        Cnt = 0;
        return;
    }

    //采样间隔时间判断,如果STR_Osci.SampleTime = 0,表示每次调度电流环时采样
    Cnt ++;
    if(Cnt >= STR_Osci.SampleTime) 
    {
        Cnt = 0;
    }
    else
    {
        return;
    }  

    FaultModeSampling(&STR_Osci);
}

static void SeriesModeSampling(STR_OSCILLOSCOPE * p)
{
    uint16_t ScpoeNum;
    UNI_32TYPE  DataBuff;
    uint8_t *pp;
    uint16_t BuffNum;
   
    ScpoeNum = p->SeriesStartIndex + (p->SampleCnt << 2);                       //Dword地址索引
    pp = (uint8_t *)&UNI_OsciBuffer.all_8Bits[ScpoeNum << 2];                   //Byte地址索引

    if(p->Ch1Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH1Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH1Addr) >> p->CH1Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    
    if(p->Ch2Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH2Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    else
    {
        DataBuff.all_32bit = ((*p->pCH2Addr) >> p->CH2Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }   
    
    if(p->Ch3Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH3Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH3Addr) >> p->CH3Coeff); 
        
        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
        pp++;
    }

    if(p->Ch4Type == 0x01)
    {
        DataBuff.all_32bit = (*p->pCH4Addr);
        DataBuff.all_32bit = (int32_t)DataBuff.all_16bit[0];

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
    }
    else
    {
        DataBuff.all_32bit =  ((*p->pCH4Addr) >> p->CH4Coeff);  

        *pp = (uint8_t)DataBuff.all_8bit[1];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[0];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[3];
        pp++;
        *pp = (uint8_t)DataBuff.all_8bit[2];
    }

    p->SampleCnt++;
    BuffNum = p->SampleCnt;
    if(p->SampleCnt == SEROSCIBUFFNUM_HALF) 
    {
        p->SampleCnt = 0;
        BuffNum = 0;
    }
      
    if((p->SeriesTxEndFlag == 0) && (ScopePara->SeriesEnable == 1) && (BuffNum >= 100))  //上次发送完成，且有新的发送请求
    {
        if(BuffNum >= SEROSCIMAXSENDNUM)
        {
           p->StartIndex = p->SeriesStartIndex  + ((BuffNum - SEROSCIMAXSENDNUM) << 2);
           p->SeriesTxNum = SEROSCIMAXSENDNUM;          
        }
        else
        {
            p->StartIndex = p->SeriesStartIndex;
            p->SeriesTxNum = BuffNum;         
        }

        if(p->SeriesStartIndex == SERIESFIRSTINDEX) 
        {
            p->SeriesStartIndex = SERIESCONEINDEX;
        }
        else
        {
            p->SeriesStartIndex = SERIESFIRSTINDEX;  
        }
        
        p->SampleCnt = 0;
        p->OsciCtrl.bit.SampleStutas = 6;
        p->SeriesTxEndFlag = 1;
        ScopePara->SeriesEnable = 0;
    }    
}

void OscilloscopeSampling_ms(void)
{
    static Uint16   Cnt = 0;
    
    //如果没有使能采样或者采样完成,计数器清零并退出
    if(STR_Osci.OsciCtrl.bit.Mode != 2)
    {
        Cnt = 0;
        return;
    }

    Cnt ++;
    if(Cnt >= STR_Osci.SampleTime)
    {
        Cnt = 0;
    }
    else
    {
        return;
    }   
    
    SeriesModeSampling(&STR_Osci);
}

#if SERVO_UpComputer == ServoTool
void Scope_SendData(void)
{
    uint8_t state;
    uint16_t ScopeNum;
    uint16_t TxWordNum;
    state = STR_Osci.OsciCtrl.bit.SampleStutas;
    
    switch(state)
    {
        //case 0:  触发模式0是未开启触发，连续模式0是正在采样中
        case 1:
        case 2:
        case 3:
        if(ScopePara->SeriesEnable == 1)
        {
            FirstSendData[0] = RamCommu->Addrss;  
            FirstSendData[1] = 0x06; 
            FirstSendData[2] = 0x76; 
            FirstSendData[3] = 0x0C; 
            FirstSendData[4] = 0x00; 
            FirstSendData[5] = 0x00; 
            
            if(SCOPE_TXEND_FLAG == 1)
            {
                SCI_SelectToTx();
                Scope_DmaSend((uint8_t *)&FirstSendData[0],6); 
                ScopePara->SeriesEnable = 0;
            }  
        }
        else
        {
            if(SCOPE_TXEND_FLAG == 1)
            {
                SCI_SelectToRx();
            }
        }
        break;
        case 4:
            if((ScopePara->SeriesEnable == 1) && (SCOPE_TXEND_FLAG == 1))
            {
                SCI_SelectToTx();
                //ScopeNum = (OSCIBUFFNUM << 3);      //字
                ScopeNum = (OSCIBUFFNUM << 4);      //字节
                FirstSendData[0] = RamCommu->Addrss;  
                FirstSendData[4] = (ScopeNum >> 8);   
                FirstSendData[5] = (ScopeNum&0x00FF); 
                
                Scope_DmaSend((uint8_t *)&FirstSendData[0],6);
                STR_Osci.OsciCtrl.bit.SampleStutas = 5;
                ScopePara->SeriesEnable = 0;
            }
        break;
        case 5:
            if(SCOPE_TXEND_FLAG == 1)
            {
                ScopeNum = ((STR_Osci.StartIndex + STR_Osci.SampleTxCnt) << 2);
                Scope_DmaSend((uint8_t *)&UNI_OsciBuffer.all_32Bits[ScopeNum],16); 
                STR_Osci.SampleTxCnt = STR_Osci.SampleTxCnt + 1;                 
            }      

            if((STR_Osci.StartIndex + STR_Osci.SampleTxCnt) == OSCIBUFFNUM)
            {
                STR_Osci.SampleTxCnt = 0; 
                STR_Osci.StartIndex = 0;
            }
            
            if((STR_Osci.StartIndex + STR_Osci.SampleTxCnt) == STR_Osci.TringEndIndex)
            {
                STR_Osci.OsciCtrl.bit.SampleStutas = 8;  //触发采集和发送结束  
            }          
        break;
        case 6:  
            if(SCOPE_TXEND_FLAG == 1)
            {
                SCI_SelectToTx();
                //TxWordNum = (STR_Osci.SeriesTxNum << 3);       //Word数量
                TxWordNum = (STR_Osci.SeriesTxNum << 4);         //Byte数量
                FirstSendData[0] = RamCommu->Addrss;  
                FirstSendData[4] = (uint8_t)(TxWordNum >> 8); 
                FirstSendData[5] = (uint8_t)(TxWordNum&0x00FF);  
                
                Scope_DmaSend(FirstSendData,6);  
                STR_Osci.OsciCtrl.bit.SampleStutas = 7;
            }
        break;
        case 7:
            if(SCOPE_TXEND_FLAG == 1)
            {
                ScopeNum = STR_Osci.StartIndex;
                Scope_DmaSend((uint8_t *)&UNI_OsciBuffer.all_32Bits[ScopeNum],(STR_Osci.SeriesTxNum << 4));  
                STR_Osci.OsciCtrl.bit.SampleStutas = 8;
            }
        break;
        case 8:
            if(SCOPE_TXEND_FLAG == 1)
            {
                STR_Osci.SeriesTxEndFlag = 0;
                STR_Osci.SampleTxCnt = 0;  
                STR_Osci.SeriesTxNum = 0;
                STR_Osci.OsciCtrl.bit.SampleStutas = 0;
                STR_Osci.OsciCtrl.bit.TringStatus = 0;
                SCI_SelectToRx();
            }
        break;
        default:
	    break;
    }
}
#endif

void HarmoCore_Scope_SendData(void)
{
    uint8_t state;
    uint16_t ScopeNum;
    uint16_t TxWordNum;
    
    state = STR_Osci.OsciCtrl.bit.SampleStutas;
    
    switch(state)
    {
        //case 0:  触发模式0是未开启触发，连续模式0是正在采样中
        case 1:
        case 2:
        case 3:
        if(ScopePara->SeriesEnable == 1)
        {
            HarmoCore_FirstSendData[0] = 0x3C;
            HarmoCore_FirstSendData[1] = 0x3C;  
            HarmoCore_FirstSendData[2] = RamCommu->Addrss;  
            HarmoCore_FirstSendData[3] = 0x64; 
            HarmoCore_FirstSendData[4] = 0x76; 
            HarmoCore_FirstSendData[5] = 0x0C; 
            HarmoCore_FirstSendData[6] = 0x00; 
            HarmoCore_FirstSendData[7] = 0x00; 
            HarmoCore_FirstSendData[8] = 0x3E;
            HarmoCore_FirstSendData[9] = 0x3E;
            if(SCOPE_TXEND_FLAG == 1)
            {
                SCI_SelectToTx();
                Scope_DmaSend((uint8_t *)&HarmoCore_FirstSendData[0],10); 
                ScopePara->SeriesEnable = 0;
            }  
        }
        else
        {
            if(SCOPE_TXEND_FLAG == 1)
            {
                SCI_SelectToRx();
            }
        }
        break;
        case 4:
            if((ScopePara->SeriesEnable == 1) && (SCOPE_TXEND_FLAG == 1))
            {
                SCI_SelectToTx();
                //ScopeNum = (OSCIBUFFNUM << 3);      //字
                ScopeNum = (uint16_t)(((uint16_t)OSCIBUFFNUM) << 4u);       //字节
                HarmoCore_FirstSendData[2] = RamCommu->Addrss;
                HarmoCore_FirstSendData[6] = (ScopeNum >> 8); 
                HarmoCore_FirstSendData[7] = (ScopeNum&0x00FF);
                
                Scope_DmaSend((uint8_t *)&HarmoCore_FirstSendData[0],8);
                STR_Osci.OsciCtrl.bit.SampleStutas = 5;
                ScopePara->SeriesEnable = 0;
            }
        break;
        case 5:
            if(SCOPE_TXEND_FLAG == 1)
            {
                ScopeNum = ((STR_Osci.StartIndex + STR_Osci.SampleTxCnt) << 2);
                Scope_DmaSend((uint8_t *)&UNI_OsciBuffer.all_32Bits[ScopeNum],16); 
                STR_Osci.SampleTxCnt = STR_Osci.SampleTxCnt + 1;                 
            }    

            if((STR_Osci.StartIndex + STR_Osci.SampleTxCnt) == OSCIBUFFNUM)
            {
                STR_Osci.SampleTxCnt = 0; 
                STR_Osci.StartIndex = 0;
            }
            
            if((STR_Osci.StartIndex + STR_Osci.SampleTxCnt) == STR_Osci.TringEndIndex)
            {
                STR_Osci.OsciCtrl.bit.SampleStutas = 8;  //触发采集和发送结束  
            }          
        break;
        
        case 6:
            if(SCOPE_TXEND_FLAG == 1)
            {
                SCI_SelectToTx();
                //TxWordNum = (STR_Osci.SeriesTxNum << 3);       //Word数量
                TxWordNum = (STR_Osci.SeriesTxNum << 4);         //Byte数量
                HarmoCore_FirstSendData[2] = RamCommu->Addrss;   
                HarmoCore_FirstSendData[6] = (uint8_t)(TxWordNum >> 8); 
                HarmoCore_FirstSendData[7] = (uint8_t)(TxWordNum&0x00FF); 
                
                Scope_DmaSend(HarmoCore_FirstSendData, 8);  
                STR_Osci.OsciCtrl.bit.SampleStutas = 7;
            }
        break;
        case 7:
            if(SCOPE_TXEND_FLAG == 1)
            {
                ScopeNum = STR_Osci.StartIndex;
                Scope_DmaSend((uint8_t *)&UNI_OsciBuffer.all_32Bits[ScopeNum], (STR_Osci.SeriesTxNum << 4)); 
                STR_Osci.OsciCtrl.bit.SampleStutas = 8;
            }
        break;
        case 8:
            if(1 == SCOPE_TXEND_FLAG)  //mqb新上位机触发和连续统一反馈帧尾
            {
                Scope_DmaSend(HarmoCore_SendEndFrame, 2); 
                STR_Osci.OsciCtrl.bit.SampleStutas = 9;
            }
        break;
        case 9:
            if(SCOPE_TXEND_FLAG == 1)
            {
                STR_Osci.SeriesTxEndFlag = 0;
                STR_Osci.SampleTxCnt = 0;  
                STR_Osci.SeriesTxNum = 0;
                STR_Osci.OsciCtrl.bit.SampleStutas = 0;
                STR_Osci.OsciCtrl.bit.TringStatus = 0;
                SCI_SelectToRx();
            
                if(STR_Osci.OsciCtrl.bit.Mode == 2)//连续采样
                {
                }
                else
                {
                    ScopePara->Enable = 0;
                    ScopePara->FaultTriEnable = 0;
                    STR_Osci.FaultFlag = 0;
                    ScopePara->FaultTriFlag = 0;
                }
            }
        break;
        default:
	    break;
    }
}


void Scope_Process(void)
{
    ScopeCh_Init(&STR_Osci);
    OscilloscopeSampling_ms();
    #if SERVO_UpComputer == HarmoCore
    HarmoCore_Scope_SendData();
    #elif SERVO_UpComputer == ServoTool
    Scope_SendData();
    #endif
}


