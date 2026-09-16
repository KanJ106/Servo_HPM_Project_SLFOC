#include "TorClosedCtrLoop.h"
#include "SV_FaultProtect.h"//
#include "SV_FuncVar.h"
#include "DrvCoeff.h"
#include "SV_StateMachine.h" 
#include "Drive.h"//
#include "s_tsensor_init.h"

float TargetTorqueClose = 0;
float ActualTorqueClose = 0;
CURFULLCLOSE_PIREG CurFullReg = {0};
//extern int32_t TorQueData;

void CurFullPiReg_init(CURFULLCLOSE_PIREG *v)
{
    float temp;
    
    v->Up2  = 0.0f;
    v->Up  = 0.0f;
    v->Ui  = 0.0f;
    v->Ud  = 0.0f;
    v->Out = 0;
    v->Ref = 0.0f;
    v->PiFlag = 0;
    v->PdFlag = 0;
    v->OutPreSat = 0.0f;
    v->cnt1 = 0;
    v->Time1 = RamPC->Time1;

    
    temp = DrvCoeff.MotTorqe * RamPD->Reduction; // 0.01 NM
    v->Coeff  = 100000.0f / temp;                // 0.1%
    v->Coeff2  = v->Coeff;                       // 0.1%
    
   // v->OutMax = (float)DPI_CCW_TorqMaxLmt;
   // v->OutMin = (float)(0 - (int16_t)DPI_CW_TorqMaxLmt);
   
    if(RamPC->Tor_Kp == 0) 
        v->Kp = 0;
    else
        v->Kp = (float)RamPC->Tor_Kp / 1000.0f;
    
    if(RamPC->Tor_Ki == 0) 
        v->Ki = 0;
    else
        v->Ki = (float)RamPC->Tor_Ki / 10000.0f;

    if(RamPC->Tor_Kd == 0) 
        v->Kd = 0;
    else
        v->Kd = (float)RamPC->Tor_Kd / 10000.0f;

    if(RamPC->Tor_Kp2 == 0) 
        v->Kp2 = 0;
    else
        v->Kp2 = (float)RamPC->Tor_Kp2 / 1000.0f;

    if(RamPC->TorKsf == 0)
        v->TorKsf = 0;
    else
        v->TorKsf = (float)RamPC->TorKsf / 1000.0f;

    if(RamPC->TorAcc == 0)
    {
        v->RampAccDelta = 0;
        v->RampDecDelta = 0;
    }
    else
    {
        v->RampAccDelta = (float)RamPC->TorAcc / 1000.0f;
        v->RampDecDelta = v->RampAccDelta;
    }

    v->TorRefRev = RamPC->TorRefRev;

    if(RamPC->ErrValLmt == 0)
    {
        v->ErrValLmt = 0;
    }
    else
    {
        v->ErrValLmt = (float)RamPC->ErrValLmt / 1000.0f;
    }

    v->ErrTimLmt = RamPC->ErrTimLmt;

    
    if(RamPC->TorOutLmt == 0)
    {
        v->OutMax = 0;
        v->OutMin = 0;
    }
    else
    {
        v->OutMax = (float)RamPC->TorOutLmt*10;
        v->OutMin = -v->OutMax;
    }
    

    v->TorDonFlag = 0;
}

void CurFullPiReg_rst(CURFULLCLOSE_PIREG *v)
{
    v->Up2  = 0.0f;
    v->Up  = 0.0f;
    v->Ui  = 0.0f;
    v->Ud  = 0.0f;
    v->Err = 0;
    v->Err_pre = 0;
    v->Out = 0;
    v->Ref = 0.0f;
    v->PiFlag = 0;
    v->PdFlag = 0;
    v->OutPreSat = 0.0f;
    v->Fdb   = -TorQueData * v->Coeff;
    v->cnt1 = 0;
    v->Time1 = RamPC->Time1;

    if(RamPC->Tor_Kp == 0) 
        v->Kp = 0;
    else
        v->Kp = (float)RamPC->Tor_Kp / 1000.0f;
    
    if(RamPC->Tor_Ki == 0) 
        v->Ki = 0;
    else
        v->Ki = (float)RamPC->Tor_Ki / 10000.0f;

    if(RamPC->Tor_Kd == 0) 
        v->Kd = 0;
    else
        v->Kd = (float)RamPC->Tor_Kd / 10000.0f;

    if(RamPC->Tor_Kp2 == 0) 
        v->Kp2 = 0;
    else
        v->Kp2 = (float)RamPC->Tor_Kp2 / 1000.0f;

    if(RamPC->TorKsf == 0)
        v->TorKsf = 0;
    else
        v->TorKsf = (float)RamPC->TorKsf / 1000.0f;

    if(RamPC->TorAcc == 0)
    {
        v->RampAccDelta = 0;
        v->RampDecDelta = 0;
    }
    else
    {
        v->RampAccDelta = (float)RamPC->TorAcc / 1000.0f;
        v->RampDecDelta = v->RampAccDelta;
    }

    v->TorRefRev = RamPC->TorRefRev;

    if(RamPC->ErrValLmt == 0)
    {
        v->ErrValLmt = 0;
    }
    else
    {
        v->ErrValLmt = (float)RamPC->ErrValLmt / 1000.0f;
    }

    v->ErrTimLmt = RamPC->ErrTimLmt;

    
    if(RamPC->TorOutLmt == 0)
    {
        v->OutMax = 0;
        v->OutMin = 0;
    }
    else
    {
        v->OutMax = (float)RamPC->TorOutLmt*10;
        v->OutMin = -v->OutMax;
    }
    

    v->TorDonFlag = 0;
}

void CurFullPiReg_updatat(CURFULLCLOSE_PIREG *v)
{
    v->Time1 = RamPC->Time1;

    if(RamPC->Tor_Kp == 0) 
        v->Kp = 0;
    else
        v->Kp = (float)RamPC->Tor_Kp / 1000.0f;
    
    if(RamPC->Tor_Ki == 0) 
        v->Ki = 0;
    else
        v->Ki = (float)RamPC->Tor_Ki / 10000.0f;

    if(RamPC->Tor_Kd == 0) 
        v->Kd = 0;
    else
        v->Kd = (float)RamPC->Tor_Kd / 10000.0f;

    if(RamPC->Tor_Kp2 == 0) 
        v->Kp2 = 0;
    else
        v->Kp2 = (float)RamPC->Tor_Kp2 / 1000.0f;

    if(RamPC->TorKsf == 0)
        v->TorKsf = 0;
    else
        v->TorKsf = (float)RamPC->TorKsf / 1000.0f;

    if(RamPC->TorAcc == 0)
    {
        v->RampAccDelta = 0;
        v->RampDecDelta = 0;
    }
    else
    {
        v->RampAccDelta = (float)RamPC->TorAcc / 1000.0f;
        v->RampDecDelta = v->RampAccDelta;
    }

    v->TorRefRev = RamPC->TorRefRev;

    if(RamPC->ErrValLmt == 0)
    {
        v->ErrValLmt = 0;
    }
    else
    {
        v->ErrValLmt = (float)RamPC->ErrValLmt / 1000.0f;
    }

    v->ErrTimLmt = RamPC->ErrTimLmt;

    
    if(RamPC->TorOutLmt == 0)
    {
        v->OutMax = 0;
        v->OutMin = 0;
    }
    else
    {
        v->OutMax = (float)RamPC->TorOutLmt*10;
        v->OutMin = -v->OutMax;
    }
    
}

void TorRampGenerator_calc(CURFULLCLOSE_PIREG *v)//斜坡函数
{
	v->RampErr = v->RampTarget-v->RampOutput;

    if(v->RampAccDelta==0||v->RampDecDelta==0)
    {
        v->RampOutput = v->RampTarget;
    }
    else
    {
        if(v->RampErr > 0)
    	{
    		v->RampOutput += v->RampAccDelta;
    		v->RampOutput = v->RampOutput>v->RampTarget?v->RampTarget:v->RampOutput;
    	}
    	else if(v->RampErr < 0)
    	{
    		v->RampOutput -= v->RampDecDelta;
    		v->RampOutput = v->RampOutput<v->RampTarget?v->RampTarget:v->RampOutput;
    	}
    }
}



int16_t CurFullPiReg_Calc(CURFULLCLOSE_PIREG *v)
{
    float Err;
    
    Err = v->Ref - v->Fdb;
    
	v->Err = (int16)Err;
    
	v->Up = v->Kp * Err;                      // Compute the proportional output

    v->Up2 = v->Kp2 * v->Ref;

    v->Ud = v->Kd * (Err - v->Err_pre);         // v->Ud = v->Kd * (Err-v->Err_pre)/v-dt; 

    v->Err_pre = Err;
    
    if(v->PiFlag == 1 && Err < 0.0f)//积分抗饱和处理
    {
        v->Ui += v->Ki * Err;
    }
    else if(v->PiFlag == 2 && Err > 0.0f)
    {
        v->Ui += v->Ki * Err;
    }
    else if(v->PiFlag == 0)
    {
        v->Ui += v->Ki * Err;
    }
    
    v->OutPreSat = v->Up + v->Ui + v->Up2 + v->Ud;
     
    if(v->OutPreSat > v->OutMax)//输出限幅
    {
        v->Out = (int16_t)v->OutMax;
        v->PiFlag = 1;
    }
    else if(v->OutPreSat < v->OutMin)
    {
        v->Out = (int16_t)v->OutMin;
        v->PiFlag = 2;
    }
    else
    {
        v->Out = (int16_t)v->OutPreSat;
        v->PiFlag = 0;
    }

    v->Out = v->Out + v->TorKsf*(v->TorRefPre - v->Out);//一阶低通滤波
    v->TorRefPre = v->Out;


    /*
    if(v->cnt1<32768)//力矩环力矩到位信号处理
    {
        if(Err<v->ErrValLmt&&Err>-v->ErrValLmt)
        {
            v->cnt1++;
        }
        else
        {
            v->cnt1 = 0;
        }

        if(v->cnt1>v->ErrTimLmt)
        {
            v->TorDonFlag = 1;
        }
        else
        {
            v->TorDonFlag = 0;
        }
    }
    */
    
    return v->Out;
}

/*
int16_t CurFullPdReg_Calc(CURFULLCLOSE_PIREG *v)
{
    float Err;
    
    Err = v->Ref - v->Fdb;
    
	v->Err = (int16)Err;
    
	v->Up = v->Kp * Err;                      // Compute the proportional output
    
    v->Ud = v->Kd * (Err - v->Err_pre);         // v->Ud = v->Kd * (Err-v->Err_pre)/v-dt; 

    v->Err_pre = Err;
    
    v->OutPreSat = v->Up + v->Ud + v->Kp2 * v->Ref;
     
    if(v->OutPreSat > v->OutMax)
    {
        v->Out = (int16_t)v->OutMax;
    }
    else if(v->OutPreSat < v->OutMin)
    {
        v->Out = (int16_t)v->OutMin;
    }
    else
    {
        v->Out = (int16_t)v->OutPreSat;
    }
    
    return v->Out;
}
*/


int16_t TorCtrLoop_Calc(CURFULLCLOSE_PIREG *v)
{
    float temp1;
    int16 temp2; 

    if(CurFullReg.Time1 > 0)//当开启内部两段力矩阶跃指令测试
    {
        if(CurFullReg.cnt1 < 32768)
        {
            CurFullReg.cnt1++;
        }

        if(CurFullReg.cnt1<CurFullReg.Time1)
        {
            temp1 = (float)DPI_TorRefCmd*CurFullReg.Coeff2;
        }
        else if(CurFullReg.cnt1>=CurFullReg.Time1 && CurFullReg.cnt1<2*CurFullReg.Time1)
        {
            temp1 = 0;
        }
        else
        {
            CurFullReg.cnt1 = 0;
            temp1 = 0;
        }
    }
    else//不开启测试
    {
        temp1 = (float)DPI_TorRefCmd*CurFullReg.Coeff2;
    }

    CurFullReg.RampTarget = (CurFullReg.TorRefRev==0)? temp1:(-temp1);//力矩命令方向取反

    TorRampGenerator_calc(&CurFullReg);//力矩斜坡函数

    Spd_PiReg.ToroutRef = CurFullReg.RampOutput;//DPI_TrqCmd;//temp17;//CM_BusCtrl.TargetTorqueA;
    CurFullReg.Ref      = CurFullReg.RampOutput;//DPI_TrqCmd;//temp17;//(float)CM_BusCtrl.TargetTorqueA;
    CurFullReg.Fdb      = TorQueData * CurFullReg.Coeff;//10 * CurFullReg.Coeff;//
    temp2 = CurFullPiReg_Calc(&CurFullReg);//使用PI控制器

    return temp2;
}
