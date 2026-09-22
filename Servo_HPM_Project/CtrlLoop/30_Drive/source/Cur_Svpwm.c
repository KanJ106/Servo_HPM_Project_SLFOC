/*
 * Cur_Svpwm.c
 *
 *  Created on: 2015-12-16//
 *      Author: w
 */
#include "Drive.h"
#include "SensorlessConfig.h"//
uint16_t PwmTestU;
uint16_t PwmTestV;
uint16_t PwmTestW;
/******************************************************************************
 **函 数 名：
 **描    述：SVPWM计算
 **调    用：
 **输    入：
 **输    出：
 **返    回：无
 **其    它：无
 **日    期：2015-12-17
 *******************************************************************************/
void Cur_Svpwm_init(CUR_SVPWM *v)
{
    Uint16 temp;
    Uint64 temp64;
	v->AvrSw = 0;
	v->MaxDuty = _IQ(0.5);
    
    if(RamServo->DbComp > 10)
        RamServo->DbComp = 10;
    
    temp = PWMCLKASE / 1000000;
    temp = (temp * RamServo->DbComp) / 10;
    
    v->OnDelayComp = temp;
    
    if(RamServo->DbSlope == 0)
    {
        v->Slope = 0;
    }
    else
    {
        temp64 = _IQdiv(RamServo->DbSlope, 32678); 
        
        v->Slope = 16777216L / temp64;
    }
}

void Cur_Svpwm_rst(CUR_SVPWM *v)
{
	v->AvrSw = 0;
	v->MaxDuty = _IQ(0.5);
    v->SamWinMode = 0;
}


void Cur_Svpwm_calc(CUR_SVPWM *v)
{
#if FUNCTYPEN
	_iq Vatemp,Vbtemp,Vctemp,Vmax,Vmin;
	_iq Ualpha,Ubeta;
	if(v->AvrSw == 0)
	{
		Ualpha = _IQmpy(v->Ualpha,_IQdiv(DrvCoeff.UdcEpu,v->Udc));
		Ubeta = _IQmpy(v->Ubeta,_IQdiv(DrvCoeff.UdcEpu,v->Udc));//电压补偿
	}
	else
	{
		Ualpha = v->Ualpha;
		Ubeta = v->Ubeta;//电压补偿
	}

	//2S->3S
	Vatemp = _IQmpy(_IQ(1.154700538379),Ualpha);
	Vbtemp = _IQmpy(_IQ(-0.57735026919),Ualpha) + _IQmpy(_IQ(1.0),Ubeta);
	Vctemp = _IQmpy(_IQ(-0.57735026919),Ualpha) - _IQmpy(_IQ(1.0),Ubeta);

	Vmax = FindMax(Vatemp,Vbtemp,Vctemp);
	Vmin = FindMin(Vatemp,Vbtemp,Vctemp);
    
	v->Vz = _IQmpy(_IQ(-0.5),(Vmax+Vmin));//谐波计算
	Vatemp = _IQmpy(_IQ(0.5),(_IQ(1.0)+Vatemp+v->Vz));
	v->Va = _IQsat(Vatemp,_IQ(1.0),_IQ(0.0));
	Vbtemp = _IQmpy(_IQ(0.5),(_IQ(1.0)+Vbtemp+v->Vz));
	v->Vb = _IQsat(Vbtemp,_IQ(1.0),_IQ(0.0));
	Vctemp = _IQmpy(_IQ(0.5),(_IQ(1.0)+Vctemp+v->Vz));
	v->Vc = _IQsat(Vctemp,_IQ(1.0),_IQ(0.0));
	v->MaxDuty = FindMax(v->Va,v->Vb,v->Vc);//最大占空比，占空比越大延时越大
#endif
}

#ifdef __riscv
__attribute__((section(".ramfunctiontext")))
#endif
void Svpwm_Calc(void)
{
	_iq Vatemp,Vbtemp,Vctemp,Vmax,Vmin;
	_iq Ualpha,Ubeta;
	_iq rats;
    _iq ia,ib,ic,duty;
    uint16_t dutyu,dutyv,dutyw;
    uint16_t epwmPrd,maxduty;
    
	if(SENSORLESS_SOURCE_FOC_BUILD || (Svpwm.AvrSw == 0 && DPI_ACR_AVR == 0))//电压补偿
	{
		rats = _IQdiv(DrvCoeff.UdcEpu,Svpwm.Udc);//1.28us
		Ualpha = _IQmpy(Svpwm.Ualpha,rats);
		Ubeta = _IQmpy(Svpwm.Ubeta,rats);//电压补偿
	}
	else
	{
		Ualpha = Svpwm.Ualpha;
		Ubeta = Svpwm.Ubeta;
	}
    
    #if 0
    //2S->3S
	Vatemp = _IQmpy(19372660L,Ualpha);
	Vbtemp = _IQmpy(-9686330L,Ualpha) + Ubeta;
	Vctemp = _IQmpy(-9686330L,Ualpha) - Ubeta;

	Vmax = Vatemp>Vbtemp?(Vatemp>Vctemp?Vatemp:Vctemp):(Vbtemp>Vctemp?Vbtemp:Vctemp);//0.36us
	Vmin = Vatemp<Vbtemp?(Vatemp<Vctemp?Vatemp:Vctemp):(Vbtemp<Vctemp?Vbtemp:Vctemp);
	Svpwm.Vz = _IQmpy(-8388608L,(Vmax+Vmin));//谐波计算
	Vatemp = _IQmpy(8388608L,(16777216L + Vatemp+Svpwm.Vz));
	Svpwm.Va = _IQsat(Vatemp,_IQ24_P,_IQ24_N);
	Vbtemp = _IQmpy(8388608L,(16777216L + Vbtemp+Svpwm.Vz));
	Svpwm.Vb = _IQsat(Vbtemp,_IQ24_P,_IQ24_N);
	Vctemp = _IQmpy(8388608L,(16777216L + Vctemp+Svpwm.Vz));
	Svpwm.Vc = _IQsat(Vctemp,_IQ24_P,_IQ24_N);
    #else
    //2S->3S
    {
        int32 Va_tmp = Ualpha >> 1;
        int32 Vb_tmp = _IQmpy(14529495L,Ubeta);  
          
        Vatemp = Ualpha;                 // Va = Ualpha
	    Vbtemp = -Va_tmp + Vb_tmp;       // Vb = -0.5 * Ualpha + Ubeta * sqrt(3)/2 
	    Vctemp = -Va_tmp - Vb_tmp;       // Vc = -0.5 * Ualpha - Ubeta * sqrt(3)/2
    }
    
    Vmax = Vatemp>Vbtemp?(Vatemp>Vctemp?Vatemp:Vctemp):(Vbtemp>Vctemp?Vbtemp:Vctemp);
	Vmin = Vatemp<Vbtemp?(Vatemp<Vctemp?Vatemp:Vctemp):(Vbtemp<Vctemp?Vbtemp:Vctemp);
    
	Svpwm.Vz = (Vmax + Vmin) >> 1;
    
    Vatemp = 8388608L + (Vatemp - Svpwm.Vz);
	Svpwm.Va = _IQsat(Vatemp,_IQ24_P,_IQ24_N);
    
	Vbtemp = 8388608L + (Vbtemp - Svpwm.Vz);
	Svpwm.Vb = _IQsat(Vbtemp,_IQ24_P,_IQ24_N);
    
	Vctemp = 8388608L + (Vctemp - Svpwm.Vz);
	Svpwm.Vc = _IQsat(Vctemp,_IQ24_P,_IQ24_N);
    #endif
    
    /**************************死区补偿************************************/
    if(Svpwm.OnDelayComp > 0)
    {
        //2S->3S
        Ualpha = Svpwm.Ialpha >> 1;
        Ubeta = _IQmpy(14529495L,Svpwm.Ibeta);  
          
        Vatemp = Ualpha;                 // Va = Ualpha
        Vbtemp = -Ualpha + Ubeta;        // Vb = -0.5 * Ualpha + Ubeta * sqrt(3)/2 
        Vctemp = -Ualpha - Ubeta;        // Vc = -0.5 * Ualpha - Ubeta * sqrt(3)/2
        Svpwm.Ia = Ualpha;               //测试使用
        
        if(Svpwm.Slope == 0)
        {
            ia = (int16)_IQsign(Vatemp) * Svpwm.OnDelayComp;   //矩形补偿
            ib = (int16)_IQsign(Vbtemp) * Svpwm.OnDelayComp;   //矩形补偿
            ic = (int16)_IQsign(Vctemp) * Svpwm.OnDelayComp;   //矩形补偿
        }
        else
        {
            ia = (int16)_IQmpy_HP(Vatemp, Svpwm.Slope);         //梯形补偿
            ia = _IQsat(ia,Svpwm.OnDelayComp,-Svpwm.OnDelayComp);
            ib = (int16)_IQmpy_HP(Vbtemp, Svpwm.Slope);         //梯形补偿
            ib = _IQsat(ib,Svpwm.OnDelayComp,-Svpwm.OnDelayComp);
            ic = (int16)_IQmpy_HP(Vctemp, Svpwm.Slope);         //梯形补偿
            ic = _IQsat(ic,Svpwm.OnDelayComp,-Svpwm.OnDelayComp);
        }
    }
    else
    {
        ia = 0;
        ib = 0;
        ic = 0;
    }
    /**************************死区补偿************************************/
    
    epwmPrd = DrvCoeff.EpwmPrd >> 1;
    if(DrvCoeff.DrvFlg.bit.SvpwmMode == 0)
    {
        //duty = Svpwm.Va;
        duty = 16777216L - Svpwm.Va;    //电流未取反
        duty = _IQmpyI32int(duty,epwmPrd);
        duty = duty - ia;
        dutyu = _IQsat(duty,epwmPrd,0);     
        //PWMA_SET_COMPAREVALUE = duty;
        
        //duty = Svpwm.Vb;
        duty = 16777216L - Svpwm.Vb;     //电流未取反
        duty = _IQmpyI32int(duty,epwmPrd);
        duty = duty - ib;
        dutyv = _IQsat(duty,epwmPrd,0);      
        //PWMB_SET_COMPAREVALUE = duty;
        
        //duty = Svpwm.Vc;
        duty = 16777216L - Svpwm.Vc;     //电流未取反
        duty = _IQmpyI32int(duty,epwmPrd);
        duty = duty - ic;
        dutyw = _IQsat(duty,epwmPrd,0);     
        //PWMC_SET_COMPAREVALUE = duty;
    }
    else
    {
        //duty = Svpwm.Va;
        duty = 16777216L - Svpwm.Vc;    //电流未取反
        duty = _IQmpyI32int(duty,epwmPrd);
        duty = duty - ic;
        dutyu = _IQsat(duty,epwmPrd,0);     
        //PWMC_SET_COMPAREVALUE = duty;
        
        //duty = Svpwm.Vb;
        duty = 16777216L - Svpwm.Vb;     //电流未取反
        duty = _IQmpyI32int(duty,epwmPrd);
        duty = duty - ib;
        dutyv = _IQsat(duty,epwmPrd,0);      
        //PWMB_SET_COMPAREVALUE = duty;
        
        //duty = Svpwm.Vc;
        duty = 16777216L - Svpwm.Va;     //电流未取反
        duty = _IQmpyI32int(duty,epwmPrd);
        duty = duty - ia;
        dutyw = _IQsat(duty,epwmPrd,0);     
        //PWMA_SET_COMPAREVALUE = duty; 
    }
    PwmDutyUpdata(dutyu,dutyv,dutyw,DrvCoeff.EpwmPrd); 

    maxduty = dutyu>dutyv?(dutyu>dutyw?dutyu:dutyw):(dutyv>dutyw?dutyv:dutyw);
   
    if(maxduty == dutyu)
    {
        Svpwm.SamWinMode = 1;
    }
    else if(maxduty == dutyv)
    {
        Svpwm.SamWinMode = 2;
    }
    else
    {
        Svpwm.SamWinMode = 0;
    }

    PwmTestU = dutyu;
    PwmTestV = dutyv;
    PwmTestW = dutyw;
}


