/*
 * ToqLoop.c
 *
 *  Created on: 2015-12-24//
 *      Author: w
 */
#include "SV_AppDriveInterface.h"
#include "Drive.h"
#include <math.h>
#include "SensorlessShadow.h"
#include "SensorlessCanopen.h"
#include "StartupTiming.h"
#include "s_analog_init.h"
extern void Sensorless_GlobalFaultHook(uint32_t fault_flags);
#include "SV_PanelCtl.h"//
#include "s_pwm_init.h"
extern int32_t g_sl_sdm_raw_u, g_sl_sdm_raw_v;
extern uint16_t PwmTestU, PwmTestV, PwmTestW;

#define SPDRESPONSTESTEN 0//速度环路频宽测试
#define IQREFLPFEN 0      //iqref一阶低通滤波

/* One-shot transaction: keep its locals/code out of the per-PWM XIP frame.
 * The caller and transaction use the same acquired Clarke sample. */
#if defined(__riscv)
__attribute__((noinline,section(".ramfunctiontext")))
#elif defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__)
__attribute__((noinline))
#endif
static uint16_t ToqLoop_CommitFrame(float voltage_limit,float *sine,float *cosine)
{
    uint16_t current_failure,current_ok;
    SL_PROBE *p=&g_sensorless_shadow.startup.probe;
    SOURCE_FOC_CURRENT next;
    int32_t new_theta=p->previous_smo;
    float new_angle=(float)((double)new_theta*DIV_PI_MPY_2);
    float ns=sinf(new_angle),nc=cosf(new_angle);
    _iq nsq=(_iq)(ns*_IQ24_P),ncq=(_iq)(nc*_IQ24_P);
    _iq nd=_IQmpy(Clark.Alpha,ncq)+_IQmpy(Clark.Beta,nsq);
    _iq nq=_IQmpy(Clark.Beta,ncq)-_IQmpy(Clark.Alpha,nsq);
    current_failure=(new_theta<0 || new_theta>=16777216)
        ? SOURCE_FOC_INPUT_INVALID : SourceFoc_PrepareFrame(&g_source_foc_current,&next,
        g_sensorless_diag.id_command_pu,g_sensorless_diag.iq_command_pu,
        _IQtoF(Park.Ds),_IQtoF(Park.Qs),_IQtoF(nd),_IQtoF(nq),
        nc*(*cosine)+ns*(*sine),ns*(*cosine)-nc*(*sine),voltage_limit);
    current_ok=current_failure==SOURCE_FOC_OK;
    if(current_ok) {
        /* Single writer in the PWM ISR. No fallible operation after commit
         * and before the normal inverse-Park/PWM path. */
        g_source_foc_current=next;
        Park.Ds=nd;Park.Qs=nq;ToqLoop.idfb=nd;ToqLoop.iqfb=nq;
        (*sine)=ns;(*cosine)=nc;
        Etheta.Ethetapk=Etheta.EtaInerGet=new_theta;
        p->frame_id=p->last_id=next.id_reference_pu;
        p->frame_iq=p->last_iq=next.iq_reference_pu;
        p->frame_tick=g_sensorless_shadow.pwm_counter;
        p->cycles=1U;p->phase=SL_P_HOLD;p->ratio=1.0f;
        p->offset=p->delta;
        g_sensorless_diag.id_command_pu=p->frame_id;
        g_sensorless_diag.iq_command_pu=p->frame_iq;
        g_sensorless_shadow.startup.output.id_ref_pu=p->frame_id;
        g_sensorless_shadow.startup.output.iq_ref_pu=p->frame_iq;
        g_sensorless_shadow.startup.output.theta_e_q24=new_theta;
        g_sensorless_shadow.startup.output.source=ROTOR_SOURCE_SENSORLESS;
        g_sensorless_diag.control_source=ROTOR_SOURCE_SENSORLESS;
        IdRef.IdRefInner=_IQ(p->frame_id);IqRef.IqRefInner=_IQ(p->frame_iq);
        g_probe_trace.h.fast_trigger_tick=p->frame_tick;
        p->frame_committed=1U;
    }
    return current_failure;
}

void ToqLoop_Calc(void)
{
    float angle, sine, cosine, bus_ratio, voltage_limit;
    uint16_t neutral, current_ok, current_failure=SOURCE_FOC_OK;
    uint32_t part_begin=StartupTiming_Cycle();
    g_startup_timing.current_pi=g_startup_timing.pwm=g_startup_timing.monitor=0;
    g_sensorless_chain.chain_valid = 0;
    g_sensorless_chain.chain_tick = (int32_t)g_sensorless_shadow.pwm_counter;
    Get_AdcValue();
    AdRead.SamWinMode = Svpwm.SamWinMode;
    AdRead.ClarkMode = (Uint16)DrvCoeff.DrvFlg.bit.SvpwmMode;
    IuvRead_Calc();
    UdcRead_Calc();
    AdAdjCal_Calc();
    SensorlessSampling_Record(StateMachine.RegulFlg ? 3U :
        (StateMachine.InjectStep==2U ? 2U : (StateMachine.InjectStep==1U ? 1U : 0U)),
        (int16_t)ADCBuf[0], (int16_t)ADCBuf[1], AdRead.IuZero, AdRead.IvZero);
    g_startup_timing.acquisition=StartupTiming_Cycle()-part_begin;
    part_begin=StartupTiming_Cycle();
    Clark.As = AdRead.Iu3;
    Clark.Bs = AdRead.Iv3;
    clark();
    angle = (float)((double)Etheta.Ethetapk * DIV_PI_MPY_2);
    sine = sinf(angle);
    cosine = cosf(angle);
    Park.Alpha = Clark.Alpha;
    Park.Beta = Clark.Beta;
    park((_iq)(cosine * _IQ24_P), (_iq)(sine * _IQ24_P));
    g_sensorless_chain.sdm_u = g_sl_sdm_raw_u;
    g_sensorless_chain.sdm_v = g_sl_sdm_raw_v;
    g_sensorless_chain.adc_u = (int16_t)ADCBuf[0];
    g_sensorless_chain.adc_v = (int16_t)ADCBuf[1];
    g_sensorless_chain.adc_w = (int16_t)ADCBuf[2];
    g_sensorless_chain.zero_u = AdRead.IuZero;
    g_sensorless_chain.zero_v = AdRead.IvZero;
    g_sensorless_chain.zero_w = AdRead.IwZero;
    g_sensorless_chain.iu_q24 = AdRead.Iu;
    g_sensorless_chain.iv_q24 = AdRead.Iv;
    g_sensorless_chain.iw_q24 = AdRead.Iw;
    g_sensorless_chain.clark_u_q24 = Clark.As;
    g_sensorless_chain.clark_v_q24 = Clark.Bs;
    g_sensorless_chain.alpha_q24 = Clark.Alpha;
    g_sensorless_chain.beta_q24 = Clark.Beta;
    g_sensorless_chain.sample_window = AdRead.SamWinMode;
    g_sensorless_chain.phase_mode = AdRead.ClarkMode;
    ToqLoop.idfb = Park.Ds;
    ToqLoop.iqfb = Park.Qs;
    g_startup_timing.transform=StartupTiming_Cycle()-part_begin;
    /* Probe release completion removes gate drive in this PWM period. */
    if (SENSORLESS_IS_PROBE(g_sensorless_shadow.startup.config.debug_mode)
        && g_sensorless_shadow.startup.probe.phase == SL_P_OFF)
        PWMOutDisable();
    /* Only an armed sensorless run owns the output in this production build. */
    if (StateMachine.RegulFlg != 1U || !g_source_foc_ready
        || !g_sensorless_diag.control_override_active) {
        SourceFoc_Reset(&g_source_foc_current);
        IdPiReg.Out = IqPiReg.Out = 0;
        Svpwm.Ualpha = Svpwm.Ubeta = 0;
        Svpwm.Va = Svpwm.Vb = Svpwm.Vc = _IQ(0.5);
        CurMonitor.IeeValuPu = 0;
        CurMonitor.TorqRatsDispS = 0;
        CurMonitor.TorqRatsRef = 0;
        neutral = DrvCoeff.EpwmPrd >> 2;
        PwmDutyUpdata(neutral, neutral, neutral, DrvCoeff.EpwmPrd);
        return;
    }
    part_begin=StartupTiming_Cycle();
    bus_ratio = (DrvCoeff.UdcEpu > 0)
        ? _IQtoF(AdRead.Udc) / _IQtoF(DrvCoeff.UdcEpu) : 0.0f;
    voltage_limit = fminf((float)MAXPIOUT, 0.5f * bus_ratio);
    if(g_sensorless_shadow.startup.probe.frame
        && g_sensorless_shadow.startup.probe.phase==SL_P_RISE
        && !g_sensorless_shadow.startup.probe.frame_committed) {
        current_failure=ToqLoop_CommitFrame(voltage_limit,&sine,&cosine);
        current_ok=current_failure==SOURCE_FOC_OK;
    } else {
        current_ok = SourceFoc_Step(&g_source_foc_current,
            g_sensorless_diag.id_command_pu, g_sensorless_diag.iq_command_pu,
            _IQtoF(Park.Ds), _IQtoF(Park.Qs), voltage_limit);
        current_failure=g_source_foc_current.failure_reason;
    }
    g_startup_timing.current_pi=StartupTiming_Cycle()-part_begin;
    if (!current_ok) {
        g_source_foc_current.failure_reason=current_failure;
        IdPiReg.Out = IqPiReg.Out = 0;
        Svpwm.Ualpha = Svpwm.Ubeta = 0;
        Svpwm.Va = Svpwm.Vb = Svpwm.Vc = _IQ(0.5);
        neutral = DrvCoeff.EpwmPrd >> 2;
        PwmDutyUpdata(neutral, neutral, neutral, DrvCoeff.EpwmPrd);
        SensorlessShadow_RecordAlignCurrent(voltage_limit, CURRENT_MODE == CURRENT_SDFM);
        SensorlessShadow_CurrentFault(current_failure);
        return;
    }
    part_begin=StartupTiming_Cycle();
    IdRef.Ref = _IQ(g_source_foc_current.id_reference_pu);
    IqRef.IqRef = _IQ(g_source_foc_current.iq_reference_pu);
    IdPiReg.Ref = IdRef.Ref;
    IqPiReg.Ref = IqRef.IqRef;
    IdPiReg.Fdb = Park.Ds;
    IqPiReg.Fdb = Park.Qs;
    IdPiReg.Out = _IQ(g_source_foc_current.voltage_d);
    IqPiReg.Out = _IQ(g_source_foc_current.voltage_q);
    Ipark.Ds = IdPiReg.Out;
    Ipark.Qs = IqPiReg.Out;
    ipark_Calc((_iq)(cosine * _IQ24_P), (_iq)(sine * _IQ24_P));
    Svpwm.OnDelayComp = 0U;
    Svpwm.Ualpha = Ipark.Alpha;
    Svpwm.Ubeta = Ipark.Beta;
    Svpwm.Udc = AdRead.Udc;
    Svpwm_Calc();
    g_sensorless_chain.ualpha_q24 = Svpwm.Ualpha;
    g_sensorless_chain.ubeta_q24 = Svpwm.Ubeta;
    g_sensorless_chain.duty_a_q24 = Svpwm.Va;
    g_sensorless_chain.duty_b_q24 = Svpwm.Vb;
    g_sensorless_chain.duty_c_q24 = Svpwm.Vc;
    g_sensorless_chain.pwm_u = PwmTestU;
    g_sensorless_chain.pwm_v = PwmTestV;
    g_sensorless_chain.pwm_w = PwmTestW;
    g_sensorless_chain.pwm_period = DrvCoeff.EpwmPrd;
    g_sensorless_chain.bus_reference_q24 = DrvCoeff.UdcEpu;
#if CURRENT_MODE == CURRENT_SDFM
    g_sensorless_chain.chain_valid = 1;
#endif
    g_startup_timing.pwm=StartupTiming_Cycle()-part_begin;
    SensorlessShadow_RecordAlignCurrent(voltage_limit, CURRENT_MODE == CURRENT_SDFM);
    part_begin=StartupTiming_Cycle();
    Monitor_Calc();
    g_startup_timing.monitor=StartupTiming_Cycle()-part_begin;
}

/*
void ToqCaclFloat(void)
{
    int32 tempu32,tempv32;
    float Angle;
    float Sine,Cosine;
    float temp1,temp2,temp3,temp4,temp5;
    
    tempu32 = AdRead.iu;
    tempv32 = AdRead.iv;
    
    if(StateMachine.RegulFlg > 0)
    {
        tempu32 = tempu32 - AdRead.IuZero;
        tempv32 = tempv32 - AdRead.IvZero;   
    }
    
    tempu32 = _IQsat(tempu32,32767,-32768);
    tempv32 = _IQsat(tempv32,32767,-32768);
    
    if(StateMachine.InjectStep == 2)    //if(StateMachine.RegulFlg < 1)
    {
        AdRead.SumIu += AdRead.iu;
        AdRead.SumIv += AdRead.iv;
        AdRead.Cnt++;
        if(AdRead.Cnt > 127)
        {
            AdRead.IuZero = AdRead.SumIu >> 7;
            AdRead.IvZero = AdRead.SumIv >> 7;
            AdRead.SumIu  = 0;
            AdRead.SumIv  = 0;
            AdRead.Cnt    = 0;
      
            if(_IQabs(AdRead.IuZero) > 1000 || _IQabs(AdRead.IvZero) > 1000)
            {
                FaultPrtt_FaultInterface(LostPhase);
            }
            else
            {
                CLEAR_SDFM_STATUS();
            }
        }
    }
    else
    {
        AdRead.SumIu = 0;
        AdRead.SumIv = 0;
        AdRead.Cnt = 0;
    }
    
    //alpha
    temp1 = tempu32 / 32768.0;
    
    //beta
    //temp2 = tempv32 / 32768.0;
    //temp2 = (temp1 + temp2 * (float)2.0f) * (float)0.577351f;
    temp2 = tempv32 / 16384.0;
    temp2 = (temp1 + temp2) * (float)0.577351f;
    
    Sine = sin(Angle);
    Cosine = cos(Angle);
    
    //DS
	temp3 = temp1 * Sine + temp2 * Cosine;
	//Qs
	temp4 = temp2 * Sine - temp1 * Cosine;
    
    IqPiReg.Fdb = temp3;
    IdPiReg.Fdb = temp4;
    if(StateMachine.RegulFlg == 1)
    {
         //PID_Q
        IqPiReg.Ref = (float32_t)QsRefCpuToAsic / 16777216.0f;
		
		// Compute the error
		IqPiReg.Err = IqPiReg.Ref - IqPiReg.Fdb;

		// Compute the proportional output
		IqPiReg.Up = IqPiReg.Kp * IqPiReg.Err;

		if(IqPiReg.PiSatFlg == 0)//说明之前调节器未饱和
		{
			IqPiReg.Ui += IqPiReg.Ki * IqPiReg.Err;
		}
		else if(IqPiReg.PiSatFlg == 1)//说明之前调节器正向饱和
		{
			if(IqPiReg.Err < 0)//误差小于0才积分
				IqPiReg.Ui += IqPiReg.Ki * IqPiReg.Err;
		}
		else//说明之前调节器负向饱和
		{
			if(IqPiReg.Err > 0)//误差大于0才积分
				IqPiReg.Ui += IqPiReg.Ki * IqPiReg.Err;
		}

		IqPiReg.OutPreSat = IqPiReg.Up + IqPiReg.Ui;

		if(IqPiReg.OutPreSat < IqPiReg.OutMin)
		{
			IqPiReg.Out = IqPiReg.OutMin;
			IqPiReg.PiSatFlg = 2;//负向饱和
		}
		else if(IqPiReg.OutPreSat > IqPiReg.OutMax)
		{
			IqPiReg.Out = IqPiReg.OutMax;
			IqPiReg.PiSatFlg = 1;//正向饱和
		}
		else
		{
			IqPiReg.Out = IqPiReg.OutPreSat;
			IqPiReg.PiSatFlg = 0;
		}
		temp1 = IqPiReg.Out;

		//PID_D
		IdPiReg.Ref = (float32_t)DsRefCpuToAsic / 16777216.0f;
        
		// Compute the error
		IdPiReg.Err = IdPiReg.Ref - IdPiReg.Fdb;

		// Compute the proportional output
		IdPiReg.Up = IdPiReg.Kp * IdPiReg.Err;

		if(IdPiReg.PiSatFlg == 0)//说明之前调节器未饱和
		{
		    IdPiReg.Ui += IdPiReg.Ki * IdPiReg.Err;
		}
		else if(IdPiReg.PiSatFlg == 1)//说明之前调节器正向饱和
		{
		    if(IdPiReg.Err < 0)//误差小于0才积分
			    IdPiReg.Ui += IdPiReg.Ki * IdPiReg.Err;
		}
		else//说明之前调节器负向饱和
		{
		    if(IdPiReg.Err > 0)//误差大于0才积分
			    IdPiReg.Ui += IdPiReg.Ki * IdPiReg.Err;
		}

		IdPiReg.OutPreSat = IdPiReg.Up + IdPiReg.Ui;

		if(IdPiReg.OutPreSat < IdPiReg.OutMin)
		{
		    IdPiReg.Out = IdPiReg.OutMin;
		    IdPiReg.PiSatFlg = 2;//负向饱和
		}
		else if(IdPiReg.OutPreSat > IdPiReg.OutMax)
		{
		    IdPiReg.Out = IdPiReg.OutMax;
		    IdPiReg.PiSatFlg = 1;//正向饱和
		}
		else
		{
		    IdPiReg.Out = IdPiReg.OutPreSat;
		    IdPiReg.PiSatFlg = 0;
		}
		temp2 = IdPiReg.Out;

		//D 轴交叉解耦补偿
		tempu32 = SpdWfbCpuToAsic;
		temp3 = -Ds_Fbd * (float)tempu32 / 16777216.0f;
		temp3 = temp3 * UFwdCoeff1Cla;
		temp3 = _IQsat(temp3,0.577350f,-0.577350f);

		//Q 轴交叉解耦补偿
		temp4 = 0;
		//Q 轴反电动势
		temp4 = temp4 + UFwdCoeff2Cla * (float)tempu32 / 16777216.0f;
		temp4 = _IQsat(temp4,0.577350f,-0.577350f);

		temp1 = temp1 + temp4;
		temp2 = temp2 + temp3;

		//过调制
		temp3 = temp1 * temp1 + temp2 * temp2;
		temp3 = sqrtf(temp3);
        if(temp3 > 0.577350f)
        {
            temp4 = 0.577350f / temp3;
            temp1 = temp1 * temp4;
            temp2 = temp2 * temp4;
        }

		//IPARK
		temp3 = temp1 * Cosine + temp2 * Sine; //Alpha
		temp4 = temp2 * Cosine - temp1 * Sine; //Beta

		//电压补偿
		temp1 = MYDIVF32(ADCBuf[10], 4096.0f);
		UdcAsicToCpu = (long)(temp1 * 16777216.0f);
		temp1 = temp1 / UDCEuCla;
		temp3 = temp3 * temp1;
		temp4 = temp4 * temp1;

		//SVPWM
		temp1 = temp3 * 0.5f;
		temp2 = temp4 * 0.8660253882f;

		temp3 = temp1;           //U
		temp4 = -temp1 + temp2;  //V
		temp5 = -temp1 - temp2;  //W

		temp1 = temp3>temp4?(temp3>temp5?temp3:temp5):(temp4>temp5?temp4:temp5);//MAX
		temp2 = temp3<temp4?(temp3<temp5?temp3:temp5):(temp4<temp5?temp4:temp5);//MIN
		temp1 = (temp1 + temp2) * 0.5f;
		temp3 = 0.5f + (temp3 - temp1);
		temp4 = 0.5f + (temp4 - temp1);
		temp5 = 0.5f + (temp5 - temp1);

		//死区补偿

		//PWM赋值
		temp1 = 1.0f - temp3;
		temp1 = _IQsat(temp1,1.0f,0.0f);
		PwmDuty = (uint16_t)(temp1 * (float32)EpwmPrdCpuToAsic);
		//HWREGH(0x0000306BU) = 20;

		temp1 = 1.0f - temp4;
		temp1 = _IQsat(temp1,1.0f,0.0f);
		PwmDuty = (uint16_t)(temp1 * (float32)EpwmPrdCpuToAsic);
		//HWREGH(0x0000326BU) = 20;

		temp1 = 1.0f - temp5;
		temp1 = _IQsat(temp1,1.0f,0.0f);
		PwmDuty = (uint16_t)(temp1 * (float32)EpwmPrdCpuToAsic);
		//HWREGH(0x0000346BU) = 20;
     }
    
    
}

*/



//void CurResponseTest(void)
//{
//	if(DPI_CURSTEPTEST > 0)
//	{
//		UFwd.Sw = 0;//关闭电压补偿
//		StepGen.RefLow = _IQmpy(DrvCoeff.MotIe,_IQ(0.3));
//		StepGen.RefHigh = _IQmpy(DrvCoeff.MotIe,_IQ(1.5));
//		StepGen.Const = 0;
//		StepGen.Times = 900L;
//		StepGen.calc(&StepGen);
//		IdRef.Ref = StepGen.Ref;
//		IqRef.IqRef = 0;
//		Etheta.EtaGetSwt = 1;//电角度内部强制给定
//		Etheta.EtaInerGet = 0;//强制拉到d轴
//	}
//}


    /*
    if((IqRef.IqRef != 0  || IdRef.Ref != 0 || RxStartFlag == 1) && RxEndFlg == 0)
    {
        RxStartFlag = 1;
        testiuf[Rxcnt]   = (int16)((int64)IqPiReg.Ref * CurMonitor.CoeffIQ16  >> 24);
        testivf[Rxcnt]   = (int16)((int64)IqPiReg.Fdb * CurMonitor.CoeffIQ16  >> 24);
        testidf[Rxcnt]   = (int16)((int64)IdPiReg.Fdb * CurMonitor.CoeffIQ16  >> 24);
        testiwf[Rxcnt++] = (int16)(IqPiReg.Err >> 9);
        
        if(Rxcnt == 1024)
        {
            RxEndFlg = 1;
            RxStartFlag = 0;
            Rxcnt = 0;
        }
    }
    */






