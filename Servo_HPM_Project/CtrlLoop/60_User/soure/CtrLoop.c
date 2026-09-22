/*
 * CtrLoop.c//
 *
 *  Created on: 2015-12-23
 *      Author: w
 */
#include "Drive.h"
#include "StartupTiming.h"
#include "SV_PanelCtl.h"
#include "SV_FuncVar.h"
#include "SV_IncEncode.h"
#include "userdefine.h"

#include "SV_MagEncode.h"
#include "Pos_FullClosedLoop.h"
#include "SV_CanbusCtrl.h"
#include "SV_ModbusConTrolVar.h"
#include "TorClosedCtrLoop.h"
#include "s_tsensor_init.h"//
#include "SV_KingKongEnc.h"
#include "SV_RtT036.h"
#include "Mit_CtrLoop.h"
#include "SensorlessProduction.h"
#include "SensorlessShadow.h"
#include "SensorlessCanopen.h"

extern float ActualTorqueClose;
extern float TargetTorqueClose;
extern volatile uint8_t EcatSnycCmdFlag;
extern volatile uint8_t EcatSnycFlag;
extern volatile uint8_t CanxPosFlag;
extern void CanSyncCalibCacl(void);
void ScopeCacl(void);

void CtrLoop_Interface(CTRLOOP_TYPE *v)//Code in 1ms timer isr
{

}

void CtrLoop_init(CTRLOOP_TYPE *v)//Code in system init,各个控制模块初始化
{
//位置环路初始化
	CtrLoop.PosLoopCnt = 0;//
	PosRef.init(&PosRef);//位置环路位置给定模块初始化
	PosCmxCdv.init(&PosCmxCdv);//电子齿轮模块初始化
	PosSmooth.init(&PosSmooth);//位置指令一阶低通模块初始化
	PosFirflt.init(&PosFirflt);//位置指令FIR滤波模块初始化
	PosDamping.init(&PosDamping);//位置指令减震模块初始化
	PosFb.Tsamp = DrvCoeff.PosTsamp;//位置环路调度周期，单位0.01us
	PosFb.init(&PosFb);//位置环路位置反馈计算模块初始化
	SpdGainAdpt.init(&SpdGainAdpt);//增益自整定计算模块初始化
	PosReg.init(&PosReg);//位置调节器模块初始化，包括速度前馈模块
	CtrLoop.PosLoopTimes = DrvCoeff.PosLoopDivCnt;//位置环路调度次数
	PosMonitor.init(&PosMonitor);//位置环监控模块初始化，或者叫位置环路接口函数
	RampKPp.init(&RampKPp);//位置增益切换时斜坡给定模块初始化
	MultiPos.init(&MultiPos);//内部多段位置初始化
//速度环路初始化
	Spd_Ref.init(&Spd_Ref);//速度环路速度指令给定模块初始化
	CtrLoop.SpdLoopTimes = DrvCoeff.SpdLoopDivCnt;//速度环路调度次数
	Spd_PiReg.init(&Spd_PiReg);//速度调节器模块初始化
	SpdPosFb.Tsamp = DrvCoeff.SpdTsamp;//速度环路调度周期单位0.01us
	//SpdPosFb.Tsamp = DrvCoeff.Tpwm;
	SpdPosFb.init(&SpdPosFb);//速度环路为测速设置的位置反馈模块初始化
	SpdFb.Tao = DPI_T_FdSpdLpf;//测速滤波器一阶滤波时间常数，单位us
	SpdFb.Tsamp = DrvCoeff.SpdTsamp;//速度环路调度周期单位0.01us
	//SpdFb.Tsamp = DrvCoeff.Tpwm;
	SpdFb.init(&SpdFb);//测速模块初始化
	InstSpdObser.Tsamp = DrvCoeff.SpdTsampPu;//速度调度时间
	InstSpdObser.init(&InstSpdObser);//速度观测器初始化
	//InstSpdObser.mask = 1;//速度观测器打开
	SpdMonitor.init(&SpdMonitor);//速度监控模块初始化
	SpdScurve.TSpdAcc = DPI_T_SpdAcc;//速度模式时加速时间，单位ms
	SpdScurve.TSpdDec = DPI_T_SpdDec;//速度模式时减速时间，单位ms
	SpdScurve.TSpdAccDecLpf = DPI_T_SpdAccDecLpf;//速度模式时圆弧时间，单位ms
	SpdScurve.TZstop = DPI_T_ZeroStop;//零速停车时间，单位ms
	SpdScurve.TNPot = DPI_T_Overtravel;//超程保护停车时间，单位ms
	SpdScurve.Tjog = DPI_JOGTAccDec;
	SpdScurve.Tsamp = DrvCoeff.SpdTsamp;//速度环路调度周期单位0.01us
	SpdScurve.init(&SpdScurve);//速度加减速模块初始化
	MultiSpd.init(&MultiSpd);//内部多段速度模块初始化
	RampKp.init(&RampKp);//速度增益切换斜坡给定模块初始化
	RampKi.init(&RampKi);//速度积分切换斜坡给定模块初始化
	AccFwd.init(&AccFwd);//加速度前馈模块初始化
	OffLineJidt.init(&OffLineJidt);//三角波形式离线惯量辨识模块初始化
	OnLineJidt.Ib = DrvCoeff.DrvIbase;//在线惯量辨识需要的基值电流，单位0.01A
	OnLineJidt.Kt = DrvCoeff.Kt;//在线惯量辨识需要的转矩系数，单位0.01N.M/A
	OnLineJidt.gama = _IQ(0.01);//在线惯量辨识自适应增益
	OnLineJidt.nb = DrvCoeff.SpdBase;//在线惯量辨识需要的速度基值
	OnLineJidt.ts = _IQmpy(_IQ(2.0),DrvCoeff.SpdTsampPu);//在线惯量辨识调度周期
	OnLineJidt.AccLimt = 5;//在线惯量辨识限制的加速度，每个辨识周期内速度的改变量单位rpm
	OnLineJidt.init(&OnLineJidt);//在线惯量辨识模块初始化
//电流环路初始化
	Etheta.ZoffsetAngle = DPI_ZeroElecAngle;//Z信号处的偏移电角度
	//Etheta.UVWoffsetAngle = DPI_UElecAngle;//U相处的偏移电角度，只对普通带UVW信号的增量型编码器有用
	Etheta.UVWoffsetAngle = Hall.OffsetAngle;
	Etheta.init(&Etheta);//电角度计算模块初始化
	IdRef.init(&IdRef);//D轴电流给定模块初始化
    
	//IqRef.Tsamp = DrvCoeff.Tpwm;    //电流环调度周期
    IqRef.Tsamp = DrvCoeff.SpdTsamp;  //速度环调度周期 0.01us
	IqRef.init(&IqRef);//Q轴电流给定模块初始化
	IdPiReg.init(&IdPiReg);//D轴电流调节器初始化
	IqPiReg.init(&IqPiReg);//Q轴电流调节器初始化
	Svpwm.init(&Svpwm);
	SinGen.init(&SinGen);//正弦波生成模块，做为自测用
	SinGen1.init(&SinGen1);//电机参数自学习中使用的正弦波生成模块
	SinGen2.init(&SinGen2);//速度环频率响应测试使用的正弦波生成模块
    Cur_Monitor_init(&CurMonitor);

	SinGentst.init(&SinGentst);//自测用
	SinGentst.AngleDelta = _IQ30div(100L*DrvCoeff.Tpwm,100000000L);//100Hz，原始频率
	SinGentst.Amp = _IQ(0.5);
	SinGentst1.init(&SinGentst1);//自测用
	SinGentst1.AngleDelta = _IQ30div(600L*DrvCoeff.Tpwm,100000000L);//600Hz，干扰频率
	SinGentst1.Amp = _IQ(0.5);

	SpdResponseTestUpdateInit();//速度环频率响应测试模块初始化
	MotPaLearn.init(&MotPaLearn);//电机参数自学习模块初始化
	UFwd.init(&UFwd);//D和Q轴电压解耦模块初始化
	StepGen.init(&StepGen);//电流环阶跃相应测试使用的阶跃给定模块初始化
    NotchFilter1.init(&NotchFilter1);
	ftvftst_init();//简单vf初始化
	Frctoqcps.init(&Frctoqcps);//摩擦力补偿
	Zcheck.Tsamp = DrvCoeff.Tpwm;
	Zcheck.init(&Zcheck);//z信号检测
//外环初始化
    
    ExPosFb_Init();
    PosFullClose_Init();
    CurFullPiReg_init(&CurFullReg);
    SensorlessShadow_Init();
    SensorlessProduction_Init();
    SensorlessCanopen_Init();
    
    #if SERVOTYPE == SERVO_MODBUS
    CM_busCtrl_init(&CM_BusCtrl);
    #endif
    
    #if SERVOTYPE == SERVO_CAN
    CM_busCtrl_init(&CM_BusCtrl);
    #endif  
    
    #if SERVOTYPE == SERVO_ETHERCAT
    Mit_Init();  
    #endif        
}

void CtrLoop_rst(CTRLOOP_TYPE *v)//Code in Servo Stop,1ms timer isr
{
    (void)v;
    SensorlessShadow_Reset();
}

#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
void Updata_EcatCmd(void)
{
    PosRef.PosNow      = Cia402_PP.PPBuffer0.TargetPostion;
    Spd_Ref.EcatIn     = Cia402_PP.PPBuffer0.TargetVelo;
    IqRef.IqEcatCmd    = (int16)Cia402_PP.PPBuffer0.TargetTorque;
    PosRef.VelOffset   = Cia402_PP.PPBuffer0.VelOffset60B1;
    IqRef.IqEcatOffset = Cia402_PP.PPBuffer0.TorqueOffset60B2; 
    TargetTorqueClose  = Cia402_PP.PPBuffer0.TargetTorqueClose; 

    PosRef.SyncFlag = 1;     //赋值结束才能置标志位，否则一出现DC抖动，位置不准确 2025.06.24

    if(LocalAxes.Objects.objModesOfOperation == (int8)ETHERCATSTATE_MIT)
    {
        Mit_PdoReceive();
    }
}

void Updata_EcatFb(void)
{
    if(FullCloseLoop.FullFlag == 1)
    {
        Cia402_PP.GetPos6063 = ExtPosFb.PosFbTotle;
        Cia402_PP.GetVel606C = (ExtPosFb.SpdMode == 2) ? ExtPosFb.SpdFbOut : SpdFb.Fb; 
    }
    else
    {
        Cia402_PP.GetPos6063 = PosFb.PosFbTotle;
        Cia402_PP.GetVel606C = SpdFb.Fb;  
    }
          
    Cia402_PP.GetTor6077 = CurMonitor.TorqRatsDispS;
    
    Cia402_PP.PosDemVal60FC = PosRef.PosNow;         
    Cia402_PP.VelDemVal606B = Spd_Ref.Ref;           
    Cia402_PP.TorDemVal6074 = CurMonitor.TorqRatsRef;
    Cia402_PP.GetTorqueClose = ActualTorqueClose;

    if(LocalAxes.Objects.objModesOfOperation == (int8)ETHERCATSTATE_MIT)
    {
        Mit_PdoSend();
    }
}
#endif

void CtrLoop_CalcFirst(void)//Code in PWM ISR
{
    /* Dedicated sensorless test firmware: no legacy outer-loop ownership. */
    uint32_t timing_mark=StartupTiming_Cycle();
    SensorlessCanopen_PwmGuard();
    SensorlessShadow_PreFoc();
    Etheta.Ethetapk = Etheta.EtaInerGet;
    g_startup_timing.pre=StartupTiming_Cycle()-timing_mark;
    timing_mark=StartupTiming_Cycle();
    ToqLoop_Calc();
    g_startup_timing.torque=StartupTiming_Cycle()-timing_mark;
    timing_mark=StartupTiming_Cycle();
    SensorlessShadow_PostFoc();
    g_startup_timing.post=StartupTiming_Cycle()-timing_mark;
    return;
    //第一编码器
    if(DPI_EncType == 2 || DPI_EncType == 3)
    {
        DPT_Process();
    }
    else if(DPI_EncType == 4 || DPI_EncType == 5)
    {
        T_FormatProcess();
    }
    else if(DPI_EncType == 8 || DPI_EncType == 9)
    {
        EccodeBissPosCacl_First(); 
    }
    else if(DPI_EncType == 12 || DPI_EncType == 13)
    {
        RT_Process(); 
    }
    else
    {
        
    }
    
    ////第二编码器
    //if(RamPD->ExEncType > 0)     
    //{
    //    if(MagReg.EncTxflg == 1)
    //    {
    //        MagEncodePosRead_Second(); //接收 
    //        MagReg.EncTxflg = 0;
    //    }
        
    //    if(CtrLoop.PosLoopCnt == MagReg.BissTxNum)
    //    {
    //        MagEncodePosReq_Second();  //发送 
    //        MagReg.EncTxflg = 1;
    //    }
    //}

    //解析编码器信息得到单圈位置值
    EncPosFb_Calc();	   // 48/150

    #if SERVOTYPE == SERVO_ETHERCAT
    if((++CtrLoop.PosLoopCnt == CtrLoop.PosLoopTimes) || (EcatSnycFlag == 1) )      
    { 
        PosLoop_Calc(); 
        CtrLoop.PosLoopCnt = 0;
    }
    #endif

    #if SERVOTYPE == SERVO_MODBUS 
    if(++CtrLoop.PosLoopCnt == CtrLoop.PosLoopTimes) 
    {
        PosLoop_Calc(); 
        CtrLoop.PosLoopCnt = 0;
    }
    #endif 

    #if (SERVOTYPE == SERVO_CAN) || (SERVOTYPE == SERVO_CANOPEN) 
    CtrLoop.PosLoopCnt++;
    if(CanxPosFlag == 1)
    { 
        PosLoop_Calc(); 
        CanSyncCalibCacl();
        CtrLoop.PosLoopCnt = 0;
        CanxPosFlag = 0;
    }
    #endif 
    
    #if SERVOTYPE == SERVO_ETHERCAT
    if((++CtrLoop.SpdLoopCnt == CtrLoop.SpdLoopTimes) || (EcatSnycFlag == 1))
    #else
    if(++CtrLoop.SpdLoopCnt == CtrLoop.SpdLoopTimes)        
    #endif
    {
        SpdLoop_Calc();
        CtrLoop.SpdLoopCnt = 0;
    }
    
#if SERVOTYPE == SERVO_ETHERCAT
    EcatSnycFlag = 0;
#endif
    
    //计算电角度，Park和ipark使用		
    Etheta.Pos = EncPosFb.EncSinglePos;
    Etheta.calc(&Etheta);  //97/150

	ToqLoop_Calc();// 1800/150   1400/150(增量式编码器)

    #if SERVOTYPE == SERVO_ETHERCAT
    if(LocalAxes.Objects.objModesOfOperation == (int8)ETHERCATSTATE_MIT)
    {
        Mit_FbCacl(PosFb.PosFbTotle,SpdFb.Fb,CurMonitor.TorqRatsDispS,StateMachine.RegulFlg);
    }
    #endif
}

void CtrLoop_CalcSecond(void)//Code in PWM ISR
{
    uint32_t timing_mark=StartupTiming_Cycle();
    SensorlessCanopen_PwmGuard();
    SensorlessShadow_PreFoc();
    Etheta.Ethetapk = Etheta.EtaInerGet;
    g_startup_timing.pre=StartupTiming_Cycle()-timing_mark;
    timing_mark=StartupTiming_Cycle();
    ToqLoop_Calc();
    g_startup_timing.torque=StartupTiming_Cycle()-timing_mark;
    timing_mark=StartupTiming_Cycle();
    SensorlessShadow_PostFoc();
    g_startup_timing.post=StartupTiming_Cycle()-timing_mark;
}











//    if(VIC.RAIS4.LONG & 0x00040000UL)
//    {
//        PosLoop_Calc(); 
//        VIC.PIC4.LONG = 0x00040000UL;
//    }
//    
//    if(VIC.RAIS4.LONG & 0x00300000UL)
//    {
//        SpdLoop_Calc();
//        VIC.PIC4.LONG = 0x00300000UL;
//    }

//    {//有外部放电
//    #if (SERVO_HARDWARE == HARDWARE_DC0)
//        int16 temp = 0;
//        
//        temp = (ADCBuf[10] + ADCBuf[11]) >> 1;
//    
//        if(temp > 2713)  
//            PWM_BRAKE_HIGH;
//        else if(temp < 2700)
//            PWM_BRAKE_LOW;  
//    #endif  
//    }
    
//    //OnLineJidtLoop:在线惯量辨识
//	if(++CtrLoop.JidtLoopCnt >= 2*CtrLoop.SpdLoopTimes)
//	{
//		CtrLoop.JidtLoopCnt = 0;
//		OnLineJidt.te_new = IqPiReg.Ref;
//		OnLineJidt.w_new = SpdFb.Fb;
//		OnLineJidt.Spd = SpdMonitor.SpdAvrg;
//		OnLineJidt.RegSatFlg = Spd_PiReg.Flg.bit.PiSatFlg;
//		OnLineJidt.StFlg = StateMachine.RegulFlg;
//		OnLineJidt.calc(&OnLineJidt);
//	}