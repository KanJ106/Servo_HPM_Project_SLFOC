#include "userdefine.h"
#include "SensorlessCanopen.h"
#include "s_encode_init.h"
#include "r_cg_Project.h"
#include "SV_PanelCtl.h"
#include "SV_I2c.h"
#include "SV_AbsEncode.h"
#include "SV_StateMachine.h"////
#include "SV_FaultProtect.h"
#include "SV_OLProtect.h"
#include "SV_Sci.h"
#include "SV_UdcCtrl.h"
#include "SV_Di.h"
#include "SV_Do.h"
#include "SV_CtrlMode.h"
#include "SV_Global.h"
#include "SV_Fault.h"
#include "SV_UVWShortProtect.h"
#include "SV_FunCodeAttrLit.h"
#include "SV_Fanctl.h"
#include "SV_Temperctl.h"
#include "SV_OTProtect.h"
#include "SV_Factorytest.h"
#include "SV_RunTime.h"
#include "SV_IncEncode.h"
#include "SV_Scope.h"
#include "s_sys_init.h"
#include "SV_CanFirm.h"
#include "SV_KingKongEnc.h"
#include "SV_RtT036.h"

extern void Monitor_1ms(void);

Uint16 SysFtest = 0;
Uint16 Time1msMainLoopFlag = 0;
void EncodeErrPeocess(void);
void AppTime1Ms(void)
{   
    //ServiceDog();
    SensorlessCanopen_Tick1ms();
    IIC_Process();				//IIC存储类 3.21us

	if(PARAREDY == Glo_IIcReadAll)	//参数未初始化完毕不执行应用程序
	{
        Time1msMainLoopFlag = 1;
		if(SysFtest)
		{
			FactortTest();	//工厂自检
			return;
		}
        
        Pos_Monitor1ms_Calc();
        Spd_Monitor1ms_Cacl();          //电流环移出到此处
        CurMonitor.monit(&CurMonitor);  //电流有关的监控计算
        
        #if (SERVO_HARDWARE == HARDWARE_AC0)
        DI_Handle();
        #endif
        
        //#if SERVOTYPE == SERVO_ETHERCAT
        //if(bRunApplication == TRUE)
        //{
        //    ECAT_CheckTimer();        //1ms
        //    //CheckIfEcatError();
        //    CiA402_StateMachine();
        //}
        //#endif
        
        #if SERVOTYPE == SERVO_CAN 
        CanSyncCheck();
        #endif
            
        StateMachine_Process();		            //状态机处理抱闸处理
        
        CtrlModeSwitch();			            //控制模式处理 
        SM_ParaRefreshFixtime();
        //时序要求高处理放到以上

		//PanelCtl();                           //键盘显示及操作
        
		UdcCtrl_Schedule();			//母线电压控制
		Fault_DrvOLProtect();		//驱动器过载保护
		Fault_MotOLProtect();		//电机过载保护
        Fault_MotorLockedProtect(); //电机堵转保护
        RunAwayMointor();           //飞车保护
        #if SERVOPOWER == SERVO_AC
		Fault_BrakeResProtect();	//制动电阻过载保护
        #endif

#if SERVOPOWER == SERVO_AC	//只有交流伺服判断主电源，低压伺服只有一个电源不需判断
		Fault_Check();
#else
        FaultJudge.PowOffFlag = 0;
#endif

		Fault_OTProtect();		    //超程保护
		FaultPrtt_Schedule();		//故障保护调度处理
        
		FAN_Ctl();					//风扇控制
		IGBTTemperCtl();			//IGBT温度计算
        MortorTemperCtl();          //电机温度计算
        MCUTemperCtl();             //MCU温度计算
		Monitor_1ms();
		TS_Store_10Min();
        SensorlessCanopen_Service1ms();
#if defined(SENSORLESS_CANOPEN_BUILD)
        SensorlessEncoder_Service1ms(StateMachine.RegulFlg);
#endif
             
        #if (SERVO_HARDWARE == HARDWARE_AC0)
		DO_Handle();
        #endif 
	}
    else if(2 == Glo_IIcReadAll)
    {
        Time1msMainLoopFlag = 2;    
    }
    else
    {}
}

void AppTime1msMainLoop(void)
{
    while(UartBootFlag)
    {
        if (gptmr_check_status(BOARD_CALLBACK_TIMER, GPTMR_CH_RLD_STAT_MASK(BOARD_CALLBACK_TIMER_CH)))//判断1ms定时
        {
            SCI_Process(SCI_CHA);		//485通讯 
            ServiceDog();
            gptmr_clear_status(BOARD_CALLBACK_TIMER, GPTMR_CH_RLD_STAT_MASK(BOARD_CALLBACK_TIMER_CH));
        }
    }

    if(Time1msMainLoopFlag == 0)
    {
        return;
    }
    Time1msMainLoopFlag = 0;
    
    EncodeErrPeocess();
    
    PanelCtl();                         //键盘显示及操作

    if((1 == RamAid->SoftReset) && (I2cBusState == IIC_BUS_IDLE))	//软件复位
    {
        RamAid->SoftReset = 0;
        ResetCPU();  
    }    
    
    SCI_Process(SCI_CHA);		//485通讯 
    
    Scope_Process();
    
    Panel_mainloop();
    FuncLitProc_sch();
}

void EncodeErrPeocess(void)
{
    /* No encoder acquisition in this sensorless-only candidate. */
    return;
    //故障保护类
    if(!RamServo->VF_Test)
    {
        if((RamMotor->EncType == 2) || (RamMotor->EncType == 3))
        {
            DPT_FaultHandle();
        }
        else if((RamMotor->EncType == 4) || (RamMotor->EncType == 5))
        {
            EncFaultHandle();			//编码器相关故障
        }
        else if((RamMotor->EncType == 12) || (RamMotor->EncType == 13))
        {
           RT_FaultHandle();
        }
        else
        {
                            // 增量式编码器断线检测
        }
    }  
}


////零速判断
//#define ZeroSpdDlyDefine	(100)	//100ms
//#define ZeroSpdThre			(2^10)	//一圈2^17脉冲  TODO
//#define ZeroSpdThreSpd		(5)	//5rpm
//Uint16 ZeroSpdCnt = 0;
//Uint16 ZeroSpdJudge(void)	//指令为0，当前转速为0
//{
//	Uint16 ret = 0;
//
//#if SERVOTYPE == SERVO_ETHERCAT
//	switch(CtrlMode.ECATCtrlMode)
//	{
//		case ETHERCATSTATE_VL:
//		case ETHERCATSTATE_PV:
//		case ETHERCATSTATE_CSV:
//			if((Cia402_PP.PPBuffer0.TargetVelo == 0) && (SpdMonitor.SpdAvrg < ZeroSpdThreSpd))
//			{
//				ret = 1;
//			}
//			break;
//		case ETHERCATSTATE_PP:	//速度模式 SpdRefSource_Var9
//		case ETHERCATSTATE_CSP:
//		case ETHERCATSTATE_IP:
//		    //if((abs(Cia402_PP.PPBuffer0.TargetPostion - LocalAxes.Objects.objPositionActualValue) <= ZeroSpdThre)
//#if OPENABSENCODE
//		    if((abs(Cia402_PP.PPBuffer0.TargetPostion - (LocalAxes.Objects.objPositionActualInterValue-Cia402_PP.PosFbInitValue)) <= ZeroSpdThre)//YH 2017.9.14
//#else
//		    if((abs(Cia402_PP.PPBuffer0.TargetPostion - LocalAxes.Objects.objPositionActualValue) <= ZeroSpdThre)
//#endif
//		        && (SpdMonitor.SpdAvrg < ZeroSpdThreSpd))
//			{
//				ret = 1;
//			}
//			break;
//		case ETHERCATSTATE_TQ:	//转矩模式 g_IqRefSource_var6
//		case ETHERCATSTATE_CST:
//			if((Cia402_PP.PPBuffer0.TargetTorque == 0)
//				&& (SpdMonitor.SpdAvrg < ZeroSpdThreSpd))
//			{
//				ret = 1;
//			}
//			break;
//		default://非402使能-包括home
//			if(SpdMonitor.SpdAvrg < ZeroSpdThreSpd)
//			{
//				ret = 1;
//			}
//			break;
//	}
//#elif SERVOTYPE == SERVO_PULSE
//	switch(CtrlMode.CtrlMode)
//	{
//		case CTRLMODE_SPD:	//速度模式 SpdRefSource_Var9
//			if((Spd_Ref.Ref == 0) && (SpdMonitor.SpdAvrg == 0))
//			{
//				ret = 1;
//			}
//			break;
//		case CTRLMODE_POS:	//位置模式 PosRefSource_var10
//		default:
//			if((PosReg.Pospd == 0) && (SpdMonitor.SpdAvrg == 0))
//			{
//				ret = 1;
//			}
//			break;
//		case CTRLMODE_TRQ:	//转矩模式 g_IqRefSource_var6
//			if((IqRef.Ref == 0) && (SpdMonitor.SpdAvrg == 0))
//			{
//				ret = 1;
//			}
//			break;
//	}
//#endif
//	if(StateMachine.RegulFlg == 0)
//	{
//		ret = 1;
//	}
//
//	if(ret)//零速判断滤波
//	{
//		if(ZeroSpdCnt < ZeroSpdDlyDefine)
//		{
//			ZeroSpdCnt ++;
//			ret = 0;
//		}
//	}
//	else
//	{
//		ZeroSpdCnt = 0;
//	}
//
//	return ret;
//}
