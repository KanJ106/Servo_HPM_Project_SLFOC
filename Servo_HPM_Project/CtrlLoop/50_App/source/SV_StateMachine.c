/*
 * SV_StateMachine.c
 *
 *  Created on: 2015-12-16
 *      Author: zhangwei////
 */
#include "userdefine.h"
#include "r_cg_Project.h"
#include "SV_StateMachine.h"
#include "SV_FuncVar.h"
#include "SV_FuncCode.h"
#include "Common_Lib.h"
#include "SV_FaultProtect.h"
#include "SV_UdcCtrl.h"
#include "Drive.h"
#include "SV_Di.h"
#include "SV_Do.h"
#include "cia402appl.h"
#include "SV_PanelCtl.h"
#include "SV_OTProtect.h"
#include "Pos_FullClosedLoop.h"
#include "SV_ModbusConTrolVar.h"
#include "SV_CanbusCtrl.h"
#include "TorClosedCtrLoop.h"
#include "Mit_CtrLoop.h"
#include "canopen_interface.h"
#ifdef SENSORLESS_CANOPEN_BUILD
#include "SensorlessCanopen.h"
#endif

Uint8 PwmBrkFlag = 0;

TYPE_STATEMACHINE StateMachine = {	\
/*	Ready;									*/{0x0003},	\
/*	CmdIn;									*/{0},	\
/*	PowerUpDly;								*/0,	\
/*	OutSRDY_F1;								*/0,	\
/*	SrvOnIn;								*/0,	\
/*	SrvOnStatus;							*/0,	\
/*	SrvOnStatusOld;							*/0,	\
/*	SrvStopCmd;								*/0,	\
/*	SrvOffCnt;								*/0,	\
/*	SrvOffCntBase;							*/0,	\
/*	InjectStep;								*/0,	\
/*	PWM_Charge;								*/0,	\
/*	ChargeTimer;							*/0,	\
/*	RegulFlg;								*/0,	\
/*	RegulFlgISR;							*/0,	\
/*	SvpwmOnFlg;								*/0,	\
/*	StopMode;								*/0,	\
/*	OffZSpdStopFlg;							*/0,	\
/*	HaltStopFlg;							*/0,	\
/*	BrakeStatus;							*/SM_BRAKESTATUS_BRAKE,	\
/*	SrvOnToLooseCnt;						*/0,	\
/*	LooseBrakeCnt;							*/0,	\
/*	SrvOffToBrakeCnt;						*/0,	\
/*	HoldBraekCnt;							*/0,	\
/*	BrakeChangePowerCnt;					*/0,	\
/*	OzSvStFlg;								*/0,	\
/*  OffDbFlg;                               */0,    \
/*  EStopFlag                               */0,    \
};

static void SM_ReadyJudge(void);
static void SM_CmdInCheck(void);
static void SM_SonCmdInChk(void);
static void SM_SrvBrakeCloseDeal(void);
static void SM_SrvStatusDeal(void);

static Uint16 SM_StopModeDeal(void);
void SM_ParaRefresh(void);				//状态参数更新
static void SM_ParaRefreshAnytime(void);//参数刷新任何时候
static void SM_ParaRefreshOff(void);	//参数刷新OFF

//----------------------------------------------------------------------------//
//---状态机处理---//
void StateMachine_Process(void)
{
//    #if SERVOTYPE == SERVO_ETHERCAT
//	EcatSM_1MsProcess();
//    #endif
//    
//    #if SERVOTYPE == SERVO_PULSE
//    PulseSM_Process();
//    #endif
    
	TYPE_STATEMACHINE *sm = &StateMachine;
    
    #if SERVOTYPE != SERVO_ETHERCAT
	if(sm->PowerUpDly < SM_POWERUPDELAY)
	{
		sm->PowerUpDly ++;
		return;
	}
    #endif
    
	//准备好检测
	SM_ReadyJudge();                             //准备判断
    
	if(sm->Ready.all == 0)	                     //运行准备好
	{
		SM_DO_SRDY_F1 = 1;
		//检测输入命令
		SM_SonCmdInChk();                        //sm->SrvOnIn = 1;	在该处处理
	}
	else	                                     //运行未准备好
	{
		SM_DO_SRDY_F1 = 0;
		sm->SrvOnIn = 0;	//输入命令清零
	}
    
	//抱闸处理
	SM_SrvBrakeCloseDeal();//抱闸处理
    
	//SRV状态转换
	SM_SrvStatusDeal();    //自举充电,停车方式处理    
    
}

static Uint16 SM_StopModeDeal(void)
{
	TYPE_STATEMACHINE *sm = &StateMachine;
    Uint16 stopmode = 0;
    
	if(sm->Ready.bit.ERR)
	{
		if(FAULT_DEAL_BIT(FaultP.ErrCode,FAULT_BIT_LEVEL) == FAULT_LEVEL1)//在1ms中进行处理是否时间太短
		{
			stopmode = 0;//1级故障，自由停车
		}
		else
		{
			stopmode = RamBasePara->ErrStopMode > 0 ? STOPMODEFAULT : 0;//2级故障，零速停车（？？？按停车方式停车）
		}
	}
	else
	{
		stopmode = RamBasePara->StopMode > 0 ? STOPMODEZERO : 0;//正常，按设定方式停车
	}
    
	return stopmode;
}

//static void SM_1MsProcess(void)
//{
//	TYPE_STATEMACHINE *sm = &StateMachine;
//
//	//准备好检测
//	SM_ReadyJudge();                             //准备判断
//    
//	if(sm->Ready.all == 0)	                         //运行准备好
//	{
//		SM_DO_SRDY_F1 = 1;
//		//检测输入命令
//		SM_SonCmdInChk();                        //sm->SrvOnIn = 1;	在该处处理
//	}
//	else	//运行未准备好
//	{
//		SM_DO_SRDY_F1 = 0;
//		sm->SrvOnIn = 0;	//输入命令清零
//	}
//    
//	//抱闸处理
//	SM_SrvBrakeCloseDeal();//抱闸处理
//    
//	//SRV状态转换
//	SM_SrvStatusDeal();    //自举充电,停车方式处理
//}


static void SM_ReadyJudge(void)
{
	TYPE_STATEMACHINE *sm = &StateMachine;
	sm->Ready.bit.ERR = (FaultP.FaultStatus & 0x0002)>>1;	//故障状态判断
	sm->Ready.bit.DC = (UdcCtrl.State != UDC_INIT ? 0 : 1);	//母线状态判断
}

static void SM_CmdInCheck()
{
#ifdef SENSORLESS_CANOPEN_BUILD
    /* Exclusive debug owner supplies only the command; normal ready, charge,
       brake sequencing and fault handling below remain authoritative. */
    if (SensorlessJlink_OwnsControl()) {
        StateMachine.CmdIn.all = 0;
        StateMachine.CmdIn.bit.INRUN = SensorlessJlink_EnableRequested();
        return;
    }
#endif
	TCiA402Axis*pLocalAxes = &LocalAxes;
	TYPE_STATEMACHINE *sm  = &StateMachine;

	if(Servo2Panel.Bit.JOG|Servo2Panel.Bit.FFt|Servo2Panel.Bit.StartLeanInertia|RamAid->SON|RamAid->JodRun|Servo2Panel.Bit.StartSelfLean)
	{
		//pLocalAxes->CmdSource = CMD_INTERNAL;//命令来自内部       2025.01.18屏蔽
	}
	else if(pLocalAxes->bAxisFunctionEnabled)
	{
		sm->CmdIn.bit.ECAT = pLocalAxes->bAxisFunctionEnabled;
		//pLocalAxes->CmdSource = CMD_REMOTE;//命令来自外部，ECAT   2025.01.18屏蔽      
	}
    else if(CM_BusCtrl.Run)
    {
        if(CM_BusCtrl.EstopFlag == 0)
        {
            sm->CmdIn.bit.Mbusrun = 1;
        }
        else
        {
            sm->CmdIn.bit.Mbusrun = 0;
        }
        
		//pLocalAxes->CmdSource = CMD_CMBUS;//命令来自外部，Modbus 2025.01.18屏蔽    
    }
    else if(ScopeCtrl->MortorSon)
    {
        sm->CmdIn.bit.Scoperun = 1;
    }
	else
	{
		//sm->CmdIn.bit.ECAT = 0;
        //pLocalAxes->CmdSource = CMD_NO;//命令来自内部
        //以上为修改前 2022.5.27   
            
        sm->CmdIn.bit.Scoperun = 0;
        sm->CmdIn.bit.ECAT = 0;
        sm->CmdIn.bit.Mbusrun = 0;
        if(sm->RegulFlg == 0) //停车时也需要命令来自哪里 2023.10.07
        {
		    //pLocalAxes->CmdSource = CMD_NO;//命令来自内部           2025.01.18屏蔽
        }
	}

	/*****************************2019.5.13修改前***********************************************/
//	sm->CmdIn.bit.JOG = Servo2Panel.Bit.JOG;                       //JOG
//	sm->CmdIn.bit.FFt = Servo2Panel.Bit.FFt;                       //FFT
//	sm->CmdIn.bit.Inertia = Servo2Panel.Bit.StartLeanInertia;      //惯量辨识
//	sm->CmdIn.bit.LRN = Servo2Panel.Bit.StartSelfLean;             //内部自学习
//	sm->CmdIn.bit.INRUN = RamAid->SON;			                  //P8-05 用于工厂自检
	/*****************************2019.5.13修改前***********************************************/

	/*****************************2019.5.13修改后***********************************************/
    #if SERVOTYPE == SERVO_CANOPEN
    if(co[0]->NMT->operatingState == CO_NMT_OPERATIONAL)
    #else
    if((nAlStatus & 0x0F) == STATE_OP)
    #endif
	//if(nAlStatus == STATE_OP)
	{
        sm->CmdIn.bit.JOG = 0;                       //JOG
        sm->CmdIn.bit.FFt = 0;                       //FFT
        sm->CmdIn.bit.Inertia =0;                    //惯量辨识
        sm->CmdIn.bit.LRN = 0;                       //内部自学习
        sm->CmdIn.bit.INRUN = 0;                     //P8-05 用于工厂自检

        Servo2Panel.Bit.JOG = 0;
        sm->CmdIn.bit.FFt = 0;
        Servo2Panel.Bit.StartLeanInertia = 0;
        Servo2Panel.Bit.StartSelfLean = 0;
        RamAid->SON = 0;
        sm->CmdIn.bit.Scoperun = 0;
	}
	else
	{
	    sm->CmdIn.bit.JOG = Servo2Panel.Bit.JOG | RamAid->JodRun;      //JOG
        sm->CmdIn.bit.FFt = Servo2Panel.Bit.FFt;                       //FFT
        sm->CmdIn.bit.Inertia = Servo2Panel.Bit.StartLeanInertia;      //惯量辨识
        sm->CmdIn.bit.LRN = Servo2Panel.Bit.StartSelfLean;             //内部自学习
        sm->CmdIn.bit.INRUN = RamAid->SON | sm->CmdIn.bit.Mbusrun;     //P8-05 用于工厂自检
	}
	/*****************************2019.5.13修改后***********************************************/
}

static void SM_SonCmdInChk(void)
{
	TYPE_STATEMACHINE *sm = &StateMachine;
	SM_CmdInCheck();
	if(SM_CMDIN)//ECAT给出运行命令
	{
//		if(sm->SrvOnStatusOld == 1)//已经处于使能状态
//		{
//			sm->SrvOnIn = 1;
//		}
//		else//未使能
//		{
//			sm->SrvOnIn = 1;	//启动标志有效
//		}
      
        sm->SrvOnIn = 1;	//启动标志有效
	}
	else
	{
		sm->SrvOnIn = 0;
	}
}

static void SM_SrvBrakeCloseDeal(void)
{
	TYPE_STATEMACHINE *sm = &StateMachine;
    uint16_t temp;

	//母线异常，直接抱闸
	if(sm->Ready.bit.DC == 1)
	{
        RamAid->BrakeLogic = 0;
		sm->SrvOnToLooseCnt = 0;		         //运行指令到达延时
		sm->LooseBrakeCnt = 0;                   //松闸延时
		sm->SrvOffToBrakeCnt = 0;		         //运行指令撤销延时
		sm->HoldBraekCnt = 0;			         //抱闸延时
		sm->BrakeStatus = SM_BRAKESTATUS_BRAKE;  //抱闸状态
		SM_BRAKECTRL_OUT = SM_BRAKECTRL_HOLD;
        DPI_SON = 0;           //2025.02.14
        PWMOutDisable();       //2025.10.31
        RamAid->JodRun = 0;
		sm->SrvOnStatus = 0;
        ScopeCtrl->MortorSon = 0;
	}
	else	//母线正常，状态调度
	{
		switch(sm->BrakeStatus)
		{
			case SM_BRAKESTATUS_BRAKE:	//抱闸状态：命令输入有效，且延时＞PF-44
				if(sm->SrvOnIn == 1)//命令给定有效
				{
					sm->SrvOnStatus = 1;
                    temp = (RamServo->SrvOnBrkDlyTime < 50) ? 50 : RamServo->SrvOnBrkDlyTime;
					if(++sm->SrvOnToLooseCnt > temp)//PrF.44  50ms
					{
						sm->SrvOnToLooseCnt = 0;
						sm->BrakeStatus = SM_BRAKESTATUS_2LOOSE;//准备松闸
					}
				}
				else	//S-ON命令撤销后立马停机（抱闸）
				{
					//sm->RegulFlg = 0;     //2022.1.7
					sm->SvpwmOnFlg = 0;
					sm->SrvOnStatus = 0;
                    LocalAxes.u16PendingOptionCode = 0;
					if(sm->RegulFlg == 1)	//调节器未关
					{
						PWMOutDisable();
                        sm->RegilFlgISR = 0;
						sm->RegulFlg = 0;		//调节器关闭
					    sm->OffZSpdStopFlg = 0;	//伺服OFF零速停车无效
					    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
					    LocalAxes.OffStop_Other = 0;	//伺服OFF零速停车无效
					    #endif
					}
					sm->HoldBraekCnt = 0;
					sm->SrvOnToLooseCnt = 0;
				}               
				break;
			case SM_BRAKESTATUS_2LOOSE://释放抱闸过程：延时＞P1-54
				SM_BRAKECTRL_OUT = SM_BRAKECTRL_LOOSE;//DO有效            
  
				if(++sm->LooseBrakeCnt > (RamBasePara->LooseBrakeDelay))//延时P1-54时间 200 ms才可以接收运动命令
				{
					sm->LooseBrakeCnt = 0;
					sm->BrakeStatus = SM_BRAKESTATUS_LOOSE;
				}
				else if(sm->SrvOnIn == 0)	//释放抱闸过程中，若S-ON命令撤销，立马抱闸
				{
					sm->SrvOnStatus = 0;
					sm->LooseBrakeCnt = 0;
					sm->BrakeStatus = SM_BRAKESTATUS_2BRAKE;
				}
				break;
			case SM_BRAKESTATUS_LOOSE://松闸状态：命令输入无效，延时＞P1-57或者速度≤P1-56
                //if((sm->SrvOnIn == 0) || (sm->SrvOnStatus == 0))//即使S-ON信号有效，仍然先抱闸
                if(sm->SrvOnStatusOld == 0)//即使S-ON信号有效，仍然先抱闸
				{
//					//SrvOFF瞬间电机静止-速度≤24rpm
//					if((sm->SrvOnStatus == 1) && (SM_GLOSPD <= STOPSPEED))
//					{
//						sm->SrvOffToBrakeCnt = 0;
//						sm->BrakeStatus = SM_BRAKESTATUS_2BRAKE;//转换到准备抱闸状态
//					}
                    if(sm->OffZSpdStopFlg > 0)  //Zero spd stop
                    {
                        if((SM_GLOSPD < STOPSPEED))
                        {
                            sm->SrvOffToBrakeCnt = 0;
						    sm->BrakeStatus = SM_BRAKESTATUS_2BRAKE;//转换到准备抱闸状态 
                        }
                    }
					//SrvOFF时旋转-要求P1-57＞当前减速时间，都这会旋转中抱闸
    	    		else if((++sm->SrvOffToBrakeCnt > (RamBasePara->SrvOffToBrakeDealy)) || \
							(SM_GLOSPD <= RamBasePara->SrvOffToBrakeSpd) )
					{
						sm->SrvOffToBrakeCnt = 0;
						sm->BrakeStatus = SM_BRAKESTATUS_2BRAKE;//转换到准备抱闸状态
					}
					sm->SrvOnStatus = 0;
				}
                sm->SrvOnStatus = sm->SrvOnIn;
				break;
			case SM_BRAKESTATUS_2BRAKE://抱闸过程：延时＞P1-55或者发生故障
				SM_BRAKECTRL_OUT = SM_BRAKECTRL_HOLD;
                
                #if SERVOPOWER == SERVO_DC
                RamAid->BrakeLogic = 0;
                //SM_BRAKECTRL_OUT1_LOW;
                //SM_BRAKECTRL_OUT2_LOW;
                //PwmBrkFlag = 0;
                //sm->BrakeChangePowerCnt = 0;
                #endif
                
				if((++sm->HoldBraekCnt > (RamBasePara->HoldBraekDelay)) || \
						(SM_FAULTSTATUS == 1) )
				{
                    if(sm->OffDbFlg > 0)
                    {
                        PWMOutDisableDB();
                        sm->OffDbFlg = 0;
                        LocalAxes.EcatDbFlag = 0;
                    }
                    else
                    {
                        PWMOutDisable();
                    }
                    sm->RegilFlgISR = 0;
                    sm->RegulFlg = 0;		//调节器关闭
                    sm->OffZSpdStopFlg = 0;	//伺服OFF零速停车无效
                    sm->HoldBraekCnt = 0;
					sm->BrakeStatus = SM_BRAKESTATUS_BRAKE;
                    sm->EStopFlag = 0;     //2025.03.11
                    DPI_SON = 0;           //2025.02.14
                    RamAid->JodRun = 0;    //2025.10.31
                    ScopeCtrl->MortorSon = 0;
                    //CM_BusCtrl.Run = 0;     //2025.02.14
                    //CM_BusCtrl.EstopFlag = 0;   //2025.03.11
                    sm->HaltStopFlg = 0;          //2025.05.14
                    
                    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN                     
                    LocalAxes.EcatStopMode = 0;
                    LocalAxes.OffStop_Other = 0;	//伺服OFF零速停车无效
                    LocalAxes.OffStop_FaultStop = 0;
                    LocalAxes.OffStop_Other = 0;
                    LocalAxes.OffStop_QuickStop = 0;
                    LocalAxes.u16PendingOptionCode = 0;
                    #endif
				}
				break;
			default:
				break;
		}
	}

    if((SM_BRAKECTRL_OUT == SM_BRAKECTRL_LOOSE) || (RamAid->BrakeLogic == 128))
    {
        #if SERVOPOWER == SERVO_DC
        sm->BrakeChangePowerCnt++;
        if(sm->BrakeChangePowerCnt < (RamAid->Brake48V_Delay))
        {
            #if !PROTIMETEST
            SM_BRAKECTRL_OUT1_HIGH;
            SM_BRAKECTRL_OUT2_HIGH;
            PwmBrkFlag = 1;
            #endif
        }
        else
        {
            #if !PROTIMETEST
            SM_BRAKECTRL_OUT1_LOW;
            SM_BRAKECTRL_OUT2_HIGH;
            PwmBrkFlag = 2;
            #endif
            sm->BrakeChangePowerCnt = (RamAid->Brake48V_Delay) + 1;
        }
        #endif
    }
    else
    {
        SM_BRAKECTRL_OUT1_LOW;
        SM_BRAKECTRL_OUT2_LOW;
        PwmBrkFlag = 0;
        sm->BrakeChangePowerCnt = 0;
    }
}

static void SM_SrvStatusDeal(void)
{
	TYPE_STATEMACHINE *sm = &StateMachine;
	TCiA402Axis *pCiA402Axis = &LocalAxes;
    Uint8 stopmode = 0;
    uint16_t temp;

	//上升沿：OFF→ON
    if((sm->SrvOnStatusOld == 0) && (sm->SrvOnStatus == 1))
    {
        PWMOutEnable();            //下桥打开自举充电中开始
        sm->InjectStep = 1;
    }
    else if((sm->InjectStep == 1) && (sm->SrvOnStatus == 1))
    {
        sm->ChargeTimer++;
        if(sm->ChargeTimer > 10)
        {
            sm->InjectStep = 2;
            sm->ChargeTimer = 0;
        }
    }
    else if((sm->InjectStep == 2) && (sm->SrvOnStatus == 1)) 
    {
        sm->ChargeTimer++;
        //sm->InjectStep = 2;
        temp = (RamServo->SrvOnBrkDlyTime < 50) ? 50 : RamServo->SrvOnBrkDlyTime;
        if(sm->ChargeTimer > temp)  //F.44(50ms~100ms)//50ms
        {
            sm->RegilFlgISR = 1;
            sm->SvpwmOnFlg = 1;
            sm->InjectStep = 0; 
            sm->ChargeTimer = 0;
            sm->BootStrapFlg = 1;
        }
    }
    else if(sm->SrvOnStatus == 1)        //用于PP PV PT HM 的暂停
    {
        sm->HaltStopFlg = LocalAxes.OffStop_HaltStop;    
    }
    else if((sm->SrvOnStatusOld == 1) && (sm->SrvOnStatus == 0)) //if(sm->SrvOnStatus == 0)//
    {
//        if(pCiA402Axis->CmdSource == CMD_INTERNAL)
//        {
//            stopmode = SM_StopModeDeal();
//        }
//        else if(pCiA402Axis->CmdSource == CMD_REMOTE)
//        {
//            stopmode = CiA402_Stop(pCiA402Axis);
//        }
//        else if(pCiA402Axis->CmdSource == CMD_CMBUS)
//        {
//            stopmode = CMbus_Stop();
//        }
//        else
//        {
//            stopmode = 0;
//        }
//        //2025.01.18修改前
        
        if(CtrlMode.CtrlSoure == CMD_REMOTE)
        {
            #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
            stopmode = CiA402_Stop(pCiA402Axis);
            #else
            stopmode = 0;
            #endif
        }
        else if(CtrlMode.CtrlSoure == CMD_CMBUS)
        {
            stopmode = CMbus_Stop();
        }
        else
        {
            stopmode = SM_StopModeDeal();
        }
     
        if(stopmode == 0)               //自由停车模式
        {
            if(SM_GLOSPD > STOPSPEED)
            {
                PWMOutDisable();
                sm->RegilFlgISR = 0;
                sm->RegulFlg = 0;	    //调节器关闭
            }
            sm->OffZSpdStopFlg = 0;     //零速停车标志置零
        }
        else
        {
            if(CtrlMode.CtrlSoure == CMD_INTERNAL)
            {
                sm->OffZSpdStopFlg = stopmode;
                sm->OffDbFlg       = 0;
            }
            else if(CtrlMode.CtrlSoure == CMD_REMOTE)
            {
                sm->OffZSpdStopFlg = pCiA402Axis->EcatStopMode;
                sm->OffDbFlg       = pCiA402Axis->EcatDbFlag;
            }
            else if(CtrlMode.CtrlSoure == CMD_CMBUS)
            {
                sm->OffZSpdStopFlg = stopmode;
                sm->OffDbFlg       = 0;
            }
            else
            {
                sm->OffZSpdStopFlg = stopmode;
                sm->OffDbFlg       = 0;
            }    
        }
        
        sm->InjectStep = 0;
        sm->ChargeTimer = 0;
    }
    else
    {
      
    }
    
	sm->SrvOnStatusOld = sm->SrvOnStatus;
    //sm->OffZSpdStopFlg = pCiA402Axis->EcatStopMode;
    //sm->OffDbFlg       = pCiA402Axis->EcatDbFlag;
    
	if(sm->SrvOnStatus)
	{
		Glo_DO_FUNC.Flag_Func0.bit.S_RUN = 1;
	}
	else
	{
		Glo_DO_FUNC.Flag_Func0.bit.S_RUN = 0;
	}
    
	if(sm->RegulFlg)
	{
		Glo_DO_FUNC.Flag_Func0.bit.PWMOUT = 1;
	}
	else
	{
		Glo_DO_FUNC.Flag_Func0.bit.PWMOUT = 0;
	}
}

void PWMOutDisable(void)
{
    StateMachine.BootStrapFlg = 0;
    MTU3PWMOff();
}

void PWMOutDisableDB(void)
{
    StateMachine.BootStrapFlg = 0;
    MTU3PWMOffDB();
}

//----------------------------------------------------------------------------//
void SM_ParaRefresh(void)
{
	TYPE_STATEMACHINE *sm = &StateMachine;

	if(sm->RegulFlg == 0)	//调节器已关闭
	{
		SM_ParaRefreshOff();
	}
    
	SM_ParaRefreshAnytime(); //主函数调用
}

static void SM_ParaPosResh1ms(void)
{
    static uint16_t ExecuteFlag = 2;
    
	//if(PosLoop.PosFlg.bit.Clr == 1 && StateMachine.RegulFlg > 0)//位置变量1清零,只要不是松闸状态就清零，在超程保护时零速锁轴状态有用StateMachine.RegulFlg == 1也会清除
	if(((OTP.ZeroSpdStopFlg > 0) || (StateMachine.HaltStopFlg > 0)) && (ExecuteFlag > 1)) //零速停车后，在转换到位置模式，不会跳变
    {
        ExecuteFlag--;
        
		PosRef.rst(&PosRef);
		PosCmxCdv.rst(&PosCmxCdv);
		PosSmooth.rst(&PosSmooth);
		PosFirflt.clr(&PosFirflt);
		PosDamping.rst(&PosDamping);		
        PosFullClose_Rst();
		Poscia402Gen.rst(&Poscia402Gen);     
		MultiPos.rst(&MultiPos);
		PosGenerator.rst(&PosGenerator);
		Glo_DO_FUNC.Flag_Func0.bit.PNEAR = 0x0; //位置接近输出清零
		Glo_DO_FUNC.Flag_Func0.bit.COIN = 0x0;  //位置到达输出清零
	}
    else if((OTP.ZeroSpdStopFlg > 0) || (StateMachine.HaltStopFlg > 0))
    {
    }
    else
    {
        ExecuteFlag = 2;
    }
}

//---参数刷新-1ms中断调度执行 使能中参数可以刷新---//
void SM_ParaRefreshFixtime(void)
{
	SpdGainAdpt.calc(&SpdGainAdpt);
    SM_ParaPosResh1ms();
	Glo_DO_FUNC.Flag_Func0.bit.S_RUN = StateMachine.RegulFlg;
}

//---参数刷新-任何时候执行 主函数调用---//
static void SM_ParaRefreshAnytime(void)//约172us
{
//位置增益
	PosReg.update(&PosReg);
    PosSmooth.updata(&PosSmooth);
    MultiPos.update(&MultiPos);
    
//速度增益
//	SpdGainAdpt.calc(&SpdGainAdpt);//需放入定时中断
	Spd_PiReg.update(&Spd_PiReg);
	AccFwd.update(&AccFwd);
    
//电流增益
	IdPiReg.update(&IdPiReg);
	IqPiReg.update(&IqPiReg);
    CurFullPiReg_updatat(&CurFullReg);
    
//滤波时间常数
	SpdFb.Tao = DPI_T_FdSpdLpf;//单位0.01ms
	SpdFb.update(&SpdFb);
	IqRef.Tao = DPI_T_TrqCmdLpf;//单位0.01ms
    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
    IqRef.EcatMax_P = Cia402_PP.PPBuffer0.MaxPosTorque;
    IqRef.EcatMax_N = Cia402_PP.PPBuffer0.MaxPosTorque;
    #else
    IqRef.EcatMax_P = DPI_SysMaxTorq;
    IqRef.EcatMax_N = DPI_SysMaxTorq;
    #endif
	IqRef.update(&IqRef);//滤波时间常数以及来自ECAT的力矩限幅
	//AdptFlt.pacalc(&AdptFlt);//自适应滤波系数计
	InstSpdObser.update(&InstSpdObser);
    
    #if SERVOTYPE == SERVO_MODBUS || SERVOTYPE == SERVO_CAN
    CM_busCtrl_updata(&CM_BusCtrl);
    #endif
}
//----------------------------------------------------------------------------//
//---参数刷新-OFF执行//
static void SM_ParaRefreshOff(void)//约80us
{
    uint32_t temp;
//旋转极性设置
#if SERVOTYPE == SERVO_PULSE
	DrvCoeff.DrvFlg.bit.RotPolar = DPI_RotPolar;
#endif
    
	Spd_PiReg.rst(&Spd_PiReg);
	IdRef.rst(&IdRef);
	IqRef.rst(&IqRef);
	IdPiReg.rst(&IdPiReg);
	IqPiReg.rst(&IqPiReg);
	PwmDrv.rst(&PwmDrv);
	AccFwd.rst(&AccFwd);
	UFwd.rst(&UFwd);
	NotchFilter1.rst(&NotchFilter1);
	Etheta.rst(&Etheta);
	Clark.rst(&Clark);
	Park.rst(&Park);
	Ipark.rst(&Ipark);
	Svpwm.rst(&Svpwm);
	ftvftst_rst();
	Frctoqcps.rst(&Frctoqcps);
    
    //位置模式反馈和给定的位置清零
    /***************移入位置环处理************************/
	PosRef.rst(&PosRef);
    
    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN

    LocalAxes.u32CycleTime = CumObj.x1006_communicationCyclePeriod;//1006 同步周期 us

    temp = (LocalAxes.u32CycleTime == 0UL) ? 2000UL :LocalAxes.u32CycleTime;
    PosRef.SYNCPeriodRatio = temp * 100UL / DrvCoeff.PosTsamp;
    #endif
    
    #if SERVOTYPE == SERVO_CAN
    PosRef.SYNCPeriodRatio = CM_BusCtrl.SyncTime * 100L / DrvCoeff.PosTsamp;
    #endif

    #if SERVOTYPE == SERVO_ETHERCAT
    Mit_rst();
    #endif
    
	PosFb.rst(&PosFb);
    PosReg.rst(&PosReg);       
	PosCmxCdv.rst(&PosCmxCdv);
	PosSmooth.rst(&PosSmooth);
	PosFirflt.rst(&PosFirflt); // 转移到中断处理，以防止使能自转
    
	PosDamping.rst(&PosDamping);
	PosMonitor.rst(&PosMonitor);
    MultiPos.rst(&MultiPos);	
    PosGenerator.rst(&PosGenerator);
	Poscia402Gen.rst(&Poscia402Gen);
    
    PosFullClose_Rst();
    CurFullPiReg_rst(&CurFullReg);
    ExPosFb_rst();

    #if (SERVOTYPE == SERVO_MODBUS || SERVOTYPE == SERVO_CAN)
    CM_busCtrl_rst(&CM_BusCtrl);
    #endif
    /***************移入位置环处理************************/

//速度指令清零
	Spd_Ref.rst(&Spd_Ref);
//自学习模块清零
	MotPaLearn.rst(&MotPaLearn);
	OffLineJidt.rst(&OffLineJidt);
	OnLineJidt.rst(&OnLineJidt);
	InstSpdObser.rst(&InstSpdObser);
	SinGen.rst(&SinGen);
	SinGen1.rst(&SinGen1);
	SpdResponseTestUpdateInit();
//速度S曲线更新及复位清零
	SpdScurve.rst(&SpdScurve);
	SpdScurve.TSpdAcc = DPI_T_SpdAcc;
	SpdScurve.TSpdDec = DPI_T_SpdDec;
	SpdScurve.TSpdAccDecLpf = DPI_T_SpdAccDecLpf;
	SpdScurve.TZstop = DPI_T_ZeroStop;
	SpdScurve.TNPot = DPI_T_Overtravel;
	SpdScurve.Tjog = DPI_JOGTAccDec;
	SpdScurve.update(&SpdScurve);
	InstSpdObser.rst(&InstSpdObser);
//内部多段速清零
	MultiSpd.rst(&MultiSpd);
//Do输出清零
	Glo_DO_FUNC.Flag_Func0.bit.V_CLS = 0x0;//速度接近信号清零
	Glo_DO_FUNC.Flag_Func0.bit.V_CMP = 0x0;//速度到达信号清零
	Glo_DO_FUNC.Flag_Func0.bit.C_LT = 0x0;//转矩限制输出清零
	Glo_DO_FUNC.Flag_Func0.bit.V_LT = 0x0;//速度限制输出清零
	Glo_DO_FUNC.Flag_Func0.bit.T_CMP = 0x0;//转矩到达输出清零
	Glo_DO_FUNC.Flag_Func0.bit.PNEAR = 0x0;//位置接近输出清零
	Glo_DO_FUNC.Flag_Func0.bit.COIN = 0x0;//位置到达输出清零
	Glo_DO_FUNC.Flag_Func0.bit.S_RUN = 0x0;//伺服使能输出清零
	Glo_DO_FUNC.Flag_Func0.bit.Home = 0x0;//回零输出清零
}














//#if SERVOTYPE == SERVO_PULSE1
///**
// * 脉冲型伺服状态机
// */
//static void PulseSM_Process(void)
//{
//	TYPE_STATEMACHINE *sm = &StateMachine;
//	//1、上电延时
//	if(sm->PowerUpDly < SM_POWERUPDELAY)
//	{
//		sm->PowerUpDly ++;
//		return;
//	}
//	//2、准备条件判断
//	SM_ReadyJudge();	//运行准备完成判断
//	//3、输入命令检测
//	if(sm->Ready.all == 0)	//运行准备好
//	{
//		SM_DO_SRDY_F1 = 1;
//		SM_SonCmdInChk();	//SRV-ON判断处理
//	}
//#if 0//这个分支应该不用，SZX注
//	else if(SM_CMDIN&&sm->Ready.all)//运行命令已经下发，但是未准备好
//	{
//		if(sm->Ready.bit.PowerOn)//置1
//		{
//
//		}
//		SM_DO_SRDY_F1 = 0;
//		sm->SrvOnIn = 0;	//输入命令清零
//	}
//#endif
//	else	//运行未准备好
//	{
//		SM_DO_SRDY_F1 = 0;
//		sm->SrvOnIn = 0;	//输入命令清零
//	}
//	//4、抱闸处理
//	SM_SrvBrakeCloseDeal();		//抱闸处理
//	//5、状态转换
//	SM_SrvStatusDeal();			//SRV状态处理
//	//6、状态记录
//	sm->SrvOnStatusOld = sm->SrvOnStatus;
//}
//#endif
//
//#if SERVOTYPE == SERVO_PULSE
////----------------------------------------------------------------------------//
////---运行准备完成判断---//
//static void SM_ReadyJudge(void)
//{
//	TYPE_STATEMACHINE *sm = &StateMachine;
//
//	sm->Ready.bit.ERR = (FaultP.FaultStatus & 0x0002)>>1;	//故障状态判断
//	sm->Ready.bit.DC = (UdcCtrl.State != UDC_INIT ? 0 : 1);	//母线状态判断
//}
//#endif
//
//#if SERVOTYPE == SERVO_PULSE
////---SRV-ON判断处理---//
//static void SM_CmdInCheck(void)
//{
//
//	TYPE_STATEMACHINE *sm = &StateMachine;
//	sm->CmdIn.bit.TML = SM_DI_RUN & 0x01;	//Di端子
//	sm->CmdIn.bit.INRUN = RamAid->SON;			//P8-05
//	sm->CmdIn.bit.JOG = Servo2Panel.Bit.JOG;
//}
//#endif
//
//#if SERVOTYPE == SERVO_PULSE
//static void SM_SonCmdInChk(void)
//{
//	TYPE_STATEMACHINE *sm = &StateMachine;
//	SM_CmdInCheck();	//输入命令检测
//	if(SM_CMDIN != 0)	//任意运行输入有效,此处不包括ECAT命令给定
//	{
//		//1、上次状态为运行，且本次运行输入仍然有效
//		if(sm->SrvOnStatusOld == 1)
//		{
//			sm->SrvOnIn = 1;
//		}
//		//2a、再启动关于之前停车方式不同的处理
//		else if((RamAid->StopCondition == 1) || ((RamAid->StopCondition == 0) && (RamBasePara->StopMode == 0)) )
//		//P8-14=1,自由停车和零速停车均有效;
//		//或P8-14=0,仅自由停车有效，且P1-53=0为自由停车
//		{
//			switch(RamAid->SONCondition)//P8-15使能条件判断
//			{
//				case 0:		//0：按S-OFF后时间P8-16条件启动使能
//					//停车时间≥P8-16
//					if(sm->SrvOffCnt >= RamAid->T_SONDelay)
//					{
//						sm->SrvOnIn = 1;
//					}
//					else
//					{
//						sm->SrvOnIn = 0;
//					}
//					break;
//				case 1:		//1：按速度P8-17条件启动使能
//					//当前速度≥P8-17
//					if(SM_GLOSPD <= RamAid->SpdSON)
//					{
//						sm->SrvOnIn = 1;
//					}
//					else
//					{
//						sm->SrvOnIn = 0;
//					}
//					break;
//				case 2:		//2：按时间及速度速度条件启动使能
//					//时间到P8-16，速度满足P8-17
//					if((sm->SrvOffCnt >= RamAid->T_SONDelay) && (SM_GLOSPD <= RamAid->SpdSON))
//					{
//						sm->SrvOnIn = 1;
//					}
//					else
//					{
//						sm->SrvOnIn = 0;
//					}
//					break;
//				case 3:		//3：立即使能
//				default:
//					sm->SrvOnIn = 1;	//启动标志有效
//					break;
//			}
//		}
//		//2b、P8-14=0，且P1-53=1为零速停车
//		else
//		{
//			sm->SrvOnIn = 1;	//启动标志有效
//		}
//	}
//	else	//无运行信号输入
//	{
//		sm->SrvOnIn = 0;
//	}
//}
//#endif
//
//#if SERVOTYPE == SERVO_PULSE
////----------------------------------------------------------------------------//
////---SRV状态处理---//
//
//static Uint16 SM_StopModeDeal(void)
//{
//	TYPE_STATEMACHINE *sm = &StateMachine;
//	if(sm->Ready.bit.ERR)
//	{
//		if(FAULT_DEAL_BIT(FaultP.ErrCode,FAULT_BIT_LEVEL) == FAULT_LEVEL1)//在1ms中进行处理是否时间太短
//		{
//			sm->StopMode = 0;//1级故障，自由停车
//		}
//		else
//		{
//			sm->StopMode = 1;//2级故障，零速停车（？？？按停车方式停车）
//		}
//	}
//	else
//	{
//		sm->StopMode = RamBasePara->StopMode;//正常，按设定方式停车
//	}
//	return sm->StopMode;
//}
//#endif
//
//#if SERVOTYPE == SERVO_PULSE
//static void SM_SrvStatusDeal(void)
//{
//	TYPE_STATEMACHINE *sm = &StateMachine;
//
//	//低电平：一直OFF
//	if((sm->SrvOnStatusOld == 0) && (sm->SrvOnStatus == 0))
//	{
//		//调节器处理
//		if(sm->RegulFlg == 1)		//调节器未关
//		{
//		}
//		//停机时间积累
//		else if(sm->SrvOffCntBase >= 9)	//P8-16 0.01s
//		{
//			if(sm->SrvOffCnt < FuncAttr.P8[16].Max.U)
//			{
//				sm->SrvOffCnt ++;	//停机时间累积
//			}
//			sm->SrvOffCntBase = 0;
//		}
//		else
//		{
//			sm->SrvOffCntBase ++;
//		}
//	}
//	//上升沿：OFF→ON
//	else if((sm->SrvOnStatusOld == 0) && (sm->SrvOnStatus == 1))
//	{
//       #if SERVOPOWER == SERVO_AC
//       sm->RegulFlg = 1;//调节器使能标志
//       sm->SvpwmOnFlg = 1;//打开PWM标志
//       PWMOutEnable();
//       #else
//       if(sm->InjectStep == 0)
//       {
//           //sm->PWM_Charge = 1;     //开始充电
//       }
//       else
//       {
//           //sm->PWM_Charge = 0;
//       }
//       #endif
//	}
//	//下升沿：ON→OFF
//	else if((sm->SrvOnStatusOld == 1) && (sm->SrvOnStatus == 0))
//	{
//		sm->SrvOffCnt = 0;			//停机计时清零
//		sm->SrvOffCntBase = 0;      //计时器基值
//
//		if(SM_StopModeDeal() == 0)	//自由停车处理
//		{
//			sm->OffZSpdStopFlg = 0;		//伺服OFF零速停车无效
//			sm->InjectStep = 0;			//自举电容充电步骤清零，暂时没有用到
//			if((SM_GLOSPD > STOPSPEED))	//且非静止时才封管
//			{
//				PWMOutDisable();
//                sm->RegilFlgISR = 0;
//				sm->RegulFlg = 0;			//调节器关闭
//			}
//		}
//		else //零速停车
//		{
//			sm->OffZSpdStopFlg = 1;		//伺服OFF零速停车有效（需转换为速度模式）
//			SpdScurve.Output = SpdFb.Fb;//记录当时的转速作为零速停车的初始速度
//			SpdScurve.out = SpdFb.Fb<<7;//记录当时的转速作为零速停车的初始速度
//		}
//	}
//	//高电平：一直ON
//	else if((sm->SrvOnStatusOld == 1) && (sm->SrvOnStatus == 1))
//	{
//		//以前为自举电容充电
//        #if SERVOPOWER == SERVO_DC
//        if((sm->InjectStep == 2) && (sm->SrvOnStatus == 1))    //充电完成判断
//        {
//            //sm->PWM_Charge = 0;
//            sm->InjectStep = 3;
//            sm->RegulFlg = 1;
//            sm->SvpwmOnFlg = 1;
//            PWMOutEnable();
//        }
//        #endif
//	}
//
//	SM_ParaRefreshFixtime();
//}
//#endif
/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
