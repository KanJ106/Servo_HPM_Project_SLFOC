/*
 * SV_OTProtect.c
 *
 *  Created on: 2016-2-23
 *      Author: zhangwei//
 */

//外部引用-------------------------------------------------------------//
#include "userdefine.h"
#include "r_cg_Project.h"
#include "SV_OTProtect.h"
#include "Common_Lib.h"
#include "SV_FuncVar.h"
#include "SV_FaultProtect.h"
#include "SV_CtrlMode.h"
#include "SV_StateMachine.h"
#include "Drive.h"
#include "SV_Di.h"
#include "cia402appl.h"
#include "Pos_FullClosedLoop.h"
#include "SV_CanbusCtrl.h"
#include "SV_ModbusConTrolVar.h"//
//调试输入变量-----------------------------------------------------------//

//宏定义/结构体定义--------------------------------------------------------//

TYPE_OTPROTECT OTP = {0,0,0,0,0,1,0};

//停车处理
static void OTP_OTStopDeal(void)
{
	TYPE_OTPROTECT *ot = &OTP;

	if(ot->ZeroPosLockFlg != 1)
	{
		//todo:LCR   F_PosRegDatClr
		//TODO:汇川/安川，水平轴可功能码选择自由停车
		if(ot->ZeroSpdStopFlg == 0)
		{
			ot->ZeroSpdStopFlg = 1;
			SpdScurve.Output = SpdFb.Fb;//记录当时的转速作为零速停车的初始速度
			SpdScurve.out = SpdFb.Fb<<7;//记录当时的转速作为零速停车的初始速度
		}

		if(abs(RamMonitor->RotatingSpd) <= 24)
		{
			ot->ZeroSpdStopFlg = 0;
			ot->ZeroPosLockFlg = 1;
		}
	}
}

//退出处理
static void OTP_OTReleaseDeal(void)
{
	TYPE_OTPROTECT *ot = &OTP;

	ot->ZeroSpdStopFlg = 0;
	ot->ZeroPosLockFlg = 0;
	ot->RefDirRec = 0;
    ot->Ccw_CwFlg = 0;
    ot->POTNOTFlg = 0;
}

static uint16_t ECATOTP_RefDirChk(void)
{
    uint16_t RefDirRec = 0;
    
	//TYPE_OTPROTECT *ot = &OTP;
	Cia402PP*pCia402_PP = &Cia402_PP;
    
	switch(CtrlMode.ECATCtrlMode)
	{
		case ETHERCATSTATE_VL:	//位置模式 PosRefSource_var10
		case ETHERCATSTATE_PV:
		case ETHERCATSTATE_CSV:
			if(pCia402_PP->PPBuffer0.TargetVelo > 0)
			{
				RefDirRec = 1;
			}
			else if(pCia402_PP->PPBuffer0.TargetVelo < 0)
			{
				RefDirRec = 2;
			}
			break;
		case ETHERCATSTATE_PP:	//速度模式 SpdRefSource_Var9
		case ETHERCATSTATE_CSP:
		case ETHERCATSTATE_IP:
            if(Cia402_PP.PPBuffer0.TargetPos607A > LocalAxes.Objects.objPositionActualValue)
            {
                RefDirRec = 1;
            }
            else if(Cia402_PP.PPBuffer0.TargetPos607A < LocalAxes.Objects.objPositionActualValue)
            {
                RefDirRec = 2;
            }
			break;
		case ETHERCATSTATE_TQ:	//转矩模式 g_IqRefSource_var6
		case ETHERCATSTATE_CST:
			if(pCia402_PP->PPBuffer0.TargetTorque > 0)
			{
				RefDirRec = 1;
			}
			else if(pCia402_PP->PPBuffer0.TargetTorque < 0)
			{
				RefDirRec = 2;
			}
			break;
		default:
			if(PosReg.Pospd > 0)
			{
				RefDirRec = 1;
			}
			else if(PosReg.Pospd < 0)
			{
				RefDirRec = 2;
			}
			break;
	}
    
    return RefDirRec;
}

static uint16_t CMBus_OTP_RefDirChk(void)
{
    uint16_t RefDirRec = 0;  
    
	switch(CM_BusCtrl.Mode)
	{
        case CMBUSMODE_POS:  //位置模式
//            if(CM_BusCtrl.PosTurn == POSCONTINUOUS)
//            {
//                if(CM_BusCtrl.TargetPosVelA > 0 )
//                {
//                    RefDirRec = 1;
//                }
//                else if(CM_BusCtrl.TargetPosVelA < 0 )
//                {
//                    RefDirRec = 2;
//                }
//            }
			break;
		case CMBUSMODE_SPD:	
		case CMBUSMODE_SYNCSPD:
			if(CM_BusCtrl.TargetSpdVelA > 0)
			{
				RefDirRec = 1;
			}
			else if(CM_BusCtrl.TargetSpdVelA < 0)
			{
				RefDirRec = 2;
			}
			break;
		case CMBUSMODE_SYNCPOS:	//速度模式 SpdRefSource_Var9
            if(CM_BusCtrl.TargetSyncPos > CM_BusCtrl.PosFbActualValue)
            {
                RefDirRec = 1;
            }
            else if(CM_BusCtrl.TargetSyncPos < CM_BusCtrl.PosFbActualValue)
            {
                RefDirRec = 2;
            }
			break;
		case CMBUSMODE_TOR:	//转矩模式 g_IqRefSource_var6
		case CMBUSMODE_SYNCTOR:
			if(CM_BusCtrl.TargetTorqueA > 0)
			{
				RefDirRec = 1;
			}
			else if(CM_BusCtrl.TargetTorqueA < 0)
			{
				RefDirRec = 2;
			}
			break;
		default:
		    RefDirRec = 0;
			break;
	}
    
    return RefDirRec;
}

static uint16_t OTP_RefDirChk(void)
{
    uint16_t RefDirRec = 0;
    
    if(CtrlMode.CtrlSoure == CMD_INTERNAL)
    {
        RefDirRec = 0;
    }
    else if(CtrlMode.CtrlSoure == CMD_REMOTE)
    {
        RefDirRec = ECATOTP_RefDirChk();
    }
    else if(CtrlMode.CtrlSoure == CMD_CMBUS)
    {
        RefDirRec = CMBus_OTP_RefDirChk();
    }
    else
    {
        RefDirRec = 0;
    } 
    
    return RefDirRec;
}

void Fault_OTProtect(void)
{
	TYPE_OTPROTECT *ot = &OTP;
    static Uint8 LimFlag = 0;
    static Uint16 Limcnt = 0;
    
//	if(ot->ZeroPosLockFlg != 1)
//	{
//		FaultPrtt_WarnInterfaceClr(POTWarn);
//		FaultPrtt_WarnInterfaceClr(NOTWarn);
//	}
//	ot->DirOT = 0;	//OT标志清零   从这里清零，中断如果被打断，可能会报位置偏差

	//原点回归,且不为原点回归
	//if(CtrlMode.ECATCtrlMode != ETHERCATSTATE_HM)
	{
		//松闸状态，可接收命令;且为非零速
		if((StateMachine.BrakeStatus >= SM_BRAKESTATUS_LOOSE) && \
				(ot->ZeroCLAMPFlg != 0))
		{
			//软限位有效
			if(DPI_SoftOTSel == 1)
			{
				//P-OT判断
				if((Common_Check16Bit2(ot->Ccw_CwFlg,Common_BIT0)))
				{
					Common_Set16Bit(ot->DirOT,Common_BIT0);
				}
				//N-OT判断
				else if((Common_Check16Bit2(ot->Ccw_CwFlg,Common_BIT1)))
				{
					Common_Set16Bit(ot->DirOT,Common_BIT1);
				}
                else
                {
                    ot->DirOT = 0;
                }
			}
			//软限位无效
			else
			{
				//P-OT判断
				if(Glo_DI_FUNC.Flag_Func0.bit.P_OT == FuncState_Valid)
				{
					Common_Set16Bit(ot->DirOT,Common_BIT0);
				}
				//N-OT判断
				else if(Glo_DI_FUNC.Flag_Func0.bit.N_OT == FuncState_Valid)
				{
					Common_Set16Bit(ot->DirOT,Common_BIT1);
				}
                else
                {
                    ot->DirOT = 0;
                }
			}

			if(ot->DirOT >= 3)		//POT/NOT同时有效
			{
				OTP_OTStopDeal();
                LimFlag = 1;
                FaultPrtt_FaultInterface(POTWarn);
                FaultPrtt_FaultInterface(NOTWarn);
			}
			else if(ot->DirOT == 0)	//无超程
			{
				OTP_OTReleaseDeal();
                if(LimFlag == 1)
                {
                    LimFlag = 0;
                    FaultPrtt_WarnInterfaceClr(POTWarn);
		            FaultPrtt_WarnInterfaceClr(NOTWarn);    
                }
			}
			else					//超程有效
			{
				ot->RefDirRec = OTP_RefDirChk();//判断超程方向与运动方向是否一致
				if(ot->RefDirRec == ot->DirOT)// || (ot->ZeroPosLockFlg == 1))
				{
					//POT
					if(ot->DirOT == 1)
					{
						if(StateMachine.RegulFlg &&
								(PosRef.Homing.HomingFlg.bit.PotlimtFlg == 0))
						{
                            LimFlag = 1;
							FaultPrtt_FaultInterface(POTWarn);
							OTP_OTStopDeal();
						}
					}
					//NOT
					else
					{
						if(StateMachine.RegulFlg &&
								(PosRef.Homing.HomingFlg.bit.NotlimtFlg == 0))
						{
                            LimFlag = 1;
							FaultPrtt_FaultInterface(NOTWarn);
							OTP_OTStopDeal();
						}
					}
                    ot->POTNOTFlg = ot->DirOT;
				}
				else
				{
					OTP_OTReleaseDeal();
                    if(LimFlag == 1)
                    {
                        LimFlag = 0;
                        FaultPrtt_WarnInterfaceClr(POTWarn);
		                FaultPrtt_WarnInterfaceClr(NOTWarn);    
                    }
				}
			}
            Limcnt = 0;
		}
		//不可接收命令；或零速
		else
		{
            LimFlag = 0;
			OTP_OTReleaseDeal();
            Limcnt++;
            if(Limcnt > 1000)
            {
                Limcnt = 1001;
                FaultPrtt_WarnInterfaceClr(POTWarn);
		        FaultPrtt_WarnInterfaceClr(NOTWarn); 
            }
		}
	}
}


//
////接口函数定义-----------------------------------------------------------//
////驱动器限位保护
//#if SERVOTYPE == SERVO_PULSE
////内部函数定义-----------------------------------------------------------//
////当前有效方向判断
//static void OTP_RefDirChk(void)
//{
//	TYPE_OTPROTECT *ot = &OTP;
//	switch(CtrlMode.CtrlMode)
//	{
//		case CTRLMODE_SPD:	//速度模式 SpdRefSource_Var9
//			if(Spd_Ref.Ref > 0)
//			{
//				ot->RefDirRec = 1;
//			}
//			else if(Spd_Ref.Ref < 0)
//			{
//				ot->RefDirRec = 2;
//			}
//			break;
//		case CTRLMODE_POS:	//位置模式 PosRefSource_var10
//		default:
//			if(PosReg.Pospd > 0)
//			{
//				ot->RefDirRec = 1;
//			}
//			else if(PosReg.Pospd < 0)
//			{
//				ot->RefDirRec = 2;
//			}
//			break;
//		case CTRLMODE_TRQ:	//转矩模式 g_IqRefSource_var6
//			if(IqRef.Ref > 0)
//			{
//				ot->RefDirRec = 1;
//			}
//			else if(IqRef.Ref < 0)
//			{
//				ot->RefDirRec = 2;
//			}
//			break;
//	}
//}
//
//void Fault_OTProtect(void)
//{
//	TYPE_OTPROTECT *ot = &OTP;
//
//	if(ot->ZeroPosLockFlg != 1)
//	{
//		FaultPrtt_WarnInterfaceClr(POTWarn);
//		FaultPrtt_WarnInterfaceClr(NOTWarn);
//	}
//	ot->DirOT = 0;	//OT标志清零
//
//	//PB-03＞1,且不为原点回归
//	if(RamZeroReturn->DetectMode > 1)
//	{
//		//松闸状态，可接收命令;且为非零速
//		if((StateMachine.BrakeStatus >= SM_BRAKESTATUS_LOOSE) && \
//				(ot->ZeroCLAMPFlg != 0))
//		{
//			//软限位有效
//			if(RamServo->SoftOTSel == 1)
//			{
//				//P-OT判断
//				if((Common_Check16Bit2(ot->Ccw_CwFlg,Common_BIT0)))
//				{
//					Common_Set16Bit(ot->DirOT,Common_BIT0);
//				}
//				//N-OT判断
//				if((Common_Check16Bit2(ot->Ccw_CwFlg,Common_BIT1)))
//				{
//					Common_Set16Bit(ot->DirOT,Common_BIT1);
//				}
//			}
//			//软限位无效
//			else
//			{
//				//P-OT判断
//				if(Glo_DI_FUNC.Flag_Func0.bit.P_OT == FuncState_Valid)
//				{
//					Common_Set16Bit(ot->DirOT,Common_BIT0);
//				}
//				//N-OT判断
//				if(Glo_DI_FUNC.Flag_Func0.bit.N_OT == FuncState_Valid)
//				{
//					Common_Set16Bit(ot->DirOT,Common_BIT1);
//				}
//			}
//
//			if(ot->DirOT >= 3)		//POT/NOT同时有效
//			{
//				OTP_OTStopDeal();  //零速停车处理
//			}
//			else if(ot->DirOT == 0)	//无超程
//			{
//				OTP_OTReleaseDeal();
//			}
//			else					//超程有效
//			{
//				OTP_RefDirChk();
//				if((ot->RefDirRec == ot->DirOT))// || (ot->ZeroPosLockFlg == 1))
//				{
//					//POT
//					if(ot->DirOT == 1)
//					{
//						if(StateMachine.RegulFlg &&
//								(PosRef.Homing.HomingFlg.bit.PotlimtFlg == 0))
//						{
//							FaultPrtt_FaultInterface(POTWarn);
//							OTP_OTStopDeal();
//						}
//					}
//					//NOT
//					else
//					{
//						if(StateMachine.RegulFlg &&
//								(PosRef.Homing.HomingFlg.bit.NotlimtFlg == 0))
//						{
//							FaultPrtt_FaultInterface(NOTWarn);
//							OTP_OTStopDeal();
//						}
//					}
//				}
//				else
//				{
//					OTP_OTReleaseDeal();
//				}
//			}
//		}
//		//不可接收命令；或零速
//		else
//		{
//			OTP_OTReleaseDeal();
//		}
//	}
//}
//#endif
//

//
//#if SERVOTYPE == SERVO_ETHERCAT
//static void ECATOTP_RefDirChk(void)
//{
//	TYPE_OTPROTECT *ot = &OTP;
//	Cia402PP*pCia402_PP = &Cia402_PP;
//	switch(CtrlMode.ECATCtrlMode)
//	{
//		case ETHERCATSTATE_VL:	//位置模式 PosRefSource_var10
//		case ETHERCATSTATE_PV:
//		case ETHERCATSTATE_CSV:
//			if(pCia402_PP->PPBuffer0.TargetVelo > 0)
//			{
//				ot->RefDirRec = 1;
//			}
//			else if(pCia402_PP->PPBuffer0.TargetVelo < 0)
//			{
//				ot->RefDirRec = 2;
//			}
//			break;
//		case ETHERCATSTATE_PP:	//速度模式 SpdRefSource_Var9
//		case ETHERCATSTATE_CSP:
//		case ETHERCATSTATE_IP:
//#if OPENABSENCODE
//            if(FullCloseLoop.FullFlag > 0)///2023.4.7   
//            {
//                if((LocalAxes.Objects.objTargetPosition + LocalAxes.Objects.objPositionoffset) > LocalAxes.Objects.objPositionActualValue)//YH 2017.9.14
//                {
//                    ot->RefDirRec = 1;
//                }
//                else if((LocalAxes.Objects.objTargetPosition + LocalAxes.Objects.objPositionoffset) < LocalAxes.Objects.objPositionActualValue)//YH 2017.9.14
//                {
//                    ot->RefDirRec = 2;
//                }
//            }
//            else
//            {
//                if(Cia402_PP.PPBuffer0.TargetPostion > (LocalAxes.Objects.objPositionActualInterValue-Cia402_PP.PosFbInitValue))//YH 2017.9.14
//                {
//                    ot->RefDirRec = 1;
//                }
//                else if(Cia402_PP.PPBuffer0.TargetPostion < (LocalAxes.Objects.objPositionActualInterValue-Cia402_PP.PosFbInitValue))//YH 2017.9.14
//                {
//                    ot->RefDirRec = 2;
//                }             
//            }     
//#else
//            if(Cia402_PP.PPBuffer0.TargetPostion > LocalAxes.Objects.objPositionActualValue)
//            {
//                ot->RefDirRec = 1;
//            }
//            else if(Cia402_PP.PPBuffer0.TargetPostion < LocalAxes.Objects.objPositionActualValue)
//            {
//                ot->RefDirRec = 2;
//            }
//#endif
//			break;
//		case ETHERCATSTATE_TQ:	//转矩模式 g_IqRefSource_var6
//		case ETHERCATSTATE_CST:
//			if(pCia402_PP->PPBuffer0.TargetTorque > 0)
//			{
//				ot->RefDirRec = 1;
//			}
//			else if(pCia402_PP->PPBuffer0.TargetTorque < 0)
//			{
//				ot->RefDirRec = 2;
//			}
//			break;
//		default:
//			if(PosReg.Pospd > 0)
//			{
//				ot->RefDirRec = 1;
//			}
//			else if(PosReg.Pospd < 0)
//			{
//				ot->RefDirRec = 2;
//			}
//			break;
//	}
//}
//TODO:急停，与OT处理一致
//===========================================================================
// End of file.
//===========================================================================

