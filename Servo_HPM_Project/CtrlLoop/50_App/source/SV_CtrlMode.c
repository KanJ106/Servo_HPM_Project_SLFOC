/*
 * SV_CtrlMode.c
 *
 *  Created on: 2016-1-11//
 *      Author: zhangwei
 */
#include "SV_CtrlMode.h"
#include "SV_StateMachine.h"
#include "SV_FuncVar.h"
#include "Common_Lib.h"
#include "SV_Di.h"
#include "cia402appl.h"
#include "PosLoop.h"
#include "SV_AppDriveInterface.h"
#include "SV_ModbusConTrolVar.h"
#include "SV_CanbusCtrl.h" //
#include "canopen_interface.h"
     
void CtrlModeSwitch(void);					//控制模式切换


TYPE_CtrlMode CtrlMode = {	\
/*	CtrlMode;	*/							CTRLMODE_POS,	\
/*	CtrlMode;	*/							CTRLMODE_POS,	\
/*  EUNM_ETHERCAT_STATE */                  ETHERCATSTATE_PV,\
/*  LastECATCtrlMode    */                  ETHERCATSTATE_PV,\
/*	ActualMode         	*/					0,	\
/*	CtrSoure        	*/				    0,	\
/*	CtrlModExHibitFlg;	*/					0,	\
/*	(*deal)();	*/						    CtrlModeSwitch,	\
};

static void Ctrl_Soure_Switch(void)
{
    #if SERVOTYPE == SERVO_CANOPEN
    if(co[0]->NMT->operatingState == CO_NMT_OPERATIONAL)
    {
        CtrlMode.CtrlSoure = CMD_REMOTE;    //命令来自外部，ECAT   要考虑拔掉网线   
    }
    #else
    if((nAlStatus & 0x0F) >= STATE_SAFEOP)
    {
        CtrlMode.CtrlSoure = CMD_REMOTE;    //命令来自外部，ECAT   要考虑拔掉网线   
    }
    #endif
    else if(CM_BusCtrl.ModbusSel == 1)    
    {
        CtrlMode.CtrlSoure = CMD_CMBUS;    //命令来自外部，Modbus    
    }
    else if(ScopeCtrl->CtrlMode)
    {
        CtrlMode.CtrlSoure = CMD_SCOPE;    //命令来自外部，Modbus 
    }
    else
    {
        CtrlMode.CtrlSoure = CMD_INTERNAL;  //命令来自内部    
    }
}

static void CM_Switch_Interal(void)
{
	TYPE_CtrlMode *cm = &CtrlMode;
    
	switch(RamBasePara->DrvMode)
	{
		case PULSESTATE_SPD: 		//0：速度控制模式
			cm->CtrlMode = CTRLMODE_SPD;
            cm->ActualMode =ACTUALMODE_SPD;
			break;

		default:
		case PULSESTATE_POS:		//1：位置控制模式
			cm->CtrlMode = CTRLMODE_POS;
            cm->ActualMode =ACTUALMODE_POS;
			break;

		case PULSESTATE_TRQ:		//2:转矩控制模式
			cm->CtrlMode = CTRLMODE_TRQ;
            cm->ActualMode =ACTUALMODE_TOR;
			break;
	}
}

//---控制模式切换-EtherCAT---//
static void CM_Switch_EtherCAT(void)
{
	TYPE_CtrlMode       *cm = &CtrlMode;
	TCiA402Axis *pLocalAxes = &LocalAxes;
    
    switch(pLocalAxes->Objects.objModesOfOperation)
    {
        case PROFILE_POSITION_MODE://PP
            if(pLocalAxes->Objects.objSupportedDriveModes&SUPPORT_PP)
            {
                cm->ECATCtrlMode = ETHERCATSTATE_PP;
                cm->ActualMode = ACTUALMODE_POS;
                pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)PROFILE_POSITION_MODE;
            }
            break;
        case PROFILE_VELOCITY_MOCE://PV
            if(pLocalAxes->Objects.objSupportedDriveModes&SUPPORT_PV)
            {
                cm->ECATCtrlMode = ETHERCATSTATE_PV;
                cm->ActualMode =ACTUALMODE_SPD;
                pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)PROFILE_VELOCITY_MOCE;
            }
            break;
        case PROFILE_TORQUE_MODE://TQ
            if(pLocalAxes->Objects.objSupportedDriveModes&SUPPORT_TQ)
            {
                cm->ECATCtrlMode = ETHERCATSTATE_TQ;
                cm->ActualMode = ACTUALMODE_TOR;
                pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)PROFILE_TORQUE_MODE;
            }
            break;
        case HOMING_MODE://HM
            if(pLocalAxes->Objects.objSupportedDriveModes&SUPPORT_HM)
            {
                cm->ECATCtrlMode = ETHERCATSTATE_HM;
                cm->ActualMode = ACTUALMODE_HOME;
                pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)HOMING_MODE;
            }
            break;
        case INTERPOLATION_POSITION_MODE://IP
            if(pLocalAxes->Objects.objSupportedDriveModes&SUPPORT_IP)
            {
                cm->ECATCtrlMode = ETHERCATSTATE_IP;
                cm->ActualMode = ACTUALMODE_POS;
                pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)INTERPOLATION_POSITION_MODE;
            }
            break;
        case CYCLIC_SYNC_POSITION_MODE://CSP
            if(pLocalAxes->Objects.objSupportedDriveModes&SUPPORT_CSP)
            {
                LocalAxes.Objects.objModesOfOperationDisplay = (Uint8)CYCLIC_SYNC_POSITION_MODE;
                cm->ECATCtrlMode = ETHERCATSTATE_CSP;
                cm->ActualMode = ACTUALMODE_POS;
            }
            break;
        case CYCLIC_SYNC_VELOCITY_MODE://CSV
            if(pLocalAxes->Objects.objSupportedDriveModes&SUPPORT_CSV)
            {
                cm->ActualMode = ACTUALMODE_SPD;
                cm->ECATCtrlMode = ETHERCATSTATE_CSV;
                pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)CYCLIC_SYNC_VELOCITY_MODE;
            }
            break;
        case CYCLIC_SYNC_TORQUE_MODE://CST
            if(pLocalAxes->Objects.objSupportedDriveModes&SUPPORT_CST)
            {
                cm->ECATCtrlMode = ETHERCATSTATE_CST;
                cm->ActualMode = ACTUALMODE_TOR;
                pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)CYCLIC_SYNC_TORQUE_MODE;
            }
            break;
        case TORQUE_CLOSED_LOOP://力矩闭环
                cm->ECATCtrlMode = ETHERCATSTATE_TORCLOSE;
                cm->ActualMode = ACTUALMODE_TOR;
                pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)ETHERCATSTATE_TORCLOSE;
            break;
        case MIT_MODE://MIT
                cm->ECATCtrlMode = ETHERCATSTATE_MIT;
                cm->ActualMode = ACTUALMODE_TOR;
                pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)MIT_MODE;
            break;
        default:
            cm->ECATCtrlMode = ETHERCATSTATE_NULL0;
            cm->ActualMode = 0;
            pLocalAxes->Objects.objModesOfOperationDisplay = (Uint8)NO_MODE;
            break;
    }
}

static void CM_Switch_Scope(void)
{
	TYPE_CtrlMode *cm = &CtrlMode;
    
	switch(ScopeCtrl->CtrlMode)
	{
		case ACTUALMODE_SPD: 		//2：速度控制模式
			cm->ScopeMode = CTRLMODE_SPD;
            cm->ActualMode = ACTUALMODE_SPD;
			break;

		default:
		case ACTUALMODE_POS:		//1：位置控制模式
			cm->ScopeMode = CTRLMODE_POS;
            cm->ActualMode = ACTUALMODE_POS;
			break;

		case ACTUALMODE_TOR:		//3:转矩控制模式
			cm->ScopeMode = CTRLMODE_TRQ;
            cm->ActualMode = ACTUALMODE_TOR;
			break;
	}
}

//---控制模式切换---//
void CtrlModeSwitch(void)
{
    TYPE_STATEMACHINE   *sm = &StateMachine;
    TYPE_CtrlMode       *cm = &CtrlMode;
    
    Ctrl_Soure_Switch();
 
	if(CtrlMode.CtrlSoure == CMD_INTERNAL)
	{
        //使能模式禁止切换
        if(sm->RegulFlg > 0)
        {
            return ; 
        }  
        
        CM_Switch_Interal();    
    }
    else if(CtrlMode.CtrlSoure == CMD_REMOTE)
	{
        CM_Switch_EtherCAT();    
    }
    else if(CtrlMode.CtrlSoure == CMD_CMBUS)
    {
        cm->ActualMode = CMbusControlMode(); 
    }
    else if(CtrlMode.CtrlSoure == CMD_SCOPE)
    {
        //使能模式禁止切换
        if(sm->RegulFlg > 0)
        {
            return ; 
        } 

        CM_Switch_Scope(); 
    }
    else
    {
        
    }        
}


////----------------------------------------------------------------------------//
////---控制模式切换-Pulse---//
//#if SERVOTYPE == SERVO_PULSE
//static void CM_Switch_Pulse(void)
//{
//
//	TYPE_CtrlMode *cm = &CtrlMode;
//	Uint16 ModeSwitchCmd = CM_ModeSwitchDI;	//控制模式切换输入
//
//	cm->CtrlModExHibitFlg = (CM_MOTORSPD == 0 ? 1 : 0);		//目前仅判断是否零速
//
//	switch(RamBasePara->DrvMode)
//	{
//		case PULSESTATE_SPD: 		//0：速度控制模式
//			cm->CtrlMode = CTRLMODE_SPD;
//			break;
//
//		default:
//		case PULSESTATE_POS:		//1：位置控制模式
//			cm->CtrlMode = CTRLMODE_POS;
//			break;
//
//		case PULSESTATE_TRQ:		//2:转矩控制模式
//			cm->CtrlMode = CTRLMODE_TRQ;
//			break;
//
//		case PULSESTATE_SPD2POS:	//3：速度位置切换模式（零速条件）
//			if(cm->CtrlModExHibitFlg == 0)	//模式切换禁止无效
//			{
//				if((ModeSwitchCmd == FuncState_UpTrig) || (ModeSwitchCmd == FuncState_DnTrig))
//				{	//切换有效，变量清零
//					CM_SwitchClr_Spd2Pos();
//				}
//
//				if(ModeSwitchCmd == FuncState_Invalid)
//				{
//					cm->CtrlMode = CTRLMODE_SPD;
//				}
//				else//此处应该判断为FuncState_Valid最合理
//				{
//					cm->CtrlMode = CTRLMODE_POS;
//				}
//			}
//			break;
//
//		case PULSESTATE_TRQ2SPD:	//4：转矩速度切换模式（零速条件）
////			if(cm->CtrlModExHibitFlg == 0)	//模式切换禁止无效
//			{
//				if((ModeSwitchCmd == FuncState_UpTrig) || (ModeSwitchCmd == FuncState_DnTrig))
//				{	//切换有效，变量清零
//					CM_SwitchClr_Trq2Spd();
//				}
//
//				if(ModeSwitchCmd == FuncState_Invalid)
//				{
//					cm->CtrlMode = CTRLMODE_TRQ;
//				}
//				else
//				{
//					cm->CtrlMode = CTRLMODE_SPD;
//				}
//			}
//			break;
//
//		case PULSESTATE_POS2TRQ:	//5：位置转矩切换模式（零速条件）
//			if(cm->CtrlModExHibitFlg == 0)	//模式切换禁止无效
//			{
//				if((ModeSwitchCmd == FuncState_UpTrig) || (ModeSwitchCmd == FuncState_DnTrig))
//				{	//切换有效，变量清零
//					CM_SwitchClr_Pos2Trq();
//				}
//
//				if(ModeSwitchCmd == FuncState_Invalid)
//				{
//					cm->CtrlMode = CTRLMODE_POS;
//				}
//				else
//				{
//					cm->CtrlMode = CTRLMODE_TRQ;
//				}
//			}
//			break;
//
//		case PULSESTATE_SPD2POS2:	//6：速度位置切换模式2（直接）
//			if((ModeSwitchCmd == FuncState_UpTrig) || (ModeSwitchCmd == FuncState_DnTrig))
//			{	//切换有效，变量清零
//				CM_SwitchClr_Spd2Pos();
//			}
//
//			if(ModeSwitchCmd == FuncState_Invalid)
//			{
//				cm->CtrlMode = CTRLMODE_SPD;
//			}
//			else
//			{
//				cm->CtrlMode = CTRLMODE_POS;
//			}
//			break;
//
//		case PULSESTATE_TRQ2SPD2:	//7：转矩速度切换模式2（直接）
//			if((ModeSwitchCmd == FuncState_UpTrig) || (ModeSwitchCmd == FuncState_DnTrig))
//			{	//切换有效，变量清零
//				CM_SwitchClr_Trq2Spd();
//			}
//
//			if(ModeSwitchCmd == FuncState_Invalid)
//			{
//				cm->CtrlMode = CTRLMODE_TRQ;
//			}
//			else
//			{
//				cm->CtrlMode = CTRLMODE_SPD;
//			}
//			break;
//
//		case PULSESTATE_POS2TRQ2:	//8：位置转矩切换模式2（直接）
//			if((ModeSwitchCmd == FuncState_UpTrig) || (ModeSwitchCmd == FuncState_DnTrig))
//			{	//切换有效，变量清零
//				CM_SwitchClr_Pos2Trq();
//			}
//
//			if(ModeSwitchCmd == FuncState_Invalid)
//			{
//				cm->CtrlMode = CTRLMODE_POS;
//			}
//			else
//			{
//				cm->CtrlMode = CTRLMODE_TRQ;
//			}
//			break;
//	}
//
//}
//#endif

////----------------------------------------------------------------------------//
////---控制模式切换变量清零-Spd2Pos---//
//static void CM_SwitchClr_Spd2Pos(void)
//{
//
//}
////----------------------------------------------------------------------------//
////---控制模式切换变量清零-Trq2Spd---//
//static void CM_SwitchClr_Trq2Spd(void)
//{
//
//}
////----------------------------------------------------------------------------//
////---控制模式切换变量清零-Pos2Trq---//
//static void CM_SwitchClr_Pos2Trq(void)
//{
//
//}
/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
