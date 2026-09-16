#include "SV_PanelCtl.h"
#include "SV_FuncCode.h"
#include "SV_StateMachine.h"
#include "Canopenappl.h"
#include "SV_CtrlMode.h"
#include "Common_Lib.h"
#include "Drive.h"
#include "SV_FaultProtect.h"
#include "Pos_FullClosedLoop.h"//
#include "canopen_interface.h"


#if SERVOTYPE == SERVO_CANOPEN

void Cia402_FirstPwoerUp_Init_M(void)
{
    UINT32 temp; 
    UINT16 Spd,MaxSpd;
    temp = RamFuncCode.PE[17];
    temp = (temp << 16) + RamFuncCode.PE[16];
    temp = temp << 2;

    Spd = RamFuncCode.PE[6];
    MaxSpd = RamFuncCode.PE[7];

    LocalAxes.Objects.objFollowingErrorWindow = temp << 2;  //
    LocalAxes.Objects.objProfileAcceleration  = (UINT32)((UINT64)Spd * temp / 30);  //500ms
    LocalAxes.Objects.objProfileDeceleration  = (UINT32)((UINT64)Spd * temp / 30);  //500ms
    LocalAxes.Objects.objQuickStopDeclaration = (UINT32)((UINT64)Spd * temp / 15);  //250ms
    LocalAxes.Objects.objMaxMotorSpeed        = (UINT32)((UINT64)MaxSpd * temp / 60);
    LocalAxes.Objects.objMaxProfileVelocity   = (UINT32)((UINT64)Spd * temp / 60);
    LocalAxes.Objects.objHomingSpeeds.i32Speedforswitch = (UINT32)((UINT64)100L * temp / 60);
    LocalAxes.Objects.objHomingSpeeds.i32Speedforzero = (UINT32)((UINT64)10L * temp / 60);
    LocalAxes.Objects.objHomingAcceleration = (UINT32)((UINT64)Spd * temp / 60);  //1000ms
}

void Cia402_FirstPwoerUp_Init_O(void)
{
    UINT32 temp,temp1; 
    UINT16 Spd,MaxSpd,Reduct;
    temp = RamFuncCode.PE[17];
    temp = (temp << 16) + RamFuncCode.PE[16];
    temp = temp << 2;

    temp1 = RamFuncCode.PD[5];
    temp1 = (temp1 << 16) + RamFuncCode.PD[4];
    temp1 = temp1 << 2;
    temp1 = temp1 / temp;
    temp1 = (temp1 == 0) ? 1 : temp1;

    Spd = RamFuncCode.PE[6];
    MaxSpd = RamFuncCode.PE[7];
    Reduct = RamFuncCode.PD[6];

    LocalAxes.Objects.objFollowingErrorWindow = (((temp << 2U) * temp1) / Reduct);  //
    LocalAxes.Objects.objProfileAcceleration  = (UINT32)((UINT64)Spd * temp / (30L *Reduct));  //500ms
    LocalAxes.Objects.objProfileDeceleration  = (UINT32)((UINT64)Spd * temp / (30L *Reduct));  //500ms
    LocalAxes.Objects.objQuickStopDeclaration = (UINT32)((UINT64)Spd * temp / (15L *Reduct));  //250ms
    LocalAxes.Objects.objMaxMotorSpeed        = (UINT32)((UINT64)MaxSpd * temp / (60 *Reduct));
    LocalAxes.Objects.objMaxProfileVelocity   = (UINT32)((UINT64)Spd * temp / (60 *Reduct));
    LocalAxes.Objects.objHomingSpeeds.i32Speedforswitch = (UINT32)((UINT64)10L * temp / 60);
    LocalAxes.Objects.objHomingSpeeds.i32Speedforzero = (UINT32)((UINT64)1L * temp / 60);
    LocalAxes.Objects.objHomingAcceleration = (UINT32)((UINT64)Spd * temp / (60 *Reduct));  //1000ms
}
/**
 \return    0 Init CiA402 device successful
            ALSTATUSCODE_XX Init CiA402 device failed
 \brief    This function initializes the Axes structures
*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_Init(void)//在初始化时调用
{
    LocalAxes.Objects.objSoftwarePositionLimit.u16SubIndex0 = 2;//软限位
    LocalAxes.Objects.objGearRatio.u16SubIndex0 = 2;//减速比
    LocalAxes.Objects.objHomingSpeeds.u16SubIndex0 = 2;//回零速度
    LocalAxes.Objects.objInterpolationDataRecord.u16SubIndex0 = 1;//插补数据
    LocalAxes.Objects.objInterpolationTimePeriod.u16SubIndex0 = 2;//插补时间

    LocalAxes.bAxisIsActive = FALSE;//未使能,在状态机初始化的时候使能
    LocalAxes.bAxisFunctionEnabled = FALSE;
    LocalAxes.i16State = STATE_NOT_READY_TO_SWITCH_ON;//状态机
    LocalAxes.u16PendingOptionCode = 0x00;//未处理状态
    LocalAxes.u16PendState = 0;
    //LocalAxes.fCurPosition = 0;
    LocalAxes.u32CycleTime = 0;
    LocalAxes.OffStop_QuickStop = 0;      //ECAT伺服中Qick Stop停机方式
    LocalAxes.OffStop_HaltStop = 0;       //ECAT伺服中Halt停机方式
    LocalAxes.OffStop_Other = 0;          //其他模块发出的停车方式
    LocalAxes.OffStop_FaultStop = 0;
    LocalAxes.TpN = 0;                    //多久更新一次
    LocalAxes.InitReady = 1;
    
    LocalAxes.Encoderres = 60.0 / DrvCoeff.MotEncSglRevLns;  //编码器分辨率

    LocalAxes.Objects.objMotorRatedTorque  = (Uint32)RamMotor->Te * 10;//电机额定转矩
    LocalAxes.Objects.objMotorRatedCurrent = (Uint32)RamMotor->Ie * 10;//电机额定电流
    
    CumObj.sIdentity.u32Productcode = RamServo->ServoCode;
    Cia402_PP.PosOffset = (INT32)(((INT64)RamAid->PosOffset * LocalAxes.Objects.objGearRatio.MotorRevolutions) / LocalAxes.Objects.objGearRatio.ShaftRevolutions);
}

void CiA402_rst(void)
{
    double temp1 = 0;
    double temp2 = 0; 
    double temp3 = 0;
    double temp5 = 0;
    double temp6 = 0;

    UINT64 temp4 = 0;
  
    if(StateMachine.RegulFlg == 0)
    {
        temp1 = DrvCoeff.MotEncSglRevLns * PosCmxCdv.PosCdv; 
        temp1 = temp1 * DrvCoeff.SpdBase;
        temp2 = PosCmxCdv.PosCmxNum * 60.0;
        temp2 = temp2 * 16777216.0;
        temp1 = temp1 / temp2; 
        //LocalAxes.SpdFbPu = temp1 / temp2; 
        
        temp6 = (float64)ExtPosFb.ExtEncSglRevLns * DrvCoeff.SpdBase / (60.0 * 16777216.0);
        temp6 = temp6 * (float64)PosCmxCdv.PosCdv / PosCmxCdv.PosCmxNum;
        
        temp2 = (60.0*16777216.0)/((float64)DrvCoeff.MotEncSglRevLns*DrvCoeff.SpdBase);
        temp2 = temp2 * PosCmxCdv.PosCmxNum / PosCmxCdv.PosCdv;
//        LocalAxes.SpdCmdPU = temp2;
        
        temp4 = (UINT64)DrvCoeff.MotEncSglRevLns * DrvCoeff.SpdBase * 100000000L;
        temp3 = 2147483648.0 * 60.0 * DrvCoeff.PosTsamp;                      
        temp3 = temp3 / temp4;                                                   //加减速扩大128
        temp3 = temp3 * PosCmxCdv.PosCmxNum / PosCmxCdv.PosCdv;
        
        temp5 = temp3 * DrvCoeff.SpdTsamp / DrvCoeff.PosTsamp;
        if(FullCloseLoop.FullFlag > 0)
        {
            LocalAxes.PosPU  = temp2 * FullCloseLoop.ExtRev / DrvCoeff.MotEncSglRevLns; //转换到电机端的编码器单位
            LocalAxes.PosAccDecPu = temp3 * FullCloseLoop.ExtRev / DrvCoeff.MotEncSglRevLns;
            
            LocalAxes.SpdCmdPU  = temp2 * FullCloseLoop.InnerRev / FullCloseLoop.ExtRev;
            
            if(ExtPosFb.SpdMode == 2)
            {
                LocalAxes.SpdFbPu = temp6;     
            }
            else
            {
                LocalAxes.SpdFbPu = temp1 * FullCloseLoop.ExtRev / FullCloseLoop.InnerRev; 
            }
            
            LocalAxes.SpdAccDecPu = temp5 * FullCloseLoop.InnerRev / FullCloseLoop.ExtRev; 
        }
        else
        {
            LocalAxes.PosPU    = temp2;
            LocalAxes.SpdCmdPU = temp2;
            LocalAxes.SpdFbPu  = temp1;

            LocalAxes.SpdAccDecPu = temp5;
            LocalAxes.PosAccDecPu = temp3;
        }
        
        temp1 = DrvCoeff.SpdTsamp * 65536;
        LocalAxes.TorquePU = (float)(temp1 / 100000000.0);
        
        Cia402_PP.PPBuffer0.ConBit.All = 0;
        Cia402_PP.PPBuffer1.ConBit.All = 0;

        Cia402_PP.PosOffset = (INT32)(((INT64)RamAid->PosOffset * PosCmxCdv.PosCmxNum) / PosCmxCdv.PosCdv);
    }
}
    
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    CiA402-Statemachine
        This function handles the state machine for devices using the CiA402 profile.
        called cyclic from MainLoop()
        All described transition numbers are referring to the document
        "ETG Implementation Guideline for the CiA402 Axis Profile" located on the EtherCAT.org download section
状态机转化函数
*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_StateMachine(void)//状态转换函数
{
    TCiA402Axis *pCiA402Axis;
    TYPE_STATEMACHINE *sm = &StateMachine;
    Cia402PP*pCia402_PP = &Cia402_PP;
    UINT16 StatusWord = 0;
    UINT16 ControlWord6040 = 0;
    
    if(!LocalAxes.bAxisIsActive)//轴有效
    {
    	return;
    }
    pCiA402Axis = &LocalAxes;//得到当前轴
    ControlWord6040 = pCiA402Axis->Objects.objControlWord;//控制字
    
    //2024.03.22由函数底部移到这里，在u16PendingOptionCode = 0时也能执行。
    if((FaultP.FaultStatus&0x1) == 1)//发生了警告
    {
    	pCiA402Axis->Objects.objStatusWord |= STATUSWORD_WARNING;
    }
    else
    {
    	pCiA402Axis->Objects.objStatusWord &= ~STATUSWORD_WARNING;
    }
    
    if(sm->Ready.bit.ERR)
    {
        pCiA402Axis->i16State = STATE_FAULT;
        pCiA402Axis->Objects.objStatusWord |= STATUSWORD_STATE_FAULT;
    }
    else
    {
        pCiA402Axis->Objects.objStatusWord &= ~STATUSWORD_STATE_FAULT;    
    }
    
    /*skip state state transition if the previous transition is pending挂起*/
    if(pCiA402Axis->u16PendingOptionCode != 0x0)//如果没有按照要求的方式停车完成，则停止本次状态机的执行
    {
        if(!sm->Ready.bit.ERR) //只有不报错的状态才停止转换 2024.03.25
        {
    	    return;
        }
    }
    
    pCiA402Axis->bAxisInitReady = 1;

    switch(pCiA402Axis->i16State)//该变量表示当前所处的状态
    {
    	case STATE_NOT_READY_TO_SWITCH_ON: //0x01
    		StatusWord |= (STATUSWORD_STATE_NOTREADYTOSWITCHON);//0x0000
    	    if(sm->PowerUpDly < SM_POWERUPDELAY)
    	    {
    	    	sm->PowerUpDly ++;
    	        return;
    	    }
    	    if(co[0]->NMT->operatingState == CO_NMT_OPERATIONAL && pCiA402Axis->bAxisInitReady)
    	    {
    	        pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 1，当前状态提升一个等级，为SwitchOnDisabled
    	    }
    	    else
    	    {
    	    	/*     CiA402 statemachine shall stay in "STATE_NOT_READY_TO_SWITCH_ON" if EtherCAT state is not OP.*/
    	    	pCiA402Axis->i16State = STATE_NOT_READY_TO_SWITCH_ON; // stay in current state，保持在原状态
    	    }
    	    break;
        case STATE_SWITCH_ON_DISABLED://当前所处的状态 0x02
        	StatusWord = (STATUSWORD_STATE_SWITCHEDONDISABLED);//0x0040
        	if ((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)//0x0006
        	{
        		pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 2
        	}
        	break;
        case STATE_READY_TO_SWITCH_ON://自动转换可转换到这一步
        	StatusWord = (STATUSWORD_STATE_READYTOSWITCHON);
        	if (((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)
            || ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
        	{
        		pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 7
        	}
        	else if (((ControlWord6040 & CONTROLWORD_COMMAND_SWITCHON_MASK) == CONTROLWORD_COMMAND_SWITCHON) ||
            ((ControlWord6040 & CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION))
            {
        		if(!sm->Ready.bit.DC)//加入判断母线电压标志
        		{
        			pCiA402Axis->i16State = STATE_SWITCHED_ON;           // Transition 3
        		}
            }
            break;
        case STATE_SWITCHED_ON:
        	StatusWord = (STATUSWORD_STATE_SWITCHEDON);
        	if(sm->Ready.bit.DC)
        	{
        		pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 6
        		break;
        	}
            
        	if (((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN))
        	{
        		pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 6
        	}
        	else if (((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP
            || (ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
        	{
        		pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 10
        	}
        	else if ((ControlWord6040 & CONTROLWORD_COMMAND_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_ENABLEOPERATION&&(!sm->Ready.bit.PowerOn))
        	{
        		pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
        		pCia402_PP->FollowErrtime = 0;
        		pCia402_PP->ReachedTime = 0;
        		pCia402_PP->VelReachedTime = 0;
        		pCia402_PP->VelZeroTime = 0;
        		pCiA402Axis->Objects.objStatusWord &=~(1<<13);
        		pCiA402Axis->OffStop_HaltStop = 0;
        		pCiA402Axis->OffStop_Other = 0;
        		pCiA402Axis->OffStop_FaultStop = 0;
        		pCiA402Axis->OffStop_QuickStop = 0;
        		pCiA402Axis->i16State = STATE_OPERATION_ENABLED;  // Transition 4
                pCiA402Axis->u16PendState = 0;
                pCiA402Axis->u16PendingOptionCode = 0;
        	}
        	break;
        case STATE_OPERATION_ENABLED://在允许运行的状态
        	StatusWord = (STATUSWORD_STATE_OPERATIONENABLED);

            if(co[0]->NMT->operatingState != CO_NMT_OPERATIONAL)
            {
                // Transition 5
                /*disable operation pending*/
                pCiA402Axis->u16PendingOptionCode = 0x605B;//参照台达的停车方式进行停车
                pCiA402Axis->u16PendState = 1;
                pCiA402Axis->i16State = STATE_NOT_READY_TO_SWITCH_ON;
                //return;
            }
            
            // ---> 0x0007
        	if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEOPERATION_MASK) == CONTROLWORD_COMMAND_DISABLEOPERATION )
        	{
        		// Transition 5
        		/*disable operation pending*/
        		pCiA402Axis->u16PendingOptionCode = 0x605B;////参照台达的停车方式进行停车
        		pCiA402Axis->u16PendState = 1;
                pCiA402Axis->i16State = STATE_SWITCHED_ON;
        		//return;
        	}
            // ---> 0x0002
        	else if ((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)
            {
        		pCiA402Axis->i16State = STATE_QUICK_STOP_ACTIVE;  // Transition 11，注意发出停车命令的状态
                return;
            }
            // ---> 0x0006
        	else if ((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
        		// Transition 8
        		/*shutdown operation required*/
        		pCiA402Axis->u16PendingOptionCode = 0x605B;////STATE_TRANSITION (STATE_OPERATION_ENABLED,STATE_READY_TO_SWITCH_ON);
                pCiA402Axis->u16PendState = 2;
                pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON;
                //return;
            } 
            // ---> 0x0000
            else if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE)
            {
            	// Transition 9
            	/*shutdown operation required*/
            	pCiA402Axis->u16PendingOptionCode = 0x605B;// //STATE_TRANSITION (STATE_OPERATION_ENABLED,STATE_READY_TO_SWITCH_ON);
            	pCiA402Axis->u16PendState = 3;
                pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED;
            	//return;
            }
            break;
        case STATE_QUICK_STOP_ACTIVE:
        	StatusWord = STATUSWORD_STATE_QUICKSTOPACTIVE;
        	pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
        	if((pCiA402Axis->Objects.objStatusWord & STATUSWORD_STATE_MASK)!= STATUSWORD_STATE_QUICKSTOPACTIVE)
        	{
        		/*Only execute quick stop action in state transition 11*/
        		//if(pCiA402Axis->Objects.objQuickStopOptionCode != DISABLE_DRIVE)
        		pCiA402Axis->u16PendingOptionCode = 0x605A;//STATE_TRANSITION (STATE_OPERATION_ENABLED,STATE_QUICK_STOP_ACTIVE);
                pCiA402Axis->u16PendState = 2;
                pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON;
        	}
			if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE||sm->Ready.bit.DC)
			{
				pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 12
			}
            
//			else if ((ControlWord6040 & CONTROLWORD_COMMAND_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_ENABLEOPERATION)
//			{
//				//pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
//			    pCiA402Axis->i16State = STATE_OPERATION_ENABLED;  // Transition 16
//			    pCiA402Axis->OffStop_QuickStop = 0;
//			    sm->OffZSpdStopFlg = 0;
//			    //The Axis function shall be enabled and all internal set-points cleared.
//			}//这个时候切换到运行状态，电机会跳
			break;
        case STATE_FAULT:
			StatusWord = (STATUSWORD_STATE_FAULT);
            if(sm->Ready.bit.ERR == 0)
            {
                if(co[0]->NMT->operatingState == CO_NMT_OPERATIONAL)
                {
                    pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED;// Transition 15 
                }
                else
                {
                    pCiA402Axis->i16State = STATE_NOT_READY_TO_SWITCH_ON; //防止其他操作没有退出 
                }
            }
			break;
        case STATE_FAULT_REACTION_ACTIVE:
        	StatusWord = (STATUSWORD_STATE_FAULTREACTIONACTIVE);
        	break;
        default:    //the sate variable is set to in invalid value => rest Axis
        	StatusWord = (STATUSWORD_STATE_NOTREADYTOSWITCHON);
        	pCiA402Axis->i16State = STATE_NOT_READY_TO_SWITCH_ON;
        	break;
    }// switch(current state)
    
    if(((pCiA402Axis->i16State == STATE_QUICK_STOP_ACTIVE&&pCiA402Axis->Objects.objQuickStopOptionCode != DISABLE_DRIVE) \
    || pCiA402Axis->i16State == STATE_OPERATION_ENABLED \
    || pCiA402Axis->i16State == STATE_FAULT_REACTION_ACTIVE))
    {
    	pCiA402Axis->bAxisFunctionEnabled = TRUE;
    }
    else
    {
    //	pCia402_PP->PPBuffer0.TargetPostion = pCia402_PP->GetPos6063;
    	pCiA402Axis->bAxisFunctionEnabled = FALSE;
    }
    
    //母线电压显示
    if( sm->Ready.bit.DC)
    	pCiA402Axis->Objects.objStatusWord &= ~STATUSWORD_VOLTAGE_ENABLED;
    else
    	pCiA402Axis->Objects.objStatusWord |= STATUSWORD_VOLTAGE_ENABLED;
    
    //状态显示
    pCiA402Axis->StatusWord = StatusWord;
}

UINT8 CiA402_DecCacl(INT16 Characteristic)
{   
    UINT8 Dbflag = 0;
    Uint32 Dec_user = 0;
    float64 Dec_GivetoDrive = 0;               //标幺值单位
    Cia402PP*pCia402_PP = &Cia402_PP;
	TCiA402Axis *pLocalAxes = &LocalAxes;
    
    switch(Characteristic)
    {
        case 0:
            Dbflag = 0;            
        break;
        case 1:
            Dec_user = (pLocalAxes->Objects.ojbMaxDeceleration > pLocalAxes->Objects.objProfileDeceleration) ?\
            pLocalAxes->Objects.objProfileDeceleration : pLocalAxes->Objects.ojbMaxDeceleration;  

            if(Dec_user == 0)
            {
                Dec_user = (INT32)((INT64)DrvCoeff.MotEncSglRevLns * DrvCoeff.SpdOrgNe / 60L);//  1S ;
            }
            Dec_GivetoDrive = pLocalAxes->SpdAccDecPu * (float64)Dec_user;           
            pCia402_PP->SlowDownRampTime = (INT32)Dec_GivetoDrive;
            Dbflag = 0;            
        break; 
        case 2:
            Dec_user = pLocalAxes->Objects.objQuickStopDeclaration;

            if(Dec_user == 0)
            {
                Dec_user = (INT32)((INT64)DrvCoeff.MotEncSglRevLns * DrvCoeff.SpdOrgNe / 60L);//  1S ;
            }
            Dec_GivetoDrive = pLocalAxes->SpdAccDecPu * (float64)Dec_user;
            pCia402_PP->QuickStopRampTime = (INT32)Dec_GivetoDrive; 
            Dbflag = 0;
        break;
        case 3:
            Dec_user = (pLocalAxes->Objects.ojbMaxDeceleration > pLocalAxes->Objects.objProfileDeceleration) ?\
            pLocalAxes->Objects.objProfileDeceleration : pLocalAxes->Objects.ojbMaxDeceleration;  

            if(Dec_user == 0)
            {
                Dec_user = (INT32)((INT64)DrvCoeff.MotEncSglRevLns * DrvCoeff.SpdOrgNe / 60L);//  1S ;
            }
            Dec_GivetoDrive = pLocalAxes->SpdAccDecPu * (float64)Dec_user;           
            pCia402_PP->SlowDownRampTime = (INT32)Dec_GivetoDrive;
            Dbflag = 1;            
        break; 
        case 4:
            Dec_user = pLocalAxes->Objects.objQuickStopDeclaration;

            if(Dec_user == 0)
            {
                Dec_user = (INT32)((INT64)DrvCoeff.MotEncSglRevLns * DrvCoeff.SpdOrgNe / 60L);//  1S ;
            }
            Dec_GivetoDrive = pLocalAxes->SpdAccDecPu * (float64)Dec_user;
            pCia402_PP->QuickStopRampTime = (INT32)Dec_GivetoDrive; 
            Dbflag = 1;
        break;
        default:
            Dec_user = (pLocalAxes->Objects.ojbMaxDeceleration > pLocalAxes->Objects.objProfileDeceleration) ?\
            pLocalAxes->Objects.objProfileDeceleration : pLocalAxes->Objects.ojbMaxDeceleration;  

            if(Dec_user == 0)
            {
                Dec_user = (INT32)((INT64)DrvCoeff.MotEncSglRevLns * DrvCoeff.SpdOrgNe / 60L);//  1S ;
            }
            Dec_GivetoDrive = pLocalAxes->SpdAccDecPu * (float64)Dec_user;           
            pCia402_PP->SlowDownRampTime = (INT32)Dec_GivetoDrive;
            Dbflag = 0; 
        break;  
    }
    
    return Dbflag;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return TRUE if moving on predefined ramp is finished

 \brief    CiA402-TransitionAction
 \brief this function shall calculate the desired Axis input values to move on a predefined ramp
 \brief if the ramp is finished return TRUE
*////////////////////////////////////////////////////////////////////////////////////////
UINT8 CiA402_TransitionAction(INT16 Characteristic)//用于处理605b
{
    UINT8 dbflag = 0;
	TCiA402Axis *pLocalAxes = &LocalAxes;
    
    if(Characteristic > 4)  Characteristic = 4;

    dbflag = CiA402_DecCacl(Characteristic);
    
    pLocalAxes->EcatStopMode = (Characteristic > 2) ? (Characteristic - 2) : Characteristic;
    pLocalAxes->EcatDbFlag = dbflag;
    
    return Characteristic;
}

/*************************快速停车处理*****************************/
UINT8 CiA402_QuickStopAction(INT16 Characteristic)
{
    UINT8 dbflag = 0;
	TCiA402Axis *pLocalAxes = &LocalAxes;
    
    if(Characteristic > 4)
        Characteristic = 4;
    
    dbflag = CiA402_DecCacl(Characteristic);
    
    pLocalAxes->EcatStopMode = (Characteristic > 2) ? (Characteristic - 2) : Characteristic;
    pLocalAxes->EcatDbFlag = dbflag;
    
	return Characteristic;
}

/*************************暂停处理*****************************/
/**
 * 记录到最大执行时间为普通10us
 */
UINT8 CiA402_HaltStopAction(INT16 Characteristic)
{
	TCiA402Axis *pLocalAxes = &LocalAxes;
    
    if(Characteristic == 2)
    {
        Characteristic = 2;
    }
    else
    {
        Characteristic = 1;
    }
    
    CiA402_DecCacl(Characteristic);
    
    if(SM_GLOSPD <= STOPSPEED)	//速度≤24rpm
    {
        pLocalAxes->Objects.objStatusWord |= (1<<10);
    }
    else
    {
        pLocalAxes->Objects.objStatusWord &= ~(1<<10);
    }
    
    return Characteristic;
}

UINT8 CiA402_FaultStopAction(INT16 Characteristic)
{
    UINT8 dbflag = 0;
	TCiA402Axis *pLocalAxes = &LocalAxes;
    
    if(Characteristic > 4)
        Characteristic = 4;
    
    dbflag = CiA402_DecCacl(Characteristic);
 
    pLocalAxes->EcatStopMode = (Characteristic > 2) ? (Characteristic - 2) : Characteristic;
    pLocalAxes->EcatDbFlag = dbflag;
    
	return Characteristic;
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    CiA402-Application
 \brief check if a state transition is pending挂起 and pass desired ramp-code to CiA402TransitionAction()
 \brief if this functions returns true the state transition is finished.
*////////////////////////////////////////////////////////////////////////////////////////
UINT8 CiA402_Stop(TCiA402Axis *pCiA402Axis)
{
	UINT16 ramp = 0;
	TYPE_STATEMACHINE *sm = &StateMachine;
    UINT8 stopmode = 0;
    
	if(sm->Ready.bit.ERR)
	{
		if(FAULT_DEAL_BIT(FaultP.ErrCode,FAULT_BIT_LEVEL) == FAULT_LEVEL1)//在这里重复执行，在报错函数有执行
		{
			//if((SM_GLOSPD > STOPSPEED))	//且非静止时才封管 //2023.5.24去掉转速限制
			{
				PWMOutDisable();
                sm->RegilFlgISR = 0;
				sm->RegulFlg = 0;	    //调节器关闭
			}
            stopmode = 0;
			//pCiA402Axis->i16State = STATE_FAULT; 
		}
		else
		{
			stopmode = CiA402_FaultStopAction(pCiA402Axis->Objects.objFaultOptionCode);
			//pCiA402Axis->i16State = STATE_FAULT; 
		}
	}
    else
    {
        switch(pCiA402Axis->u16PendingOptionCode)//停机选择方式
        {
            case 0x605A:   //由QuickStop命令发出控制
                ramp = pCiA402Axis->Objects.objQuickStopOptionCode;
                stopmode = CiA402_QuickStopAction(ramp);
            break;
            case 0x605B:   //
                ramp = pCiA402Axis->Objects.objDisableOptionCode;
                stopmode = CiA402_TransitionAction(ramp);
            break;
            default:       // 内部模式用此处停车
                stopmode = 0;              
            break;       
        }
    }
    
    return stopmode;
}

void CiA402_Application(TCiA402Axis *pCiA402Axis)
{
//    if(bDcSyncActive && (pCiA402Axis->u32CycleTime != 0)\
//        && ((pCiA402Axis->Objects.objSupportedDriveModes/*支持的模式*/ >> (pCiA402Axis->Objects.objModesOfOperationDisplay/*请求的模式*/ - 1)) & 0x1)) //Mode of Operation (0x6060) - 1 specifies the Bit within Supported Drive Modes (0x6502)
    //if((pCiA402Axis->Objects.objSupportedDriveModes/*支持的模式*/ >> (pCiA402Axis->Objects.objModesOfOperationDisplay/*请求的模式*/ - 1)) & 0x1)
    {
        Control_Mode();
    }
    pCiA402Axis->Objects.objStatusWord  &= ~(STATUSWORD_STATE_MASK | STATUSWORD_REMOTE);
    
    if(CtrlMode.CtrlSoure != CMD_INTERNAL)//保证在OP状态时才使能该位
    	pCiA402Axis->Objects.objStatusWord |= (pCiA402Axis->StatusWord | STATUSWORD_REMOTE);
    else
    	pCiA402Axis->Objects.objStatusWord |= (pCiA402Axis->StatusWord);
    
    pCiA402Axis->LastControlWord6040 = pCiA402Axis->Objects.objControlWord;
}

void CiA402_AppFeedbackHanle(void)
{
    cia402_FeedbackHanle(&Cia402_PP);//反馈接收
}

#endif

