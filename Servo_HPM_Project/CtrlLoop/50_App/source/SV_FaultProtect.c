/******************************************************************************
**                           深圳市同川科技有限公司
**                               TC200伺服驱动器
**                               www.tc-tech.net//
**
----------------------------------文件信息-------------------------------------
**文   件   名：SV_FaultProtect.c
**创   建   人：杨玉亮
**最后修改日期：2015-12-22
**对外接口函数：
**	void FaultPrtt_Schedule(void)								1ms调度执行
**	void FaultPrtt_WarnInterfaceClr(Uint16 fault)				警告消失时调用
**			例电机过热警告消失：FaultPrtt_WarnInterfaceClr(MotOvHeatWarn)
**	void FaultPrtt_ErrInterface(Uint16 fault)					故障或警告发生时调用
**			例电机过热警告出现：FaultPrtt_ErrInterface(MotOvHeatWarn)
**			例电机过热故障出现: FaultPrtt_ErrInterface(MotOvHeat)
**	interrupt void epwm6TZ_isr(void)							TZ故障中断
**注意：故障屏蔽由各分模块自己处理//
******************************************************************************/
#include "userdefine.h"
#include "SV_FaultProtect.h"
#include "Common_Lib.h"
#include "SV_FuncVar.h"
#include "SV_StateMachine.h"
#include "cia402appl.h"
#include "SV_Do.h"
#include "SV_Di.h"
#include "SV_I2c.h"
#include "cia402ControlMode.h"
#include "drive.h"
#include "SV_UdcCtrl.h"
#include "canopen_interface.h"

void FaultPrtt_Schedule(void);	//故障调度函数
static void FaultPrtt_FaultRst(void);	//故障复位函数
void FaultPrtt_WarnInterfaceClr(Uint16 fault);	//警告清除接口函数
void FaultPrtt_FaultInterface(Uint16 fault);	//故障发生调用接口函数
static void FaultPrtt_MsgRecord(void);	//故障信息记录函数

TYPE_FAULTPROTECT FaultP = {	\
/*	FaultStatus;		*/					FP_NORMAL1,	\
/*	FaultDisp,ErrCode,WarnFlg;*/			0,0,0,	\
/*	ErrDoFlg,WarnDoFlg;		*/				0,0,	\
/*	EepFlg,EepNewFlg;		*/				0,0,	\
/*	ErrDiRst;		*/						0,	\
};

TYPE_FAULTINFO FaultInfo = {	\
/*	InfoPtr;	*/					FP_FAULTINFOMAX-1,	\
/*	Info;	*/						{{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}},	\
};

//----------------------------------------------------------------------------//
//---故障调度函数--//
void FaultPrtt_Schedule(void)
{
	TYPE_FAULTPROTECT *fp = &FaultP;
//	TYPE_FAULTINFO *info = &FaultInfo;

	switch(fp->FaultStatus)
	{
		case FP_NORMAL1:	//正常
		default:		//或异常值
			fp->WarnDoFlg = 0;	//do输出清零
			fp->ErrDoFlg = 0;
			break;
		case FP_WARN:
			fp->WarnDoFlg = 1;	//do警告输出
			fp->ErrDoFlg = 0;
			break;
		case FP_WARNERR:
			fp->WarnDoFlg = 1;	//do警告/故障均输出
			fp->ErrDoFlg = 1;
			break;
		case FP_ERR:
			fp->WarnDoFlg = 0;
			fp->ErrDoFlg = 1;	//do故障输出
			break;
	}
	Glo_DO_FUNC.Flag_Func0.bit.WARN = fp->WarnDoFlg;
	Glo_DO_FUNC.Flag_Func0.bit.ALM = fp->ErrDoFlg;

	FaultPrtt_FaultRst();//故障复位处理
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
	LocalAxes.LastFaultRest = LocalAxes.Objects.objControlWord;
#endif
}
#define FP_ERRRST_DI	(Glo_DI_FUNC.Flag_Func0.bit.ALM_RST)
#define FP_ERRRST_FUNC	(RamAid->FaultReset)
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
#define FP_ERRRST_ECAT6040 ((LocalAxes.Objects.objControlWord&CONTROLWORD_COMMAND_FAULTRESET)\
							&&((LocalAxes.LastFaultRest&CONTROLWORD_COMMAND_FAULTRESET) == 0))
#else
#define FP_ERRRST_ECAT6040 0
#endif
//---故障复位函数--//
static void FaultPrtt_FaultRst(void)
{
	TYPE_FAULTPROTECT *fp = &FaultP;

#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
	/************************************
	 * 需添加EtherCAT状态机中故障转换部分
	 */
	TCiA402Axis *pCiA402Axis = &LocalAxes;
#endif

	if(fp->FaultStatus == FP_NORMAL1)
	{
        if(FP_ERRRST_FUNC == 1)
        {
            FP_ERRRST_FUNC = 0;
            IIcInterfaceB(&FP_ERRRST_FUNC,1);
        }
        
		return;	//正常状态，直接退出
	}

	if(fp->WarnFlg == 0)
	{
		//警告自动恢复
		if(fp->FaultStatus != FP_ERR)
		{
			fp->FaultStatus -= 1;//此处由状态枚举特性决定可以如此计算
		}
	}//else警告不为0时，需由判断模块持续调用FaultPrtt_ErrInterface实现

#if !OPALLFAULTRESET	//调试期间所有故障均可复位
	if((FAULT_DEAL_BIT(fp->ErrCode,FAULT_BIT_RESET) == FAULT_RESET_NO) && (fp->FaultStatus > FP_WARN)) //2025.05.06
	{
		if(FP_ERRRST_FUNC == 1) 
        {
            FP_ERRRST_FUNC = 0;	//不可复位故障P8-01不可更改
            IIcInterfaceB(&FP_ERRRST_FUNC,1);
        }
		return;	//非可复位故障，或非故障状态，直接退出
	}
#endif

	//可复位故障需外部清除
	if((FP_ERRRST_DI == FuncState_UpTrig) || (FP_ERRRST_FUNC == 1) || (FP_ERRRST_ECAT6040))
	{
		FP_ERRRST_FUNC = 0;
		IIcInterfaceB(&FP_ERRRST_FUNC,1);
		//fp->FaultStatus -= 2;//此处由状态枚举特性决定可以如此计算
        fp->FaultStatus = FP_NORMAL1; //2025.05.06
		fp->ErrCode = 0;
        fp->FaultDisp = 0;        
        
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
		/********************
		 * 需添加故障处理转换部分,该转换为故障处理完成
		 */
		//pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED;// Transition 15
        pCiA402Axis->u16PendingOptionCode = 0x0;
		pCiA402Axis->Objects.objStatusWord &= ~(1<<13);
		Cia402_PP.FollowErrtime = 0;
#endif
	}
}
//----------------------------------------------------------------------------//
//---警告屏蔽判断--//
Uint16 *FaultMask[] = {	\
		&RamFuncCode.PF[27],\
		&RamFuncCode.PF[28],\
		&RamFuncCode.PF[29],\
		&RamFuncCode.PF[30],\
		&RamFuncCode.PF[32],\
		&RamFuncCode.PF[33],\
		&RamFuncCode.PF[34],\
		&RamFuncCode.PF[35],\
};
Uint16 FaultPrtt_GetFaultMask(Uint16 fault)
{
	Uint16 mask = 0;
	if(FAULT_DEAL_BIT(fault,FAULT_BIT_LEVEL) != FAULT_LEVEL3)	//故障，非警告
	{
		Uint16 faultDisp = FAULT_DEAL_BIT(fault,FAULT_BIT_DISP), \
				faultGrup = (faultDisp - 1) >> 3,		/* disp/8 */ \
				faultIndex = (faultDisp - 1) & 0x0007   /* disp%8 */;
		if(Common_Check16Bit2(*FaultMask[faultGrup], faultIndex) != 0)
		{
			mask = 1;
		}
	}

	return mask;
}
//----------------------------------------------------------------------------//
//---警告清除接口函数--//
void FaultPrtt_WarnInterfaceClr(Uint16 fault)
{
	TYPE_FAULTPROTECT *fp = &FaultP;
//	TYPE_FAULTINFO *info = &FaultInfo;
    
    if(fp->FaultDisp == (fault & 0x00FF)) //2025.05.12
    {
        fp->FaultDisp = 0;
    }

	fp->WarnFlg &= ~FAULT_GETWARNBIT(fault);//清警告标志
}
//---故障即时处理函数--//
#define FP_ERRDEAL()	{PWMOutDisable();	\
						StateMachine.RegulFlg = 0;	\
						StateMachine.RegilFlgISR = 0;	\
                        }	//调节器关闭，PWM关断
//---故障接口函数--//
void FaultPrtt_FaultInterface(Uint16 fault)
{
	TYPE_FAULTPROTECT *fp = &FaultP;

	if(FaultPrtt_GetFaultMask(fault))return;//故障屏蔽，跳出

	//状态机处理(退出故障or警告状态由故障复位模块处理)
	switch(fp->FaultStatus)
	{
		case FP_NORMAL1:		//0：正常状态
		default:	//异常值
			if(FAULT_DEAL_BIT(fault,FAULT_BIT_LEVEL) == FAULT_LEVEL3)
			{
				fp->FaultStatus = FP_WARN;//切换为警告状态
				fp->WarnFlg |= FAULT_GETWARNBIT(fault);//置警告标志
			}
			else
			{
				fp->FaultStatus = FP_ERR;//切换为故障状态
				fp->ErrCode = fault;
				fp->EepNewFlg = 1;
				if(FAULT_DEAL_BIT(fault,FAULT_BIT_LEVEL) == FAULT_LEVEL1)
				{
					FP_ERRDEAL();//一级故障，立即封管
                                 // 是否要立即抱闸

                    SM_BRAKECTRL_OUT = SM_BRAKECTRL_HOLD;
                    
                    #if SERVOPOWER == SERVO_DC
                    RamAid->BrakeLogic = 0;
                    SM_BRAKECTRL_OUT1_LOW;
                    SM_BRAKECTRL_OUT2_LOW;
                    #endif
				}
			}
			fp->FaultDisp = FAULT_DEAL_BIT(fault,FAULT_BIT_DISP);	//直接显示
			break;
		case FP_WARN:		//1:警告状态
			if(FAULT_DEAL_BIT(fault,FAULT_BIT_LEVEL) == FAULT_LEVEL3)
			{
				//更新警告为最新，后面语句执行fp->FaultDisp = fault;
				//状态不切换
				fp->WarnFlg |= FAULT_GETWARNBIT(fault);//置警告标志
			}
			else
			{
				fp->FaultStatus = FP_WARNERR;	//切换为故障且警告状态
				fp->ErrCode = fault;
				fp->EepNewFlg = 1;              //这里还需要吗，会不会一直存储
				if(FAULT_DEAL_BIT(fault,FAULT_BIT_LEVEL) == FAULT_LEVEL1)
				{
					FP_ERRDEAL();//一级故障，立即封管
                    
                    SM_BRAKECTRL_OUT = SM_BRAKECTRL_HOLD;

                    #if SERVOPOWER == SERVO_DC
                    RamAid->BrakeLogic = 0;
                    SM_BRAKECTRL_OUT1_LOW;
                    SM_BRAKECTRL_OUT2_LOW;
                    #endif
				}
			}
			fp->FaultDisp = FAULT_DEAL_BIT(fault,FAULT_BIT_DISP);	//直接显示
			break;
		case FP_ERR:		//2：故障状态
		case FP_WARNERR:	//3：故障且警告状态
			if(FAULT_DEAL_BIT(fault,FAULT_BIT_PRIORITY) < FAULT_DEAL_BIT(fp->ErrCode,FAULT_BIT_PRIORITY))	//发生更高优先级故障
			{
				//状态不变
				fp->ErrCode = fault;
				fp->EepNewFlg = 1;
				fp->FaultDisp = FAULT_DEAL_BIT(fault,FAULT_BIT_DISP);	//条件显示，状态不变
				if(FAULT_DEAL_BIT(fault,FAULT_BIT_LEVEL) == FAULT_LEVEL1)
				{
					FP_ERRDEAL();//一级故障，立即封管
                    
                    SM_BRAKECTRL_OUT = SM_BRAKECTRL_HOLD;

                    #if SERVOPOWER == SERVO_DC
                    RamAid->BrakeLogic = 0;
                    SM_BRAKECTRL_OUT1_LOW;
                    SM_BRAKECTRL_OUT2_LOW;
                    #endif
				}
			}
			else if(FAULT_DEAL_BIT(fault,FAULT_BIT_LEVEL) == FAULT_LEVEL3)	//警告
			{
				fp->FaultStatus = FP_WARNERR;	//切换为故障且警告状态
				fp->WarnFlg |= FAULT_GETWARNBIT(fault);//置警告标志
			}//低阶别故障或警告忽略
			break;
	}

	//故障信息需存储
	if(fp->EepNewFlg)
	{
		fp->EepNewFlg = 0;
		FaultPrtt_MsgRecord();//故障信息获取
	}
    
//#if 0
//#if SERVOTYPE == SERVO_ETHERCAT
//	/********************
//	 * 需添加故障处理转换部分,该转换为故障处理完成
//	 */
//	if(pCiA402Axis->i16State == STATE_FAULT_REACTION_ACTIVE)
//	{
//		pCiA402Axis->i16State = STATE_FAULT;
//	}
//#endif
//#endif  
}
//---故障信息记录函数--//
static void FaultPrtt_MsgRecord(void)
{
	TYPE_FAULTPROTECT *fp = &FaultP;
	TYPE_FAULTINFO *info = &FaultInfo;

	if(fp->EepFlg)	//前一故障存储任务未响应
	{
		//故障信息指针不处理（直接覆盖上次未存储内容）
	}
	else
	{
		fp->EepFlg = 1;
		Uint16 tmp = info->InfoPtr + 1;	//故障信息指针后移
		info->InfoPtr = (tmp >= FP_FAULTINFOMAX) ? (0) : (tmp);
	}
    
	info->Info[info->InfoPtr].FaultCode = FP_INFO_CODE;	     //故障代码
	info->Info[info->InfoPtr].Spd = FP_INFO_SPD;	         //故障速度
	info->Info[info->InfoPtr].BusVolt = FP_INFO_BUSVOLT;	 //故障母线电压
	info->Info[info->InfoPtr].Curr = FP_INFO_CURR;	         //故障电流
	//info->Info[info->InfoPtr].RunTime = FP_INFO_RUNTIME;	 //故障运行时间
    info->Info[info->InfoPtr].ServoTemper = RamMonitor->TIPM;//故障温度
    
    
//    info->Info[info->InfoPtr].FaultCode = FP_INFO_CODE;	     //故障代码
//	info->Info[info->InfoPtr].Spd = SpdFb.SpdFbDecm;	         //故障速度
//	info->Info[info->InfoPtr].BusVolt = FP_INFO_BUSVOLT;	 //故障母线电压
//	info->Info[info->InfoPtr].Curr = FP_INFO_CURR;	         //故障电流
//	info->Info[info->InfoPtr].RunTime = FP_INFO_RUNTIME;	 //故障运行时间
}

void Fault_Prtt(void)
{
    RamServo->MASK_E01 = 0;
    RamServo->MASK_E02 = 0;
    RamServo->MASK_E03 = 0;
    RamServo->MASK_E04 = 0;
    RamServo->MASK_E05 = 0;
    RamServo->MASK_E06 = 0;
    RamServo->MASK_E07 = 0;

    RamServo->MASK_E12 = 0;
    RamServo->MASK_E35 = 0;
}

/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
