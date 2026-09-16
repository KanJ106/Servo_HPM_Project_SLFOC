/******************************************************************************
**                           深圳市同川科技有限公司
**                               TC200伺服驱动器
**                               www.tc-tech.net//
**
----------------------------------文件信息-------------------------------------
**文   件   名：SV_FuncVar.c
**创   建   人：杨玉亮
**最后修改日期：2015-4-13
**对外接口函数 		void InitFuncVar(void)
** 调用驱动函数 	
**描        述：定义功能码变量并初始化
**其        它：无
**函 数 清  单：
**历        史：修改历史记录列表，每条记录包括修改日期、修改者及修改内容简述
                1.日期：
                作者：
                描述：
                2....//
******************************************************************************/

#include "userdefine.h"
#include "r_cg_Project.h"
#include "SV_FuncVar.h"
#include "SV_FuncCode.h"
#include "SV_Sci.h"
#include "SV_I2c.h"
#include "SV_UdcCtrl.h"
#include "SV_Di.h"
#include "SV_Do.h"
#include "SV_OLProtect.h"
#include "SV_FunCodeAttrLit.h"
#include "SV_Servocode.h"
#include "SV_AbsEncode.h"
#include "Drive.h"
#include "SV_Motorcode.h"
#include "SV_DataType.h"
#include "cia402appl.h"
#include "coeappl.h"
#include "SV_ModbusConTrolVar.h"

/***********掉电参数组P00****************/
TYPE_PowerDownArg *PowerDownArg;

/***********监视参数组P0****************/
TYPE_MONITOR *RamMonitor;//监视参数组

/**********P1：基本参数组*****************/
TYPE_BASEPARA *RamBasePara;//基本参数组

/***********多段位置参数组*P2****************/
TYPE_MULTIPOS *RamMultiPos;//多段位置参数组

/***********多段速及转矩控制功能组*P3****************/
TYPE_MULTISPD *RamMultiSpeed;//多段速及转矩控制参数组

/**********P4组：转矩控制参数组*******************/
TYPE_TRQCTL *RamTrqCtrl;//转矩控制参数组

/**********P5组：调谐参数组********************/
TYPE_TUNING *RamTuning;//调谐参数组

/**********P6组：输入输出参数组*******************/
TYPE_IO *RamIO;

/***********通讯组*P7****************/
TYPE_COMMU *RamCommu;//通讯参数组

/***********P8组：辅助功能功能码组************/
TYPE_AID *RamAid;//辅助功能参数组

/***********P9组：保留参数组**********************/
TYPE_P9 *RamP9;//保留参数组

/**********PA 保留参数组***************************/
TYPE_PA *RamPA;//保留参数组

/**********PB 保留参数组***************************/
TYPE_ZERORETURN *RamZeroReturn;//原点回归参数组

/**********PC 保留参数组***************************/
TYPE_PC *RamPC;//保留参数组

/**********PD 保留参数组***************************/
TYPE_PD *RamPD;//保留参数组

/**********PE 电机参数组***************************/
TYPE_MOTOR *RamMotor;//电机参数组

/**********PF 驱动器参数组***************************/
TYPE_SERVO *RamServo;//驱动器参数组

/************通信下发命令组*********/
TYPE_SCICmd     *SCICmd;       //通信下发命令组

/**********驱动器状态参数组***************/
//TYPE_ServoStatus  *ServoStatus; //驱动器状态组

/******************产品信息组*************************************/
TYPE_SCI_Ifo *SCI_Ifo;

/******************软件示波器数据*************************************/
FUN76H_RARA *ScopePara;

/******************软件示波器数据*************************************/
FUN77H_RARA *ScopeCtrl;

FUN78H_RARA ProductInf;

/***********重新上电功能码参数结构体****************/
TYPE_REPOWERON PowerOn;

/***********电机静止更新功能码参数结构体****************/
TYPE_HALTREFRESH HaltFresh;

const Uint16 FuncLit[] =
{
		P0_FUN_GROUP_SIZE,
		P1_FUN_GROUP_SIZE,
		P2_FUN_GROUP_SIZE,
		P3_FUN_GROUP_SIZE,
		P4_FUN_GROUP_SIZE,
		P5_FUN_GROUP_SIZE,
		P6_FUN_GROUP_SIZE,
		P7_FUN_GROUP_SIZE,
		P8_FUN_GROUP_SIZE,
		P9_FUN_GROUP_SIZE,
		PA_FUN_GROUP_SIZE,
		PB_FUN_GROUP_SIZE,
		PC_FUN_GROUP_SIZE,
		PD_FUN_GROUP_SIZE,
		PE_FUN_GROUP_SIZE,
		PF_FUN_GROUP_SIZE,

		F76H_FUN_GROUP_SIZE
};

/******************************************************************************
**函 数 名：void InitFuncVar(void)
**描    述：功能码结构体变量初始化
**调    用：
**输    入：
**输    出：
**返    回：
**其    它：无
**日    期：2013-7—12
*******************************************************************************/
void InitFuncVar(void)
{
	PowerDownArg = (TYPE_PowerDownArg *)RamFuncCode.P00;
	RamMonitor = (TYPE_MONITOR *)RamFuncCode.P0;
	RamBasePara = (TYPE_BASEPARA *)RamFuncCode.P1;
	RamMultiPos = (TYPE_MULTIPOS *)RamFuncCode.P2;
	RamMultiSpeed = (TYPE_MULTISPD *)RamFuncCode.P3;
	RamTrqCtrl = (TYPE_TRQCTL *)RamFuncCode.P4;
	RamTuning = (TYPE_TUNING *)RamFuncCode.P5;
	RamIO =(TYPE_IO *)RamFuncCode.P6;
	RamCommu = (TYPE_COMMU *)RamFuncCode.P7;
	RamAid = (TYPE_AID *)RamFuncCode.P8;
	RamP9 = (TYPE_P9 *)RamFuncCode.P9;
	RamPA = (TYPE_PA *)RamFuncCode.PA;
	RamZeroReturn = (TYPE_ZERORETURN *)RamFuncCode.PB;
	RamPC = (TYPE_PC *)RamFuncCode.PC;
	RamPD = (TYPE_PD *)RamFuncCode.PD;
	RamMotor = (TYPE_MOTOR *)RamFuncCode.PE;
	RamServo = (TYPE_SERVO *)RamFuncCode.PF;
    
	ScopePara = (FUN76H_RARA *)(RamFuncCode.F76H);
    ScopeCtrl = (FUN77H_RARA *)(RamFuncCode.F77H);

    memcpy(&RamFuncCode,&FactoryFunc,sizeof(TYPE_FuncCodeVal));
    memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));
    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
    memcpy(RamCumObj_StartAddr,FanCumObj_StartAddr,sizeof(CommunicationObjects));		
    memcpy(&LocalAxes.Objects,&DefCiA402ObjectValues,sizeof(CiA402Objects));
    #endif

    memcpy(&RamFuncCode.F76H,&FactoryFunc.F76H,sizeof(FUN76H_RARA));//赋出厂值，例外处理
}
/******************************************************************************
**函 数 名：void VarInit(void)
**描    述：TC200所有不能定义初始化的变量在此初始化
**调    用：
**输    入：
**输    出：
**返    回：
**其    它：无
**日    期：2013-7—12
*******************************************************************************/
void VarInit(void)
{
	///RamBasePara->ServoType = SERVOTYPE;
	ServoparaInit_Fix();			//驱动器代码相关参数初始化
    MotoParaInit(EncInfo.ReadMotorcode);//电机参数初始化
	FuncLitInit();					//功能码限定相关初始化

	DI_Init();						//DI模块初始化
	DO_Init();						//DO模块初始化
	
	UdcCtrl_Init();					//母线控制初始化
	Fault_OLInit();					//过载初始化

    DrvCoeff.init(&DrvCoeff);		//驱动器和电机参数初始化
    #if SERVOTYPE == SERVO_CAN  || SERVOTYPE == SERVO_CANOPEN
    DrvCoeff.PosTsamp = 100000;
    //位置环分频系数
    DrvCoeff.PosLoopDivCnt = DrvCoeff.PosTsamp / DrvCoeff.Tpwm;   
    #endif
    CtrLoop.init(&CtrLoop);			//各个控制模块初始化

    F6700_PowerUpInit();

    Fault_Prtt();//初始化不可屏蔽故障

    ScopeCtrl->PosAcc = DPI_T_CCW_AccDecPos;
    ScopeCtrl->PosDec = DPI_T_CW_AccDecPos;
    ScopeCtrl->SpdAcc = DPI_JOGTAccDec;
    
#if SCOPE_EN == 1
	Scope.Init(&Scope);				//示波器数据
#endif

}
/******************************************************************************
**函 数 名：Uint16 AddrRangLit(Uint16 Functioncode)
**描    述：功能码地址范围限定
** 调用
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
Uint16 AddrRangLit(Uint16 Functioncode)
{
	TYPE_WORD Rx_Func;
	Uint16 ret = 0 ;

	Rx_Func.all= Functioncode;//当前功能代码

	//连续标准部分功能码判断
	if(Rx_Func.Byte.High <= F_LAST_FUN_GROUP)//SCI读取的功能码高位地址在0~15组范围内
	{
        if(Rx_Func.Byte.High == F_LAST_FUN_GROUP)
        {
            if(Rx_Func.Byte.Low >= F_FUN_MAXNUM)
            {
                ret = ERR_ADDR_INVALID;			//返回非法地址错误              
            } 
        }
        else
        {
            if(Rx_Func.Byte.Low >= FuncLit[Rx_Func.Byte.High])
            {
                ret = ERR_ADDR_INVALID;			//返回非法地址错误              
            }    
        }
	}
	else
	{
		//非连续部分功能码判断
		switch(Rx_Func.Byte.High)
		{
			case E0x1000_GROUP://0x1000
				if(Rx_Func.Byte.Low >= ModbusCumObj_SIZE)
				{
					ret = ERR_ADDR_INVALID;
				}
				break;

			case E0x6000_GROUP://0x6000
				if(Rx_Func.Byte.Low >= ModbusCIA402OBJ_SIZE)
				{
					ret = ERR_ADDR_INVALID;
				}
				break;
        
			case F76H_FUN_GROUP://F76H组示波器设置参数
				if(Rx_Func.Byte.Low >= F76H_FUN_GROUP_SIZE)
				{
					ret = ERR_ADDR_INVALID;
				}
				break;

            case F77H_FUN_GROUP://F76H组示波器设置参数
				if(Rx_Func.Byte.Low >= F77H_FUN_GROUP_SIZE)
				{
					ret = ERR_ADDR_INVALID;
				}
				break;
            case F78H_FUN_GROUP://F76H组示波器设置参数
				if(Rx_Func.Byte.Low >= F78H_FUN_GROUP_SIZE)
				{
					ret = ERR_ADDR_INVALID;
				}
				break;
                
			default:ret = ERR_ADDR_INVALID;
				break;
		}
	}
	return ret;
}

/******************************************************************************
**函 数 名：Uint16 AddrRangLit(Uint16 Functioncode)
**描    述：功能码地址范围限定
** 调用
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
Uint16 CanAddrRangLit(Uint16 Functioncode)
{
	TYPE_WORD Rx_Func;
	Uint16 ret = 0 ;

	Rx_Func.all= Functioncode;//当前功能代码

	//连续标准部分功能码判断
	if(Rx_Func.Byte.High <= F_LAST_FUN_GROUP)//SCI读取的功能码高位地址在0~15组范围内
	{
        if(Rx_Func.Byte.High == F_LAST_FUN_GROUP)
        {
            if(Rx_Func.Byte.Low >= F_FUN_MAXNUM)
            {
                ret = 1;			//返回非法地址错误              
            } 
        }
        else
        {
            if(Rx_Func.Byte.Low >= FuncLit[Rx_Func.Byte.High])
            {
                ret = 1;			//返回非法地址错误              
            }    
        }
	}
	else
	{
        ret = 1;			//返回非法地址错误 	
	}

	return ret;
}

uint16_t P0_PF_Write_Limit(uint16_t SCI_Limit_Addr, uint16_t *Limit_Data) //mqb 参数2修改为指针类型的
{
	TYPE_Attribute *Attr;
	uint16_t ret = SCI_NO_ERR;
    
	Attr = FuncAttrStartAddr + GetFuncAddr(SCI_Limit_Addr);

	if(Attr->Operation == ATTR_OPE_ReadOnly)
	{
        ret = 2;	
	}
	else
	{
        if(Attr->Sign == USIGN)
        {
            if((*Limit_Data > Attr->Max.U) || (*Limit_Data < Attr->Min.U))
            {
                ret = 4;
            }
            else
            {
                ret = 0;
            }
        }
        else
        {
            if(((int16_t)*Limit_Data > Attr->Max.S) || ((int16_t)*Limit_Data < Attr->Min.S))
            {
                ret = 4;
            }
            else
            {
                ret = 0;
            }
        }
    }
	return ret;
}

/******************************************************************************
**函 数 名：Uint16 GetFuncNum(Uint16 *RamFunc)
**描    述：根据功能码的内存地址查找功能代码号
** 调用
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
Uint16 GetFuncNum(Uint16 *RamFunc)
{
	TYPE_WORD Rx_Func;
	if(RamFunc > RamFuncEndAddr)//如果地址超过最大地址，则等于最大地址
	{
		RamFunc = RamFuncEndAddr;
	}
	else if(RamFunc < RamFuncStartAddr)//如果地址小于最小地址，则等于最小地址
	{
		RamFunc = RamFuncStartAddr;
	}

	if(RamFunc >=  RamFuncCode.PF)
	{
		Rx_Func.Byte.High = PF_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.PF;
	}
	else if(RamFunc >=  RamFuncCode.PE)
	{
		Rx_Func.Byte.High = PE_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.PE;
	}
	else if(RamFunc >=  RamFuncCode.PD)
	{
		Rx_Func.Byte.High = PD_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.PD;
	}
	else if(RamFunc >=  RamFuncCode.PC)
	{
		Rx_Func.Byte.High = PC_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.PC;
	}
	else if(RamFunc >=  RamFuncCode.PB)
	{
		Rx_Func.Byte.High = PB_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.PB;
	}
	else if(RamFunc >=  RamFuncCode.PA)
	{
		Rx_Func.Byte.High = PA_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.PA;
	}
	else if(RamFunc >=  RamFuncCode.P9)
	{
		Rx_Func.Byte.High = P9_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P9;
	}
	else if(RamFunc >=  RamFuncCode.P8)
	{
		Rx_Func.Byte.High = P8_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P8;
	}
	else if(RamFunc >=  RamFuncCode.P7)
	{
		Rx_Func.Byte.High = P7_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P7;
	}
	else if(RamFunc >=  RamFuncCode.P6)
	{
		Rx_Func.Byte.High = P6_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P6;
	}
	else if(RamFunc >=  RamFuncCode.P5)
	{
		Rx_Func.Byte.High = P5_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P5;
	}
	else if(RamFunc >=  RamFuncCode.P4)
	{
		Rx_Func.Byte.High = P4_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P4;
	}
	else if(RamFunc >=  RamFuncCode.P3)
	{
		Rx_Func.Byte.High = P3_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P3;
	}
	else if(RamFunc >=  RamFuncCode.P2)
	{
		Rx_Func.Byte.High = P2_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P2;
	}
	else if(RamFunc >=  RamFuncCode.P1)
	{
		Rx_Func.Byte.High = P1_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P1;
	}
	else if(RamFunc >=  RamFuncCode.P0)
	{
		Rx_Func.Byte.High = P0_FUN_GROUP;
		Rx_Func.Byte.Low  = RamFunc - RamFuncCode.P0;
	}
	return Rx_Func.all;
}

void F6700_PowerUpInit(void)
{
    ScopePara->MotSingRev = DrvCoeff.MotEncSglRevLns;
    ScopePara->OutSingRev = DrvCoeff.OutEncSglRevLns;
}
