/*
 * SV_Monitor.c
 *
 *  Created on: 2016-1-8
 *      Author: zhangwei//
 */

#include "userdefine.h"
#include "r_cg_Project.h"
#include "SV_UdcCtrl.h"
#include "Common_Lib.h"
#include "SV_FuncVar.h"
#include "Drive.h"
#include "SV_FaultProtect.h"
#include "SV_Do.h"
#include "SV_RunTime.h"
#include "SV_MagEncode.h"   
#include "r_cg_Project.h"
#include "Pos_FullClosedLoop.h"//
#include "SV_KingKongEnc.h"
#include "SV_RtT036.h"
#include "s_tsensor_init.h"

extern int32_t TorQueData;
extern UINT16 EcatErrCode;
extern uint8_t NMT_State;

Uint16 MonCnt = 0;
void Monitor_1ms(void)
{
	Uint32 temp;
	Uint16 tmp16;
    int16  temp1;

    Etheta.EtheataDecm = _IQmpyI32int(Etheta.Etheta,3600);
    LocalAxes.Objects.objDCLinkActualValue = UdcCtrl.UdcFltr * 1000;
	if(++MonCnt > 100)//100ms刷一次,实验结果显示100ms比较合适
	{
		MonCnt = 0;
		//按照数字大小顺序排列
		RamMonitor->RotatingSpd = SpdMonitor.SpdAvrg;//P0-00显示转速
		RamMonitor->LoadFactor = CurMonitor.TorqRatsDisp;//P0-01电机平均负载率
		//Etheta.EtheataDecm = _IQmpyI32int(Etheta.Etheta,3600);
		RamMonitor->ElecAngle = Etheta.EtheataDecm;//P0-02电角度
		RamMonitor->DILevel = Xi_Logic.XILog_All & 0x00FF;//P0-03DI输入电平
		RamMonitor->DOLevel = DoFlag_Last.all;//P0-04DO输入电平
        RamMonitor->EncMulturnPos = EncPosFb.MultiCircle;//P0-05多圈信息
//		Uint16	T_SysRunL;						//P0-06系统运行时间(模块内赋值)
//		Uint16	T_SysRunH;
		//RuntimeCal();
//		Uint16	AI1;							//P0-08AI1电压值
//		Uint16	AI2;							//P0-09AI2电压值

		RamMonitor->EncSector = Hall.State;   //用于显示

		RamMonitor->Udc = UdcCtrl.UdcFltr;//P0-11直流母线电压
        //RamMonitor->Udc = ((int64)AdRead.Udc * 800) >> 24;
        
		RamMonitor->Irms = CurMonitor.IeeValuDisp;//P0-12电流有效值

//		RamMonitor->BrkFactor = ;//P0-14制动负载率(模块内赋值)
//		RamMonitor->TIPM = ;//P0-15IPM模块温度(模块内赋值)

		RamMonitor->PulseAquNumL = PosRef.PosRefTotle;//P0-16接收的总脉冲数

        //		temp = (Uint32)PosRef.PosRefTotle;
//		RamMonitor->PulseAquNumL = Common_GetU32L(temp);
//		RamMonitor->PulseAquNumH = Common_GetU32H(temp);

		tmp16 = FP_INFO_P160PTR;
        if(tmp16 >= FP_FAULTINFOMAX)//防止数组越界
        {
            tmp16 = FP_FAULTINFOMAX - 1;
            RamBasePara->FaultDisplay = tmp16;
        }
		RamMonitor->FaultType = FaultInfo.Info[tmp16].FaultCode;	//P0-18故障类型
		RamMonitor->FaultSpd = FaultInfo.Info[tmp16].Spd;	//P0-19故障时转速
		RamMonitor->FaultUdc = FaultInfo.Info[tmp16].BusVolt;	//P0-20故障时母线电压
		RamMonitor->FaultIrms = FaultInfo.Info[tmp16].Curr;	//P0-21故障时电流有效值
		RamMonitor->T_FaultL = FaultInfo.Info[tmp16].ServoTemper;//Common_GetU32L(FaultInfo.Info[tmp16].RunTime);	//P0-22故障时运行时间
		RamMonitor->T_FaultH = 0;//Common_GetU32H(FaultInfo.Info[tmp16].RunTime);

		temp = (Uint32)EncPosFb.EncSinglePos;
		RamMonitor->EncSingleturnPosL = Common_GetU32L(temp);//P0-24单圈绝对值编码器单圈位置值
		RamMonitor->EncSingleturnPosH = Common_GetU32H(temp);

//		RamMonitor->LoadInertia = ;						//P0-26负载惯量
//		RamMonitor->LoadInertiaRatio = ;					//P0-27负载惯量比

		RamMonitor->FdPulseNumL = PosFb.ScopeFbTotle;//P0-28反馈的总脉冲数
        //		temp = (Uint32)PosFb.PosFbTotle;
//		RamMonitor->FdPulseNumL = Common_GetU32L(temp);
//		RamMonitor->FdPulseNumH = Common_GetU32H(temp);
		temp = (Uint32)PosMonitor.frqvalue;
		RamMonitor->IputPulseFreqL = Common_GetU32L(temp);//P0-30位置指令脉冲频率，单位0.01Khz
		RamMonitor->IputPulseFreqH = Common_GetU32H(temp);
       
        if(DPI_EncType == 2 || DPI_EncType == 3)
        {
            RamMonitor->CRC_ErrCount = DPT.ErrCRCcnt;         //P0-34 CRC错误计数
            RamMonitor->NoAckTime_Count = DPT.NoAckTime;      //P0-35 编码器无数据计数
            RamMonitor->CE_ErrCount  = DPT.S.All;             //P0-36 CE
        }
        else if(DPI_EncType == 4 || DPI_EncType == 5)
        {
            RamMonitor->CRC_ErrCount = EncInfo.ErrCRCcnt;         //P0-34 CRC错误计数
            RamMonitor->NoAckTime_Count = EncInfo.NoAckTime;      //P0-35 编码器无数据计数
            RamMonitor->CE_ErrCount  = 0;                         //P0-36 CE
        }
        else if(DPI_EncType == 6 || DPI_EncType == 7)
        {
            RamMonitor->CRC_ErrCount = 0;                          //P0-34 CRC错误计数
            RamMonitor->NoAckTime_Count = 0;                       //P0-35 编码器无数据计数
            RamMonitor->CE_ErrCount  = 0;                          //P0-36 CE
            
            RamMonitor->NoAckTime_CountS = MagReg.RxErrCount;
            RamMonitor->CE_ErrCountS = MagReg.RxLostCount;
            RamMonitor->CRC_ErrCountS = MagReg.CeErrCount;  
        }
        else if(DPI_EncType == 8 || DPI_EncType == 9)
        {
            RamMonitor->CRC_ErrCount = MagFirst.RxErrCount;        //P0-34 CRC错误计数
            RamMonitor->NoAckTime_Count = MagFirst.RxLostCount;    //P0-35 编码器无数据计数
            RamMonitor->CE_ErrCount  = MagFirst.CeErrCount;        //P0-36 CE
            
            RamMonitor->CRC_ErrCountS = MagReg.RxErrCount;
            RamMonitor->NoAckTime_CountS = MagReg.RxLostCount;   
            RamMonitor->CE_ErrCountS = MagReg.CeErrCount;
        }
        else if(DPI_EncType == 12 || DPI_EncType == 13)
        {
            RamMonitor->CRC_ErrCount    = RTData.ErrCRCcnt;      //P0-34 CRC错误计数
            RamMonitor->NoAckTime_Count = RTData.NoAckTime;      //P0-35 编码器无数据计数
            RamMonitor->CE_ErrCount     = RTData.S.All;          //P0-36 CE
        }

        #if SERVOTYPE == SERVO_ETHERCAT
        RamMonitor->EcatStatus = (nAlStatus & 0x0F);
        #elif SERVOTYPE == SERVO_CANOPEN
        RamMonitor->EcatStatus = NMT_State;
        #else
        RamMonitor->EcatStatus = 1;
        #endif 
        
		RamMonitor->SpdCmd = Spd_Ref.SpdRefDecm;              	   //p0-41 速度指令值
		RamMonitor->MagSingle = EncPosFb.OutEncSinglePos;          //p0-42 磁编码器值
        RamMonitor->ServoErrCode = (FaultP.FaultStatus == 0) ? 0: FaultP.FaultDisp;

        #if SERVOTYPE == SERVO_ETHERCAT
        RamMonitor->EcatErrCode  = EcatErrCode;
        #endif

        RamMonitor->MotorIe = RamMotor->Ie;
        RamMonitor->ServoCode = RamServo->ServoCode;                        
        RamMonitor->CtrSoure = CtrlMode.CtrlSoure;

        //RamMonitor->EEASer1  = CUSTOMER;
	}
    
    RamMonitor->ExPos = ExtPosFb.PosFbTotle;
    RamMonitor->MortorPos = PosFb.PosFbTotle;

    RamMonitor->Torque_Nm = TorQueData;

    if(DPI_EncType == 12 || DPI_EncType == 13)
    {
        ProductInf.EncStatus = RTData.CalNote1;
        ProductInf.AngleErr  = RTData.CalMotorAglErr2;
        ProductInf.OAngleErr = RTData.CalReducerAglErr2;

        ProductInf.NsinVpp = RTData.NsinVpp;
        ProductInf.NsinOffset = RTData.NsinOffset;
        ProductInf.NcosVpp = RTData.NcosVpp;
        ProductInf.NcosOffset = RTData.NcosOffset;
        ProductInf.MsinVpp = RTData.MsinVpp;
        ProductInf.MsinOffset = RTData.MsinOffset;
        ProductInf.McosVpp = RTData.McosVpp;
        ProductInf.McosOffset = RTData.McosOffset;

        ProductInf.ONsinVpp = RTData.ONsinVpp;
        ProductInf.ONsinOffset = RTData.ONsinOffset;
        ProductInf.ONcosVpp = RTData.ONcosVpp;
        ProductInf.ONcosOffset = RTData.ONcosOffset;
        ProductInf.OMsinVpp = RTData.OMsinVpp;
        ProductInf.OMsinOffset = RTData.OMsinOffset;
        ProductInf.OMcosVpp = RTData.OMcosVpp;
        ProductInf.OMcosOffset = RTData.OMcosOffset;

        ProductInf.EncStatus1 = RTData.CalNote1;
        temp1 = RTData.CalMotorAglErr2;
        ProductInf.EncErr1 = (temp1 < 0) ? (-temp1) : temp1;
        temp1 = RTData.CalReducerAglErr2;
        ProductInf.EncErr2 = (temp1 < 0) ? (-temp1) : temp1;
    }

    ProductInf.Vendor  = Sensor.Vendor;       //0x782F
    ProductInf.ProId   = Sensor.ProId;        //0x7830
    ProductInf.Version = Sensor.Version;      //0x7831
}

/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
//计算当前电流有效值，当前负载率




