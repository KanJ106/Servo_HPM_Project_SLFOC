/*
 * SV_UdcCtrl.c
 *
 *  Created on: 2015-12-29
 *      Author: zhangwei////
 */

#include "userdefine.h"
#include "r_cg_Project.h"
#include "SV_UdcCtrl.h"
#include "Common_Lib.h"
#include "SV_FuncVar.h"
#include "Drive.h"
#include "SV_FaultProtect.h"
#include "SV_Fault.h"

TYPE_UDCCTRL UdcCtrl = {
/*	State;		*/				UDC_INIT,	\
/*	Udc;		*/				0,	\
/*	UdcFltr,UdcFltrRem;		*/	0,0,	\
/*	SoftOnCnt;		*/			0,	\
/*	PLRTCnt,PLRTTime,PLRTFlag;		*/	0,300,0,	\
/*	UdcVoltBase;		*/		UDC_VOLTBASE_220,	\
/*	PowerUpLevel;		*/		UDC_POWERUPLEVEL_220,	\
/*	OverVoltLevel;		*/		UDC_OVERVOLTLEVEL_220,	\
/*	BrakeVoltUpLevle;	*/		UDC_BRKVOLTUPLEVEL_220,	\
/*	BrakeVoltDnLevle;	*/		UDC_BRKVOLTDNLEVEL_220,	\
/*	UnderVoltLevel;		*/		UDC_UNDERVOLTLEVEL_220,	\
};

/************************************************************************
 * 函数名：void UdcCtrl_Schedule(void)
 * 描述：电压控制调度程序。
 ************************************************************************/
void UdcCtrl_Schedule(void)
{
	TYPE_UDCCTRL *Udc = &UdcCtrl;

	//UdcRead_Calc();
	//Udc->Udc = _IQmpy(_IQmpy(AdRead.Udc, _IQ(0.01)*RamServo->Kadccorr), UDC_VOLTBASE) + 5;
	Udc->Udc = _IQmpy(_IQmpy(AdRead.Udc, _IQ(0.01)*RamServo->Kadccorr), UDC_VOLTBASE);////
	Udc->UdcFltr = Common_FliterUint16(Udc->Udc, Udc->UdcFltr, &Udc->UdcFltrRem, UDC_FLTRTIME);

	switch(Udc->State)
	{
		case UDC_INIT :				//初始化
		default:
			if(Udc->Udc > UDC_POWERUPLEVEL)
			{
                #if (SERVO_HARDWARE == HARDWARE_AC0)
				UDC_SOFTON();
                #endif
                
				if(++Udc->SoftOnCnt > UDC_SOFTONDELAY)//软起吸合延时之后才允许运行
				{
					Udc->SoftOnCnt = 0;
					Udc->State = UDC_NORMAL;
				}
			}
			else
			{
				Udc->SoftOnCnt = 0;
			}
			break;
		case UDC_NORMAL:			//正常
            #if (SERVO_HARDWARE == HARDWARE_AC0)
			UDC_SOFTON();
            #endif

			if(Udc->Udc > UDC_BRKVOLTUPLEVEL)
			{
				Udc->State = UDC_BRAKE;
				SetG7Duty(0);               
			}
			else if(Udc->Udc < UDC_UNDERVOLTLEVEL)
			{
				Udc->State = UDC_UNDERVOLT;	//不立即报警，需判断瞬停不停
			}
            Udc->PLRTCnt = 0;
			break;
		case UDC_UNDERVOLT:			//欠压
			Udc->PLRTCnt ++;

			if(Udc->Udc < UDC_SOFTOFFLEVEL)
			{
				Udc->State = UDC_INIT;
				if(FaultJudge.PowOffFlag == 0)//掉电时，不报欠压故障
				{
					FaultPrtt_FaultInterface(UnderUdc);	//以防电压直接跌入此范围
				}
                #if (SERVO_HARDWARE == HARDWARE_AC0)
				UDC_SOFTOFF();	//电压＜软起点，报故障，关闭软起
                #endif
				Udc->PLRTCnt = 0;
				Udc->PLRTFlag = 0;
			}
			else if(Udc->Udc > UDC_UNDERVOLTLEVEL + 2)
			{
				Udc->State = UDC_NORMAL;
				Udc->PLRTCnt = 0;	//恢复正常
				Udc->PLRTFlag = 0;
			}
			else if(Udc->PLRTCnt > Udc->PLRTTime)	//电压在软起和正常电压之间时，判断瞬停不停
			{
				if(FaultJudge.PowOffFlag == 0)//掉电时，不报欠压故障
				{
					FaultPrtt_FaultInterface(UnderUdc);
				}
				Udc->PLRTCnt = Udc->PLRTTime;
				Udc->PLRTFlag = 0;	//瞬停不停无效，报故障
			}
			else if(Udc->PLRTCnt > 10)
			{
				Udc->PLRTFlag = 1;	//瞬停不停有效，开始力矩限制
			}
			break;
		case UDC_BRAKE:				//能耗制动
			if(Udc->Udc > UDC_OVERVOLTLEVEL)	//过压判断禁用滤波值
			{
				Udc->State = UDC_OVERVOLT;
				FaultPrtt_FaultInterface(OverUdc);
			}
			else if(Udc->Udc < UDC_BRKVOLTDNLEVEL)
			{
				Udc->State = UDC_NORMAL;
				SetG7Duty(0);            
			}
			else
			{
				if(FaultP.ErrCode != RstOvLoad)
				{
					SetG7Duty(RamAid->DutyBrk*100);                    
				}
				else
				{
					SetG7Duty(0);                    
				}
			}
			break;
		case UDC_OVERVOLT:			//过压
			if(Udc->Udc < UDC_OVERVOLTLEVEL - UDC_OVERVOLTTHRE)
			{
				Udc->State = UDC_BRAKE;
			}
			break;
	}
}
/************************************************************************
 * 函数名：void UdcCtrl_Init(void)
 * 描述：电压控制初始化程序。
 ************************************************************************/
void UdcCtrl_Init(void)
{
	TYPE_UDCCTRL *Udc = &UdcCtrl;

	if(UDC_DRVVOLT <= 100)		//≤100都算低压
	{
		Udc->UdcVoltBase = UDC_VOLTBASE_48;
		Udc->PowerUpLevel = UDC_POWERUPLEVEL_48;
		Udc->OverVoltLevel = (RamAid->UpPowerValue == 0) ? UDC_OVERVOLTLEVEL_48 : RamAid->UpPowerValue;
		Udc->UnderVoltLevel = UDC_UNDERVOLTLEVEL_48;
		Udc->BrakeVoltUpLevle = UDC_DRVVOLT + 5; //UDC_BRKVOLTUPLEVEL_48;
		Udc->BrakeVoltDnLevle = UDC_DRVVOLT + 4; //UDC_BRKVOLTDNLEVEL_48 ;
		Udc->SoftOffLevel = UDC_SOFTOFFLEVEL_48;
	}
	else if(UDC_DRVVOLT <= 250)	//≤250V都算220V电压等级
	{
		Udc->UdcVoltBase = UDC_VOLTBASE_220;
		Udc->PowerUpLevel = UDC_POWERUPLEVEL_220;
		Udc->OverVoltLevel = UDC_OVERVOLTLEVEL_220;
		Udc->UnderVoltLevel = UDC_UNDERVOLTLEVEL_220;
		Udc->BrakeVoltUpLevle = UDC_BRKVOLTUPLEVEL_220;
		Udc->BrakeVoltDnLevle = UDC_BRKVOLTDNLEVEL_220;
		Udc->SoftOffLevel = UDC_SOFTOFFLEVEL_220;
	}
	else	//＞250V都算380V电压等级
	{
		Udc->UdcVoltBase = UDC_VOLTBASE_380;
		Udc->PowerUpLevel = UDC_POWERUPLEVEL_380;
		Udc->OverVoltLevel = UDC_OVERVOLTLEVEL_380;
		Udc->UnderVoltLevel = UDC_UNDERVOLTLEVEL_380;
		Udc->BrakeVoltUpLevle = UDC_BRKVOLTUPLEVEL_380;
		Udc->BrakeVoltDnLevle = UDC_BRKVOLTDNLEVEL_380;
		Udc->SoftOffLevel = UDC_SOFTOFFLEVEL_380;
	}
}
/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
