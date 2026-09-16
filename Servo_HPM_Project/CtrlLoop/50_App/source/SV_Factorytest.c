/*
 * SV_Factorytest.c
 *
 *  Created on: 2016-3-8
 *      Author: rd0217//
 */

#include "SV_Factorytest.h"
#include "userdefine.h"
#include "SV_Di.h"
#include "SV_Do.h"
#include "Drive.h"
#include "SV_UdcCtrl.h"
#include "SV_Fanctl.h"

extern void SCI_Process(Uint16 ch);
extern void PanelCtl(void);
void FactortTest(void)
{
#if 0
	SCI_Process(SCI_CHB);		//485通讯
	PanelCtl();					//键盘显示及操作
	if(1 == RamAid->SoftReset)	//软件复位
	{
		RamAid->SoftReset = 0;
		ResetCPU();
	}

	IuvRead_Calc();
	RamPA->Iu =  _IQmpy(AdRead.iu,3000);	 //Iu  0~3000mv
	RamPA->Iv =  _IQmpy(AdRead.iv,3000);	 //Iv  0~3000mv
	AdRead.Udcalc(&AdRead);
	RamPA->Udc =  _IQmpy(AdRead.Udc,3000);	 //Udc 0~3000mv
	IGBTemperRead_Calc();
	RamPA->IGBTemper =  _IQmpy(AdRead.IGBTemper,3000);	 //IGBTemper 0~2400mv

	RamPA->AI1 = 0;		//AI1模拟输入1检测，总线型没有
	RamPA->AI2 = 0;		//AI2模拟输入2检测，总线型没有
	RamPA->AI3 = 0;		//AI3模拟输入3检测，总线型没有
	RamPA->AO1 = 0;		//AO0模拟输出1检测，总线型没有
	RamPA->AO2 = 0;		//AO1模拟输出2检测，总线型没有

	RamPA->DI.bit.X1 = !GetX1_Pin;	//X1~X8数字输入检测
	RamPA->DI.bit.X2 = !GetX2_Pin;
#if (SERVO_HARDWARE == HARDWARE_AC0 ||SERVO_HARDWARE == HARDWARE_AC1)
	RamPA->DI.bit.X3 = !GetX3_Pin;
	RamPA->DI.bit.X4 = !GetX4_Pin;
	RamPA->DI.bit.X5 = !GetX5_Pin;
	RamPA->DI.bit.X6 = !GetX6_Pin;
	RamPA->DI.bit.X7 = !GetX7_Pin;
	RamPA->DI.bit.X8 = !GetX8_Pin;
#endif
//	RamPA->DI.bit.ERR_OC = !GpioDataRegs.GPADAT.bit.GPIO13;
//	RamPA->DI.bit.ERR_IPM = !GpioDataRegs.GPADAT.bit.GPIO14;

	if(0 == RamPA->DO.bit.Y1)
	{
		SetY1_Valid;	//y1数字输出检测
	}
	else
	{
		SetY1_Invaild;
	}
	if(0 == RamPA->DO.bit.Y2)
	{
		SetY2_Valid;	//y2数字输出检测
	}
	else
	{
		SetY2_Invaild;
	}
#if (SERVO_HARDWARE == HARDWARE_AC0 ||SERVO_HARDWARE == HARDWARE_AC1)
	if(0 == RamPA->DO.bit.Y3)
	{
		SetY3_Valid;	//y3数字输出检测
	}
	else
	{
		SetY3_Invaild;
	}
	if(0 == RamPA->DO.bit.Y4)
	{
		SetY4_Valid;	//y4数字输出检测
	}
	else
	{
		SetY4_Invaild;
	}
#endif
	if(1 == RamPA->DO.bit.FAN_OPEN)
	{
		FAN_ON;	//FAN输出检测
	}else
	{
		FAN_OFF;
	}
	if(1 == RamPA->DO.bit.SOFT_OPEN)
	{
		UDC_SOFTON();	//SOFT输出检测
	}else
	{
		UDC_SOFTOFF();
	}

	RamPA->ENC.all = 0;			//增量式编码器端口检测，总线型没有
	RamPA->ENC.bit.LP = !GpioDataRegs.GPADAT.bit.GPIO26;
	RamPA->EEPROM = 2;			//EEPROM自检(0:缺省；1：故障；2：正常)

	//G1~G6输出8K占空比50%PWM波形
	if(RamPA->PWM.all)
	{
		PWMOutEnable();
	}
	else
	{
		PWMOutDisable();
	}
	if(1 == RamPA->PWM.bit.PWMW_P)//G5/G6
	{
		EPwm3Regs.CMPA.bit.CMPA =	EPWM_PRD_DEFAULT / 2;  //默认占空比50%
	}
	else
	{
		EPwm3Regs.CMPA.bit.CMPA =	0;
	}
	if(1 == RamPA->PWM.bit.PWMW_N)
	{

	}
	else
	{

	}
	if(1 == RamPA->PWM.bit.PWMV_P)//G3/G4
	{
		EPwm2Regs.CMPA.bit.CMPA =	EPWM_PRD_DEFAULT / 2;  //默认占空比50%
	}
	else
	{
		EPwm2Regs.CMPA.bit.CMPA = 0;
	}
	if(1 == RamPA->PWM.bit.PWMV_N)
	{

	}
	else
	{

	}
	if(1 == RamPA->PWM.bit.PWMU_P)//G1/G2
	{
		EPwm1Regs.CMPA.bit.CMPA =	EPWM_PRD_DEFAULT / 2;  //默认占空比50%
	}
	else
	{
		EPwm1Regs.CMPA.bit.CMPA = 0;
	}
	if(1 == RamPA->PWM.bit.PWMU_N)
	{

	}
	else
	{

	}
	if(1 == RamPA->PWM.bit.BREAK)
	{
		SetG7Duty(5000);
	}
	else
	{
		SetG7Duty(0);
	}
#endif
}
