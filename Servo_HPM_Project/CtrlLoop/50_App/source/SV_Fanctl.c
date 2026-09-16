/*
 * SV_Fanctl.c
 *
 *  Created on: 2016-1-26
 *      Author: rd0217//
 */
#include "SV_Fanctl.h"
#include "SV_Fault.h"
#include "userdefine.h"
#include "r_cg_Project.h"
#include "SV_FuncVar.h"
#include "SV_StateMachine.h"
#include "SV_FaultProtect.h" //

void FAN_Ctl(void)
{
#if SERVO_HARDWARE == HARDWARE_AC0
	if(0 == RamAid->FanCtl) 	//伺服使能或者警告故障时运行
	{
		if((1 == StateMachine.RegulFlg)||(0 != FaultP.FaultStatus))
		{
			FAN_ON;
		}
		else
		{
			FAN_OFF;
		}
	}
	else if(1 == RamAid->FanCtl)	//上电运行
	{
		FAN_ON;
	}
	else
	{
		FAN_OFF;
	}
#endif
}
