/*
 * SV_RunTime.c
 *
 *  Created on: 2016-4-16
 *      Author: zhangwei//
 */
#include "SV_RunTime.h"
#include "SV_FuncVar.h"
#include "SV_StateMachine.h"
#include "SV_I2c.h"//


#define STORE_TIME_10MIN	(600000)
#define STORE_NUM			(1)
Uint16 RunMissecond = 0;
Uint32 StorePeriod = 0;

void RuntimeCal(void)
{
	if(1 == StateMachine.SrvOnStatus)
	{
		RunMissecond++;
	}
	if(RunMissecond >= 60000 )
	{
		RamMonitor->T_SysRun++;
		RunMissecond =0;
	}
}
void TS_Store_10Min(void)
{
	RuntimeCal();	//	����ʱ��ͳ��
    
	if(++StorePeriod >= STORE_TIME_10MIN/STORE_NUM )
	{
		StorePeriod = 0;
		IIcInterfaceB((Uint16*)(&RamMonitor->T_SysRun),2);
	}
}

