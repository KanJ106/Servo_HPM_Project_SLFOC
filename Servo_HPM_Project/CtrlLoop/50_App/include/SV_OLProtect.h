/*
 * SV_OLProtect.h
 *
 *  Created on: 2015-12-25
 *      Author: zhangwei
 */

#ifndef SV_OLPROTECT_H_
#define SV_OLPROTECT_H_

#include "Datatype.h"

typedef struct{
	Uint32	DrvSum32B_var15;	//驱动器热积累
	Uint32	MotSum32B_var15;	//电机热积累
	Uint32	SysPower;			//"额定功率"值，不为实际值
	float   StopRota;
	float   StopRotb;

	Uint32	ResSum32B;			//制动电阻热积累
	Uint32	BrakeAblePoint;		//制动电阻过载点
	Uint32	BrakeDec;			//制动电阻减速速率
	Uint16	BrakeDecCnt;		//制动电阻减速计时
	Uint16	BrakeAvalid;		//制动有效
}TYPE_OL;

extern void Fault_OLInit(void);				//过载初始化
extern void Fault_DrvOLProtect(void);		//驱动器过载保护
extern void Fault_MotOLProtect(void);		//电机过载保护
extern void Fault_BrakeResProtect(void);	//制动电阻保护
extern void Fault_MotorLockedProtect(void);
extern void RunAwayMointor(void);
extern TYPE_OL Ol;

#endif /* SV_OLPROTECT_H_ */
