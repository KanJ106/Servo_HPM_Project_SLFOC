/*
 * SV_OTProtect.h
 *
 *  Created on: 2016-2-23
 *      Author: zhangwei
 */

#ifndef SV_OTPROTECT_H_
#define SV_OTPROTECT_H_

#include "Datatype.h"

#define OTP_CTRLMODE			(CtrlMode.CtrlMode)			//当前控制模式
#define OTP_DI_POT				(Glo_DI_FUNC.Flag_Func0.bit.P_OT)
#define OTP_DI_NOT				(Glo_DI_FUNC.Flag_Func0.bit.N_OT)

typedef struct{
	Uint16 DirOT;				//超程方向:1|0 NOT|POT
	Uint16 RefDirRec;			//伺服当前方向 1：正;2：反
	Uint16 Ccw_CwFlg;			//软限位标志（PF-47=1有效）1|0 NOT|POT

	Uint16 ZeroSpdStopFlg;		//零速停车使能
	Uint16 ZeroPosLockFlg;		//零位固定功能有效（运行于位置模式）
	Uint16 ZeroCLAMPFlg;		//零速钳位功能 F_SpdRefGet模块确定(模拟量给定相关)
	Uint16 POTNOTFlg;			//POT OR NOT
}TYPE_OTPROTECT;

extern void Fault_OTProtect(void);
extern TYPE_OTPROTECT OTP;

#endif /* SV_OTPROTECT_H_ */
