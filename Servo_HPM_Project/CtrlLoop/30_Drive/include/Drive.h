/*
 * Drive.h
 *
 *  Created on: 2015-12-19
 *      Author: w
 *///

#ifndef DRIVE_H_
#define DRIVE_H_

#define FUNCTYPEN 0 //函数实现类型
#include "IQmathLib.h"         /* Include header for IQmath library *///
/* Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file */
#include "dataType.h"
#include "SV_AppDriveInterface.h"
#include "cia402ControlMode.h"
#include "Filter.h"
#include "SV_Di.h"
#include "SV_Do.h"
#include "SV_CtrlMode.h"
#include "SV_FaultProtect.h"
#include "SV_AbsEncode.h"
#include "SV_IncEncode.h"
#include "SV_StateMachine.h"                //
#include "CtrLoop.h"
#include "PosLoop.h"
#include "SpdLoop.h"
#include "ToqLoop.h"
#include "AdRead.h"
#include "Spd_PiReg.h"
#include "DrvCoeff.h"
#include "Spd_Ref.h"
#include "Cur_Clark.h"
#include "Cur_Park.h"
#include "Cur_Etheta.h"
#include "Cur_IdRef.h"
#include "Cur_IqRef.h"
#include "Cur_IdPiReg.h"
#include "Cur_IqPiReg.h"
#include "Cur_Ipark.h"
#include "Cur_Svpwm.h"
#include "Cur_PwmDrv.h"
#include "Cur_UdFwd.h"
#include "Cur_NotchFilter.h"
#include "Cur_FrictionTorqCps.h"
#include "FtVfTest.h"
#include "ReferenceTest.h"
#include "Pos_Reg.h"
#include "EncPosFb.h"
#include "Pos_Ref.h"
#include "PosFb.h"
#include "SpdFb.h"
#include "Cur_MotPaLearn.h"
#include "Cur_Monitor.h"
#include "Spd_Monitor.h"
#include "ScurveAccDec.h"
#include "MultiSpd.h"
#include "PosGenerator.h"
#include "MultiPos.h"
#include "Pos_Monitor.h"
#include "Spd_GainAdpt.h"
#include "Spd_OfflineJidt.h"
#include "Spd_OnlineJidt.h"
#include "Spd_InstsSpdObser.h"
#include "Cur_PiGainAdpt.h"
#include "Pos_CmxCdv.h"
#include "Pos_Smooth.h"
#include "Pos_FirFilter.h"
#include "Pos_Damping.h"
#include "SpdResponseTest.h"
#include "cia402appl.h"
#include "OD.h"
#include "cia402Ctrl.h"

//#include "SV_ModbusConTrolVar.h"

//extern ADREAD AdRead;
//extern CTRLOOP_TYPE CtrLoop;
//extern SPDLOOP SpdLoop;
//extern SPD_PIREG Spd_PiReg;
//extern ACCFWD AccFwd;
//extern DRVCOEFF DrvCoeff;
//extern SPD_REF Spd_Ref;
//extern CUR_CLARKE Clark;
//extern CUR_PARK Park;
//extern CUR_ETHETA Etheta;
//extern CUR_IDREF IdRef;
//extern CUR_IQREF IqRef;
//extern CUR_IDPIREG IdPiReg;
//extern CUR_IQPIREG IqPiReg;
//extern CUR_IPARK Ipark;
//extern CUR_SVPWM Svpwm;
//extern CUR_PWMDRV PwmDrv;
//extern CUR_UDFWD UFwd;
//extern ADPTFLT AdptFlt;
//extern FTVFTEST Ftvftst;
//extern REFERENCETEST RefTest;
//extern SINGEN SinGen;
//extern SINGEN SinGen1;
//extern SINGEN SinGen2;
//extern SINGEN SinGentst;
//extern SINGEN SinGentst1;
//extern POS_REG PosReg;
//extern ENCPOSFB	EncPosFb;
//extern POS_REF	PosRef;
//extern POSFB PosFb;
//extern POSFB SpdPosFb;
//extern SPDFB SpdFb;
//extern CUR_MOTPALEARN MotPaLearn;
//extern CUR_MONITOR CurMonitor;
//extern SPD_MONITOR SpdMonitor;
//extern SCURVEACCDEC SpdScurve;
//extern SCURVEACCDEC PosScuv;
//extern MULTISPD MultiSpd;
//extern POSGENERATOR PosGenerator;
//extern POSGENERATOR Poscia402Gen;
//extern POSGENERATOR HomingGen;
//extern POSGENERATOR SysAdjustGen;
//extern MULTIPOS MultiPos;
//extern POS_MONITOR PosMonitor;
//extern SPD_GAINADPT SpdGainAdpt;
//extern RAMPGENERATOR RampKp;
//extern RAMPGENERATOR RampKi;
//extern RAMPGENERATOR RampKPp;
//extern RAMPGENERATOR TorqGen;
//extern SPD_OFFLINEJIDT OffLineJidt;
//extern SPD_ONLINEJIDT OnLineJidt;
//extern SPD_INSTSSPDOBSER InstSpdObser;
//extern CUR_PIGAINADPT CurPiGainAdpt;
//extern POS_CMXCDV PosCmxCdv;
//extern POS_SMOOTH PosSmooth;
//extern POS_DAMPING PosDamping;
//extern STEPGEN StepGen;
//extern SPDRESPONSETEST SpdRespTest;
#define CSPMETHOD 2//位置同步模式处理方式选择
#endif /* DRIVE_H_ */





