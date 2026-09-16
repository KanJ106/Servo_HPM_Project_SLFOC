/*
 * SV_CtrlMode.h
 *
 *  Created on: 2016-1-11
 *      Author: zhangwei
 */

#ifndef SV_CTRLMODE_H_
#define SV_CTRLMODE_H_
//
#include "Datatype.h"//

#define CM_ModeSwitchDI			(Glo_DI_FUNC.Flag_Func0.bit.MSEL)
#define CM_MOTORSPD				(Common_Abs(RamMonitor->RotatingSpd))

#define CMD_NO        0//没有给定命令
#define CMD_INTERNAL  1//当前命令来自内部
#define CMD_REMOTE    2//当前命令来自外部
#define CMD_CMBUS     3//当前命令来modbus
#define CMD_SCOPE     4

#define  ACTUALMODE_POS   1
#define  ACTUALMODE_SPD   2
#define  ACTUALMODE_TOR   3
#define  ACTUALMODE_HOME  4

typedef enum
{
	PULSESTATE_SPD = 0,					//0：速度控制模式
	PULSESTATE_POS = 1,					//1：位置控制模式
	PULSESTATE_TRQ = 2,					//2：转矩控制模式
	PULSESTATE_SPD2POS = 3,				//3：速度位置切换模式（零速条件）
	PULSESTATE_TRQ2SPD = 4,				//4：转矩速度切换模式（零速条件）
	PULSESTATE_POS2TRQ = 5,				//5：位置转矩切换模式（零速条件）
	PULSESTATE_SPD2POS2 = 6,			//6：速度位置切换模式2（直接）
	PULSESTATE_TRQ2SPD2 = 7,			//7：转矩速度切换模式2（直接）
	PULSESTATE_POS2TRQ2 = 8				//8：位置转矩切换模式2（直接）
}ENUM_PULSE_STATE;

typedef enum
{
    ETHERCATSTATE_MIT = -6,
    ETHERCATSTATE_TORCLOSE = -1,
	ETHERCATSTATE_NULL0 = 0,			//NULL0
	ETHERCATSTATE_PP = 1,				//Profile position mode 轨迹位置模式
	ETHERCATSTATE_VL = 2,				//Velocity mode 速度模式（一般不支持）
	ETHERCATSTATE_PV = 3,				//Profile velocity mode 轨迹速度模式
	ETHERCATSTATE_TQ = 4,				//Profile torque mode 轨迹转矩模式
	ETHERCATSTATE_NULL5 = 5,			//NULL5
	ETHERCATSTATE_HM = 6,				//Homing mode 原点回归模式
	ETHERCATSTATE_IP = 7,				//Interpolated position	mode 插补位置模式（汇川不支持）
	ETHERCATSTATE_CSP = 8,				//Cyclic sync position mode 周期同步位置模式
	ETHERCATSTATE_CSV = 9,				//Cyclic sync velocity mode 周期同步速度模式
	ETHERCATSTATE_CST = 10,				//Cyclic sync torque mode 周期同步转矩模式
    ETHERCATSTATE_PVT = 128				//PVT mode 自定义
}EUNM_ETHERCAT_STATE;//CIA 402

typedef enum
{
	CTRLMODE_SPD = 0,					//0:速度模式
	CTRLMODE_POS = 1,					//1:位置模式
	CTRLMODE_TRQ = 2,					//2:转矩模式
    CTRLMODE_HOME = 3,					//3:回零模式
}ENUM_CTRLMODE;

typedef struct{
	ENUM_CTRLMODE		CtrlMode;				//控制模式（0：速度；1：位置；2：转矩）
	ENUM_CTRLMODE       ScopeMode;
    EUNM_ETHERCAT_STATE ECATCtrlMode;           //Cia402里面的控制模式
	EUNM_ETHERCAT_STATE LastECATCtrlMode;       //Cia402里面的控制模式
    Uint16              ActualMode;             //控制模式，无论指令源来自哪里
    Uint16              CtrlSoure;              //控制指令源
	Uint16				CtrlModExHibitFlg;		//控制模式禁止切换标志
	void				(*deal)();				//切换函数
}TYPE_CtrlMode;

extern TYPE_CtrlMode CtrlMode;			//控制模式相关结构体
extern void CtrlModeSwitch(void);		//控制模式切换

#endif /* SV_CTRLMODE_H_ */
/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
