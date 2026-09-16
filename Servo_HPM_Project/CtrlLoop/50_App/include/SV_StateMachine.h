/*
 * SV_StateMachine.h
 *
 *  Created on: 2015-12-16
 *      Author: zhangwei
 */

#ifndef SV_STATEMACHINE_H_
#define SV_STATEMACHINE_H_

#include "Datatype.h"
#include "SV_Do.h"

#define STOPMODESLOW1   1
#define STOPMODEQUICK1  2
#define STOPMODESLOW2   3
#define STOPMODEQUICK2  4
#define STOPMODEFAULT   5
#define STOPMODEZERO    6
#define STOPMODEEMER    7  //急停

#define SM_POWERUPDELAY             (1000)                           //上电延时时间

#define SM_GLOSPD					(Common_Abs(RamMonitor->RotatingSpd))	//转速
#define STOPSPEED                   24        //伺服停机时间
#define	SM_FAULTSTATUS				(FaultP.ErrDoFlg)		//故障标志
#define	SM_DO_SRDY_F1				(Glo_DO_FUNC.Flag_Func0.bit.S_RDY)	//DO SRDY_F1输出
#define SM_CMDIN					(StateMachine.CmdIn.all&0xfff)	//RUN综合，除了ECAT给定外
#define ECATSM_CMDIN				(StateMachine.CmdIn.bit.ECAT|\
		                             StateMachine.CmdIn.bit.Inertia|\
		                             StateMachine.CmdIn.bit.LRN|\
		                             StateMachine.CmdIn.bit.JOG|\
									 StateMachine.CmdIn.bit.INRUN|\
									 StateMachine.CmdIn.bit.FFt)

#define	SM_DI_RUN					(Glo_DI_FUNC.Flag_Func0.bit.S_ON)	//DI_RUN

#define SM_BRAKECTRL_OUT			(Glo_DO_FUNC.Flag_Func0.bit.BK)		//制动器抱闸输出
#define SM_BRAKECTRL_HOLD			(0)			//控制有效（抱闸）
#define SM_BRAKECTRL_LOOSE			(1)			//控制无效（松闸）


typedef union{
	Uint16 all;
	struct READY{
		Uint16 DC:1;			//运行使能-母线条件（继电器吸合后，清零）
		Uint16 ERR:1;			//运行使能-故障条件（无故障清零）
		Uint16 PowerOn:1;		//运行使能-参数条件（需上电生效参数被更改，置1，禁止运行）
		Uint16 rsvd:13;
	}bit;
}TYPE_RUNREADY;

typedef union{
	Uint16 all;
	struct CMD{
		Uint16 TML:1;			//外部端子
		Uint16 COMM:1;			//通讯
		Uint16 JOG:1;			//JOG
		Uint16 INRUN:1;			//内部

		Uint16 LRN:1;			//自学习
		Uint16 Inertia:1;       //惯量自学习
		Uint16 ECAT   :1;       //EtherCAT使能
		Uint16 FFt    :1;       //FFT
        Uint16 Mbusrun:1;       //FFT
        Uint16 Scoperun:1;
		Uint16 rsvd:6;
	}bit;
#define CMDECAT_ENABLE 0x40
}TYPE_CMDINPUT;

typedef enum{
	SM_BRAKESTATUS_BRAKE = 0,		//抱闸状态
	SM_BRAKESTATUS_2LOOSE = 1,		//准备松闸
	SM_BRAKESTATUS_LOOSE = 2,		//松闸状态
	SM_BRAKESTATUS_2BRAKE = 3		//准备抱闸
}ENUM_SM_BRAKESTATUS;

typedef struct{
	TYPE_RUNREADY		Ready;					//启动准备
	TYPE_CMDINPUT		CmdIn;					//运行命令输入

	Uint16 				PowerUpDly;				//上电延时处理
	Uint16				OutSRDY_F1;				//SRDY_F1
	Uint16				SrvOnIn;				//SRV_ON IN
	Uint16				SrvOnStatus;			//SRV_ON STATUS(0/1)
	Uint16				SrvOnStatusOld;			//SRV_ON STATUS_OLD
	Uint16				SrvStopCmd;             //ECAT中用于表示停止命令下发，0表示停止命令已经执行完成，1正在执行停止命令
	Uint16				SrvOffCnt;				//SRV-OFF计时
	Uint16				SrvOffCntBase;			//0.01S
	Uint16				InjectStep;				//自举电容充电步骤（目前没有此东西）
	Uint16				BootStrapFlg;			//开始充电
	Uint16				ChargeTimer;			//充电时间计数
	Uint16				RegulFlg;				//调节器使能标志,键盘run显示调用
    Uint16              RegilFlgISR;            //1ms中先赋值，然后再主循环结束赋值给RegulFlg
	Uint16				SvpwmOnFlg;				//打开PWM标志（一次性）
	int16				StopMode;				//停机模式（最终处理结果，主要考虑故障分级）
	Uint16				OffZSpdStopFlg;			//伺服OFF零速停车标志,等于1表示慢速停车，为2表示快速停车
    Uint16				HaltStopFlg;			//伺服halt零速停车标志
	ENUM_SM_BRAKESTATUS	BrakeStatus;			//抱闸状态(0:抱闸；1：准备松闸；2：松闸；3：准备抱闸)
	Uint16				SrvOnToLooseCnt;		//运行指令到达延时
	Uint16				LooseBrakeCnt;			//松闸延时
	Uint16				SrvOffToBrakeCnt;		//运行指令撤销延时
	Uint16				HoldBraekCnt;			//抱闸延时
	Uint16				BrakeChangePowerCnt;	//制动切换电源延时（低压伺服使用，从48V电源打开切换到5V电源保持）
	Uint16				OzSvStFlg;				//伺服是否可以接收（实际怎么走）指令（不是是否使能）
    Uint16              OffDbFlg;               //关闭使能后是否打开动态制动
    Uint16              EStopFlag;              //急停标志
}TYPE_STATEMACHINE;

extern void StateMachine_Process(void);   //---状态机处理---//
extern void SM_ParaRefresh(void);	      //状态参数更新
extern void SM_ParaRefreshFixtime(void);  //参数刷新-调度执行
extern TYPE_STATEMACHINE StateMachine;
extern void PWMOutDisable(void);
extern void PWMOutDisableDB(void);
extern Uint16 ECATSM_StopModeDeal(void);
#endif /* SV_STATEMACHINE_H_ */
/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
