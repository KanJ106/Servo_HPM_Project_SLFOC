/*
 * cia402ControlMode.h
 *
 *  Created on: 2016-1-27
 *      Author: sunzhixin//
 */

#ifndef CIA402CTRL_H_
#define CIA402CTRL_H_
#include "OD.h"
#include "r_cg_Project.h"
/*
 * 0x607D
 */
typedef struct  {
   INT32    i32MinLimit; /**< \brief Minimum limit*/
   INT32    i32MaxLimit; /**< \brief Maximum limit*/
} TOBJ607DH;
/*
 * 0x607E按位置方式解析
 */
typedef struct
{
	UINT16 unused6:6;
	UINT16 VelPol:1;
	UINT16 PosPol:1;
	UINT16 unused8:8;
}TOBJ607EBit;
typedef union{
	UINT16 All;
	TOBJ607EBit Bit;
}TOBJ607E;
/**************0x6040按位置方式进行解析**********************/
//位置控制模式PP
typedef struct
{
	UINT16 unused0:4;
	UINT16 NewSetPoint:1;  //bit4
	UINT16 ChangeSetImmd:1;//bit5
	UINT16 absrel:1;
	UINT16 unused1:1;
	UINT16 Halt:1;
	UINT16 ChangeOnSetpoint:1;
	UINT16 unused8:6;
}TOBJPP6040Bit;
//插补模式IP
typedef struct
{
	UINT16 unused0:4;
	UINT16 CSPstart:1; //bit4
	UINT16 Unused1:3;
	UINT16 Halt:1;       //bit8
	UINT16 unused2:7;
}TOBJIP6040Bit;

//周期性位置模式CSP
typedef struct
{
	UINT16 unused0:4;
	UINT16 CSPstart:1; //bit4
	UINT16 unused2:11;
}TOBJCSP6040Bit;

//原点回归模式HM
typedef struct
{
	UINT16 unused0:4;
	UINT16 HMstart:1; //bit4
	UINT16 unused1:3;
	UINT16 Halt:1;    //bit8
	UINT16 unused2:7;
}TOBJHM6040Bit;
//PV模式
typedef struct
{
	UINT16 unused0:8;
	UINT16 Halt:1;    //bit8
	UINT16 unused2:7;
}TOBJPV6040Bit;
//周期性速度模式CSV
typedef struct
{
	UINT16 unused0:4;
	UINT16 CSVstart:1; //bit4
	UINT16 unused2:11;
}TOBJCSV6040Bit;

//力矩模式
typedef struct
{
	UINT16 unused0:8;
	UINT16 Halt:1;    //bit8
	UINT16 unused2:7;
}TOBJTQ6040Bit;
//周期性速度模式CST
typedef struct
{
	UINT16 unused0:4;
	UINT16 CSTstart:1; //bit4
	UINT16 unused2:11;
}TOBJCST6040Bit;

typedef union{
	UINT16 All;
	TOBJPP6040Bit  PPBit;
	TOBJIP6040Bit  IPBit;
	TOBJCSP6040Bit CSPBit;
	TOBJHM6040Bit  HMBit;
	TOBJPV6040Bit  PVBit;
	TOBJCSV6040Bit CSVBit;
	TOBJTQ6040Bit  TQBit;
	TOBJCST6040Bit CSTBit;
}TOBJ6040;
/*
 * 0x6041按位置模式进行解析
 */
typedef struct
{
	UINT16 unused0:10;
	UINT16 TargetReached:1;
	UINT16 unused1:1;
	UINT16 Setpointack:1;
	UINT16 Followingerror:1;
	UINT16 unused8:2;
}TOBJPP6041Bit;
typedef union{
	UINT16 All;
	TOBJPP6041Bit Bit;
}TOBJPP6041;

typedef struct
{
	UINT16 FullVelReached:1;        //速度到达判断
	UINT16 FullPosReached:1;        //到达判断中用于判断位置到达,表示总的路径
	UINT16 Followingerror:1;        //跟随误差模块用于表示是否有跟随误差
	UINT16 ThisPosFinished:1;       //用于表示本次路径完成，只表示该段
	UINT16 GiveLimit:1;             //用于表示给定使受到限制的
	UINT16 BufferFull:1;            //表示缓冲区满
	UINT16 Polar:1;                  //极性
	UINT16 unused1:9;               //暂时没有用到
}givebit;
typedef union{
	UINT16 All;
	givebit Bit;
}GiveControl;
//HM
typedef struct
{
	UINT16 TargetReached:1;            //
	UINT16 HomingAttained:1;           //
	UINT16 HomingError:1;              //
	UINT16 unused8:13;                 //暂时没有用到
}HMbit;
typedef union{
	UINT16 All;
	HMbit Bit;
}HMStatus;

typedef struct  {
   INT32    i32MinLimit; /**< \brief Minimum limit*/
   INT32    i32MaxLimit; /**< \brief Maximum limit*/
}
OBJ607D;

/** \brief 0x6099 (Homing Speeds) data structure*/
typedef struct  {
   UINT32    i32Speedforswitch; /**< \brief Speed during search for switch*/
   UINT32    i32Speedforzero; /**< \brief Speed during search for zero*/
}
OBJ6099;

/** \brief Object 0x60C2 (Interpolation Time Period) data structure*/
typedef struct  {
   UINT32    u8InterpolationPeriod; /**< \brief Interpolation period*/
   INT8     i8InterpolationIndex; /**< \brief Interpolation index*/
}
OBJ60C2;

typedef struct {
	GiveControl  ConBit;             //缓存是满的
	TOBJ6040  ControlWord;           //控制字
	/*位置模式PP,速度模式PV*/
	INT32   TargetPos607A;           //目标位置，用户单位
	INT32   TargetPostion;           //目标位置，总是以脉冲形式给定轨迹发生器，采用绝对位置
	INT32   TargetVelo;              //目标速度，脉冲(标幺值)
	INT32   TargetSpd;               //目标速度，脉冲（非标幺值）
	INT32   TargetAcc;               //目标加速度，脉冲
	INT32   TargetDec;               //目标减速度，脉冲
	INT32   TargetTorque;            //目标扭矩
	INT32   TargetSlop;              //目标扭矩斜率
	/*周期性位置模式*/
	INT32   LTargetPostion;          //记录上一次位置值
	INT32   VelOffset60B1;           //速度偏移0x60B1，经过转换之后的值
	INT32   TorqueOffset60B2;        //力矩偏移0x60B2
	OBJ60C2 InterTime60C2;           //插补时间0x60C2
	INT32   Posdeta;                 //插补间隔
	INT32   Posdetarem;              //插补周期余数
	INT32   MaxTorque;               //最大扭矩0x6072
    
    UINT16   MaxPosTorque;            //最大正扭矩
    UINT16   MaxNegTorque;            //最大负扭矩

	/*原点回归模式HM*/
	INT8     HomeMethod;            //原点回归方法0x6098
	OBJ6099  HomingSpeeds;          //原点回归速度0x6099
	UINT32   HomingAcc;             //原点回归加速度0x609A
	INT32    HomeOffset;            //原点偏移0x607C

    INT32    TorqueSpdLim;
    float    TargetTorqueClose;
/*速度模式PV*/
}Control_PPBuffer;


#define ControlPPBufferValue  {\
/*ConBit*/                    {0},\
/*ControlWord*/               {0},\
/*TargetPos607A*/             0,\
/*TargetPostion*/             0,\
/*TargetVelo*/                0,\
/*TargetSpd*/                 0,\
/*TargetAcc*/                 0,\
/*TargetDec*/                 0,\
/*TargetTorque*/              0,\
/*TargetSlop*/                0,\
/*LTargetPostion*/            0,\
/*VelOffset60B1*/             0,\
/*TorqueOffset60B2*/          0,\
/*InterTime60C2*/             {0,0},\
/*Posdeta*/                   0,\
/*Posdetarem*/                0,\
/*MaxTorque*/                 0,\
/*MaxPosTorque*/              0,\
/*MaxNegTorque*/              0,\
/*HomeMethod*/                0,\
/*HomingSpeeds*/              {0,0},\
/*HomingAcc*/                 0,\
/*HomeOffset*/                0,\
}

typedef struct {
	Control_PPBuffer PPBuffer0;        //给定模块一级缓冲
	Control_PPBuffer PPBuffer1;        //给定模块二级缓冲
	GiveControl StatusConbit;          //控制与状态标志位

	TOBJ6040    LastControlWord;         //0x6040,上一次的命令字
	INT32 SlowDownRampTime;              //SlowDown减速时间
	INT32 QuickStopRampTime;             //QuickDown减速时间
    
	/*反馈中间变量*/
	INT32 PosrefDeltaMax;                //CSP模式下正向边界值
	INT32 PosrefDeltaMin;                //CSP模式下负向边界值
	HMStatus HMstatusWord;               //HM反馈状态
	INT32 PosDemVal60FC;                 //轨迹发生器位置生成值
	INT32 VelDemVal606B;                 //轨迹发生器速度生成值
	INT32 TorDemVal6074;                 //轨迹发生器力矩生成值
	INT32 GetVel606C;                    //内部单位速度值,606c
	INT32 GetPos6063;                    //反馈回来的位置值
	INT32 GetTor6077;                    //反馈回来的转矩

    INT32   PosFbInitValue;
    INT32   PosFbInitValue_user;
    INT32   PosOffset;

	/*位置到达判断中间变量*/
	UINT16 ReachedTime;                 //位置到达时间判断计时器
	UINT16 FollowErrtime;               //跟随误差判断计时器
	UINT16 VelReachedTime;              //速度到达时间计数器
	UINT16 VelZeroTime;                 //零速判断到达时间
	UINT16 TqReachedTime;               //力矩到达时间
	UINT16 CycleTime;                   //同步时间单位
    float  GetTorqueClose;              //反馈回来的力矩传感器值
}Cia402PP;

#define DefCia402PPValue {\
 /*PPBuffer0*/                       ControlPPBufferValue,\
 /*PPBuffer1*/                       ControlPPBufferValue,\
 /*PPStatusConbit*/                  {0},\
 /*LastControlWord*/                 {0},\
 /*SlowDownRampTime*/                  0,\
 /*QuickStopRampTime*/                 0,\
 /*polatity*/                        {0},\
 /*polatity*/                        {0},\
 /*HMstatusWord*/                    {0},\
 /*PosDemVal60FC*/                     0,\
 /*VelDemVal606B*/                     0,\
 /*TorDemVal6074*/                     0,\
 /*GetVel606C*/                        0,\
 /*GetPos6063*/                        0,\
 /*GetTor6077*/                        0,\
 /*PosFbInitValue*/                    0,\
 /*PosFbInitValue_user*/               0,0,\
 /*ReachedTime*/                       0,\
 /*FollowErrtime*/                     0,\
 /*VelReachedTime*/                    0,\
 /*VelZeroTime*/                       0,\
 /*TqReachedTime*/                     0,\
 /*CycleTime*/                         2,\
 /*GetTorqueClose*/                 0.0f,\
 }

extern Cia402PP  Cia402_PP;/*位置控制模块*/

#if SERVOTYPE == SERVO_CANOPEN
typedef Cia402PP* Cia402PP_handle;
extern void Control_Mode(void);
extern void Location_TargetLimit(void);
extern void cia402_FeedbackHanle(Cia402PP_handle p);
#endif //#if SERVOTYPE == SERVO_CANOPEN

#endif /* CIA402CONTROLMODE_H_ */



















