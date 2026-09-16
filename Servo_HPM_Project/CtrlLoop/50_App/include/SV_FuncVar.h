#ifndef _SV_FUNCVAR_
#define _SV_FUNCVAR_
//
#include "Datatype.h"//

/***********掉电参数组P00****************/
typedef struct
{
	Uint16 Para1;			//PC-14 累计上电时间h
	Uint16 Para2;			//PC-15 累计上电时间min
	Uint16 Para3;      		//PC-16 累计运行时间h
	Uint16 Para4;    		//PC-17 累计运行时间min

	Uint16 Para5;			//F18.14 电度表MWh
	Uint16 Para6;			//F18.15 电度表KWh
	Uint16 Para7;
	Uint16 Para8;
	
	Uint16 Para9;			//F18.09 PLC运行次数
	Uint16 Para10;			//F18.10 PLC运行时间
	Uint16 Para11;			//F18.11 PLC运行阶段
	Uint16 Para12;
	
	Uint16 Para13;			//F18.39  制动检测时间显示h
	Uint16 Para14;			//F99.65 制动器检查时间min
	Uint16 Para15;
	Uint16 Check;			//保存校验字节
}TYPE_PowerDownArg;//16

/***********监视参数组*P0****************/
typedef struct
{
	int16	RotatingSpd;						//P0-00电机转速
	int16	LoadFactor;							//P0-01电机负载率
	Uint16	ElecAngle;							//P0-02电角度
	Uint16	DILevel;							//P0-03DI输入电平
	Uint16	DOLevel;							//P0-04DO输入电平

	Uint16	EncMulturnPos;						//P0-05编码器多圈位置
	Uint32	T_SysRun;							//P0-06系统运行时间
//	Uint16	T_SysRunH;
	Uint16	McuTemper;						    //P0-08AI1电压值
	Uint16	MotorTemper;					    //P0-09AI2电压值

	Uint16	EncSector;							//P0-10编码器扇区号
	Uint16	Udc;						    	//P0-11直流母线电压
	Uint16	Irms;								//P0-12电流有效值
	Uint16	ServoState;							//P0-13驱动器状态	//TODO:需细化到0~10均为什么
	Uint16	BrkFactor;							//P0-14制动负载率

	Uint16	TIPM;								//P0-15IPM模块温度
	int32	PulseAquNumL;						//P0-16采集到的脉冲总数
//	Uint16	PulseAquNumH;
	Uint16	FaultType;							//P0-18故障类型
	Uint16	FaultSpd;							//P0-19故障时转速

	Uint16	FaultUdc;							//P0-20故障时母线电压
	Uint16	FaultIrms;							//P0-21故障时电流有效值
	Uint16	T_FaultL;							//P0-22故障时温度
	Uint16	T_FaultH;                           //P0-23
	Uint16	EncSingleturnPosL;					//P0-24编码器单圈位置值

	Uint16	EncSingleturnPosH;
	Uint16	LoadInertia;						//P0-26负载惯量
	Uint16	LoadInertiaRatio;					//P0-27负载惯量比
	int32	FdPulseNumL;						//P0-28反馈脉冲数
//	Uint16	FdPulseNumH;

	Uint16	IputPulseFreqL;						//P0-30接受到的外部脉冲频率
	Uint16	IputPulseFreqH;						//
	Uint32	CPUASer1;							//P0-32 CPUA序列号1
	//Uint16	EEASer1;						    //P0-33 EEPROM序列号
    
	Uint16	CRC_ErrCount;						//P0-34 CRC错误计数
	Uint16	NoAckTime_Count;					//P0-35 编码器无数据计数
	Uint16	CE_ErrCount;						//P0-36 CE
    
	Uint16	CRC_ErrCountS;						//P0-37
	Uint16	NoAckTime_CountS;				    //P0-38 
	Uint16	CE_ErrCountS;						//P0-39

	Uint16	EcatStatus;						    //P0-40 Ecat状态
	int16   SpdCmd;                             //P0-41 速度命令
	Uint32  MagSingle;                          //P0-42 磁编码器单圈值
    
    Uint16  ServoErrCode;                       //P0-44
    Uint16  EcatErrCode;                        //P0-45
    Uint16  HardWareNum;                        //P0-46
    Uint16  MotorIe;                            //P0-47
    
    Uint16  ServoCode;                          //P0-48
    Uint16  rsd49;                              //P0-49  //扭矩千分比 
     int32  ExPos;                              //P0-50
    //Uint16  rsd51;                            //P0-51
    
    int32  MortorPos;                           //P0-52
    //Uint16  rsd53;                            //P0-53
    Uint16  CtrSoure;                           //P0-54
    Uint16  rsd55;                              //P0-55
    
    Uint16  rsd56;                              //P0-56
    Uint16  rsd57;                              //P0-57
    int32   Torque_Nm;                          //P0-58  //0.1NM
    //Uint16  rsd59;                              //P0-59
    
    int16   Torque_Per;                         //P0-60  //扭矩千分比 
    Uint16  rsd61;                              //P0-61
    Uint16  rsd62;                              //P0-62
    Uint16  rsd63;                              //P0-63
    Uint16  rsd64[48];                          //P0-64~111
}TYPE_MONITOR;//112

/***********************基本参数组*P1***************************/
typedef struct
{
	Uint16	DrvMode;							//P1-00 控制模式设定
	Uint16	PosCmdSource;						//P1-01 位置指令源选择
	Uint16	PulseCmdType;						//P1-02 外部脉冲指令输入形式
	Uint16	MulSegPosTurn;						//P1-03 内部多段位置指令执行选择
	Uint16	MotDir;							    //P1-04 电机方向

	Uint16	SpdCmdSource;						//P1-05 速度指令源
	Uint16	MulSegSpdTurn;						//P1-06 多段速选择
	Uint16	DivOutput;							//P1-07 分频输出
	Uint16	ModbusMode;						    //P1-08 ModBus 控制 0：其他控制 1：Modbus控制
	Uint16	PosJogMode;							//P1-09 上位机控制模式

	Uint16	PosInternalMode;					//P1-10 开启上位机控制
	Uint16	PosSpdJog;				            //P1-11 上位机位置速度
	Uint16	T_CCW_AccDecPos;					//P1-12 位置S型平滑正转指令加减速时间
	Uint16	T_CW_AccDecPos;						//P1-13 位置S型平滑反转指令加减速时间
	Uint16	T_PosLpf;							//P1-14 位置S型平滑曲线加减速滤波时间常数

	Uint16	T_SpdAcc;							//P1-15 速度S型平滑指令加速时间常数
	Uint16	T_SpdDec;							//P1-16 速度S型平滑指令减速时间常数
	Uint16	T_SpdAccDecLpf;						//P1-17 速度S型平滑曲线加减速滤波时间常数
	Uint16	T_ZeroStop;							//P1-18 零速停车减速时间
	Uint16	T_Overtravel;						//P1-19 超程保护减速时间  

	Uint16	S_FaultDec;						    //P1-20 二级故障零速停车时间
	Uint16	Delaytime;						    //P1-21 
	int32	TwoMultiPos1;						//P1-22 
	//Uint16	rsd23;					        //P1-23 
	int32	TwoMultiPos2;			            //P1-24

	//Uint16	rsd25;			                //P1-25
	Uint16	PrescalerNumeratorL;				//P1-26 电子齿轮分倍频分子L
	Uint16	PrescalerNumeratorH;				//P1-27 电子齿轮分倍频分子H
	Uint16	PrescalerDenominatorL;				//P1-28 电子齿轮分倍频分母L
	Uint16	PrescalerDenominatorH;				//P1-29 电子齿轮分倍频分母H

	Uint16	PrescalerSource;				    //P1-30 电子齿轮比来源   0 ECAT   1 Pr1.26
	Uint16	PrescalerNumerator4;				//P1-31 电子齿轮分倍频分子4
	Uint16	PosApproachWidth;					//P1-32 定位接近宽度
	Uint16	PosFinishWidth;						//P1-33 定位完成宽度
	Uint16	DIClearOffset;						//P1-34 外部DI信号偏差清除动作选择

	Uint16	AutoClearOffset;					//P1-35 位置偏差自动清除选择
	Uint16	OffsetFlowWarnLmt;					//P1-36 位置误差跟随警告值
	Uint16	OffsetFlowFaultLmt;					//P1-37 位置误差跟随故障值
	Uint16	TrqArriveLmt;						//P1-38 转矩到达门限值
	Uint16	SpdMaxLmt;							//P1-39 最高转速限定

	Uint16	ZeroSpdLmt;							//P1-40 零速信号输出值
	Uint16	RotatingSpdLmt;						//P1-41 旋转信号输出值
	Uint16	SpdApproachLmt;						//P1-42 速度接近门限
	Uint16	SpdArriveLmt;						//P1-43 速度到达门限
	Uint16	AnoSpdZeroLmt;						//P1-44 模拟速度零位固定值

	Uint16	Z_PulsOutputWidth;					//P1-45 Z脉冲输出宽度
	Uint16	CCW_TorqMaxLmt;						//P1-46 正转最大转矩限定
	Uint16	CW_TorqMaxLmt;						//P1-47 反转最大转矩限定
	Uint16	TrqLmtResource;						//P1-48 转矩限制来源选择
	Uint16	rsd49;					            //P1-49 

	Uint16	rsd50;						        //P1-50 
	Uint16	reserve51;							//P1-51 
	Uint16	ErrStopMode;						//P1-52 二级故障停机模式选择
	Uint16	StopMode;							//P1-53 断使能停机模式选择
	Uint16	LooseBrakeDelay;					//P1-54 伺服ON-接收指令延迟时间

	Uint16	HoldBraekDelay;						//P1-55 制动指令-伺服OFF延迟时间（静止）
	Uint16	SrvOffToBrakeSpd;					//P1-56 制动指令输出时的速度值（旋转）
	Uint16	SrvOffToBrakeDealy;					//P1-57 伺服OFF-制动指令等待时间（旋转）
	Uint16	OutputPulseDir;						//P1-58 输出脉冲反馈方向选择
												//P1-59 基本参数扩展功能
	Uint16	SpdDir:1;     			        	//P1-59 bit0:速度指令反向（速度模式有效）
	Uint16	RotPolar:1;     			        //P1-59 bit1:旋转方向极性旋转
	Uint16  usedP159:14;                        //P1-59 bit1-bit15:保留

	Uint16	FaultDisplay;						//P1-60 故障显示选择
	Uint16	ParaInit;							//P1-61 系统参数初始化
	Uint16	FactoryCode1;						//P1-62 EEPROM校验
	Uint16	FactoryCode2;						//P1-63 厂家参数2
    Uint16	reserve64[16];						//P1-64~79
}TYPE_BASEPARA;//80

/*********** 多段位置参数组*P2****************/
typedef struct
{
	int32 MultiPos1L;							//P2-00内部位置指令1脉冲数
//	Uint16 MultiPos1H;
	Uint16 MultiPos1Spd;						//P2-02内部位置指令1移动速度
	Uint16 T_Pos1Delay;							//P2-03Pr1完成后进入Pr2等待时间
	int32 MultiPos2L;							//P2-04内部位置指令2脉冲数

//	Uint16 MultiPos2H;
	Uint16 MultiPos2Spd;						//P2-06内部位置指令2移动速度
	Uint16 T_Pos2Delay;							//P2-07Pr2完成后进入Pr3等待时间
	int32 MultiPos3L;							//P2-08内部位置指令3脉冲数
//	Uint16 MultiPos3H;

	Uint16 MultiPos3Spd;						//P2-10内部位置指令3移动速度
	Uint16 T_Pos3Delay;							//P2-11Pr3完成后进入Pr4等待时间
	int32 MultiPos4L;							//P2-12内部位置指令4脉冲数
//	Uint16 MultiPos4H;
	Uint16 MultiPos4Spd;						//P2-14内部位置指令4移动速度

	Uint16 T_Pos4Delay;							//P2-15Pr4完成后进入Pr5等待时间
	int32 MultiPos5L;							//P2-16内部位置指令5脉冲数
//	Uint16 MultiPos5H;
	Uint16 MultiPos5Spd;						//P2-18内部位置指令5移动速度
	Uint16 T_Pos5Delay;							//P2-19Pr5完成后进入Pr6等待时间

	int32 MultiPos6L;							//P2-20内部位置指令6脉冲数
//	Uint16 MultiPos6H;
	Uint16 MultiPos6Spd;						//P2-22内部位置指令6移动速度
	Uint16 T_Pos6Delay;							//P2-23Pr6完成后进入Pr7等待时间
	int32 MultiPos7L;							//P2-24内部位置指令7脉冲数

//	Uint16 MultiPos7H;
	Uint16 MultiPos7Spd;						//P2-26内部位置指令7移动速度
	Uint16 T_Pos7Delay;							//P2-27Pr7完成后进入Pr8等待时间
	int32 MultiPos8L;							//P2-28内部位置指令8脉冲数
//	Uint16 MultiPos8H;

	Uint16 MultiPos8Spd;						//P2-30内部位置指令8移动速度
	Uint16 T_Pos8Delay;							//P2-31Pr8完成后进入Pr9等待时间
	int32 MultiPos9L;							//P2-32内部位置指令9脉冲数
//	Uint16 MultiPos9H;
	Uint16 MultiPos9Spd;						//P2-34内部位置指令9移动速度

	Uint16 T_Pos9Delay;							//P2-35Pr9完成后进入Pr10等待时间
	int32 MultiPos10L;							//P2-36内部位置指令10脉冲数
//	Uint16 MultiPos10H;
	Uint16 MultiPos10Spd;						//P2-38内部位置指令10移动速度
	Uint16 T_Pos10Delay;						//P2-39Pr10完成后进入Pr11等待时间

	int32 MultiPos11L;							//P2-40内部位置指令11脉冲数
//	Uint16 MultiPos11H;
	Uint16 MultiPos11Spd;						//P2-42内部位置指令11移动速度
	Uint16 T_Pos11Delay;						//P2-43Pr11完成后进入Pr12等待时间
	int32 MultiPos12L;							//P2-44内部位置指令12脉冲数

//	Uint16 MultiPos12H;
	Uint16 MultiPos12Spd;						//P2-46内部位置指令12移动速度
	Uint16 T_Pos12Delay;						//P2-47Pr12完成后进入Pr13等待时间
	int32 MultiPos13L;							//P2-48内部位置指令13脉冲数
//	Uint16 MultiPos13H;

	Uint16 MultiPos13Spd;						//P2-50内部位置指令13移动速度
	Uint16 T_Pos13Delay;						//P2-51Pr13完成后进入Pr14等待时间
	int32 MultiPos14L;							//P2-52内部位置指令14脉冲数
//	Uint16 MultiPos14H;
	Uint16 MultiPos14Spd;						//P2-54内部位置指令14移动速度

	Uint16 T_Pos14Delay;						//P2-55Pr14完成后进入Pr15等待时间
	int32 MultiPos15L;							//P2-56内部位置指令15脉冲数
//	Uint16 MultiPos15H;
	Uint16 MultiPos15Spd;						//P2-58内部位置指令15移动速度
	Uint16 T_Pos15Delay;						//P2-59Pr15完成后进入Pr16等待时间

	int32 MultiPos16L;							//P2-60内部位置指令16脉冲数
//	Uint16 MultiPos16H;
	Uint16 MultiPos16Spd;						//P2-62内部位置指令16移动速度
	Uint16 T_Pos16Delay;						//P2-63Pr16完成后进入Pr1等待时间
}TYPE_MULTIPOS;//64

/***********速度控制参数组*P3****************/
typedef struct
{
	int16  MultiSpd1;							//P3-00内部速度指令1
	Uint16 T_RunSpd1;							//P3-01内部速度指令1运行时间
	int16  MultiSpd2;							//P3-02内部速度指令2
	Uint16 T_RunSpd2;							//P3-03内部速度指令2运行时间
	int16  MultiSpd3;							//P3-04内部速度指令3

	Uint16 T_RunSpd3;							//P3-05内部速度指令3运行时间
	int16  MultiSpd4;							//P3-06内部速度指令4
	Uint16 T_RunSpd4;							//P3-07内部速度指令4运行时间
	int16  MultiSpd5;							//P3-08内部速度指令5
	Uint16 T_RunSpd5;							//P3-09内部速度指令5运行时间

	int16  MultiSpd6;							//P3-10内部速度指令6
	Uint16 T_RunSpd6;							//P3-11内部速度指令6运行时间
	int16  MultiSpd7;							//P3-12内部速度指令7
	Uint16 T_RunSpd7;							//P3-13内部速度指令7运行时间
	int16  MultiSpd8;							//P3-14内部速度指令8

	Uint16 T_RunSpd8;							//P3-15内部速度指令8运行时间
	int16  MultiSpd9;							//P3-16内部速度指令9
	Uint16 T_RunSpd9;							//P3-17内部速度指令9运行时间
	int16  MultiSpd10;							//P3-18内部速度指令10
	Uint16 T_RunSpd10;							//P3-19内部速度指令10运行时间

	int16  MultiSpd11;							//P3-20内部速度指令11
	Uint16 T_RunSpd11;							//P3-21内部速度指令11运行时间
	int16  MultiSpd12;							//P3-22内部速度指令12
	Uint16 T_RunSpd12;							//P3-23内部速度指令12运行时间
	int16  MultiSpd13;							//P3-24内部速度指令13

	Uint16 T_RunSpd13;							//P3-25内部速度指令13运行时间
	int16  MultiSpd14;							//P3-26内部速度指令14
	Uint16 T_RunSpd14;							//P3-27内部速度指令14运行时间
	int16  MultiSpd15;							//P3-28内部速度指令15
	Uint16 T_RunSpd15;							//P3-29内部速度指令15运行时间

	int16  MultiSpd16;							//P3-30内部速度指令16
	Uint16 T_RunSpd16;							//P3-31内部速度指令16运行时间
}TYPE_MULTISPD;//32

/***********转矩控制参数组*P4****************/
typedef struct
{
	Uint16 TrqCmd;								//P4-00 内部转矩指令
	Uint16 TrqSpdLmt;							//P4-01 转矩控制时速度限幅
	Uint16 TrqSpdLmtResource;					//P4-02 转矩控制时速度限幅来源
	Uint16 TrqSpdLmtGain;						//P4-03 转矩速度限制指令增益
	Uint16 TrqCmd1;							    //P4-04

	Uint16 TrqCmdTime1;						    //P4-05 
	Uint16 TrqCmd2;							    //P4-06 
	Uint16 TrqCmdTime2;						    //P4-07 
	Uint16 TrqCmd3;				                //P4-08 
    Uint16 TrqCmdTime3;							//P4-09
    
    Uint16 TrqCmd4;							    //P4-10
    Uint16 TrqCmdTime4;							//P4-11
    Uint16 reserve12;							//P4-12
    Uint16 reserve13;							//P4-13
    Uint16 reserve14;							//P4-14
    Uint16 reserve15;							//P4-15
    
    Uint16 reserve16;							//P4-16
    Uint16 reserve17;							//P4-17
    Uint16 reserve18;							//P4-18
    Uint16 reserve19;							//P4-19
    
    Uint16 reserve20;							//P4-20
    Uint16 reserve21;							//P4-21
    Uint16 reserve22;							//P4-22
    Uint16 reserve23;							//P4-23
    
    Uint16 reserve24;							//P4-24
    Uint16 reserve25;							//P4-25
    Uint16 reserve26;							//P4-26
    Uint16 reserve27;							//P4-27
    
    Uint16 reserve28;							//P4-28
    Uint16 reserve29;							//P4-29
    Uint16 reserve30;							//P4-30
    Uint16 reserve31;							//P4-31
    Uint16 reserve32[16];						//P4-32~47
}TYPE_TRQCTL;//48

/***********调谐参数组*P5****************/
typedef struct
{
    Uint16 APR_P;								//P5-00 位置调节器比例增益  
	Uint16 ASR_P;								//P5-01 速度调节器比例增益
	Uint16 ASR_Ti;								//P5-02 速度环积分时间常数 
    Uint16 RegulateMode;						//P5-03 增益调整模式
    
    Uint16 APR_Kp;								//P5-04 速度前馈前馈增益
	Uint16 T_APRLpf;							//P5-05 速度前馈滤波时间常数
   	Uint16 ACC_Kp;								//P5-06 转矩前馈增益
	Uint16 T_ACCLpf;							//P5-07 转矩前馈滤波时间常数 
    
    Uint16 T_InputPulseLpf;                     //P5-08 位置指令一阶滤波时间
    Uint16 Pos_FIR;							    //P5-09 位置指令平滑滤波时间
    Uint16 T_SpdCmdLpf;                         //P5-10 速度指令滤波 
    Uint16 T_TrqCmdLpf;							//P5-11 电流指令滤波
    
    Uint16 T_FdSpdLpf;					        //P5-12 速度反馈低通滤波时间   
	Uint16 SpdFdbFiltNum;						//P5-13 速度反馈平滑滤波
    Uint16 SpdDf;						        //P5-14 速度频宽(刚性表)
    Uint16 LoadIR;								//P5-15 负载惯量比
    
	Uint16 APR_Ap;								//P5-16 位置增益变动比率
	Uint16 ASR_Ap;								//P5-17 速度增益变动比率
	Uint16 KpSwitch;							//P5-18 增益切换条件
	Uint16 T_KpSwitch;							//P5-19 增益切换时间
    
	Uint16 T_KpSwitchDelay;						//P5-20 增益切换延迟时间
	Uint16 SpdKpSwitch;							//P5-21 增益切换阈值
	Uint16 Kpdff;								//P5-22 PDFF控制系数
	Uint16 KpCtl;								//P5-23 控制环路系数
    
	Uint16 DynamicFriction;				        //P5-24 动摩擦力补偿
	Uint16 Friction_Kp;							//P5-25 动摩擦力补偿增益
	int16  TorqueAddedValue;					//P5-26 转矩指令加算值    重力补偿
	Uint16 TorqueCompensate_P;					//P5-27 正方向转矩补偿值
    
	Uint16 TorqueCompensate_N;					//P5-28 负向转矩补偿值
	Uint16 TFricCps;							//P5-29 摩擦力补偿平滑时间常数(默认值50ms，10~1000ms)
	Uint16 LoadIR2;							    //P5-30 第二惯量比
												//P5-31 抑制性能扩展
	Uint16 Accfeedback:1;                       //P5-31.Bit0:加速度反馈功能
	Uint16 Pulsesuppression:1;                  //P5-31.Bit1:脉冲抑制功能
	Uint16 LowSpdIntegral:1;                    //P5-31.Bit2:低速积分功能
	Uint16 SPDObs_Function1:1;                  //P5-31.Bit3:速度观测器功能1
	Uint16 SPDObs_Function2:1;                  //P5-31.Bit4:速度观测器功能2,附带停止无效
    Uint16 SPDFilterMode:1;                     //P5-31.Bit5:
	Uint16 Mask_rsvd29:10;                      //P5-31.Bit6-15:保留
    
	Uint16 Spddeadline;                         //P5-32 速度观测器截止频率
	Uint16 Spdfilter;                           //P5-33 
	Uint16 Torfilter;                           //P5-34 
	Uint16 Evobjgn;							    //P5-35 
    
	Uint16 APR_Ti;							    //P5-36 
	Uint16 ICPercent;							//P5-37  离线学习惯量时注入电流百分比
	Uint16 reserve38;							//P5-38
	Uint16 reserve39;							//P5-39  
    Uint16 reserve40;							//P5-40  
    Uint16 reserve41;							//P5-41 
    Uint16 reserve42;							//P5-42  
    Uint16 reserve43;							//P5-43  
    Uint16 reserve44;							//P5-44  
    Uint16 reserve45;							//P5-45  
    Uint16 reserve46;							//P5-46 
    Uint16 reserve47;							//P5-47 
    Uint16 reserve48[32];						//P5-48~79         
}TYPE_TUNING;//80
/***********输入参数组*P6****************/
typedef struct
{
	Uint16 T_DILpf;								//P6-00 DI滤波时间
	Uint16 DILogicLevel;						//P6-01 DI电平逻辑
	Uint16 DI1Fun;								//P6-02 DI1功能号
	Uint16 DI2Fun;								//P6-03 DI2功能号
	Uint16 DI3Fun;								//P6-04 DI3功能号

	Uint16 DI4Fun;								//P6-05 DI4功能号
	Uint16 DI5Fun;								//P6-06 DI5功能号
	Uint16 DI6Fun;								//P6-07 DI6功能号
	Uint16 DI7Fun;								//P6-08 DI7功能号
	Uint16 DI8Fun;								//P6-09 DI8功能号

	Uint16 reserve10;							//P6-10
	Uint16 reserve11;							//P6-11
	Uint16 DOLogicLevel;						//P6-12 DO电平逻辑
	Uint16 DO1Fun;								//P6-13 DO1功能号
	Uint16 DO2Fun;								//P6-14 DO2功能号

	Uint16 DO3Fun;								//P6-15 DO4功能号
	Uint16 DO4Fun;								//P6-16 DO4功能号

    Uint16  Gain_Analog1;                       //P6-17 模拟量1增益
    Uint16  Filter_Analog1;                     //P6-18 模拟量1低通滤波
	Uint16  Hys_Analog1;                        //P6-19 模拟量1滞环
	 int16  Bias_Analog1;                       //P6-20 模拟量1偏置
	Uint16  ZeroDri_Analog1;                    //P6-21 模拟量1零漂

    Uint16  Gain_Analog2;                       //P6-22 模拟量2增益
    Uint16  Filter_Analog2;                     //P6-23 模拟量2低通滤波
    Uint16  Hys_Analog2;                        //P6-24 模拟量2滞环
     int16  Bias_Analog2;                       //P6-25 模拟量2偏置
    Uint16  ZeroDri_Analog2;                    //P6-26 模拟量2零漂

    Uint16  Ao1CmdSource;                       //P6-27 AO1指令来源选择
    Uint16  Ao1_Bias;                           //P6-28 AO1偏置电压
    Uint16  Ao1_Gains;                          //P6-29 AO1倍率
    Uint16  Ao1Cmdtest;                         //P6-30 AO1指令输出测试

    Uint16  Ao2CmdSource;                       //P6-31 AO2指令来源选择
    Uint16  Ao2_Bias;                           //P6-32 AO2偏置电压
    Uint16  Ao2_Gains;                          //P6-33 AO2倍率
    Uint16  Ao2Cmdtest;                         //P6-34 AO2指令输出测试 
    Uint16  Rsd;                                //P6-35 AO2指令输出测试
    Uint16  Rsd1[12];                           //P6-36~63
}TYPE_IO;//48

/***********通讯参数组*P7****************/
typedef struct
{
	Uint16 Addrss;								//P7-00 Modbus通讯地址
	Uint16 Baud;								//P7-01 通讯波特率
	Uint16 Format;								//P7-02 通讯格式
	Uint16 AckDelay;							//P7-03 通讯响应延时时间
	Uint16 AckDelay1;							//P7-04 在标准基础上增加ms数
	Uint16 reserve05;							//P7-05 主机广播数据间隔时间(保留)
	Uint16 ModbusType;							//P7-06 0:+2000H存EEP；1:不加存EEP
	Uint16 NotEEP;								//P7-07 1:不存EEPROM
    
	Uint16 CheckDCFlag;							//P7-08    
	Uint16 SyncMode;							//P7-09
	Uint16 LostCheck;							//P7-10
	Uint16 reserve11;							//P7-11
	Uint16 reserve12;							//P7-12
	Uint16 reserve13;						    //P7-13
	Uint16 reserve14;					        //P7-14 
	Uint16 reserve15;					        //P7-15 
    
	Uint16 CanID;                               //P7-16  
	Uint16 CanBaud;                             //P7-17  
    Uint16 SMLost;                              //P7-18
    Uint16 CanFdBaud;                           //P7-19   
    Uint16  CanMode;                            //P7-20
    Uint16 reserve21;                           //P7-21
    Uint16 reserve22;                           //P7-22
    Uint16 reserve23;                           //P7-23
    
    Uint16 Addrss1;                             //P7-24
    Uint16 Baud1;                               //P7-25
    Uint16 Format1;                             //P7-26
    Uint16 AckDelay2;                           //P7-27
    Uint16 reserve28;                           //P7-28
    Uint16 reserve29;                           //P7-29
    Uint16 reserve30;                           //P7-30
    Uint16 reserve31;                           //P7-31
    Uint16 reserve32[16];                       //P7-32~47
}TYPE_COMMU;//48

/***********辅助功能参数组*P8****************/
typedef struct
{
	Uint16 SoftReset;							//P8-00 软件复位
	Uint16 FaultReset;							//P8-01 故障复位
	Uint16 JodRun;								//P8-02 点动运行
	Uint16 SpdJog;								//P8-03 点动速度
	Uint16 InertiaIdentificate;					//P8-04 离线惯量辨识

	Uint16 SON;									//P8-05 内部SON指令
	Uint16 FFT;							        //P8-06 FFT
	Uint16 MonitorDisplay;						//P8-07 监控默认内容选择
	int16  Set_JodRun;							//P8-08 上位机点动方向
	Uint16 SoftOTSel;						    //P8-09 软件软限位

	Uint16 ABZFltrTime;							//P8-10 脉冲输入滤波时间（GPIO串口频率配置）
	Uint16 FanCtl;								//P8-11 风扇控制
	Uint16 ServoOLAlarmPer;						//P8-12 驱动器过载警告设置
	Uint16 MotorOLAlarmPer;						//P8-13 电机过载警告设置
	Uint16 StopCondition;						//P8-14 停车SON条件选择

	Uint16 SONCondition;						//P8-15 SON条件
	Uint16 T_SONDelay;							//P8-16 SOFF后SON有效时间间隔
	Uint16 SpdSON;								//P8-17 SON有效速度设定
	Uint16 ResistorBrk;							//P8-18 制动电阻值
	Uint16 CapacityResistorBrk;					//P8-19 制动电阻容量

	Uint16 DutyBrk;								//P8-20 制动放电占空比
	Uint16 DerateBrk;							//P8-21 制动电阻降额百分比1%
	Uint16 CursorDirection;						//P8.22 光标移动方向
	Uint16 PanelType;                           //P8.23 键盘操作类型，0是普通，1加入AC选择
	Uint16 QuickStopTime;                       //P8-24 紧急停车时间

	Uint16 JOGTAccDec;                          //p8-25 JOG加减速时间
	Uint16 JidtMaxCircle;						//P8-26 离线惯量辨识最大圈数
	Uint16 EcatMinCycle;						//P8-27 COE通讯最小周期 0：:大于等于1ms 1：小于1ms
	Uint16 AbsType;                             //P8-28 用户使用绝对式编码器的使用方法 0：单圈使用1：绝对多圈使用 带编码器电池报警
	Uint16 SoftCurProtect;					    //P8-29 软件过流

	Uint16 BrakeLogic;							//P8-30 强制抱闸打开
	Uint16 Brake48V_Delay;						//P8-31 抱闸48V工作时间
	Uint16 BrakeUdc1;						    //P8-32 保留
	Uint16 BrakeUdc2;						    //P8-33 保留
	Uint16 CheckDCFlag1;						//P8-34 是否检测SM和DC
    
    Uint16 LockIqThreshold;						//P8-35 堵转电流阈值
    Uint16 LockTime;						    //P8-36 堵转时间
    Uint16 LockSel;							    //P8-37 堵转开关
                                                //P8-38 辅助共功能
    Uint16 BootStrapSel:1;					    //Bit0  自举开关
    Uint16 P8_38Rsd:15;					        //Bit1~15 保留
    
    Uint16 UpPowerValue;						//P8-39 保留
    Uint16 RunAwaySel;							//P8-40 飞车保护
	Uint16 ServoWarnTem;						//P8-41 驱动器温度警告值
	Uint16 ServoFaultTem;						//P8-42 驱动器温度故障值
	Uint16 MotorWarnTem;						//P8-43 电机过热警告
	Uint16 MotorFaultTem;						//P8-44 电机过热故障

    Uint16 UnuseP8_45;					    	//P8-45 保留
    int32  PosOffset;			                //P8-46 Ecat偏移
    //Uint16 UnuseP8_47;					    //P8-47 保留  
    Uint16 UnuseP8_48[16];					    //P8-48~63    
}TYPE_AID;//64

/***********保留参数组*P9****************/
typedef struct
{
	Uint16	VibrationFrquency1;					//P9-00 低频抑制频率
	Uint16	VibrationFrqSelection1;				//P9-01 低频抑制增益  
    Uint16	rad2;				                //P9-02
    
 	Uint16	VibrationFrquency2;					//P9-03 低频抑制频率
	Uint16	VibrationFrqSelection2;				//P9-04 低频抑制增益  
    Uint16	rad5;				                //P9-05
    
    Uint16	VibrationFrquency3;					//P9-06 低频抑制频率
	Uint16	VibrationFrqSelection3;				//P9-07 低频抑制增益  
    Uint16	rad8;				                //P9-08
    
    Uint16	VibrationFrquency4;					//P9-09 低频抑制频率
	Uint16	VibrationFrqSelection4;				//P9-10 低频抑制增益  
    Uint16	rad11;				                //P9-11
    
    Uint16  NotchFre1;					        //P9-12 HZ
	Uint16	NotchWidth1;			            //P9-13
	Uint16	NotchDepth1;			            //P9-14
    
    Uint16  NotchFre2;					        //P9-15 HZ
	Uint16	NotchWidth2;			            //P9-16
	Uint16	NotchDepth2;			            //P9-17
    
    Uint16  NotchFre3;					        //P9-18 HZ
	Uint16	NotchWidth3;			            //P9-19
	Uint16	NotchDepth3;			            //P9-20
    
    Uint16  NotchFre4;					        //P9-21 HZ
	Uint16	NotchWidth4;			            //P9-22
	Uint16	NotchDepth4;			            //P9-23 
    
    Uint16	rad24;				                //P9-24
    Uint16	rad25;				                //P9-25
    Uint16	rad26;				                //P9-26
    Uint16	rad27;				                //P9-27
    Uint16	rad28;				                //P9-28
    Uint16	rad29;				                //P9-29
    Uint16	rad30;				                //P9-30
    Uint16	rad31;				                //P9-31
    Uint16	rad32[16];				            //P9-32~47
}TYPE_P9;//48

/***********保留参数组*PA****************/
typedef struct
{
	Uint16 ModbusDrvMode;							//PA-00    控制模式
	Uint16 RunMode;							        //PA-01    位置运动模式
	Uint16 SecFaultStop;							//PA-02    二级故障停车
	Uint16 NormalStop;							    //PA-03    正常停车
    
	Uint32 PosAcc;									//PA-04 
	//Uint16 rsd5;									//PA-05 
	Uint32 PosDec;									//PA-06 
	//Uint16 rsd7;									//PA-07
    
	Uint32 TorqueSlop;							    //PA-08 
	//Uint16 rsd9;							        //PA-09     
    Uint32 SpdTorLim;							    //PA-10 
    //Uint16 rsd11;							        //PA-11 
    
    int32 POS_PLim;							        //PA-12   
    //Uint16 rsd13;							        //PA-13 
    int32 POS_NLim;							        //PA-14 
    //Uint16 rsd15;							        //PA-15
    
	Uint32 PosErrMax;								//PA-16 
	//Uint16 rsd17;									//PA-17 
	Uint32 MaxSpd;								    //PA-18 
	//Uint16 rsd19;							        //PA-19
    
	Uint16 MaxTor;									//PA-20 
	Uint16 PosWindow;								//PA-21 
	Uint16 PosWindowTime;						    //PA-22 
	Uint16 EStopTime1;							    //PA-23
    
	Uint16 SyncTime;								//PA-24      
	Uint16 SyncFrameFbFlag;							//PA-25     
    Uint16 rsd26;							        //PA-26      
    Uint16 rsd27;							        //PA-27   
    
    Uint16 rsd28;							        //PA-28 
    Uint16 rsd29;							        //PA-29 
    Uint16 SyncMode;							    //PA-30 
    Uint16 rsd31;							        //PA-31 
    Uint16 rad32[64];				                //PA-32~95  
}TYPE_PA;//96

/***********原点回归参数组*PB****************/
typedef struct
{
	Uint16 T_ZeroFault;							//PB-00 回零失败报警时间
	Uint16 ZeroTrigger;							//PB-01 原点触发启动模式
	Uint16 ActionMode;							//PB-02 到达原点短距离移动方式
	Uint16 DetectMode;							//PB-03 原点检测器种类及寻找方向
	Uint16 HighSpeedZero;						//PB-04 回零第一段高速设定

	Uint16 LowSpeedZero;						//PB-05 回零第二段低速设定
	Uint16 ZeroOffsetPulseNumL;					//PB-06 原点回归偏移脉冲数
	Uint16 ZeroOffsetPulseNumH;					//PB-07
	Uint16 OffsetClearZero;						//PB-08 原点回归完成后是否清除偏差
	Uint16 T_ZeroSpdAcc;						//PB-09 原点回归加速时间

	Uint16 T_ZeroSpdDec;						//PB-10 原点回归减速时间
    Uint16 rsd11;						        //PB-11 
    
    Uint16 rsd12;						        //PB-12
    Uint16 rsd13;						        //PB-13
    Uint16 rsd14;						        //PB-14
    Uint16 rsd15;						        //PB-15
    
    Uint16 rsd16;						        //PB-16
    Uint16 rsd17;						        //PB-17
    Uint16 rsd18;						        //PB-18
    Uint16 rsd19;						        //PB-19
    
    Uint16 rsd20;						        //PB-20
    Uint16 rsd21;						        //PB-21
    Uint16 rsd22;						        //PB-22
    Uint16 rsd23;						        //PB-23
    
    Uint16 rsd24;						        //PB-24
    Uint16 rsd25;						        //PB-25
    Uint16 rsd26;						        //PB-26
    Uint16 rsd27;						        //PB-27
    
    Uint16 rsd28;						        //PB-28
    Uint16 rsd29;						        //PB-29
    Uint16 rsd30;						        //PB-30
    Uint16 rsd31;						        //PB-31
}TYPE_ZERORETURN;//32

/***********保留参数组*PC****************/
typedef struct
{
	Uint16 TorFullSel;                              //PC-00 力矩闭环开关
    Uint16 rsd1;                                    //PC-01
    int16  TorRefCmd;                               //PC-02 力矩闭环力矩命令给定
    Uint16 rsd3;                                    //PC-03 力矩环比例增益

    Uint16 Tor_Kp;                                  //PC-04 力矩环比例增益
    Uint16 Tor_Ki;                                  //PC-05 力矩环积分增益
    Uint16 Tor_Kd;                                  //PC-06 力矩环微分增益
    Uint16 Tor_Kp2;                                 //PC-07 力矩环前馈增益
	Uint16 TorKsf;							        //PC-08 力矩环输出低通滤波时间
	Uint16 TorAcc;                                  //PC-09 力矩环力矩命令斜坡时间
	Uint16 TorRefRev;                               //PC-10 力矩命令方向取反
	Uint16 ErrValLmt;                               //PC-11 力矩环到位误差值
	Uint16 ErrTimLmt;                               //PC-12 力矩环到位误差时间
    Uint16 TorOutLmt;                               //PC-13 力矩环输出限制
    Uint16 Time1;							        //PC-14 测试用变量
    Uint16 TorSenFrq;							    //PC-15 力矩传感器频率

    Uint16 rsd16;							        //PC-16 扫频使能
    Uint16 rsd17;									//PC-17 保留
    Uint16 rsd18;							        //PC-18 扫频起始频率
    Uint16 rsd19;							        //PC-19 扫频终止频率
	Uint16 rsd20;							        //PC-20 扫频电流幅值
    Uint16 rsd21;							        //PC-21 扫频速度幅值
	Uint16 rsd22;							        //PC-22 扫频时间
	Uint16 rsd23;							        //PC-23 扫频类型选择 

	float TargetTorClose;							//PC-24 
	//Uint16 rsd25;							        //PC-25     
    float ActualTorClose;						    //PC-26 
    //Uint16 rsd27;							        //PC-27 
    Uint16 rsd28;							        //PC-28 
    Uint16 rsd29;							        //PC-29 
    Uint16 rsd30;							        //PC-30 
    Uint16 rsd31;							        //PC-31  
    Uint16	rad32[32];				                //PC-32~63  
}TYPE_PC;//64

/***********保留参数组*PD****************/
typedef struct
{
	Uint16 FullCloseMode;					    //PD-00  //全闭环方式
    Uint16 Lpf_T;                               //PD-01  //低通滤波时间
    Uint16 ExEncType;							//PD-02  //全闭环编码器类型
    Uint16 ExEncPolar;                          //PD-03  //全闭环编码器方向
    
    Uint32 ExEncLine;							//PD-04  //全闭环编码器分辨率
    //Uint16 reserve5;							//PD-05  //全闭环编码器分辨率高位
    Uint16 Reduction;                           //PD-06  //减速比
    Uint16 SpdFbMode;                           //PD-07  // 输出端速度反馈方式
    
    Uint16 FiltNum;                             //PD-08  // 速度反馈平滑系数
    Uint16 reserve9;                            //PD-09  //
    Uint16 reserve10;                           //PD-10  //
    Uint16 reserve11;                           //PD-11  //
    
    Uint16 reserve12;                           //PD-12  //
    Uint16 TorFullSel;                          //PD-13  // 力矩闭环开关
    Uint16 Tor_Kp;                              //PD-14  //
    Uint16 Tor_Ki;                              //PD-15  //
    Uint16 rsd16[16];                           //PD-16~31  //
}TYPE_PD;//32

/***********电机参数组*PE****************/
typedef struct
{
	Uint16 MotorPSW;							//PE-00电机组参数密码
	Uint16 MotorCode;							//PE-01电机代码
	Uint16 Pe;									//PE-02电机额定功率
	Uint16 Ie;									//PE-03电机额定电流
	Uint16 Te;									//PE-04电机额定转矩

	Uint16 Ue;									//PE-05电机额定电压
	Uint16 Ne;									//PE-06电机额定转速
	Uint16 Nmax;								//PE-07电机最大转速
	Uint16 PoleNum;								//PE-08电机极对数
	Uint16 Lq;									//PE-09 Q轴线电感   0.001

	Uint16 Ld;									//PE-10 D轴线电感   0.001
	Uint16 Rs;									//PE-11 线电阻      0.001
	Uint16 TrqConst;							//PE-12 转矩参数    0.01
	Uint16 Es;									//PE-13 反电动势    0.01V
	Uint16 Jmot;								//PE-14 电机转子惯量

	Uint16 EncType;								//PE-15编码器类型
	Uint32 EncLine;							    //PE-16编码器线数低位
	//Uint16 EncLineH;							//PE-17编码器线数高位
	Uint16 ZeroElecAngle;						//PE-18编码器原点电角度
	Uint16 UElecAngle;							//PE-19编码器U相上升沿电角度

	Uint16 TuningType;							//PE-20电机参数自学习方式
	Uint16 ABSEnc;								//PE-21绝对值编码器使用方法
	Uint16 ABPhaseSequ:1;						//PE-22UVW顺序
    Uint16 OutDir:1;						    //PE-22输出端编码器方向
    Uint16 rsd22:14;						    //PE-22保留
	Uint16 ACR_Pq;								//PE-23电流调节器Q轴比例增益
	Uint16 ACR_Pd;								//PE-24电流调节器D轴比例增益

	Uint16 ACR_Iq;								//PE-25电流调节器Q轴积分增益
	Uint16 ACR_Id;								//PE-26电流调节器D轴积分增益
	Uint16 ACR_KeqGain;						    //PE-27 Q轴反电动势补偿增益
	Uint16 ACR_KedGain;							//PE-28 D轴电压补偿增益
	Uint16 ACR_KeqGain1;						//PE-29 Q轴电压补偿增益

	Uint16 ACR_GAIN_M:1;                        //PE-30 bit0
    Uint16 E30_rsd:15;                          //      bit1~bit15
    Uint16 ACR_Pq2;								//PE-31 电流调节器Q轴比例增益2用于前馈
    Uint16 EeWFlag;                             //PE-32
    Uint16 TorGain;                             //PE-33 
    Uint16 rsd34;                               //PE-34
    
    Uint16 rsd35;                               //PE-35 保留
    Uint16 rsd36;                               //PE-36 保留
    Uint16 rsd37;                               //PE-37 保留
    Uint16 rsd38;                               //PE-38 保留
    Uint16 rsd39;                               //PE-39 保留
    Uint16 rsd40;                               //PE-40 保留
    Uint16 rsd41;                               //PE-41 保留
    Uint16 rsd42;                               //PE-42 保留
    Uint16 rsd43;                               //PE-43 保留
    Uint16 rsd44;                               //PE-44 保留
    Uint16 rsd45;                               //PE-45 保留
    Uint16 rsd46;                               //PE-46 保留
    Uint16 rsd47;                               //PE-47 保留
    Uint16 rsd48[16];                           //PE-48~63 保留
}TYPE_MOTOR;//64
/***********驱动器参数组*PF****************/
typedef struct
{
	Uint16 SoftVersion;							//PF-00DSP软件版本
	Uint16 CPLDVersion;							//PF-01CPLD软件版本
	Uint16 WarnTem;								//PF-02驱动器温度警告值
	Uint16 FaultTem;							//PF-03驱动器温度故障值
	Uint16 ServoIe;								//PF-04驱动器额定电流

	Uint16 ServoVolLv;							//PF-05驱动器电压等级 (220V/380V -10%~+15%)
	Uint16 ServoCode;							//PF-06驱动器代码
	Uint16 reserve07;							//PF-07
	Uint16 IeSampleGain;						//PF-08电流放大倍数
	Uint16 TorqLimt;							//PF-09最大力矩限幅

												//PF-10清除记录器
	Uint16 Clear;						        //PF-10.Bit0:清除历史故障,bit1:清除运行时间
	Uint16 Boot_Power;							//PF-11 125使用动态制动
	Uint16 MotorWarnTem;						//PF-12 电机过热警告
	Uint16 MotorFaultTem;						//PF-13 电机过热故障
	Uint16 DbTime;							    //PF-14

	Uint16 DbComp;							    //PF-15 死区补偿
	Uint16 DbSlope;							    //PF-16 死区补偿斜率
	Uint16 Kadccorr;							//PF-17母线模拟量监测校正系数
	Uint16 ASR_AntiwindupTi;					//PF-18速度调节器积分抗饱和增益
	Uint16 ACR_AntiwindupTi;					//PF-19电流调节器积分抗饱和增益

	Uint16 AI1Dead;								//PF-20AI1死区
	Uint16 AI2Dead;								//PF-21AI2死区
												//PF-22 电流环增强功能
	Uint16 ACR_HalfPI:1;	                    //PF-22.Bit0: 电流环1/2 PI功能
	Uint16 ACR_GAIN_M:1;	                    //PF-22.Bit1: 电流环多段增益开关
	Uint16 ACR_ACK1:1;	                        //PF-22.Bit2: 电流应答1
	Uint16 ACR_ACK2:1;	                        //PF-22.Bit3: 电流应答2
	Uint16 ACR_RES2:1;                          //PF-22.Bit4: 2倍电流环响应开关
	Uint16 ACR_AVR:1;                           //PF-22.Bit5: AVR开关
	Uint16 ACR_UFWD:1;                          //PF-22.Bit6: 电压前馈开关
	Uint16 ACR_RegKiSat:1;                      //PF-22.Bit7: 电流调节器积分饱和处理选择
	Uint16 Mask_rsvd22:8;						//PF-22.Bit8:保留
												//PF-23速度环增强功能
	Uint16 ASR_Display:1;			            //PF-23.Bit0: 速度显示选择
	Uint16 ASR_RegKiSat:1;			            //PF-23.Bit1: 速度调节器积分饱和处理选择
	Uint16 Mask_rsvd23:14;						//PF-23.Bit2~7:保留
	Uint16 Pos_div;							    //PF-24位置环分频系数(总线型只能设为偶数)

	Uint16 ASR_div; 					        //PF-25速度环分频系数
	Uint16 Mask_rsvd26;	     				    //PF-26保留
	Uint16 MASK_E01:1;							//PF-27故障屏蔽开关1
	Uint16 MASK_E02:1;
	Uint16 MASK_E03:1;
	Uint16 MASK_E04:1;
	Uint16 MASK_E05:1;
	Uint16 MASK_E06:1;
	Uint16 MASK_E07:1;
	Uint16 MASK_E08:1;
	Uint16 Mask_rsvd27:8;
	Uint16 MASK_E09:1;//0						//PF-28故障屏蔽开关2
	Uint16 MASK_E0A:1;//1
	Uint16 MASK_E0B:1;//2
	Uint16 MASK_E0C:1;//3
	Uint16 MASK_E0D:1;//4
	Uint16 MASK_E0E:1;//5
	Uint16 MASK_E0F:1;//6
	Uint16 MASK_E10:1;//7
	Uint16 Mask_rsvd28:8;
	Uint16 MASK_E11:1;							//PF-29故障屏蔽开关3
	Uint16 MASK_E12:1;
	Uint16 MASK_E13:1;
	Uint16 MASK_E14:1;
	Uint16 MASK_E15:1;
	Uint16 MASK_E16:1;
	Uint16 MASK_E17:1;
	Uint16 MASK_E18:1;
	Uint16 Mask_rsvd29:8;

	Uint16 MASK_E19:1;							//PF-30故障屏蔽开关4
	Uint16 MASK_E1A:1;
	Uint16 MASK_E1B:1;
	Uint16 MASK_E1C:1;
	Uint16 MASK_E1D:1;
	Uint16 MASK_E1E:1;
	Uint16 MASK_E1F:1;
	Uint16 MASK_E20:1;
	Uint16 Mask_rsvd30:8;
	Uint16 DrvType;								//PF-31驱动器类型（0：增量式伺服；1：绝对式伺服）
	Uint16 MASK_E21:1;							//PF-32故障屏蔽开关5
	Uint16 MASK_E22:1;
	Uint16 MASK_E23:1;
	Uint16 MASK_E24:1;
	Uint16 MASK_E25:1;
	Uint16 MASK_E26:1;
	Uint16 MASK_E27:1;
	Uint16 MASK_E28:1;
	Uint16 Mask_rsvd32:8;
	Uint16 MASK_E29:1;							//PF-33故障屏蔽开关6
	Uint16 MASK_E2A:1;
	Uint16 MASK_E2B:1;
	Uint16 MASK_E2C:1;
	Uint16 MASK_E2D:1;
	Uint16 MASK_E2E:1;
	Uint16 MASK_E2F:1;
	Uint16 MASK_E30:1;
	Uint16 Mask_rsvd33:8;
	Uint16 MASK_E31:1;							//PF-34故障屏蔽开关7
	Uint16 MASK_E32:1;
	Uint16 MASK_E33:1;
	Uint16 MASK_E34:1;
	Uint16 MASK_E35:1;
	Uint16 MASK_E36:1;
	Uint16 MASK_E37:1;
	Uint16 MASK_E38:1;
	Uint16 Mask_rsvd34:8;

	Uint16 MASK_E39:1;							//PF-35故障屏蔽开关8
	Uint16 MASK_E3A:1;
	Uint16 MASK_E3B:1;
	Uint16 MASK_E3C:1;
	Uint16 MASK_E3D:1;
	Uint16 MASK_E3E:1;
	Uint16 MASK_E3F:1;
	Uint16 MASK_E40:1;
	Uint16 Mask_rsvd35:8;
	Uint16 reserve36;                           //PF-36有用到，但是不知道是什么意思
	Uint16 reserve37;							//PF-37
	Uint16 T_Stall;								//PF-38直流转速2(驱动器相关-引申为1倍对应转速，＞PF-39)
	Uint16 N_Stall;								//PF-39直流转速1（驱动器相关-引申为1.414对应转速）

	Uint16 reserve40;							//PF-40
	Uint16 reserve41;							//PF-41
	Uint16 ILPMaskFlag;							//PF-42 ILP缺相故障屏蔽标志
	Uint16 ILPLossFlag;							//PF-43 ILP主电源故障屏蔽标志
	Uint16 SrvOnBrkDlyTime;						//PF-44 伺服ON后抱闸打开最小延时时间

	Uint16 P028NoClr;							//PF-45 反馈总脉冲数不清零
	Uint16 reserve46;							//PF-46
	Uint16 reserve47;							//PF-47 
	Uint16 reserve48;							//PF-48
	Uint16 reserve49;							//PF-49

	Uint16 reserve50;							//PF-50
	Uint16 FactortTest;							//PF-51 工厂自检(改由GPIO31-GND控制)
	Uint16 DeadZoneAdj;							//PF-52 死区补偿阈值
	Uint16 DeadZoneSel;							//PF-53 死区补偿开关
	Uint16 reserve54;							//PF-54

	Uint16 VF_Test;								//PF-55VF 测试控制
	Uint16 VF_Voltage;					        //PF-56VF 给定电压
	Uint16 VF_Fre;								//PF-57VF 给定频率
	Uint16 VF_AddTimeS;					        //PF-58VF 加速时间
	Uint16 InertiaTunSpdLmt;				    //PF-59   三角波注入惯量辨识速度限制

	Uint16 IDRef;							    //PF-60 D轴电流给定
	Uint16 ACR_Test;							//PF-61 EEPROM校验
	Uint16 MechFftTorqRats;						//PF-62机械频率特性分析注入扰动力矩
	Uint16 Carrier;								//PF-63载波,0:4k,1:6k,2:8k,3:10k,高位为1时双刷

    //一下变量外部不能操作
    Uint16 rsd64[16];                           //保留64~79
    
    Uint16 Abs0;                                //PF-80   //对齐偏移 和 多摩川单圈偏移
    Uint16 Abs1;                                //PF-81   //对齐偏移 和 多摩川单圈偏移
    Uint16 Mulpos0;                             //PF-82   2：偏移
    Uint16 Mulpos1;                             //PF-83
    
    Uint16 Mulpos2;                             //PF-84
    Uint16 Mulpos3;                             //PF-85
    Uint16 Mulpos4;                             //PF-86
    //Uint16 rsd87;                             //PF-87
    //Uint16 rsd88;                             //PF-88
    float TorSensorOffsetVal;                   //PF-87
    Uint16 rsd89;                               //PF-89
}TYPE_SERVO;//90

/***********通信下发命令组*C00 7000H****************/
typedef struct
{

	Uint16 SCI_KeyCmd:8;     		//F7000 SCI下发命令
	Uint16 SCI_485Cmd:8;     		//F7000 SCI下发命令
	int16 SCI_Given;				//F7001 通信给定
	Uint16 Virtual1;     			//F7010 虚拟端子1
	Uint16 Virtual2;				//F7011 虚拟端子2
	Uint16 KeyUp;					//F7012 up功能
	Uint16 KeyDown; 				//F7012 dowm功能

}TYPE_SCICmd;

///***********伺服状态参数组*S00 7200H****************/
//typedef struct
//{
//	Uint16 RunStatus1;			//F7200  低8位是变频器运行状态1
//}TYPE_ServoStatus;


/***********变频器信息* IFO00****************/
typedef struct
{
	Uint16 CPUB_Version1;  			//7500H		性能CPU版本1
	Uint16 CPUB_Version2; 			//7501H		性能CPU版本2
	Uint16 CPUA_Version1; 			//7502H		功能软件序列号1
	Uint16 CPUA_Version2; 			//7503H		功能软件序列号2
	Uint16 KEY_Version1; 			//7504H		键盘软件序列号1

	Uint16 KEY_Version2; 			//7505H		键盘软件序列号2
	Uint16 Product_Version1; 		//7506H		产品序列号1
	Uint16 Product_Version2; 		//7507H		产品序列号2
	Uint16 Product_Version3; 		//7508H		产品序列号3
	Uint16 Product_Version4; 		//7509H		产品序列号2

	Uint16 Product_Version5; 		//750AH		产品序列号3
}TYPE_SCI_Ifo;

/***********示波器设置数据* FUN76H00****************/
typedef struct SCOPE_FUN76H{
	Uint16 AcqMode;			//7600H 采样模式 0无采样       1电流环触发采样         2 1ms连续采样
	Uint16 SampleRate;		//7601H 采样率   AcqMode = 1  1代表一个电流环         AcqMode = 2   1代表1ms
	Uint16 Enable;			//7602H 采样使能
	Uint16 BufSize;			//7603H 预采样深度（0~50）
	Uint16 ChSel1;			//7604H ch1选择（0~CHSELNUM）

	Uint16 ChSel2;			//7605H ch2选择（0~CHSELNUM）
	Uint16 ChSel3;			//7606H ch3选择（0~CHSELNUM）
	Uint16 ChSel4;			//7607H ch4选择（0~CHSELNUM）
	Uint16 ChSelTrigA;		//7608H 触发A通道选择
	Uint16 ChSelTrigEdge;	//7609H 触发A条件

	int32 TrigLevel;		//760AH 触发A水平L
	//int16 TrigLevel;	    //760BH 触发A水平H
	Uint16 SeriesEnable;	//760CH 连续采样请求
	Uint16 TrigEvent;		//760DH 触发采样使能

	Uint16 ChSelTrigB;		//760EH 触发B通道选择
	Uint16 ChSelTrigEdgeB;	//760FH 触发B条件

	int32 TrigLevelB;		//7610H 触发B水平L
	//int16 TrigLevel;	    //7611H 触发B水平H

    Uint16 FaultTriEnable;  //7612H
    Uint16 FaultTriFlag;
    Uint16 rsvd1[12];		//预留12
    Uint16 rsvd2[32];		//预留32  0x763F
    
    Uint32 MotSingRev;      //电机编码器分辨率     
    Uint32 OutSingRev;      //减速机编码器分辨率

    Uint16 rsvd3[0x3C];
	//-------------------------------------------------//
}FUN76H_RARA;

typedef struct SCOPE_FUN77H{
    Uint16 CtrlMode;        // 控制模式       7700
    Uint16 PosMode;         // 位置控制模式   7701
    Uint16 PosJog;          // 
    Uint16 PosSpd;          // 位置速度
    Uint16 PosAcc;          // 位置加速度
    Uint16 PosDec;          // 位置减速度
    Uint16 Delaytime;       // 
    Uint16 PosFre;          //               7707
    int32  PosCmd1;         // 位置1           08
    int32  PosCmd2;         // 位置2           0A
    int32  PosLim1;         // 限位1           0C
    int32  PosLim2;         // 限位2           0E
    Uint16 Posrsd[16];      // 10~1F    

    Uint16 SpdMode;                         //20
    Uint16 SpdValue;        //              //21
    int16  SpdDir;                          //22
    Uint16 SpdAcc;          // 速度加速度    //23
    Uint16 SpdDec;          // 速度减速度    //24
    Uint16 SPdrsd[11];      //25~2F

    Uint16 Torrsd1[16];     //30~3F
    Uint16 MortorSon;       //40

    Uint16 cmd;		        //7741 扫频使能         0c10
    Uint16 rsd14;			//7742 保留            0c11
    Uint16 f_start_hz;      //7743 扫频起始频率     0c12
    Uint16 f_end_hz;        //7744 扫频终止频率     0c13
	Uint16 amplitude;	    //7745 扫频电流幅值     0c14
    Uint16 amplitude2;		//7746 扫频速度幅值     0c15
	Uint16 sweep_time_s;	//7747 扫频时间         0c16
	Uint16 sel;				//7748 扫频类型选择     0c17
}FUN77H_RARA; 

typedef struct SCOPE_FUN78H{
    char SNCode[64];      //0x7800~0x0x781F
    Uint16 EncStatus1;    //0x7820
    Uint16 EncErr1;       //0x7821
    Uint16 EncErr2;       //0x7822
    char   ServoName[10]; //0x7823~7827
    Uint32 SoftCode;      //0x7828~7829
    Uint16 HardCode;      //0x782A
    Uint16 MortorIe;      //0x782B
    Uint16 MortorMaxIe;   //0x782C
    Uint16 MortorTe;      //0x782D
    Uint16 ServoType;     //0x782E

    uint16_t Vendor;      //0x782F   传感器型号
    uint16_t ProId;       //0x7830   传感器量程
    uint16_t Version;     //0x7831   传感器版本
    Uint16 rsd32_3F[14];  //0x7832~0x783F
    Uint16 rsd40_4F[16];  //0x7840~0x784F

    Uint16  EncStatus;     //0x7850 校准结果               
    int16_t NsinVpp;       //0x7851 电机端主码道正弦峰值         
    int16_t NsinOffset;    //0x7852 电机端主码道正弦中心点电压    
    int16_t NcosVpp;       //0x7853 电机端主码道余弦峰值         
    int16_t NcosOffset;    //0x7854 电机端主码道余弦中心点电压    
    int16_t MsinVpp;       //0x7855 电机端辅码道正弦峰值          
    int16_t MsinOffset;    //0x7856 电机端辅码道正弦中心点电压    
    int16_t McosVpp;       //0x7857 电机端辅码道余弦峰值          
    int16_t McosOffset;    //0x7858 电机端辅码道余弦中心点电压    
    int16_t AngleErr;      //0x7859 电机端角度偏差

    int16_t ONsinVpp;      //0x785A 输出端主码道正弦峰值
    int16_t ONsinOffset;   //0x785B 输出端主码道正弦中心点电压
    int16_t ONcosVpp;      //0x785C 输出端主码道余弦峰值
    int16_t ONcosOffset;   //0x785D 输出端主码道余弦中心点电压
    int16_t OMsinVpp;      //0x785E 输出端辅码道正弦峰值
    int16_t OMsinOffset;   //0x785F 输出端辅码道正弦中心点电压
    int16_t OMcosVpp;      //0x7860 输出端辅码道余弦峰值
    int16_t OMcosOffset;   //0x7861 输出端辅码道余弦中心点电压
    int16_t OAngleErr;     //0x7862 输出端角度偏差

}FUN78H_RARA;


/***********重新上电功能码参数结构体****************/
typedef struct
{
	Uint16	PulseCmdType;						//P1-02外部脉冲指令输入形式
}TYPE_REPOWERON;

/***********电机静止更新功能码参数结构体****************/
typedef struct
{
	Uint16 DILogicLevel;						//P6-01DI电平逻辑
}TYPE_HALTREFRESH;

/***********掉电参数组P00*******P0******/
extern TYPE_PowerDownArg *PowerDownArg;

/***********基本参数组P0****************/
extern TYPE_MONITOR *RamMonitor;//监视参数组

/**********P1：增益调整参数组*****************/
extern TYPE_BASEPARA *RamBasePara;//基本参数组

/***********多段位置参数组*P2****************/
extern TYPE_MULTIPOS *RamMultiPos;//多段位置参数组

/***********多段速及转矩控制功能组*P3****************/
extern TYPE_MULTISPD *RamMultiSpeed;//多段速及转矩控制参数组

/**********P4组：转矩控制参数组*******************/
extern TYPE_TRQCTL *RamTrqCtrl;//转矩控制参数组

/**********P5组：调谐参数组********************/
extern TYPE_TUNING *RamTuning;//调谐参数组

/**********P6组：输入输出参数组*******************/
extern TYPE_IO *RamIO;

/***********通讯组*P7****************/
extern TYPE_COMMU *RamCommu;//通讯参数组

/***********P8组：辅助功能功能码组************/
extern TYPE_AID *RamAid;//辅助功能参数组

/***********P9组：保留参数组**********************/
extern TYPE_P9 *RamP9;//保留参数组

/**********PA 保留参数组***************************/
extern TYPE_PA *RamPA;//保留参数组

/**********PB 保留参数组***************************/
extern TYPE_ZERORETURN *RamZeroReturn;//原点回归参数组

/**********PC 保留参数组***************************/
extern TYPE_PC *RamPC;//保留参数组

/**********PD 保留参数组***************************/
extern TYPE_PD *RamPD;//保留参数组

/**********PE 电机参数组***************************/
extern TYPE_MOTOR *RamMotor;//电机参数组

/**********PF 驱动器参数组***************************/
extern TYPE_SERVO *RamServo;//驱动器参数组

/************通信下发命令组*********/
extern TYPE_SCICmd     *SCICmd;       //通信下发命令组
/**********驱动器状态参数组***************/
//extern TYPE_ServoStatus  *ServoStatus; //驱动器状态组

/**********产品信息组******************************/
extern FUN76H_RARA *ScopePara;

extern FUN77H_RARA *ScopeCtrl;

extern FUN78H_RARA ProductInf;

/***********重新上电功能码参数结构体****************/
extern TYPE_REPOWERON PowerOn;

/***********电机静止更新功能码参数结构体****************/
extern TYPE_HALTREFRESH HaltFresh;

extern void VarInit(void);
void InitFuncVar(void);
extern Uint16 GetFuncNum(Uint16 *RamFunc);
void F6700_PowerUpInit(void);
#endif
