#ifndef _SV_FAULTPROTECT_
#define _SV_FAULTPROTECT_

#include "Datatype.h"//
//
//;故障码最高字节最低位表示故障是否可复位,0-不可复位; 1-可复位
//;次高字节 = E，则表示警告
//;数值越小，级别越高
#define FAULT_DEAL_BIT(x,bit)		(x&bit)
#define FAULT_BIT_DISP				(0x00FF)	//bit7~0：故障显示
#define FAULT_BIT_LEVEL				(0x0300)	//bit9~8：故障分级
#define FAULT_LEVEL1				(0x0000)	//		自由停车
#define FAULT_LEVEL2				(0x0100)	//		零速停车（？？？按停车方式停车）
#define FAULT_LEVEL3				(0x0200)	//		警告（继续运行）
#define FAULT_LEVEL4				(0x0300)	//		保留
#define FAULT_BIT_PRIORITY			(FAULT_BIT_DISP|FAULT_BIT_LEVEL)
#define FAULT_BIT_RESET				(0x1000)	//bit12：故障复位
#define FAULT_RESET_YES				(0x1000)	//		可复位
#define FAULT_RESET_NO				(0x0000)	//		不可复位

typedef enum{
//1.故障码:
	SC_ERR			= 0x0001|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.001  硬件(短路)保护
	HOC_ERR         = 0x0002|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.002  软件(短路)保护
	PramRecErr      = 0x0003|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.003  参数读异常
	PramWErr        = 0x0004|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.004  参数写异常
	SysPramErr      = 0x0005|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.005  系统参数异常
	AdSampErr       = 0x0006|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.006  Ad采样模块故障
	EncoderErr_1    = 0x0007|FAULT_RESET_YES |FAULT_LEVEL1,//ER.007  编码器异常1 断线
	EncoderErr_AB   = 0x0008|FAULT_RESET_YES |FAULT_LEVEL1,//ER.008  编码器异常2 AB干扰
	EncoderErr_Z    = 0x0009|FAULT_RESET_YES |FAULT_LEVEL1,//ER.009  编码器异常3 Z干扰
	UnderUdc	    = 0x000A|FAULT_RESET_YES |FAULT_LEVEL2,//ER.00A  母线欠压故障
	OverUdc         = 0x000B|FAULT_RESET_YES |FAULT_LEVEL1,//ER.00b  母线过压故障
	LostPhase       = 0x000C|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.00C  电机U,V电流采样故障
	MotOvLoad       = 0x000D|FAULT_RESET_YES |FAULT_LEVEL2,//ER.00d  电机过载故障
	DrvOvLoad       = 0x000E|FAULT_RESET_YES |FAULT_LEVEL2,//ER.00E  驱动器过载
	MotOvHeat       = 0x000F|FAULT_RESET_YES |FAULT_LEVEL2,//ER.00F  电机过热
	DrvOvHeat       = 0x0010|FAULT_RESET_YES |FAULT_LEVEL2,//ER.010  驱动器过热
	FanCtrlErr      = 0x0011|FAULT_RESET_YES |FAULT_LEVEL2,//ER.011  速度偏差过大
	OvSpd           = 0x0012|FAULT_RESET_YES |FAULT_LEVEL1,//ER.012  过速
	OvPosErr        = 0x0013|FAULT_RESET_YES |FAULT_LEVEL2,//ER.013  位置偏差过大故障
    
	UdcLPErr        = 0x0014|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.014  母线电压缺相故障
	PhaseErr        = 0x0015|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.015  电机相序错误
	DrvIeeIptErr    = 0x0016|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.016  驱动器额定电流输入错误(或code不存在)
	RstOvLoad       = 0x0017|FAULT_RESET_YES |FAULT_LEVEL2,//ER.017  制动电阻过载故障
	AbsEncOhErr     = 0x0018|FAULT_RESET_YES |FAULT_LEVEL2,//ER.018  绝对值编码器过热
	AbsEncBaErr     = 0x0019|FAULT_RESET_YES |FAULT_LEVEL2,//ER.019  绝对值编码器电池电压低于3.1V,电池电压偏低
	AbsEncBeErr     = 0x001A|FAULT_RESET_YES |FAULT_LEVEL2,//ER.01A  绝对值编码器电池电压低于2.5V,多圈位置信息已丢失
	DrvMotPaErr     = 0x001B|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.01b  驱动器和电机不匹配
	OrgLearnErr     = 0x001C|FAULT_RESET_YES |FAULT_LEVEL2,//ER.01C  原点回归失败
	PowLoseErr      = 0x001D|FAULT_RESET_YES |FAULT_LEVEL2,//ER.01d  主电源掉电
	MotorLockErr    = 0x001e|FAULT_RESET_YES |FAULT_LEVEL1,//ER.01E  电机堵转
	RepowErr        = 0x001F|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.01F  自学习结束复位标志
    
	ECATINIT        = 0x0020|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.020  ECAT 初始化LAN9252错误
	ECATSYNC        = 0x0021|FAULT_RESET_YES |FAULT_LEVEL2,//ER.021  DC 丢步
	ECATCMU         = 0x0022|FAULT_RESET_YES |FAULT_LEVEL2,//ER.022  ECAT 通过网线与主机通讯中断
	ECATRUNREAD     = 0x0023|FAULT_RESET_YES |FAULT_LEVEL2,//ER.023  ECAT PDO通信参数在Servo-on时只读
	ECATNOObj       = 0x0024|FAULT_RESET_YES |FAULT_LEVEL2,//ER.024  ECAT PDO通信没有要查找的索引和子索引
	ECATSYNC0       = 0x0025|FAULT_RESET_YES |FAULT_LEVEL2,//ER.025  ECAT PDO通信设置同步时间超范围
	ECATPDOLIMIT    = 0x0026|FAULT_RESET_YES |FAULT_LEVEL2,//ER.026  ECAT PDO通信数据超范围
	ECATWDTIMEOUT   = 0x0027|FAULT_RESET_YES |FAULT_LEVEL2,//ER.027  ECAT 看门狗超时
	ECATWDINVALID	= 0x0028|FAULT_RESET_YES |FAULT_LEVEL2,//ER.028  ECAT 看门狗无效 
    SeetPointErr    = 0x0029|FAULT_RESET_YES |FAULT_LEVEL2,//ER.029  ECAT 目标增量过大 
    POSLIMIT_P		= 0x002A|FAULT_RESET_YES |FAULT_LEVEL2,//ER.02A  ECAT 位置正向极限
    POSLIMIT_N		= 0x002B|FAULT_RESET_YES |FAULT_LEVEL2,//ER.02B  ECAT 位置负向极限
    
    EncoderErr_Mag  = 0x002C|FAULT_RESET_YES |FAULT_LEVEL1,//ER.02C  第二编码器断线
    EncoderErr_CE   = 0x002D|FAULT_RESET_YES |FAULT_LEVEL1,//ER.02D  第二编码器内部故障
    EncoderErr_CRC  = 0x002E|FAULT_RESET_YES |FAULT_LEVEL1,//ER.02E  第二编码器CRC故障
    EncoderConfig_Err = 0x002F|FAULT_RESET_NO|FAULT_LEVEL1,//ER.02F  第二编码器配置错误
    
    EncoderBe_Err	= 0x0030|FAULT_RESET_YES |FAULT_LEVEL2,//ER.030  第二编码器电压低报警 
    EncoderBa_Err	= 0x0031|FAULT_RESET_YES |FAULT_LEVEL2,//ER.031  第二编码器电压低警告    
    SpdCmdOv	    = 0x0032|FAULT_RESET_YES |FAULT_LEVEL2,//ER.032  速度指令超限
    OvTorErr	    = 0x0033|FAULT_RESET_YES |FAULT_LEVEL1,//ER.033  扭矩偏差过大
    TorSorErr	    = 0x0034|FAULT_RESET_YES |FAULT_LEVEL1,//ER.034  扭矩传感器数据错误 
    
    IICCheckErr	    = 0x0035|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.035 
    rs2	            = 0x0036|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.036  
    rs3	            = 0x0037|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.037   
    rs4	            = 0x0038|FAULT_RESET_NO  |FAULT_LEVEL1,//ER.038   
    SensorlessParamErr = 0x003A|FAULT_RESET_YES|FAULT_LEVEL1,
    SensorlessTrackErr = 0x003B|FAULT_RESET_YES|FAULT_LEVEL1,
    EncoderErr_M    = 0x0039|FAULT_RESET_NO  |FAULT_LEVEL2,//ER.039    
//~0x0(1)0FF故障码保留
//---------------------------------------------------------
//;2.警告代码:
	MinWarn   		= 0x0080|FAULT_RESET_YES |FAULT_LEVEL3,//警告最小值
	MotOvHeatWarn   = 0x0081|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E01 电机过热警告
	DrvOvHeatWarn   = 0x0082|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E02 驱动器过热警告
	MotOvLoadWarn   = 0x0083|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E03 电机过载警告
	DrvOvLoadWarn   = 0x0084|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E04 驱动器过载警告
	OvPosWarn       = 0x0085|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E05 位置偏差太大警告
	RstOvLoadWarn   = 0x0086|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E06 制动过载警告
	POTWarn      	= 0x0087|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E07 正向超程警告
	NOTWarn      	= 0x0088|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E08 反向超程警告
    POTLIMITWarn    = 0x0089|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E09 正向指令超程警告
	NOTLIMITWarn    = 0x008A|FAULT_RESET_YES |FAULT_LEVEL3,//ER.E0A 反向指令超程警告

//~0x1EFF保留
	MaxWarn			= 0x00FF|FAULT_RESET_YES |FAULT_LEVEL3 //警告最大值
}ENUM_FAULTDEFINE;




//;------------------------------
//;用于置警告标志
//#define FAULT_GETWARNBIT(w)			(0x0001<<(FAULT_DEAL_BIT(w,FAULT_BIT_DISP)))
#define FAULT_GETWARNBIT(w)			(0x0001<<(FAULT_DEAL_BIT((w - 0x80),FAULT_BIT_DISP)))
#define FAULT_MOTOHW				(0x0001<<(FAULT_DEAL_BIT(MotOvHeatWarn,FAULT_BIT_DISP)))
#define FAULT_DRVOHW      			(0x0001<<(FAULT_DEAL_BIT(DrvOvHeatWarn,FAULT_BIT_DISP)))
#define FAULT_MOTOLW      			(0x0001<<(FAULT_DEAL_BIT(MotOvLoadWarn,FAULT_BIT_DISP)))
#define FAULT_DRVOLW      			(0x0001<<(FAULT_DEAL_BIT(DrvOvLoadWarn,FAULT_BIT_DISP)))
#define FAULT_OVPOSW      			(0x0001<<(FAULT_DEAL_BIT(OvPosWarn,FAULT_BIT_DISP)))
#define FAULT_RSTOLW      			(0x0001<<(FAULT_DEAL_BIT(RstOvLoadWarn,FAULT_BIT_DISP)))
#define FAULT_POTW        			(0x0001<<(FAULT_DEAL_BIT(POTWarn,FAULT_BIT_DISP)))
#define FAULT_NOTW        			(0x0001<<(FAULT_DEAL_BIT(NOTWarn,FAULT_BIT_DISP)))


typedef enum{
	FP_NORMAL1 = 0,		//正常
	FP_WARN = 1,		//警告
	FP_ERR = 2,			//故障
	FP_WARNERR = 3		//警告&故障
}ENUM_FAULTSTATUS;

typedef struct{
	ENUM_FAULTSTATUS FaultStatus;		//故障状态（bit1|0:故障|警告）
	Uint16 FaultDisp;		//故障(故障或警告)显示代码
	Uint16 ErrCode;			//故障代码（不包括警告，警告只显示，不存）
	Uint16 WarnFlg;			//警告状态(位定义)
	Uint16 ErrDoFlg;		//故障Do输出标志
	Uint16 WarnDoFlg;		//警告Do输出标志
	Uint16 EepFlg;			//EEPROM存储标志（仅故障存储）
	Uint16 EepNewFlg;		//EEPROM新需求
	Uint16 ErrDiRst;		//故障复位端子
}TYPE_FAULTPROTECT;	//故障保护

typedef struct{
	Uint16 FaultCode;		//故障码
	int16  Spd;				//当前转速
	Uint16 BusVolt;			//母线电压
	Uint16 Curr;			//电流
	Uint16 ServoTemper;	    //
	Uint16 rsd;	            //    
}TYPE_FAULTINFODETAIL;

#define FP_FAULTINFOMAX		(4)	//故障信息-组数
#define FP_INFO_CODE		FAULT_DEAL_BIT(FaultP.ErrCode,FAULT_BIT_DISP)	//故障信息-代码
#define FP_INFO_SPD			(SpdMonitor.SpdAvrg)			//故障信息-转速
#define FP_INFO_BUSVOLT		(UdcCtrl.Udc)					//故障信息-母线电压
#define FP_INFO_CURR		(CurMonitor.IeeValuDisp)		//故障信息-电流
#define FP_INFO_RUNTIME		(RamMonitor->T_SysRun)			//故障信息-运行时间
#define FP_INFO_DEPTH		(sizeof(TYPE_FAULTINFODETAIL))	//故障信息空间大小（Uint16）
#define FP_INFO_CURRENTPTR	(FaultInfo.InfoPtr)				//当前故障信息指针
/*		 |	point			*/
/* P1-60 |	0	1	2	3	*/
/*-------+------------------*/
/*	0	 |	0	1	2	3	*/
/*	1	 |	3	0	1	2	*/
/*	2	 |	2	3	0	1	*/
/*	3	 |	1	2	3	0	*/
#define FP_INFO_P160PTR		( (FP_INFO_CURRENTPTR >= RamBasePara->FaultDisplay) ? \
							  (FP_INFO_CURRENTPTR - RamBasePara->FaultDisplay)    : \
							  (FP_INFO_CURRENTPTR + FP_FAULTINFOMAX-RamBasePara->FaultDisplay) \
							) //P1-60选择故障信息指针

typedef struct{
	Uint16 InfoPtr;								//故障信息指针(指向当前故障)
	TYPE_FAULTINFODETAIL Info[FP_FAULTINFOMAX];	//故障信息
}TYPE_FAULTINFO;	//故障信息(此结构体均需存储)

extern TYPE_FAULTPROTECT FaultP;		//故障保护结构体
extern TYPE_FAULTINFO FaultInfo;		//故障信息结构体
extern void FaultPrtt_Schedule(void);	//故障调度函数
extern void FaultPrtt_WarnInterfaceClr(Uint16 fault);	//警告清除接口函数
extern void FaultPrtt_FaultInterface(Uint16 fault);		//故障发生调用接口函数
extern void Fault_Prtt(void);
#endif /* _SV_FAULTPROTECT_ */
/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
