#ifndef __FuncCode_Init_H__
#define __FuncCode_Init_H__

//#include "SV_Global.h"
#include "Datatype.h"
#include "SV_FuncVar.h"//
#include "r_cg_Project.h"
//
/*********************************/
/*
//AC10400版本：使用带电池电机，默认为绝对坐标方式  编码器类型任选
//AC10500版本：增加是否显示EtherCAT错误计数
//AC10600版本：使用For循环解决电机自动旋转，修改了驱动器代码
//AC10700版本：使用变量标志修复电机使能自动旋转
//AC10800版本：修复速度偏移问题（0x60B1）
//AC10900版本：电机代码全
//AC11000版本：修改同步检测方法
//AC12000版本：默认协盛达电机代码
//AC13000版本：修改Modbus使参数存储
//AC14000版本：加入动态制动
//AC15000版本：适应摆臂机器人的参数
//AC15001版本：屏蔽主电源掉电
//AC15002版本：解决串口初始化发送一个字节数据问题
//AC15003版本：解决CST失能电机跳动；
//             根据电机代码决定是否开绝对值，不在使用Pr8.28；
//             驱动器断电后点动速度为100PRM；
//              OP状态下不能手动使能电机;
//             显示CRC错误计数
//             无驱动器过载警告和电机过载警告
//AC15004版本：加入目标位置滤波，默认多圈值电机，使用Pr8.28可以屏蔽多圈故障，
//           用户可以不改动电机代码，便可以切换多圈值电机和单圈值电机,同时兼容以前的参数
//AC15005版本：加入串口清除错误，IP，PP不限速
//AC15006版本：
//AC15007版本：三倍载时降低增益
//AC15008版本：Pr1.59可以改变电机旋转方向
 */
/*********************************/
#if SERVOPOWER == SERVO_AC
////2023.06.12调整对象字典
//#define PRDCTSer1       (23005) //DSP软件本
//#define EEPROMSer1      (6120) //DSP软件本  

//2023.08.16调整对象字典
#define PRDCTSer1       (23006) //DSP软件本
#define EEPROMSer1      (8160) //DSP软件本 
#endif

/*********************************/
//低压10400版本：霍尔元件启动电机  抱闸使用Pr8.29和Pr8.30 编码器类型任选

/*********************************/
#define CPUASer2	(0)
#define CPUBSer1	(10000)//配置文件版本：约定此处写入能兼容的最低版本
#define CPUBSer2	(0)
#define CPUCSer1	(0)
#define CPUCSer2	(0)
#define PrdctSer1	(0)
#define PrdctSer2	(0)
#define PrdctSer3	(0)

//其他系统定义////////////////////////////////////
#define MAXSPD		(10000)			//系统最大转速
#define MAXTRQ		(3000)			//系统最大转矩
#define MAXDIFUNC	(31)			//DI功能大值	TODO:最终时具体数值
#define MAXDOFUNC	(63)			//DO功能大值
#define MAXAOFUNC	(4)				//AO功能大值
#define MAXP0		(40)			//P0组参数大值
/////////////////////////////////////////////////

#define RESERVE 0

#define P0_FUN_GROUP 0
#define P1_FUN_GROUP 1
#define P2_FUN_GROUP 2
#define P3_FUN_GROUP 3
#define P4_FUN_GROUP 4
#define P5_FUN_GROUP 5
#define P6_FUN_GROUP 6
#define P7_FUN_GROUP 7
#define P8_FUN_GROUP 8
#define P9_FUN_GROUP 9
#define PA_FUN_GROUP 10
#define PB_FUN_GROUP 11
#define PC_FUN_GROUP 12
#define PD_FUN_GROUP 13
#define PE_FUN_GROUP 14
#define PF_FUN_GROUP 15
#define E0x1000_GROUP (0x10) //Ecat 0x1000
#define E0x6000_GROUP (0x18) //Ecat 0x1800
#define MODBUS_GROUP  (0x1A) //     0x1A00

#define F76H_FUN_GROUP	(0x76)	//示波器数据组
#define F77H_FUN_GROUP	(0x77)	//示波器数据组
#define F78H_FUN_GROUP	(0x78)	//示波器数据组

#define P00_FUN_GROUP_SIZE		(sizeof(TYPE_PowerDownArg)/sizeof(Uint16))
#define P0_FUN_GROUP_SIZE		(sizeof(TYPE_MONITOR)/sizeof(Uint16))
#define P1_FUN_GROUP_SIZE		(sizeof(TYPE_BASEPARA)/sizeof(Uint16))
#define P2_FUN_GROUP_SIZE		(sizeof(TYPE_MULTIPOS)/sizeof(Uint16))
#define P3_FUN_GROUP_SIZE		(sizeof(TYPE_MULTISPD)/sizeof(Uint16))
#define P4_FUN_GROUP_SIZE		(sizeof(TYPE_TRQCTL)/sizeof(Uint16))//
#define P5_FUN_GROUP_SIZE		(sizeof(TYPE_TUNING)/sizeof(Uint16))
#define P6_FUN_GROUP_SIZE		(sizeof(TYPE_IO)/sizeof(Uint16))
#define P7_FUN_GROUP_SIZE		(sizeof(TYPE_COMMU)/sizeof(Uint16))
#define P8_FUN_GROUP_SIZE		(sizeof(TYPE_AID)/sizeof(Uint16))
#define P9_FUN_GROUP_SIZE		(sizeof(TYPE_P9)/sizeof(Uint16))
#define PA_FUN_GROUP_SIZE		(sizeof(TYPE_PA)/sizeof(Uint16))
#define PB_FUN_GROUP_SIZE		(sizeof(TYPE_ZERORETURN)/sizeof(Uint16))
#define PC_FUN_GROUP_SIZE		(sizeof(TYPE_PC)/sizeof(Uint16))
#define PD_FUN_GROUP_SIZE		(sizeof(TYPE_PD)/sizeof(Uint16))
#define PE_FUN_GROUP_SIZE		(sizeof(TYPE_MOTOR)/sizeof(Uint16))
#define PF_FUN_GROUP_SIZE		(sizeof(TYPE_SERVO)/sizeof(Uint16))
#define F76H_FUN_GROUP_SIZE		(sizeof(FUN76H_RARA)/sizeof(Uint16))
#define F77H_FUN_GROUP_SIZE		(sizeof(FUN77H_RARA)/sizeof(Uint16))
#define F78H_FUN_GROUP_SIZE		(sizeof(FUN78H_RARA)/sizeof(Uint16))

#define PwoerSaveStartAddr   	(RamFuncCode.P00)  //掉电参数开始地址


#define F_LAST_FUN_GROUP 		(PF_FUN_GROUP)		//连续功能码的最后一组
#define F_FUN_MAXNUM 		    (64)		        //F组能够修改的数量   
   
//#define ModbusStartAddr   		(RamFuncCode.P0)                            //MODBUS可读写功能码起始地址
//#if SCOPE_EN == 1
//#define ModbusEndAddr     		(&RamFuncCode.F76H[F76H_FUN_GROUP_SIZE -1]) //MODBUS可读写功能码结束地址
//#else
//#define ModbusEndAddr     		(&RamFuncCode.PF[F_FUN_MAXNUM -1])          //MODBUS可读写功能码结束地址
//#endif
//#define ModbusOptSize	  		(ModbusEndAddr - ModbusStartAddr + 1) 	    //MODBUS可操作功能码总数

#define FuncAttrStartAddr 		(FuncAttr.P0) 								//属性码的开始地址
#define FactoryFuncStartAddr 	(FactoryFunc.P0)							//出厂值得开始地址

#define RamFuncStartAddr 		(RamFuncCode.P0)						   	//功能码开始地址
#define RamFuncEndAddr    		(&RamFuncCode.PF[PF_FUN_GROUP_SIZE -1])		//功能码结束地址
#define FUNC_SIZE  				(RamFuncEndAddr - RamFuncStartAddr + 1)     //功能码的个数        

#define FactoryStartAddr 		(RamFuncCode.P0)							//上电初始化起始地址         
#define FactoryEndAddr     		(&RamFuncCode.PF[PF_FUN_GROUP_SIZE -1])		//上电初始化结束地址  
#define FACTORY_SIZE   			(FactoryEndAddr - FactoryStartAddr + 1)		//上电初始化参数个数 
     
#define ResetStartAddr 		    (RamFuncCode.P0)							//恢复出厂值起始地址
#define ResetEndAddr     		(&RamFuncCode.PF[F_FUN_MAXNUM -1])		    //恢复出厂值结束地址 
#define RESET_SIZE   			(ResetEndAddr - ResetStartAddr + 1)		    //恢复出厂值参数个数

#define DefaultStartAddr		(RamFuncCode.P0)							//恢复默认值起始地址
#define DefaultEndAddr			(&RamFuncCode.PD[PD_FUN_GROUP_SIZE -1])		//恢复默认值结束地址
#define DEFAULT_SIZE			(DefaultEndAddr - DefaultStartAddr + 1)		//恢复默认值参数个数

#define ECATFuncCodeStartAddr	(RamFuncCode.P0)							//ECAT用于存储厂家参数起始位置（P1组）
#define ECATFuncCodetEndAddr	(&RamFuncCode.PD[PD_FUN_GROUP_SIZE -1])		//ECAT用于存储厂家参数末位置（PD组）
#define ECATFUNCCODE_SIZE		(DefaultEndAddr - DefaultStartAddr + 1)		//ECAT存储厂家参数

   
#define ScopeParaStartAddr 		(RamFuncCode.F76H)	   
#define F77HStartAddr 		    (RamFuncCode.F77H)
#define F78HStartAddr 		    ((Uint16 *)ProductInf.SNCode)	
// 功能码属性中小数位数
typedef enum
{
	ATTR_Dot_0 = 0,                 	// 无小数位
	ATTR_Dot_1 = 1,                 	// 1位小数
	ATTR_Dot_2 = 2,                 	// 2位小数
	ATTR_Dot_3 = 3	                 	// 3位小数
}ENUM_Dot;

// 功能码属性中的单位
typedef enum
{
	ATTR_Unit_Null, 			// null
	ATTR_Unit_Percent,          // %
	ATTR_Unit_0p1Per,			// 0.1%
	ATTR_Unit_0p1PerS,			// 0.1/S
	ATTR_Unit_Hz, 				// Hz
	ATTR_Unit_0p1Hz,            // 0.1Hz
	ATTR_Unit_Rpm,              // rpm
	ATTR_Unit_uS,               // uS
	ATTR_Unit_mS,				// ms
	ATTR_Unit_Sec,              // SEC
	ATTR_Unit_Min,              // MIN
	ATTR_Unit_Hour,             // HOUR
	ATTR_Unit_HzOrSec,          // Hz/S
	ATTR_Unit_PecOrSec,         // %/S
	ATTR_Unit_VOrSec,           // V/S
	ATTR_Unit_MOrSec,           // M/S
	ATTR_Unit_MOrMin,           // M/min
	ATTR_Unit_mm,               // mm
	ATTR_Unit_Ohm,              // Ω
	ATTR_Unit_A,                // A
	ATTR_Unit_V,                // V
	ATTR_Unit_mV,				// mV
	ATTR_Unit_kW,               // kW
	ATTR_Unit_mH,               // mH
	ATTR_Unit_kHz,              // kHz
	ATTR_Unit_EleAngle,         // 电角度
	ATTR_Unit_N,                // N
	ATTR_Unit_C,                // ℃
	ATTR_Unit_HzOrRpm,          // Hz/rpm
	ATTR_Unit_M,				// m
	ATTR_Unit_MWH,				// MW*h
	ATTR_Unit_kWH,				// kW*h
	ATTR_Unit_mOhm,				// mΩ
	ATTR_Unit_Pulse,
	ATTR_Unit_RadOrs,			// rad/s
	ATTR_Unit_W,				// W
	ATTR_Unit_Nm,				// N*m
	ATTR_Unit_NmOrA,			// N*m/A
	ATTR_Unit_KgSqucm           // Kg*cm2
}ENUM_Unit;

// 功能码属性中操作特性
typedef enum
{
	ATTR_OPE_ReadOnly = 0,				// 只读
	ATTR_EFT_Now = 1,					//立即生效
	ATTR_EFT_PowerOn = 2,				//重新上电生效
    ATTR_EFT_RunOff = 3                 //电机失能更改
}ENUM_Operation;

typedef enum
{
	ATTR_TYPE_Single = 0,          		// 参数为一个整体
	ATTR_TYPE_Hex = 1,            		// 参数为十六进制组合
	ATTR_TYPE_Bit = 2            		// 参数为位操作
}ENUM_ParaType;

// 功能码属性中参数是否显示
typedef enum
{
	DISPLAY = 0,						//显示
	HIDE = 1							//不显示
}ENUM_Dis;

// 功能码属性中参数类型
typedef enum
{
	ATTR_BIT_16 = 0,					//16位数据长度
	ATTR_BIT_32 = 1						//3位数据长度
}ENUM_Length;

// 功能码属性中参数是否显示
typedef enum
{
	USIGN = 0,							//无符号数
	SIGN  = 1							//有符号数
}ENUM_Sign;//数据符号

// 定义功能代码的属性
typedef union
{
	Uint16 U;		//无符号数
	int16 S;		//有符号数
}TYPE_AttrSign;
typedef struct
{   	
	ENUM_Dot Dotbits:2;       			// 小数位数
   	ENUM_Unit Unit:6;         	 		// 单位
   	ENUM_Operation Operation:3;     	// 操作
   	ENUM_ParaType ParaType:2;      		// 参数类型
   	ENUM_Dis IsShow:1;	     			// 是否显示
   	ENUM_Length BitSize:1;		 		// 参数位长
   	ENUM_Sign Sign:1;					// 有无符号
   	TYPE_AttrSign Max;           		// 功能代码最大值
   	TYPE_AttrSign  Min;          		// 功能代码最小值
}TYPE_Attribute;

typedef struct
{
	Uint16 P00[P00_FUN_GROUP_SIZE];
	
	Uint16 P0[P0_FUN_GROUP_SIZE];
	Uint16 P1[P1_FUN_GROUP_SIZE];
	Uint16 P2[P2_FUN_GROUP_SIZE];
	Uint16 P3[P3_FUN_GROUP_SIZE];
	Uint16 P4[P4_FUN_GROUP_SIZE];
	
	Uint16 P5[P5_FUN_GROUP_SIZE];
	Uint16 P6[P6_FUN_GROUP_SIZE];
	Uint16 P7[P7_FUN_GROUP_SIZE];
	Uint16 P8[P8_FUN_GROUP_SIZE];
	Uint16 P9[P9_FUN_GROUP_SIZE];
	
	Uint16 PA[PA_FUN_GROUP_SIZE];
	Uint16 PB[PB_FUN_GROUP_SIZE];
	Uint16 PC[PC_FUN_GROUP_SIZE];
	Uint16 PD[PD_FUN_GROUP_SIZE];
	Uint16 PE[PE_FUN_GROUP_SIZE];
    Uint16 PF[PF_FUN_GROUP_SIZE];

	Uint16 F76H[F76H_FUN_GROUP_SIZE];
    Uint16 F77H[F77H_FUN_GROUP_SIZE];
}TYPE_FuncCodeVal;

typedef struct
{
	TYPE_Attribute P0[P0_FUN_GROUP_SIZE];
	TYPE_Attribute P1[P1_FUN_GROUP_SIZE];
	TYPE_Attribute P2[P2_FUN_GROUP_SIZE];
	TYPE_Attribute P3[P3_FUN_GROUP_SIZE];
	TYPE_Attribute P4[P4_FUN_GROUP_SIZE];

	TYPE_Attribute P5[P5_FUN_GROUP_SIZE];
	TYPE_Attribute P6[P6_FUN_GROUP_SIZE];
	TYPE_Attribute P7[P7_FUN_GROUP_SIZE];
	TYPE_Attribute P8[P8_FUN_GROUP_SIZE];
	TYPE_Attribute P9[P9_FUN_GROUP_SIZE];
	
	TYPE_Attribute PA[PA_FUN_GROUP_SIZE];
	TYPE_Attribute PB[PB_FUN_GROUP_SIZE];
	TYPE_Attribute PC[PC_FUN_GROUP_SIZE];
	TYPE_Attribute PD[PD_FUN_GROUP_SIZE];
	TYPE_Attribute PE[PE_FUN_GROUP_SIZE];
	TYPE_Attribute PF[PF_FUN_GROUP_SIZE];

	TYPE_Attribute F76H[F76H_FUN_GROUP_SIZE];
}TYPE_FuncCodeAttr;
//定义功能码结构体

extern  TYPE_FuncCodeVal  RamFuncCode;
extern  TYPE_FuncCodeVal  FactoryFunc;
extern  TYPE_FuncCodeAttr FuncAttr;

extern const Uint16 tbl_identify[4];
extern Uint16 tbl_identifyRAM[4];

#define tbl_identifyaddr ((UINT16 *)(SF_FOE_BANK1_ADDR + SF_FOE_APPLI_SIZE - SF_FOE_APPLI_ID_OFFSET))

extern Uint16 GetFuncAddr(Uint16 Functioncode);
extern Uint16 AddrRangLit(Uint16 Functioncode);
extern Uint16 GetVarAddr(Uint16 *Var);
#endif
