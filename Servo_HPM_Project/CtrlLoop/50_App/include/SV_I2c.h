#ifndef _SV_IIC_H_
#define _SV_IIC_H_
#include "Datatype.h"
#include "SV_FuncCode.h"
#include "userdefine.h"
#include "Drive.h"

#define IIC_DATE_NULL 0xffff         //iic的数据是空，用于判断当前EEPROM是否有存储过
#define IIC_POWERDOWN_CHECK 0x55AA   //掉电参数校验字节
#define IIC_BUSY_TIME 1000           //iic一直处于忙状态

//留给ECAT 512word  P1~PF 1024word   CUSTOM 256word     ECATFULL 256WORD
#define IIC_0x1000_STATR (0)
#define IIC_0x6000_STATR (256)
#define IIC_INIT_ADDR    (0x200)          //iic开始存储数据的地址为第二页首地址，0x10*2=32,从第32个字节地址开始
#define IIC_POWEROFF_ADDR IIC_INIT_ADDR  //掉电参数存放地址
#define IIC_FUNC_ADDR (IIC_INIT_ADDR + P00_FUN_GROUP_SIZE) //功能码存放地址
#define IIC_ECATFULL_ADDR      (0x700)  //0x6000全闭环参数
#define ModbusIIC_ADDR_0x1000  (0)

#define IIC_PAGE_SIZE 16 //一页的空间为16字32字节
#define IIC_READ_END 1 //

#define IIC_POWERUP_TIMEMAX   (5000) //上电读写EEPROM的最大时间

#define IIC_FUNC_SAVE_SIZE 10


#define ModbusCIA402Obj_StartAddr     ((Uint16*)(&(LocalAxes.Objects)))					       //通信相关对象字典RAM区起始地址
#define ModbusCIA402OBJ_SIZE          (SIZEOF(CiA402Objects)/sizeof(UINT16))

#define RamCIA402Obj_StartAddr    	  ((Uint16*)(&(LocalAxes.Objects)))					       //通信相关对象字典RAM区起始地址
#define RAMCIA402OBJ_SIZE             (SIZEOF(CiA402Objects)/sizeof(UINT16))

#define FanCIA402Obj_StartAddr		  ((Uint16*)(&DefCiA402ObjectValues))
#define FANCIA402OBJ_SIZE             (SIZEOF(CiA402Objects)/sizeof(UINT16))

#define FanCIA402Obj_StartAddrFull    ((Uint16*)(&DefCiA402ObjectValuesFull))
#define FANCIA402OBJ_SIZE             (SIZEOF(CiA402Objects)/sizeof(UINT16))  //142

#define RamCumObj_StartAddr				(UINT16*)(&CumObj)	//????????RAM?????
#define RAMCUMOBJ_SIZE                  (sizeof(CumObj)/sizeof(UINT16))

#define FanCumObj_StartAddr             ((UINT16 *)(&DefCommuObjectsValues))
#define FANCUMOBJ_SIZE                  (sizeof(CumObj)/sizeof(UINT16))

#if SERVOTYPE == SERVO_CANOPEN 
#define ModbusCumObj_StartAddr			(UINT16*)(&CumObj.IIC_CheckStart)
#define ModbusCumObj_EndAddr			(UINT16*)(&CumObj.IIC_CheckEnd)
#define ModbusCumObj_SIZE               (ModbusCumObj_EndAddr - ModbusCumObj_StartAddr + 1)

#define CUMOBJEEPROM_STATR              (UINT16*)(&CumObj.IIC_CheckStart) 
#define CUMOBJEEPROM_EndAddr			(UINT16*)(&CumObj.IIC_CheckEnd)
#define CUMOBJEEPROM_SIZE               (CUMOBJEEPROM_EndAddr - CUMOBJEEPROM_STATR + 1) 

#define FanCumObjEEPROM_StartAddr       ((UINT16 *)(&DefCommuObjectsValues.IIC_CheckStart))
#define FANCUMOBJEEPROM_SIZE            CUMOBJEEPROM_SIZE    //205 
#else
#define ModbusCumObj_StartAddr			(UINT16*)(&CumObj.sIdentity.u16SubIndex0)
#define ModbusCumObj_EndAddr			(UINT16*)(&CumObj.sTxPDOassign.aEntries[3])
#define ModbusCumObj_SIZE               (ModbusCumObj_EndAddr - ModbusCumObj_StartAddr + 1)

#define CUMOBJEEPROM_STATR              (UINT16*)(&CumObj.sIdentity.u16SubIndex0) 
#define CUMOBJEEPROM_EndAddr			(UINT16*)(&CumObj.sTxPDOassign.aEntries[3])
#define CUMOBJEEPROM_SIZE               (CUMOBJEEPROM_EndAddr - CUMOBJEEPROM_STATR + 1) 

#define FanCumObjEEPROM_StartAddr       ((UINT16 *)(&DefCommuObjectsValues.sIdentity.u16SubIndex0))
#define FANCUMOBJEEPROM_SIZE            CUMOBJEEPROM_SIZE    //205 
#endif

 

extern Uint16 *PowerOffBuf[P00_FUN_GROUP_SIZE] ;//掉电参数所对应的功能码地址指针数组
//extern Uint16 Glo_IIcReadAll;
typedef enum
{
	IIC_WRITE = 0,  //iic写操作
	IIC_READ = 1   //iic读操作
}IIC_WRITEREADMODE;
typedef enum
{
	IIC_OPE_IDLE = 0,		//空闲状态
	IIC_OPE_WRITE_START =1,	//写操作开始状态
	IIC_OPE_READ_START =2,	//读操作开始状态
    IIC_OPE_WRITE_DELAY =3,	//写等待状态
	IIC_OPE_WRITE_NOSTOP =4,//写未完成状态
	IIC_OPE_WRITE_STOP =5,   //写操作完成状态
	IIC_OPE_READ_STOP=6//读操作完成状态
}IIC_OPE_STAYUS;

typedef enum
{
	TASK_PDSAVE =1, 				//
	TASK_FAULTSAVE1 =2, 			//故障参数存储函数
	TASK_FAULTSAVE2 =3,				//故障参数存储函数2
	TASK_SCIA =4, 					//SCIA写_KEY
	TASK_SDO =5,					//SCIb写_485
	TASK_READALL =6,				//上电全读
	TASK_FAULTREAD =7,				//上电读取故障信息
	TASK_FACSAVE = 8,				//恢复出厂值参数

	TASK_DEFSAVE = 9,  				//恢复默认值参数
	TASK_FAULTCLR = 10,				//历史故障清除
	TASK_TIMECLR =11,				//
	TASK_FUNCSAVE =12,				//功能码存储
	TASK_PANELSAVE =13,             //键盘存储功能码
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
	TASK_1000READ =14,				//1000系列读功能码
	TASK_1000SAVE =15,              //1000系列存功能码
	TASK_6000READ =16,              //6000系列读功能码

	TASK_6000SAVE =17,              //6000系列写功能码
	TASK_1000FACSAVE =18,           //1000系列恢复出厂值
	TASK_6000FACSAVE =19,           //6000系列恢复出厂值
#else
	TASK_14 =14,				    //1000系列读功能码
	TASK_15 =15,                    //1000系列存功能码
	TASK_16 =16,                    //6000系列读功能码
	TASK_17 =17,                    //6000系列写功能码
	TASK_18 =18,                    //1000系列恢复出厂值
	TASK_19 =19,                    //6000系列恢复出厂值
#endif
    
    Task_P1_PDFUNCSAVE =20,          //存储厂家参数，（ECAT用于存储P1组到PD组之间的功能码）
    Task_PE_PFFUNCSAVE =21,
	TASK_RESET =22,
    TASK_23 =23,

	TASK_6000FACSAVEFULL = 24,
	TASK_6000READFull = 25,
    
	TASK_26 =26,	
	TASK_27 =27,
	TASK_28 =28,
	TASK_29 =29,
	TASK_30 =30,
	TASK_31 =31,
	TASK_32 =32
}TaskHandle;

typedef struct
{
	Uint16 IIcAddr;    //IIC的地址,指EEPROM的绝对地址
	Uint16 *Src_Addr;   //收发数据的数据源，指
	Uint16 IIc_TR_Max;	//收发数据的最大个数

	Uint16 AddrCnt;     //数据当前发送的地址偏移量
	Uint16 IIc_TR_Size; //写数据时本次要写的数据个数

	IIC_OPE_STAYUS IIcStatus; //当前的状态
	Uint16 IIcBusyCnt;	//总线忙计数
	volatile Uint16 I2cErrFlag;  //故障标志位
	Uint16 I2cWrAllFunc;//恢复出厂值，恢复默认值后的标志位
	Uint16 ReadEerFlag;//全读参数时出错
    Uint16 WritedelayCnt;
}I2CCtlVal_Type;
#define I2CCTLlVALDEF {\
/*IIcAddr*/           0,\
/**Src_Addr*/         NULL,\
/*IIc_TR_Max*/        0,\
/*AddrCnt*/           0,\
/*IIc_TR_Size*/       0,\
/*IIC_BUS_STATUS*/    IIC_OPE_IDLE,\
/*IIcBusyCnt*/        0,\
/*I2cErrFlag*/        0,\
/*I2cWrAllFunc*/      0,\
/*ReadEerFlag*/       0,\
                      0,\
}
typedef struct
{
	Uint16 size;	//收发数据的最大个数	
	Uint16 IIcAddr;    //IIC的地址
	Uint16 *Src_Addr;   //收发数据的数据源	
	Uint16 *RomAddr;
	Uint16 *RamAddr;
}IIcArg_type;
typedef struct
{
	Uint16 Flag;
	Uint16 size;
	Uint16 FunCode;
}TYPE_IIcFunc;
/**
 * IIC任务类型，按位定义
 */
typedef struct
{
	Uint32 Task_PDSave:1; 		//掉电参数存储
	Uint32 Task_FaultSave1:1; 	//故障参数存储函数
	Uint32 Task_FaultSave2:1; 	//故障参数存储函数
	Uint32 Task_Scia:1; 		//SCIA写_KEY
	Uint32 Task_Sdo:1;			//SCIb写_485
	Uint32 Task_PURead:1;		//上电全读
	Uint32 Task_FaultRead:1;	//上电后读取故障信息
	Uint32 Task_FacWrtie:1;		//恢复出厂值参数（包括PE/PF）

	Uint32 Task_DefWrite:1;  	//恢复默认值参数（不包括PE/PF）
	Uint32 Task_FaultClr:1;		//历史故障清除
	Uint32 Task_TimeClr:1;		//上电运行时间清零
	Uint32 Task_FuncSave:1;		//功能码存储
	Uint32 Task_PanelSave:1;	//键盘参数存储
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
	Uint32 TASK_1000Rd:1;	    //402 1000系列对象字典全读
	Uint32 TASK_1000Wr:1;		//402 1000系列对象字典全写
    
	Uint32 TASK_6000Rd:1;		//402 6000系列对象字典全读
	Uint32 TASK_6000Wr:1;		//402 6000系列对象字典全写
    
	Uint32 Task_1000WrFac:1;	//写1000对象字典默认值
	Uint32 Task_6000WrFac:1;	//写6000对象字典默认值
#else
	Uint32 Task_14:1;	       //402 1000系列对象字典全读
	Uint32 Task_15:1;		   //402 1000系列对象字典全写
	Uint32 Task_16:1;		   //402 6000系列对象字典全读
	Uint32 Task_17:1;		   //402 6000系列对象字典全写
	Uint32 Task_18:1;		   //写1000对象字典默认值
	Uint32 Task_19:1;		   //写6000对象字典默认值
#endif

    Uint32 Task_P1_PDFuncSave:1; //功能码存储，（存储P1组到PD组之间的功能码）
	Uint32 Task_PE_PFFuncSave:1; //存储PE，PF参数
	Uint32 Task_Reset:1;         //复位变量
    Uint32 Task_23:1;

	Uint32 Task_6000WrFacFull:1;
	Uint32 TASK_6000RdFull:1;	
	
	Uint32 Task_26:1;
	Uint32 Task_27:1;
	Uint32 Task_28:1;
	Uint32 Task_29:1;
	Uint32 Task_30:1;
	Uint32 Task_31:1;
	Uint32 Task_32:1;
}TaskFlag_Type;
//任务结构体
typedef union
{
	volatile TaskFlag_Type bit;
	volatile TYPE_STRUCT_DWORD Dword;
	volatile Uint32 All;
}TYPE_IICFlag;


#define PENDINGTASKMAX	16
#define  SCITASKMAX      100
typedef struct
{
	Uint16 Handle;	//任务号
	Uint16 Status;		//每个任务当前状态
	Uint16 PendingTaskCnt;
	volatile Uint16 ServoInit;//表示伺服是第一次启动
    volatile Uint16 ServoAllRead;
    volatile Uint16 Cnt_1ms;
	Uint16 Front;
	Uint16 Rear;
    Uint16 FrontSci;
	Uint16 RearSci;
    Uint16 FrontSdo;
	Uint16 RearSdo;
	IIcArg_type SCIA_Arg;	//SCIA存储接口变量
	IIcArg_type SCIB_Arg;	//SCIB存储接口变量
	IIcArg_type PANEL_Arg;	//键盘存储接口变量
	IIcArg_type	Motor_Arg;
	IIcArg_type TASK[PENDINGTASKMAX];
    IIcArg_type TASKSCI[SCITASKMAX];
    IIcArg_type TASKSDO[SCITASKMAX];

	#define TASK_IDLE  0				//无任务
	#define	TASK_NEED  1				//有任务需求
	#define TASK_STATUS_IDLE  0         //无读写
	#define TASK_STATUS_WRITE  1	    //进入写状态
	#define TASK_STATUS_READ 2	        //进入读状态
	#define TASK_STATUS_END  3	        //读写结束
}IIcTaskSchedule_Type;

#define IICTASK_DEFAULT {\
/*Handle*/            TASK_IDLE,\
/*Status*/			  TASK_STATUS_IDLE,\
/*PendingTaskCnt*/    0,\
/*ServoInit*/         0,\
/*ServoAllRead*/      0,\
/*Cnt_1ms*/           0,\
/*Front*/             0,\
/*Rear*/              0,\
/*FrontSci*/          0,\
/*RearSci*/           0,\
/*FrontSci*/          0,\
/*RearSci*/           0,\
/*SCIA_Arg*/          {0},\
/*SCIB_Arg*/          {0},\
/*PANEL_Arg*/         {0},\
/*Motor_Arg*/		  {0},\
/*TASK[PENDINGTASKMAX]*/ {0},\
/*TASKSCI[PENDINGTASKMAX]*/ {0},\
/*TASKSDO[PENDINGTASKMAX]*/ {0},\
}
extern  IIcTaskSchedule_Type IIcTask;
extern  TYPE_IICFlag IICFlag ;
extern void PowerOnReadFunc(void);
extern void IIcInterfaceA(Uint16 *Func_Addr,Uint16 size);
extern void IIcInterfaceB(Uint16 *Func_Addr,Uint16 size);
extern void IIcInterfaceC(Uint16 *Func_Addr,Uint16 size);
extern I2CCtlVal_Type I2CCtlVal;
extern void IIC_Process(void);
extern void IIcVarInit(void);
extern void IIC_PowerUpCheck(void);
extern void Flash_Eeprom_Init(void);
extern void Flash_Eeprom_WrFac(void);
#endif
