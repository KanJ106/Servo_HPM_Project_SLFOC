#ifndef SV_PANELCTL_H_
#define SV_PANELCTL_H_

#include "userdefine.h"
#include "main.h"//
#include "r_cg_Project.h"
/*****************************自选数据类型********************************/
typedef struct
{
	uint8_t DataLow:8;
	uint8_t DataHigh:8;
}Uint16_2Uint8;
typedef struct
{
	uint16_t DataLow;
	uint16_t DataHigh;
}Uint32_2Uint16;
typedef union
{
	Uint32_2Uint16  Byte32;
	int32_t  Sign32;
	uint32_t UnSign32;
}UnionType32;
typedef union
{
	Uint16_2Uint8 Bit8;
	uint16_t All;
}UnionType16;
/***************************************功能码***********************************/
/**
*功能码组和每一组的数量
*/
extern const uint16_t NUMDIS_DEC2HEX[16];//={'0','1','2','3','4','5','6','7','8','9','A','b','C','d','E','F'};
#define FUNCODEGROUPMAX 14               //当前显示最大最大功能码组
typedef struct
{
	uint8_t Group_Max;//功能码最大组数
	uint8_t Group_Cnt;//当前组
	uint8_t Size_Cnt;//当前组的组内号
	uint16_t FuncGroup[FUNCODEGROUPMAX];//组名
	uint16_t FuncGroupSize[FUNCODEGROUPMAX];//组内号
	uint16_t FacTestFuncCode;
	//功能码完全信息
	//#define FunCodeAutoadjust (0x0109|0x0500|0x0501|0x0502|0x0504|0x0505|0x0506)
    #define FunCodeAutoadjust (0x0512|0x0500|0x0512|0x0513|0x0501|0x0502|0x0517)
	#define FUNCGROUP         {P0_FUN_GROUP,P1_FUN_GROUP,P2_FUN_GROUP,P3_FUN_GROUP,P4_FUN_GROUP,P5_FUN_GROUP,\
	                           P6_FUN_GROUP,P7_FUN_GROUP,P8_FUN_GROUP,P9_FUN_GROUP,PA_FUN_GROUP,PD_FUN_GROUP,PE_FUN_GROUP,PF_FUN_GROUP}
	#define FUNCGROUPSIZE     {P0_FUN_GROUP_SIZE,P1_FUN_GROUP_SIZE,P2_FUN_GROUP_SIZE,P3_FUN_GROUP_SIZE,P4_FUN_GROUP_SIZE,P5_FUN_GROUP_SIZE,\
					           P6_FUN_GROUP_SIZE,P7_FUN_GROUP_SIZE,P8_FUN_GROUP_SIZE,P9_FUN_GROUP_SIZE,PA_FUN_GROUP_SIZE,PD_FUN_GROUP_SIZE,PE_FUN_GROUP_SIZE,PF_FUN_GROUP_SIZE}
}FunCodeGroup;
typedef struct
{
	uint16_t PFPassword:1;//PF组密码是否打开
	uint16_t PEPassword:1;//PE组密码是否打开
	uint16_t ReadFuncAttr :1;//是否更新功能吗属性
	uint16_t FuncChoose:2;//对该功能码的存储操作，包括存入EEPROM，修改RAM和恢复原值
	uint16_t ShowUpDownValue:1;//当前显示的是Up/Down改变之后的值
	uint16_t Unused:10;
	/**********PE组密码状态**************/
	#define PEPASSWORDFUNCODE 0xE00
	#define PEPASSWORDNUM 11111
	/**********PF组密码状态**************/
	#define PFPASSWORDFUNCODE 0x013F
	#define PFPASSWORDNUM 11111
	#define PASSWORDOPEN 1
	/**********更新功能码属性*************/
	#define READFUNCODEATTR 1
	/*********显示的值得来源*****/
	#define SHOWREALVALUE   0  //来源于实时值
	#define SHOWUPDOWNVALUE 1  //来源于Up/Down缓冲区
	/**********功能码修改方式*************/
	#define FUNC_NOSAVE 0    //不存入EEPROM
	#define FUNC_SAVE 1      //存入EEPROM

}FunCodeBit;
/**
 * 功能码属性结构体
 */
typedef struct
{
   	uint16_t Dotbits:2;//小数点
   	uint16_t Unit:6;//单位
   	uint16_t Operation:3;//操作类型
   	uint16_t ParaType:2;//参数类型
   	uint16_t IsShow:1;//是否显示
   	uint16_t BitSize:1;
   	uint16_t Sign:1;
   	/************参数类型***************/
	#define TYPE_UNSIGNED 0
	#define TYPE_SIGNED 1
}FunCodeAttrTypeBit;
typedef union
{
	FunCodeAttrTypeBit Bit;
	uint16_t all;
}UnionFunCodeAttr;
typedef struct
{
   UnionFunCodeAttr Property;
   UnionType32 Max;
   UnionType32 Min;
}FunCodeAttribute;

typedef struct
{
	FunCodeGroup FunCodegroup;
	FunCodeBit   ControlBit;//操作属性
	UnionType16  CurAddr;//当前地址
	uint32_t     BackToPoTime;//在非监视模式下减计数器
	UnionType16  PreNotP0Addr;//非监视组地址
	UnionType16  PreIsP0Addr;//监视组地址
	UnionType32  AddDecValue;//加减之后的值
	UnionType32   Value;//当前值
	FunCodeAttribute Attr;//属性
	#define FuncCode_GetAttr(Fxx)                    FuncAttr.Fxx
	#define ChangeFuncAttr_Op(Fxx)                   FuncCode_GetAttr(Fxx).Operation
	/*********非监视组自动跳转到监视组的计时时间*************/
	#define BACKTOPOTIMEDEF 60000
	/*********功能码特殊操作地址***********************/
	#define FUNCODE_RUN     0x0805     //伺服使能功能码
	#define FUNCODE_JOG     0x0802     //JOG功能码
	#define FUNCODE_SELFLEAN 0x0E14    //自学习功能码
	#define FUNCODE_INERTIALEAN 0x0804 //惯量自学习
    #define FUNCODE_FFT    0x0806      //FFT

}FunCode;
/**
 * 功能码对外接口
 */
typedef union
{
	FunCode Key;
	FunCode Exchange;
	FunCode Led;
} FuncCodeOPeration;
#define FUNCODEDEF {\
/*FunCodegroup*/ {FUNCODEGROUPMAX,0,0,FUNCGROUP,FUNCGROUPSIZE},\
/*ControlBit*/   {0},\
/*CurAddr*/      {0},\
/*PreNotP0Addr*/ {0},\
/*PreIsP0Addr*/  {0},\
/*AddDecValue*/  {0},\
/*Value*/        {0},\
/*Attr*/         {0},\
}

typedef struct
{
	UnionFunCodeAttr Property;//操作属性
	UnionType16 CurAddr;//当前查找地址
	UnionType16 FirstAddr;//初始查找地址
	UnionType16 FinalAddr;//最终找到地址
	UnionType32 FacValue;//出厂值
	UnionType32 Value;//当前值
	uint16_t FineResult;//查找的结果
#define FINDUP 1//UP
#define FINDOUT 2//查找到了
#define FINDFULL 3//查找到一遍
#define FINTDOWN 4 //DOWN
}NoFacCodeOperation;
#define NONEFUNCODEDEF {\
/*Property*/      {0},\
/*CurAddr*/       {0},\
/*FirstAddr*/     {0},\
/*FinalAddr*/     {0},\
/*FacValue*/      {0},\
/*Value*/         {0},\
/*FineResult*/    {0}\
}

/**********************************按键**********************************/
typedef struct
{
	uint16_t KeyLongSet:1;//长时间按下
	uint16_t key_Up:1;
	uint16_t key_Down:1;
	uint16_t Key_Resv:13;
}TYPE_KeyHold;
typedef union
{
	TYPE_KeyHold Bit;
	uint16_t All;
}Type_key_Hold;
/**
 * 按键结构体
 */
typedef struct
{
	uint16_t KeyReceiveData;	//通过SPI读取的电平
	uint16_t KeyPort;			//按键采集电平
	uint16_t keyDownHold;		//按键按下保持标志
	Type_key_Hold keyHold;	//按键按下标志结构体
	uint16_t KeyDownCnt;		//按键按下计数器
	uint16_t KeyUpCnt;		//按键弹起计数器
	uint16_t PreKeyValue;		//上次按键类型
	uint16_t CurKeyValue;		//本次按键类型
	uint16_t KeyType;			//最终按键类型

	#define KEY_KEEP_DOWN_TIME 400        //按键连续按下时间间隔判断
	#define KEY_SHAKE_DELAY 20            //按键检测滤波时间

	/*************按键电平检测标志************************/
	#define KEY_CODE_IDLE 	0x1f00          //无按键按下
	#define KEY_CODE_NULL 	0XFF            //按键检测电平，无按键时为0xff
	#define KEY_CODE_SET 	(0x0f00)        //SET按键按下电平
	#define KEY_CODE_SHIFT 	(0x1700)        //SHIFT按键按下电平
	#define KEY_CODE_DOWN 	(0x1b00)        //DOWN按键按下电平
	#define KEY_CODE_UP 	(0x1d00)        //UP按键按下电平
	#define KEY_CODE_MODE 	(0x1e00)        //MODE按键按下电平

	/**************按键选择*****************************/
	#define KEY_SET 	1                   //SET按键按下跳转标志
	#define KEY_SHIFT 	2                   //SHIFT按键按下
	#define KEY_MODE 	3                   //MODE按键按下
	#define KEY_UP 		4                   //UP按键按下
	#define KEY_DOWN 	5                   //DOWN按键按下
	#define KEY_UP_UP 	11                  //UP按键弹起
	#define KEY_DOWN_UP 12                  //DWON按键弹起
} KeyInfo_Type;
#define KEYINFODEF {\
/*KeyReceiveData*/     0,\
/*KeyPort*/            0,\
/*keyDownHold*/        0,\
/*keyHold*/            {0},\
/*KeyDownCnt*/         0,\
/*KeyUpCnt*/           0,\
/*PreKeyValue*/        0,\
/*CurKeyValue*/        0,\
/*KeyType*/            0,\
}
/***********************************LED************************************/
/**
 * 对单个数码管建立起来的结构体，包括要显示的数字，小数点位置，是否显示，数字是否闪烁
 */
typedef struct
{
    uint8_t Num;//要显示的字母
    uint8_t Point;//小数点位置
    uint8_t ShowHidden;//是否显示
    uint8_t Twinkling;//是否闪烁，是指数字是否闪烁，不包括小数点
}LedAttributes;
/**
 * 数码管相关结构体
 */
typedef struct
{
	uint16_t TimeCont;//伺服上电初始化显示商标时间
	//LedAttributes DispFarmeBuf[5];//5个数码管显示相关属性4|3|2|1|0
	uint8_t Num[5];//要显示的字母
	uint8_t Point[5];//小数点位置
	uint8_t ShowHidden[5];//是否显示
	uint8_t Twinkling[5];//是否闪烁，是指数字是否闪烁，不包括小数点
	uint8_t Led_Selectshift[5];//数码管排列顺序
	uint16_t Disp_Scheduling;//单个字母的小数点闪烁时间
	uint16_t DataToLEDSeg;//通过SPI发送到数码管的数据
	uint16_t Sel_Led;     //当前选择哪个二极管亮
	#define TIME_TWINKPERD		(600)		//闪烁周期
	#define LED_PIONT_OFF 0
	#define LED_PIONT_ON 1
	#define LED_TWINK_OFF 0
	#define LED_TWINK_ON 1
	#define LED_SHOW_OFF 0
	#define LED_SHOW_ON 1
}LedControl;

#if (SERVO_HARDWARE == HARDWARE_AC0||SERVO_HARDWARE == HARDWARE_DC0)
#define LEDDISPLAYDEFAULT {\
/*TimeCont*/              0,\
/*Num[5]*/                {0},\
/*Point[5]*/              {0},\
/*ShowHidden[5]*/         {0},\
/*Twinkling[5]*/          {0},\
/*Led_Selectshift[5]*/    {4,3,2,0,1},\
/*Disp_Scheduling*/       0,\
/*DataToLEDSeg*/	      0,\
/*Sel_Led*/               0,\
}
#endif
#if (SERVO_HARDWARE == HARDWARE_AC1)
#define LEDDISPLAYDEFAULT {\
/*TimeCont*/              0,\
/*Num[5]*/                {0},\
/*Point[5]*/              {0},\
/*ShowHidden[5]*/         {0},\
/*Twinkling[5]*/          {0},\
/*Led_Selectshift[5]*/    {0,1,2,4,3},\
/*Disp_Scheduling*/       0,\
/*DataToLEDSeg*/          0,\
/*Sel_Led*/               0,\
}
#endif
/**
 *  _
 * |_|
 * |_|.
 * 对于数码管来说，第0个数码管表示最上面一个LED管，然后顺时针旋转到第5个数码管，中间一个表示6号数码管，小数点是7号数码管
 */
typedef enum{
	Seg7Disp_0		=	(0xC0),
	Seg7Disp_1		=	(0xF9),
	Seg7Disp_2		=	(0xA4),
	Seg7Disp_3		=	(0xB0),
	Seg7Disp_4		=	(0x99),
	Seg7Disp_5		=	(0x92),
	Seg7Disp_6		=	(0x82),
	Seg7Disp_7		=	(0xF8),
	Seg7Disp_8		=	(0x80),
	Seg7Disp_9		=	(0x90),
	Seg7Disp_A		=	(0x88),
	Seg7Disp_b		=	(0x83),	//小写
	Seg7Disp_C		=	(0xC6),
	Seg7Disp_d		=	(0xA1),	//小写
	Seg7Disp_E		=	(0x86),
	Seg7Disp_F		=	(0x8E),
	Seg7Disp_G		=	(0xC2),
	Seg7Disp_H		=	(0x89),
	Seg7Disp_I		=	(0xCF),
	Seg7Disp_J		=	(0xF1),
	Seg7Disp_K		=	(0x85),
	Seg7Disp_L		=	(0xC7),
	Seg7Disp_M		=	(0xAA),
	Seg7Disp_n		=	(0xAB),	//小写
	Seg7Disp_o		=	(0xA3),	//小写
	Seg7Disp_P		=	(0x8C),
	Seg7Disp_q		=	(0x98),	//小写
	Seg7Disp_r		=	(0xAF),	//小写
	Seg7Disp_s		=	(0x92),	//小写
	Seg7Disp_t		=	(0x87),	//小写
	Seg7Disp_u		=	(0xE3),	//小写
	Seg7Disp_V		=	(0x81),
	Seg7Disp_W		=	(0x95),
	Seg7Disp_X		=	(0xC9),
	Seg7Disp_y		=	(0x91),	//小写
	Seg7Disp_Z		=	(0xB6),
	Seg7Disp_NULL	=	(0xFF), //空格

	Seg7LED_0		=	(0xFE),//上横,0号数码管
	Seg7LED_1		=	(0xFD),//右上竖杠，1号数码管
	Seg7LED_2		=	(0xFB),//右下竖杠，2号数码管
	Seg7LED_3		=	(0xF7),//下横，3号数码管
	Seg7LED_4		=	(0xEF),//左下竖杠，4号数码管
	Seg7LED_5		=	(0xDF),//左上竖杠，5号数码管
	Seg7LED_6	    =	(0xBF),//中横，6号数码管
	Seg7LED_7	    =	(0x7F),//小数点，7号数码管
	Seg7LED_ALL     =   (0x00)//全部数码管

}ENUM_SEG7DISP;
#define Seg7Disp_mid Seg7LED_6
#define Seg7Disp_up  Seg7LED_0
#define Seg7Disp_dn  Seg7LED_3
/*********************************键盘状态结构体**************************************/
typedef struct
{
	uint16_t P0Count:1;			//仅在P0组监视组内切换时标志位
	uint16_t Fault :1;			//伺服故障标志位
	uint16_t Warning :1;			//伺服警告标志位
	uint16_t Brakingresistor :1;	//制动电阻标志位

	uint16_t StartSelfLean :1;	//自学习标志
	uint16_t StartLeanInertia:1;	//惯量辨识
	uint16_t PoOn:1;				//单个功能码需重新上电才有效
	uint16_t IsPoOn:1;			//表示本次操作确实有该操作，

	uint16_t FFt:1;				//FFT
	uint16_t JOG:1;				//JOG
	uint16_t LastFacTest:1;		//上一次工厂自检的值
	uint16_t Unsed :5;
}PanelCtl_Bit;
typedef struct
{
	PanelCtl_Bit Bit;
    uint8_t Status; 				//键盘状态，包括初始化、正常、警告、故障、自学习等
    uint8_t Level; 				//键盘菜单等级
    uint8_t Cursor; 				//光标位置
    uint16_t CursorMAX; 			//光标最大值
    uint16_t P0CountDown; 		//P0组倒计时
    uint16_t Level4TimeDown; 		//
    uint16_t FaultNum; 			//故障号
    uint16_t WarningNum; 			//警告号
    uint16_t ForRev; 				//正转反转标志
    uint16_t ControlType;			//控制类型，速度、力矩和位置
    uint16_t FaultCountDown;		//故障倒计时
    uint16_t WarningCountDown;	//警告倒计时
    uint16_t JOG_FWD_REV;			//停止为0，正转为1，反转为2
    uint16_t RegulateMode;		//增益调整模式选择，P5-07
    uint16_t MoterCode;			//电机代码,PE-02
    uint16_t ServoCode;			//驱动器代码,PF-06
    uint16_t ServeOn;				//伺服使能标志
    uint16_t PoOnTimeDown;		//重新上电弹出界面减计数器
    char MenuType_AC;   		//菜单模式
    /************当前菜单模式*********************************/
	#define MENUTYPE_A 'A'  	//默认，全菜单模式
	#define MENUTYPE_C 'C'  	//非出厂值模式
    /**************伺服所处的状态，用于P0.13显示******************/
	#define SERVOSTATUS_FUNCODE 0x000D  //表示伺服状态的功能码P0-13
	#define SERVOSTATUS_NRDY 0     		//未准备好，母线电压未建立
 	#define SERVOSTATUS_RDY 1      		//准备好
 	#define SERVOSTATUS_RUN 2      		//使能状态
 	#define SERVOSTATUS_FAULT 3    		//故障状态
	#define SERVOSTATUS_WORNING 4  		//警告状态
	#define SERVOSTATUS_HOMING 5   		//原点回归状态
     /*************光标移动方向***************************/
 	#define CURSORMOVELEFT 0
 	#define CURSORMOVERIGHT 1
    /***************JOG******************************/
	#define JOGFUNCODE 0x0802  	//JOG功能码*P8-02
	#define JOG_STOP 0			//停止
	#define JOG_FWD 1			//正转
	#define JOG_REV 2			//反转
    /*************P0.18显示内容************************/
	#define FAULTFUNCODE 0x0012
    /************************************************/
	#define REGULATEMODE RamTuning->RegulateMode;	//增益调整模式,P5-07
	#define MOTOR_CODE RamMotor->MotorCode;			//电机代码,PE-02
	#define SERVO_CODE RamServo->ServoCode;			//驱动器代码，PF-06
	#define MOTOR_CODE_LMTSIZE	(26)       			//电机代码更新PE组内容
    //工厂自检

    #define FUNCCODEFACTEST              ((SysFtest == 1)||(RamServo->FactortTest == 1))//工厂自检入口功能码

    #define PASYSTEST                    0 //屏蔽以前的自测模式 2025.01.20
	#define FUNCCODEGROUPNUM             PA_FUN_GROUP           //工厂自检功能码组
	#define SELFINSPECTION_KEY           0x0A1F                 //工厂自检，按键自检功能码
	#define SELFKEY_FUNCCODE             0xFFFF //RamPA->KEY	//工厂自检按键自检功能码
	#define SELFINSPECTION_LED           0x0A1E                 //工厂自检，LED自检功能码
	#define SELFLED_FUNCCODE             0xFFFF //RamPA->LED    //工厂自检LED自检功能码
    #define KEYCHECK                     0
    
}Servo_Panel ;

#define SERVO2PANELDEFAULT {\
/*Bit                 */{0},\
/*Status              */STATUS_POWERON_INIT,\
/*Level               */0,\
/*Cursor              */0,\
/*CursorMAX           */0,\
/*P0CountDown         */0,\
/*Level4TimeDown      */0,\
/*FaultNum            */0,\
/*WarningNum          */0,\
/*ForRev              */0,\
/*ControlType         */0,\
/*FaultCountDown      */0,\
/*WarningCountDown	  */0,\
/*JOG_FWD_REV		  */0,\
/* RegulateMode		  */0,\
/*MoterCode           */0,\
/*ServoCode           */0,\
/*ServeOn			  */0,\
/*PoOnTimeDown		  */0,\
/*MenuType_AC         */MENUTYPE_A,\
}
/**
 * 键盘显示状态
 */
typedef enum
{
	STATUS_POWERON_INIT = 0,	//初始化
	STATUS_SELF = 1,			//自学习
	STATUS_NORMAL = 2,			//正常状态
	STATUS_FACTORYTEST = 3,		//工厂自检状态
	STATUS_FAULT = 4,			//故障状态
	STATUS_WARNING = 5,			//警告状态
	STATUS_INERTIALEAN = 6,		//离线惯量辨识
	STATUS_FFT = 7,				//FFT
	STATUS_POWERON = 8			//重新上电有效
}ENUM_TYPE_INIT;
typedef enum
{
	RUN_STOP = 0,        //停止
    RUN_FWD = 1,         //正转
    RUN_REV = 2          //反转
}RUN_STATE;
typedef enum
{
	PANELCTL_LEVEL0 = 0,
	PANELCTL_LEVEL1 = 1,
	PANELCTL_LEVEL2 = 2,
	PANELCTL_LEVEL3 = 3,
	PANELCTL_LEVEL4 = 4,
	PANELCTL_LEVEL5 = 5
}PANEL_LEVEL;
/***************************对外接口***********************************/
/**
 * 全局变量
 */
extern FuncCodeOPeration Func;
extern Servo_Panel Servo2Panel;
/**
 * 接口函数
 */
extern void Panel_mainloop(void);
extern void PanelCtl(void);
void Panel_Init(void);
#endif
