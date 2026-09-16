/******************************************************************************
**                           深圳市同川科技有限公司
**                               TC200伺服驱动器//
**                               www.tc-tech.net
**
----------------------------------文件信息-------------------------------------
**文   件   名：SV_SV_FuncCode.c
**创   建   人：杨玉亮
**最后修改日期：2015-4-13
**对外接口函数 		void GetFuncAttr(Uint16 Functioncode,TYPE_FuncAttr *Attr)
** 调用驱动函数 	
**描        述：定义功能码的出厂值表，属性表，以及得到功能码的属性值
**其        它：无
**函 数 清  单：
**历        史：修改历史记录列表，每条记录包括修改日期、修改者及修改内容简述
                1.日期：
                作者：
                描述：
                2....//
******************************************************************************/
#include "userdefine.h"
#include "r_cg_Project.h"
#include "SV_FuncVar.h"
#include "SV_FuncCode.h"
#include "SV_Sci.h"
#include "SV_CtrlMode.h"
#include "SV_DataType.h"
#include "Version.h"

#if SERVO_MCU == TI_C2000
const Uint16 tbl_identify[4] = {PRDCTSer1,EEPROMSer1,0,0};
#elif SERVO_MCU == ST_STM32
const Uint16 tbl_identify[4] __attribute__ ((at(0x8040000 - 0x10))) = {PRDCTSer1,EEPROMSer1,0,0};
#elif SERVO_MCU == RZ_RZT1
#pragma location = "_IDENTIFY"
const Uint16 tbl_identify[4] = {PRDCTSer1,EEPROMSer1,0,0};
#elif SERVO_MCU == HPM_6E00
const uint16_t tbl_identify[4] = {PRDCTSer1,EEPROMSer1,0,0};
#else
const Uint16 tbl_identify[4] = {PRDCTSer1,EEPROMSer1,0,0};  
#endif

Uint16 tbl_identifyRAM[4] = {0,0,0,0};
TYPE_FuncCodeVal RamFuncCode;
	
TYPE_FuncCodeVal  FactoryFunc =
	
{
	/**************掉电参数组************************/
	{
	/*00.00~00.07*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	},
	/**************P0组功能码的出厂值************************/
	{
	/*P0-00~P0-07*/ 	0,		0,		0,		0,	 	0,		0,		0,		0,
	/*P0-08~P0-15*/ 	0,		0,	  100,		0,		0,		0,		0,		0,
	/*P0-16~P0-23*/ 	0,		0,		0,		0,		0,	  300,		0,		0,
	/*P0-24~P0-31*/		0,		0,		0,		0,    	0,		0,		0,		0,
	/*P0-32~P0-39*/PRDCTSer1,CUSTOMER,  0,      0,      0,      0,      0,      0,
	/*P0-40~P0-47*/		0,      0,      0,      0,      0,      0, HARDWARENUM,  0,       
	},
	{
	/**************P1组功能码的出厂值************************/
	/*P1-00~P1-07*/ 	1,		0,		4,		0,		0,		0,		0,	5000,
	/*P1-08~P1-15*/ 	0,		0,	    0,	    0,	  400,	  400,	    0,	 400,
	/*P1-16~P1-23*/ 	400,	0,	   100,	  100,	    0,	    0,		0,	   0,
	/*P1-24~P1-31*/ 	0,	    0,		1,		0,		1,	    0,		0,	   1,
	#if SERVOPOWER == SERVO_DC
    #if SERVO_PRODUCTNUM == UR_11
    /*P1-32~P1-39*/ 	20,		10,		0,		1,  25000,	25000,	    50,     3000,
	/*P1-40~P1-47*/ 	10,		10,		100,   20,		10,		0,		2100,	2100,
    #elif SERVO_PRODUCTNUM == UR_14
    /*P1-32~P1-39*/ 	20,		10,		0,		1,  25000,	25000,	    50,     3000,
	/*P1-40~P1-47*/ 	10,		10,		100,   20,		10,		0,		2100,	2100,
    #elif SERVO_PRODUCTNUM == UR_17
    /*P1-32~P1-39*/ 	20,		10,		0,		1,  25000,	25000,	    50,     3000,
    /*P1-40~P1-47*/     10,     10,     100,   20,      10,     0,      2100,   2100,
    #elif SERVO_PRODUCTNUM == UR_20
    /*P1-32~P1-39*/ 	20,		10,		0,		1,  25000,	25000,	    50,     3000,
    /*P1-40~P1-47*/     10,     10,     100,   20,      10,     0,      2100,   2100,
    #elif SERVO_PRODUCTNUM == UR_25
    /*P1-32~P1-39*/ 	20,		10,		0,		1,  25000,	25000,	    50,     3000,
    /*P1-40~P1-47*/     10,     10,     100,   20,      10,     0,      2100,   2100,
    #elif SERVO_PRODUCTNUM == UR_32
    /*P1-32~P1-39*/ 	20,		10,		0,		1,  25000,	25000,	    50,     3000,
    /*P1-40~P1-47*/     10,     10,     100,   20,      10,     0,      2100,   2100,
    #elif SERVO_PRODUCTNUM == Other1
    /*P1-32~P1-39*/ 	20,		10,		0,		1,  25000,	25000,	    50,     3000,
    /*P1-40~P1-47*/     10,     10,     100,   20,      10,     0,      3000,   3000,
    #else
    /*P1-32~P1-39*/ 	20,		10,		0,		1,  25000,	25000,	    50,     3000,
    /*P1-40~P1-47*/     10,     10,     100,   20,      10,     0,      2500,   2500,
    #endif
    #else
    /*P1-32~P1-39*/ 	20,		10,		0,		1,  25000,	25000,	    50,     3000,
	/*P1-40~P1-47*/     10,     10,     100,   20,      10,     0,      3000,   3000,
    #endif
    /*P1-48~P1-55*/ 	0,		0,	      0,	0,		0,	    0,		200,	200,
	/*P1-56~P1-63*/  6000,	  500,	      0,	0,		0,		0,		  0,	  0,
	},
	{
	/**************P2组功能码的出厂值************************/
	/*P2-00~P2-07*/     0,		0,      0,		0,		0,		0,		0,		0,
	/*P2-08~P2-15*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	/*P2-16~P2-23*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	/*P2-24~P2-31*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	/*P2-32~P2-39*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	/*P2-40~P2-47*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	/*P2-48~P2-55*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	/*P2-56~P2-63*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	},
	{
	/**************P3组功能码的出厂值************************/       
	/*P3-00~P3-07*/   100,      0,		0,	    0,		0,	    0,		0,		0,
	/*P3-08~P3-15*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	/*P3-16~P3-23*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	/*P3-24~P3-31*/ 	0,		0,		0,		0,		0,		0,		0,		0,
	},
	{
	/**************P4组功能码的出厂值************************/
	/*P4-00~P4-07*/ 	1000,	100,	0,		1000,	0,		0,		0,	    0,
	/*P4-08~P4-15*/ 	   0,     0,    0,         0,   0,      0,      0,      0,
	/*P4-16~P4-23*/ 	   0,     0,    0,         0,   0,      0,      0,      0,
    /*P4-24~P4-31*/ 	   0,     0,    0,         0,   0,      0,      0,      0,
    },
	{
	/**************P5组功能码的出厂值************************/
	/*P5-00~P5-07*/ 	320,   300,	  310,	    0,	 300,       10,	    0,	   10,
	/*P5-08~P5-15*/ 	  0,	 0,		0,	   20,	   20,       0,	   13,	  250,
	/*P5-16~P5-23*/ 	100,   100,	 0x04,	  100,	   20,     800,	  100,	   75,
	/*P5-24~P5-31*/ 	0,		0,		0,		0,      0,       0,   250,      0,
    /*P5-32~P5-39*/ 	0,		0,		0,		0,      0,       0,     0,      0,
    /*P5-40~P5-47*/ 	0,		0,		0,		0,      0,       0,     0,      0
	},
	{
	/**************P6组功能码的出厂值************************/
	/*P6-00~P6-01*/ 	2,		0,
#if SERVOTYPE == SERVO_ETHERCAT
	/*P6-02~P6-09*/     0,		0,		0,		0,		0,		0,      0,		0,
#else
	/*P6-02~P6-09*/     1,		2,		3,		4,		5,		6,      7,		8,
#endif
	/*P6-10~P6-12*/ 	0,		0,		2,
#if SERVOTYPE == SERVO_ETHERCAT
	/*P6-13~P6-16*/		1,		8,		2,      12,
#else
	/*P6-13~P6-16*/		1,		12,		2,      12,
#endif
	/*P6-17~P6-24*/ 	0,		0,		0,		0,		0,		 0,		 0,     0,
	/*P6-25~P6-32*/ 	0,		0,		0,		0,      0,       0,      0,     0,
    /*P6-33~P6-35*/     0,      0,      0
	},

	{
	/**************P7组功能码的出厂值************************/
	/*P7-00~P7-07*/ 	1,		5,		0,		1,		2,		0,		0,		0,
	/*P7-08~P7-15*/ 	4,		0,		0,		0,		0,		0,		0,		0,
#if SERVOTYPE == SERVO_CANOPEN
	/*P7-16~P7-23*/     1,      1,     10,      4,      0,      0,      0,      0,
#else
    /*P7-16~P7-23*/     1,      1,     10,      4,      1,      0,      0,      0,
#endif
    /*P7-24~P7-31*/     1,      5,      0,      0,      0,      0,      0,      0,
	},

	{
	/**************P8组功能码的出厂值************************/
	/*P8-00~P8-07*/ 	0,		0,		0,		100,	0,		0,		0,	   13,
	/*P8-08~P8-15*/ 	0,		1,		2,		0,		80,		80,		0,		3,
	/*P8-16~P8-23*/ 	500,	20,		60,    400,	    30,	    100,	0,      0,
#if SERVOPOWER == SERVO_DC
	/*P8-24~P8-31*/     50,     200,    10,     0,      2,      0,      0,    100,
#endif
#if SERVOPOWER == SERVO_AC
	/*P8-24~P8-31*/     50,     200,    10,     0,      1,      0,      0,      0,
#endif
	/*P8-32~P8-39*/      0,       0,     1,     1500,   200,    1,      0,      0,
    /*P8-40~P8-47*/      1,      90,   100,     90,    100,     0,      0,      0
	},
    
	/**************P9组功能码的出厂值************************/
	{
	/*P9-00~P9-07*/ 	0,        0,      0,     0,      0,      0,      0,    0,
    /*P9-08~P9-15*/ 	0,        0,      0,     0,      0,      0,      0,    0,
    /*P9-16~P9-23*/ 	0,        0,      0,     0,      0,      0,      0,    0,
    /*P9-24~P9-31*/ 	0,        0,      0,     0,      0,      0,      0,    0
	},
	{
	/**************PA组功能码的出厂值************************/
    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
	/*PA-00~PA-07*/ 	0,        0,      0,     0,      0,      0,      0,    0,
    /*PA-08~PA-15*/ 	0,        0,      0,     0,      0,      0,      0,    0,
    /*PA-16~PA-23*/ 	0,        0,      0,     0,      0,      0,      0,    0,
    /*PA-24~PA-31*/ 	0,        0,      0,     0,      0,      0,      0,    0        
    #endif
        
    #if SERVOTYPE == SERVO_MODBUS
	/*PA-00~PA-07*/ 	1,		0,		0,		0,		0,	 0xC8,		0,	 0xC8,
	/*PA-08~PA-15*/  5000,	    0,	 0x64,	    0,		0,		0,		0,		0,
    
    #if SERVO_PRODUCTNUM == UR_11
    /*PA-16~PA-23*/ 	0,	 0x14,		0,	 0x64,	 2500,	   50,		5,		0,
    #elif SERVO_PRODUCTNUM == UR_14
    /*PA-16~PA-23*/ 	0,	 0x14,		0,	 0x64,	 2500,	   50,		5,		0,
    #elif SERVO_PRODUCTNUM == UR_17
    /*PA-16~PA-23*/ 	0,	 0x14,		0,	 0x64,	 2500,	   50,		5,		0,
    #elif SERVO_PRODUCTNUM == UR_20
    /*PA-16~PA-23*/ 	0,	 0x14,		0,	 0x64,	 2500,	   50,		5,		0,
    #elif SERVO_PRODUCTNUM == UR_25
    /*PA-16~PA-23*/ 	0,	 0x14,		0,	 0x64,	 2500,	   50,		5,		0,
    #else
    /*PA-16~PA-23*/ 	0,	 0x14, 0xAAAA,	 0x42,	 1500,	   50,		5,		0,
    #endif
    
    /*PA-24~PA-31*/ 	0,		0,		0,		0,		0,		0,		0,		0
    #endif
        
    #if SERVOTYPE == SERVO_CAN
	/*PA-00~PA-07*/ 	1,		0,		0,		0,	  500,	    0,	  500,	    0,
	/*PA-08~PA-15*/  5000,	    0,	  546,	    0,		0,		0,		0,		0,
    
    #if SERVO_PRODUCTNUM == UR_11
    /*PA-16~PA-23*/ 	0,	 0x14,0x4000,	 0x00,	 2100,	   50,		5,	  50,
    #elif SERVO_PRODUCTNUM == UR_14
    /*PA-16~PA-23*/ 	0,	 0x14,0x4000,	 0x00,	 2100,	   50,		5,	  50,
    #elif SERVO_PRODUCTNUM == UR_17
    /*PA-16~PA-23*/ 	0,	 0x14,0x4000,	 0x00,	 2100,	   50,		5,	  50,
    #elif SERVO_PRODUCTNUM == UR_20
    /*PA-16~PA-23*/ 	0,	 0x14,0x4000,	 0x00,	 2100,	   50,		5,	  50,
    #elif SERVO_PRODUCTNUM == UR_25
    /*PA-16~PA-23*/ 	0,	 0x14,0x4000,	 0x00,	 2100,	   50,		5,	  50,
    #else
    /*PA-16~PA-23*/ 	0,	 0x14,0x2AAB,	 0x00,	 2100,	   50,		5,	  50,
    #endif
    
    /*PA-24~PA-31*/ 	4,		0,		0,		0,		0,		0,		0,		0
    #endif
	}, 

	{
	/**************PB组功能码的出厂值************************/
	/*PB-00~PB-07*/ 	0,		1,		0,		0,		500,	50,		0,		0,
	/*PB-08~PB-15*/ 	1,		100,	100,    1,        0,     0,     0,      0,
    /*PB-16~PB-23*/ 	0,		0,		0,		0,		  0,     0,		0,		0,
    /*PB-24~PB-31*/ 	0,		0,		0,		0,		  0,	 0,		0,		0
	},

	{
	/**************PC组功能码的出厂值************************/
	/*PC-00~PC-07*/     1,       0,     0,      0,  100,    100,      0,        100,
    /*PC-08~PC-15*/     0,       0,     0,      0,    0,    210,      0,        0,
    /*PC-16~PC-23*/ 	0,		 0,		0,		0,	  0,	  0,	  0,		0,
    /*PC-24~PC-31*/ 	0,		 0,		0,		0,	  0,	  0,	  0,		0
	},

	{
	/**************PD组功能码的出厂值************************/
    /*PD-00~PD-07*/     0,     512,     5,         0,        0,    2,   101,    0,
    /*PD-08~PD-11*/     0,       0,     0,         0,        0,    0,     0,    0
	},

	{
#if SERVOPOWER == SERVO_AC
	 /**************PE组电机参数的出厂值(code 203)************************/
     /*PE-00~PE-07*/         0,       204,        40,     280,    127,    220,    3000,     4500,
     /*PE-08~PE-15*/         5,      4078,      4078,    2560,     58,      0,      46,        4,
     /*PE-16~PE-23*/     32768,         0,      1800,       0,      0,      0,       0,     2000,
     /*PE-24~PE-31*/      2000,      4000,      4000,     100,    100,      0,       0,        0,
     /*PE-32~PE-33*/         0,         0
#endif
       
#if SERVOPOWER == SERVO_DC
        #if SERVO_PRODUCTNUM == UR_11
        /*PE-00~PE-07*/ 	   0,	      0,		  3,	  	216,	 12,    48,  	3000,	4000,
        /*PE-08~PE-15*/ 	   7,       760,	    760,       1500,   2100,    410,	   3,	  12,        
        /*PE-16~PE-23*/ 	32768,	      0,		 300,	      0,      0,	 0,		   3,   2500,
        /*PE-24~PE-31*/ 	2500,	   4500,	    4500,	    100,	100,	 0,        0,      0,
        /*PE-32~PE-33*/        0,         0,           0,         0,
        #elif SERVO_PRODUCTNUM == UR_14
        /*PE-00~PE-07*/ 	   0,	      0,		  4,	  	132,	 16,    48,  	3000,	4000,
        /*PE-08~PE-15*/ 	   10,      2000,	    2000,      2800,   2100,   750,		   3,	  12,
        /*PE-16~PE-23*/ 	32768,	      0,		 300,	      0,      0,	 0,		   0,   4187,
        /*PE-24~PE-31*/ 	4187,	   7019,	    7019,	    100,	100,	 0,        0,      0,
        /*PE-32~PE-33*/        0,         0,           0,         0,
        #elif SERVO_PRODUCTNUM == UR_17
        /*PE-00~PE-07*/ 	   0,	     0,		     16,	    540,	 60,    48,  	3000,	4000,
        /*PE-08~PE-15*/ 	  10,       440,	     440,       460,   2100,   680,		  11,	  12,
        /*PE-16~PE-23*/    32768,	      0,		300,	      0,      0,	 0,		   0,   2300,
        /*PE-24~PE-31*/ 	2300,	   4271,	    4271,	    100,	100,	 0,        0,      0,
        /*PE-32~PE-33*/        0,         0,           0,         0,               
        #elif SERVO_PRODUCTNUM == UR_20
        /*PE-00~PE-07*/ 	   0,	      0,		  22,	    708,	  84,    48,  	 3000,  4000,
        /*PE-08~PE-15*/ 	   10,       430,	     430,       300,	2100,   740,	   34,	 12,
        /*PE-16~PE-23*/    32768,	     0,		    300,	      0,      0,	 0,		   0,   3265,
        /*PE-24~PE-31*/ 	3265,	   2594,	    2594,	   1000,	500,	 0,        0,      0,
        /*PE-32~PE-33*/        0,         0,           0,         0,                             
        #elif SERVO_PRODUCTNUM == UR_25
        /*PE-00~PE-07*/ 	   0,	      0,		  44,	   1800,	168,    48,  	3000,	4000,
        /*PE-08~PE-15*/ 	  10,       160,	     160,       130,   2100,   730,		  80,	  12,
        /*PE-16~PE-23*/    32768,	      0,		300,	      0,      0,	 0,		   0,   2300,
        /*PE-24~PE-31*/ 	2300,	   4271,	    4271,	    100,	100,	 0,       10,      0,
        /*PE-32~PE-33*/        0,         0,           0,         0,          
        #elif SERVO_PRODUCTNUM == UR_32
        /*PE-00~PE-07*/ 	   0,	      0,		  90,	   2450,	270,    48,  	2500,	3100,
        /*PE-08~PE-15*/ 	  11,        30,	     30,        70,    2100,   730,		 120,	  12,
        /*PE-16~PE-23*/    32768,	      0,		300,	      0,      0,	 0,		   0,   2000,
        /*PE-24~PE-31*/ 	2000,	   1500,	    1500,	    100,	100,	 0,        0,      0,
        /*PE-32~PE-33*/        0,         0,           0,         0, 
        #elif SERVO_PRODUCTNUM == Other1
        /*PE-00~PE-07*/ 	   0,		  0,		  40,	  	250,	 13,    48,  	3000,	5000,
        /*PE-08~PE-15*/ 	   7,       100,	     100,       280,   2500,   600,		  10,	   2,
        /*PE-16~PE-23*/ 	   0,	     32,		300,	      0,      0,	 0,		   0,   2300,
        /*PE-24~PE-31*/ 	2300,	   4271,	    4271,	    100,	100,	 0,        0,      0,
        /*PE-32~PE-39*/        0,         0,           0,         0,   
        #endif
#endif
	},

	{
	 /**************PF组功能码的出厂值(code 2)************************/
#if SERVOPOWER == SERVO_AC
	/*PF-00~PF-07*/ EEPROMSer1,  0,      100,    100,    570,    220,     4,      0,
	/*PF-08~PF-15*/   397,  3000,   0,      125,      0,      0,      0,      0,
#endif

#if SERVOPOWER == SERVO_DC
    #if SERVO_PRODUCTNUM == UR_11
	/*PF-00~PF-07*/  EEPROMSer1,	0,		100,	100,	501,	 48,	  0,    0,
	/*PF-08~PF-15*/   113,       2500,        0,    125,      0,      0,     50,    0,
    #elif SERVO_PRODUCTNUM == UR_14
	/*PF-00~PF-07*/  EEPROMSer1,	0,		100,	100,	401,	 48,	  0,    0,
	/*PF-08~PF-15*/   113,       2500,        0,    125,      0,      0,     50,    0,
    #elif SERVO_PRODUCTNUM == UR_17
	/*PF-00~PF-07*/  EEPROMSer1,	0,		100,	100,   1335,	 48,	  0,	0,
	/*PF-08~PF-15*/   113,        2500,       0,    125,      0,      0,     50,    0,
    #elif SERVO_PRODUCTNUM == UR_20
	/*PF-00~PF-07*/  EEPROMSer1,	0,		100,	100,    1602,	 48,	  0,	0,
	/*PF-08~PF-15*/   113,        2500,       0,    125,      0,      0,      50,   0,
    #elif SERVO_PRODUCTNUM == UR_25
	/*PF-00~PF-07*/  EEPROMSer1,	0,		100,	100,   4000,	 48,	  0,    0,
	/*PF-08~PF-15*/   113,       2500,        0,    125,      0,      0,    100,    0,
    #elif SERVO_PRODUCTNUM == UR_32
	/*PF-00~PF-07*/  EEPROMSer1,	0,		100,	100,   8000,	 48,	  0,    0,
	/*PF-08~PF-15*/   113,       1600,        0,    125,      0,      0,    100,    0,
    #elif SERVO_PRODUCTNUM == Other1
    /*PF-00~PF-07*/ EEPROMSer1,	    0,		100,	100,	550,	 48,	  0,    0,
	/*PF-08~PF-15*/   386,       3000,        0,    125,     90,    100,    100,    0,
    #endif
#endif
	/*PF-16~PF-23*/ 	0,		100,	5,		10,		5,		5,		0,		0,
	/*PF-24~PF-31*/ 	1,		0,		0,		0,		0,		0,		0,		1,
	/*PF-32~PF-39*/ 	0,		0x0,   0x00,	0,		0,		0,		30,		10,
	/*PF-40~PF-47*/		0,		0,		1,		1,		50,		0,		0,		0,
	/*PF-48~PF-55*/		0,		0,		1,		0,		0,		0,		0,		0,
    #if SERVOPOWER == SERVO_AC
	/*PF-56~PF-63*/		10,	   10,		2,	 1600,	    0,		0,		50,	 0x12,
    #endif
    #if SERVOPOWER == SERVO_DC
	/*PF-56~PF-63*/		10,	   10,		2,	 1600,	    0,		0,		50,	 0x04,
    #endif
	},
		
	{
	/**************F76H组功能码的出厂值************************/
	/*S00.00~S00.07*/ 		0,		16,	    0,		  10,	0,		1,		17,		33,
	/*S00.08~S00.15*/ 		16,		19,		4,		   1,	0,		2,		0,		0
	},
};

TYPE_FuncCodeAttr  FuncAttr =
{
	{
	/**************P0组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*P0-00*/ 	{ATTR_Dot_0,	ATTR_Unit_Rpm,  	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	(Uint16)-MAXSPD },
	/*P0-01*/ 	{ATTR_Dot_1,   	ATTR_Unit_Percent,  ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	4000,	(Uint16)-4000 },
	/*P0-02*/	{ATTR_Dot_1, 	ATTR_Unit_EleAngle,	ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3599,			0 },
	/*P0-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0xff,			0 },
	/*P0-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x0f,			0 },

	/*P0-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	32767,	(Uint16)-32767 },
	/*P0-06*/	{ATTR_Dot_0, 	ATTR_Unit_Min,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*P0-06*/	{ATTR_Dot_0,	ATTR_Unit_Min,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	USIGN,	65535,			0 },
	/*P0-08*/	{ATTR_Dot_0, 	ATTR_Unit_mV,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	32767,	(Uint16)-32767 },
	/*P0-09*/	{ATTR_Dot_0, 	ATTR_Unit_mV,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	10000,	(Uint16)-10000 },

	/*P0-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	5,				0 },
	/*P0-11*/	{ATTR_Dot_0, 	ATTR_Unit_V,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			0 },
	/*P0-12*/	{ATTR_Dot_2, 	ATTR_Unit_A,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	   10,      	0 },
	/*P0-14*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	4000,			0 },

	/*P0-15*/	{ATTR_Dot_0, 	ATTR_Unit_C,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	200,			0 },
	/*P0-16*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*P0-16*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	65535,			0 },
	/*P0-18*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	0xFF,			0 },
	/*P0-19*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	(Uint16)-MAXSPD },
	
	/*P0-20*/	{ATTR_Dot_0, 	ATTR_Unit_V,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			0 },
	/*P0-21*/	{ATTR_Dot_2, 	ATTR_Unit_A,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-22*/	{ATTR_Dot_0, 	ATTR_Unit_Min,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-23*/	{ATTR_Dot_0, 	ATTR_Unit_Min,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-24*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },

	/*P0-24*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	USIGN,	65535,			0 },
	/*P0-26*/	{ATTR_Dot_2, 	ATTR_Unit_KgSqucm,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-27*/	{ATTR_Dot_2, 	ATTR_Unit_Percent,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-28*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*P0-28*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	65535,			0 },

	/*P0-30*/	{ATTR_Dot_2, 	ATTR_Unit_kHz,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*P0-30*/	{ATTR_Dot_2, 	ATTR_Unit_kHz,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	USIGN,	65535,			0 },
	/*P0-32*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*P0-33*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*P0-34*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },

	/*P0-35*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-36*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-36*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-38*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-38*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },

	/*P0-40*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P0-41*/ 	{ATTR_Dot_0,	ATTR_Unit_Rpm,  	ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	(Uint16)-MAXSPD },
    /*P0-42*/   {ATTR_Dot_0,    ATTR_Unit_Pulse,    ATTR_OPE_ReadOnly,  ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_32,    USIGN,  65535,          0 },
    /*P0-42*/   {ATTR_Dot_0,    ATTR_Unit_Pulse,    ATTR_OPE_ReadOnly,  ATTR_TYPE_Single,   HIDE,       ATTR_BIT_32,     SIGN,  65535,          0 },

	/*P0-44*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-45*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-46*/	{ATTR_Dot_1, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-47*/	{ATTR_Dot_2, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	
	/*P0-48*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-49*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-50*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
    /*P0-51*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	 SIGN,	65535,			0 },    

	/*P0-52*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
    /*P0-53*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	 SIGN,	65535,			0 },
    /*P0-54*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-55*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*P0-56*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-57*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-58*/	{ATTR_Dot_2, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
    /*P0-59*/	{ATTR_Dot_2, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	SIGN,	32767,	(Uint16)-32768 },
    
    /*P0-60*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	 SIGN,	32767,	(Uint16)-32768 },
    /*P0-61*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-62*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P0-63*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly, 	ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    },
	{
    /************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
    /*P1-00*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_RunOff,    ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  8,              0 },
    /*P1-01*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1,              0 },
    /*P1-02*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  5,              0 },
    /*P1-03*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  4,              0 },
    /*P1-04*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1,              0 },

    /*P1-05*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  5,              0 },
    /*P1-06*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  2,              0 },
    /*P1-07*/   {ATTR_Dot_0,    ATTR_Unit_Pulse,    ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  16383,         165},
    /*P1-08*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  3,              0 },
    /*P1-09*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  3,              0 },

    /*P1-10*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_RunOff,    ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1,              0 },
    /*P1-11*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P1-12*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  10000,          50 },
    /*P1-13*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  10000,          50 },
    /*P1-14*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1000,           0 },

    /*P1-15*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_RunOff,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  30000,          50 },
    /*P1-16*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_RunOff,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  30000,          50 },
    /*P1-17*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_RunOff,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  2000,           0 },
    /*P1-18*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_RunOff,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  30000,          50 },
    /*P1-19*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_RunOff,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  30000,          50 },

    /*P1-20*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  30000,         50 },
    /*P1-21*/   {ATTR_Dot_0,    ATTR_Unit_uS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P1-22*/   {ATTR_Dot_2,    ATTR_Unit_uS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P1-23*/   {ATTR_Dot_0,    ATTR_Unit_Percent,  ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,           0 },
    /*P1-24*/   {ATTR_Dot_0,    ATTR_Unit_Hz,       ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,           0 },

    /*P1-25*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,           0 },
    /*P1-26*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_32,    USIGN,  65535,          1 },
    /*P1-27*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_32,    USIGN,  65535,          0 },
    /*P1-28*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_32,    USIGN,  65535,          1 },
    /*P1-29*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_32,    USIGN,  65535,	        0 },

    /*P1-30*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1,              0 },
    /*P1-31*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          1 },
    /*P1-32*/   {ATTR_Dot_0,    ATTR_Unit_Pulse,    ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          1 },
    /*P1-33*/   {ATTR_Dot_0,    ATTR_Unit_Pulse,    ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P1-34*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  3,              0 },

    /*P1-35*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  2,              0 },
    /*P1-36*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P1-37*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P1-38*/   {ATTR_Dot_1,    ATTR_Unit_Percent,  ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1000,           0 },
    /*P1-39*/   {ATTR_Dot_0,    ATTR_Unit_Rpm,      ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  MAXSPD,         0 },

    /*P1-40*/   {ATTR_Dot_0,    ATTR_Unit_Rpm,      ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  100,            0 },
    /*P1-41*/   {ATTR_Dot_0,    ATTR_Unit_Rpm,      ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1000,           0 },
    /*P1-42*/   {ATTR_Dot_0,    ATTR_Unit_Rpm,      ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  MAXSPD,         10 },
    /*P1-43*/   {ATTR_Dot_0,    ATTR_Unit_Rpm,      ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  MAXSPD,         1 },
    /*P1-44*/   {ATTR_Dot_0,    ATTR_Unit_Rpm,      ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  300,            0 },

    /*P1-45*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  3,              0 },
    /*P1-46*/   {ATTR_Dot_1,    ATTR_Unit_Percent,  ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  MAXTRQ,         0 },
    /*P1-47*/   {ATTR_Dot_1,    ATTR_Unit_Percent,  ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  MAXTRQ,         0 },
    /*P1-48*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  3,              0 },
    /*P1-49*/   {ATTR_Dot_0,    ATTR_Unit_Hz,       ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },

    /*P1-50*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P1-51*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0},
    /*P1-52*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1,              0 },
    /*P1-53*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1,              0 },
    /*P1-54*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  500,            0 },

    /*P1-55*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  500,            0 },
    /*P1-56*/   {ATTR_Dot_0,    ATTR_Unit_Rpm,      ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          1 },
    /*P1-57*/   {ATTR_Dot_0,    ATTR_Unit_mS,       ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1000,           1 },
    /*P1-58*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1,              0 },
    /*P1-59*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Bit,      DISPLAY,    ATTR_BIT_16,    USIGN,  3,              0 },

    /*P1-60*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  3,              0 },
    /*P1-61*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P1-62*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_OPE_ReadOnly,  ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P1-63*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
    },
	{
	/**************P2组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*P2-00*/ 	{ATTR_Dot_0,   	ATTR_Unit_Pulse,  	ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-00*/ 	{ATTR_Dot_0,   	ATTR_Unit_Null,   	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-02*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-03*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-04*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },

	/*P2-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-06*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-07*/	{ATTR_Dot_0,	ATTR_Unit_mS,		ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-08*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	
	/*P2-10*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-11*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-12*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-14*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },

	/*P2-15*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-16*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-18*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-19*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	
	/*P2-20*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-20*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-22*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-23*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-24*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },

	/*P2-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-26*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-27*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-28*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },

	/*P2-30*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-31*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-32*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-32*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-34*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },

	/*P2-35*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-36*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-36*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-38*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-39*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },

	/*P2-40*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-40*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-42*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-43*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-44*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },

	/*P2-44*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-46*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-47*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-48*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-48*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },

	/*P2-50*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-51*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-52*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-52*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-54*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },

	/*P2-55*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*P2-56*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-56*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-58*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-59*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },

	/*P2-60*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xffff,		0x0000 },
	/*P2-60*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7fff,		0x8000 },
	/*P2-62*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P2-63*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	},
	{
	/**************P3组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*P3-00*/ 	{ATTR_Dot_0,   	ATTR_Unit_Rpm,  	ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-01*/ 	{ATTR_Dot_1,   	ATTR_Unit_Sec,   	ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-02*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-03*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-04*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },

	/*P3-05*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-06*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-07*/	{ATTR_Dot_1,	ATTR_Unit_Sec,		ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-08*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-09*/	{ATTR_Dot_1, 	ATTR_Unit_Sec, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },

	/*P3-10*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-11*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-12*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-13*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-14*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },

	/*P3-15*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-16*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-17*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-18*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-19*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	
	/*P3-20*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-21*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-22*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-23*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-24*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },

	/*P3-25*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-26*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-27*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P3-28*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-29*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },

	/*P3-30*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXSPD,	 (Uint16)-MAXSPD },
	/*P3-31*/	{ATTR_Dot_1, 	ATTR_Unit_Sec,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	},
	{
	/**************P4组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*P4-00*/ 	{ATTR_Dot_1,   	ATTR_Unit_Percent,  ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	MAXTRQ,	(Uint16)-MAXTRQ },
	/*P4-01*/ 	{ATTR_Dot_0,   	ATTR_Unit_Rpm,   	ATTR_EFT_Now,	 	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P4-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	3,				0 },
	/*P4-03*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P4-04*/	{ATTR_Dot_1, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	 SIGN,	3000,	 (Uint16)-3000},

	/*P4-05*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			0 },
	/*P4-06*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	 SIGN,	3000,	 (Uint16)-3000},
	/*P4-07*/	{ATTR_Dot_0,	ATTR_Unit_Null,		ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,				0 },
	/*P4-08*/	{ATTR_Dot_1, 	ATTR_Unit_mS, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	 SIGN,	3000,	 (Uint16)-3000},
	/*P4-09*/	{ATTR_Dot_0, 	ATTR_Unit_mS, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			0 },    
    /*P4-10*/	{ATTR_Dot_1, 	ATTR_Unit_mS, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	 SIGN,	3000,	 (Uint16)-3000},
	/*P4-11*/	{ATTR_Dot_0, 	ATTR_Unit_mS, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			0 },    
	/*P4-12*/	{ATTR_Dot_0, 	ATTR_Unit_mS, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	10000,			0 },    
	/*P4-13*/	{ATTR_Dot_0, 	ATTR_Unit_mS, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	10000,			0 },    
	/*P4-14*/	{ATTR_Dot_0, 	ATTR_Unit_mS, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	10000,			0 },    
	/*P4-15*/	{ATTR_Dot_0, 	ATTR_Unit_mS, 		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	10000,			0 },
    
    /*P4-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-17*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-18*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-19*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*P4-20*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },

    /*P4-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },

    /*P4-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P4-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    },
	{
	/**************P5组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*P5-00*/ 	{ATTR_Dot_1,   	ATTR_Unit_RadOrs,  	ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	20000,			10 },
	/*P5-01*/ 	{ATTR_Dot_1,   	ATTR_Unit_RadOrs, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	50000,			 1 },
	/*P5-02*/	{ATTR_Dot_1, 	ATTR_Unit_mS,	    ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			 1 },
	/*P5-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	2,			     0 },
	/*P5-04*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			 0 },

	/*P5-05*/	{ATTR_Dot_1, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			 0 },
	/*P5-06*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			 0 },
	/*P5-07*/	{ATTR_Dot_1,	ATTR_Unit_mS,		ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			 0 },
	/*P5-08*/	{ATTR_Dot_1, 	ATTR_Unit_mS, 	    ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,		     0 },
	/*P5-09*/	{ATTR_Dot_1, 	ATTR_Unit_mS, 	    ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1280,			 0 },
	
	/*P5-10*/	{ATTR_Dot_1, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	10000,			 0 },
	/*P5-11*/	{ATTR_Dot_2, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	10000,			 0 },
	/*P5-12*/	{ATTR_Dot_2, 	ATTR_Unit_mS,	    ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	2000,			 0 },
	/*P5-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	4,			     0 },
	/*P5-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	31,		   	     1 },

	/*P5-15*/	{ATTR_Dot_2, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	12000,		   100 },
	/*P5-16*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	500,			10 },
	/*P5-17*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	500,			10 },
	/*P5-18*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,	    ATTR_TYPE_Hex, 	    DISPLAY,	ATTR_BIT_16,	USIGN,	0xFF,			 0 },
	/*P5-19*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,			 0 },

	/*P5-20*/	{ATTR_Dot_0, 	ATTR_Unit_mS,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,	         0 },
	/*P5-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	32767,		   	 0 },
	/*P5-22*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			 0 },
	/*P5-23*/	{ATTR_Dot_0,   	ATTR_Unit_Null,     ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN, 4000,			 0 },
	/*P5-24*/	{ATTR_Dot_1,   	ATTR_Unit_Null,     ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN, 1000,		     0 },

	/*P5-25*/	{ATTR_Dot_0,   	ATTR_Unit_Percent,  ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,	          0 },
	/*P5-26*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,  ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	 SIGN,	1000, (Uint16)-1000 },
	/*P5-27*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			  0 },
	/*P5-28*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,	   		  0 },
	/*P5-29*/	{ATTR_Dot_0, 	ATTR_Unit_mS,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			  0 },
	/*P5-30*/	{ATTR_Dot_2, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	12000,			  0 },
	/*P5-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null,     ATTR_EFT_Now,		ATTR_TYPE_Bit,   	DISPLAY,	ATTR_BIT_16,	USIGN,	0x3F,			  0 },
	/*P5-32*/	{ATTR_Dot_0, 	ATTR_Unit_Hz,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			  0 },
	/*P5-33*/	{ATTR_Dot_2, 	ATTR_Unit_mS,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			  0 },
    /*P5-34*/	{ATTR_Dot_2, 	ATTR_Unit_mS,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			  0 },	
    /*P5-35*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			  0 },
    /*P5-36*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,				0 },
    /*P5-37*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,				0 },
	/*P5-38*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,				0 },
    /*P5-39*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,				0 },
    
    /*P5-40*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P5-41*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P5-42*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P5-43*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*P5-44*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P5-45*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P5-46*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P5-47*/	{ATTR_Dot_0, 	ATTR_Unit_Null,	    ATTR_EFT_Now,		ATTR_TYPE_Single, 	   HIDE,	ATTR_BIT_16,	USIGN,	65535,			0 },
    },
	{
	/**************P6组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*P6-00*/ 	{ATTR_Dot_0,   	ATTR_Unit_mS,  		ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	20,				0 },
	/*P6-01*/ 	{ATTR_Dot_0,   	ATTR_Unit_Null,   	ATTR_EFT_Now, 		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0xff,			0 },
	/*P6-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDIFUNC,		0 },
	/*P6-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDIFUNC,		0 },
	/*P6-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDIFUNC,		0 },

	/*P6-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDIFUNC,		0 },
	/*P6-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDIFUNC,		0 },
	/*P6-07*/	{ATTR_Dot_0,	ATTR_Unit_Null,		ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDIFUNC,		0 },
	/*P6-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDIFUNC,		0 },
	/*P6-09*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDIFUNC,		0 },

	/*P6-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P6-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P6-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x0f,			0 },
	/*P6-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDOFUNC,		0 },
	/*P6-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDOFUNC,		0 },
	
	/*P6-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDOFUNC,		0 },
	/*P6-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXDOFUNC,		0 },	
    /*P6-17*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },
    /*P6-18*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },    
    /*P6-19*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },
    /*P6-20*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },
    /*P6-21*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },

    /*P6-22*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },
    /*P6-23*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },
    /*P6-24*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },
    /*P6-25*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },	 
    /*P6-26*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },
    /*P6-27*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  MAXAOFUNC,      0 },
    
    /*P6-28*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P6-29*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P6-30*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P6-31*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  65535,          0 },
    
    /*P6-32*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P6-33*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P6-34*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*P6-35*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,      HIDE,    ATTR_BIT_16,    USIGN,  65535,          0 },
    },
	{
	/**************P7组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*P7-00*/ 	{ATTR_Dot_0,   	ATTR_Unit_Null,  	ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	254,			1 },
	/*P7-01*/ 	{ATTR_Dot_0,   	ATTR_Unit_Null,   	ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	6,	 		    0 },
	/*P7-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	5,				0 },
	/*P7-03*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	20,				1 },
	/*P7-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	20,			    0 },

	/*P7-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P7-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P7-07*/	{ATTR_Dot_0,	ATTR_Unit_Null,		ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	2,			    0 },
	/*P7-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			1 },
	/*P7-09*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,			    0 },
	
	/*P7-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0},
	/*P7-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P7-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P7-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P7-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },

	/*P7-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P7-16*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  127,            1 },
	/*P7-17*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  1,              0 },
	/*P7-18*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  255,            4 },
	/*P7-19*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_Now,       ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  4,              0 },

	/*P7-20*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,              0 },
	/*P7-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,          0 },
	/*P7-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P7-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },

	/*P7-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	255,			1 },
	/*P7-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	  5,			0 },
	/*P7-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	  5,			0 },	
	/*P7-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },     
    
    /*P7-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P7-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*P7-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },	
	/*P7-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
    },
	{
	/**************P8组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*P8-00*/ 	{ATTR_Dot_0,   	ATTR_Unit_Null,  	ATTR_EFT_Now,  		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*P8-01*/ 	{ATTR_Dot_0,   	ATTR_Unit_Null,   	ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	10,				0 },
	/*P8-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	2,				0 },
	/*P8-03*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,	 		0 },
	/*P8-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },

	/*P8-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*P8-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,  			0 },
	/*P8-07*/	{ATTR_Dot_0,	ATTR_Unit_Null,		ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXP0,			0 },
	/*P8-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,       ATTR_BIT_16,	SIGN,	1,			    -1 },
	/*P8-09*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	1,		     	0 },
	
	/*P8-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	255,			0 },
	/*P8-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*P8-12*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,		   20 },
	/*P8-13*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,		   20 },
	/*P8-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },

	/*P8-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	3,				0 },
	/*P8-16*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single, 	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,	 		1 },
	/*P8-17*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*P8-18*/	{ATTR_Dot_0, 	ATTR_Unit_Ohm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,		   20 },
	/*P8-19*/	{ATTR_Dot_0, 	ATTR_Unit_W,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,		   10 },
	
	/*P8-20*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			0 },
	/*P8-21*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			1 },
	/*P8-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*P8-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*P8-24*/	{ATTR_Dot_0, 	ATTR_Unit_mS, 	    ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	30000,	    	1 },
	/*P8-25*/	{ATTR_Dot_0, 	ATTR_Unit_mS, 	    ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,	    	1 },
	/*P8-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,     ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	20,	    	    1 },
	/*P8-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null,     ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,	    		0 },
	/*P8-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_PowerOn,   ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	2,	        	0 },
	/*P8-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,	    	    0 },

	/*P8-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null,     ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	256,	    	0 },
	/*P8-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,	    	0 },
	/*P8-32*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,	        0 },
	/*P8-33*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,	    	0 },
	/*P8-34*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,	        	0 },

    /*P8-35*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,	    	0 },    
    /*P8-36*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,	    	0 },    
    /*P8-37*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,	    	    0 },    
    /*P8-38*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Bit,	    DISPLAY,	ATTR_BIT_16,	USIGN,	65535,	    	0 },    
    
    /*P8-39*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,	    	0 },    
    /*P8-40*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,	    	0 },    
    /*P8-41*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	125,	    	0 },
    /*P8-42*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	125,	    	0 },
    /*P8-43*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	125,	        0 },
	
    /*P8-44*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	125,	    	0 },    
    /*P8-45*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,	        0 },
    /*P8-46*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,	    	0 },
    /*P8-47*/	{ATTR_Dot_0, 	ATTR_Unit_Null, 	ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	SIGN,	0x7fff,	   0x8000 },
    },
	{
	/**************P9组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*P9-00*/	{ATTR_Dot_1, 	ATTR_Unit_Hz,		ATTR_EFT_RunOff,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	 1000,			0 },
    /*P9-01*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_RunOff,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	  100,			0 },
    /*P9-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-07*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*P9-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,    	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-09*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-12*/	{ATTR_Dot_0, 	ATTR_Unit_Hz,		ATTR_EFT_RunOff,    ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	5000,			0 },
    /*P9-13*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_RunOff,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			0 },
    /*P9-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_RunOff,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			0 },
    /*P9-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*P9-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-17*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-18*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-19*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-20*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*P9-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*P9-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    },

	{
	/**************PA组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
    /*PA-00*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	   10,			0 },
	/*PA-01*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	    4,			0 },
	/*PA-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	    1,			0 },
	/*PA-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	    1,			0 },
	
    /*PA-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-07*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	
    /*PA-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-09*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	
    /*PA-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	 SIGN,	0x7fff,	   0x8000 },
	/*PA-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	 SIGN,	0x7fff,	   0x8000 },
	
    /*PA-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-17*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-18*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-19*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	
    /*PA-20*/	{ATTR_Dot_1, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,			0 },
	/*PA-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	
    /*PA-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			1 },
	/*PA-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	  2,			0 },
	/*PA-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	
    /*PA-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    #endif
    
    #if SERVOTYPE == SERVO_MODBUS
    /*PA-00*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	   10,			0 },
	/*PA-01*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	    4,			0 },
	/*PA-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	    1,			0 },
	/*PA-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	    1,			0 },
	
    /*PA-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-07*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	
    /*PA-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-09*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	
    /*PA-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	 SIGN,	0x7fff,	   0x8000 },
	/*PA-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	 SIGN,	0x7fff,	   0x8000 },
	
    /*PA-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-17*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-18*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-19*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	
    /*PA-20*/	{ATTR_Dot_1, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,			0 },
	/*PA-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	
    /*PA-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			1 },
	/*PA-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	  2,			0 },
	/*PA-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	
    /*PA-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },    
    #endif
    
    #if SERVOTYPE == SERVO_CAN
    /*PA-00*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	   10,			0 },
	/*PA-01*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	    4,			0 },
	/*PA-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	    1,			0 },
	/*PA-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	    1,			0 },
	
    /*PA-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,		   50 },
	/*PA-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	0,			    0 },
	/*PA-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,		   50 },
	/*PA-07*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	0,			    0 },
	
    /*PA-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-09*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	32767,	        0 },
	/*PA-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	0,		        0 },
	
    /*PA-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	 SIGN,	0x7fff,	   0x8000 },
	/*PA-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	 SIGN,	0x7fff,	   0x8000 },
	
    /*PA-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-17*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PA-18*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,	        0 },
	/*PA-19*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	0,		        0 },
	
    /*PA-20*/	{ATTR_Dot_1, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,			0 },
	/*PA-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,		   50 },
	
    /*PA-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	 12,			4 },
	/*PA-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	  2,			0 },
	/*PA-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	
    /*PA-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PA-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },    
    #endif
    },

	{
	/**************PB组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*PB-00*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PB-01*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	35,				1 },
	/*PB-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	2,				0 },
	/*PB-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	5,				0 },
	
    /*PB-04*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,			0 },
	/*PB-05*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,			0 },
	/*PB-06*/	{ATTR_Dot_0, 	ATTR_Unit_Pulse,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	0xFFFF,	   0x0000 },
	/*PB-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_32,	SIGN,	0x7FFF,	   0x8000 },
	
    /*PB-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*PB-09*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			1 },
	/*PB-10*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			1 },
    /*PB-11*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	30000,			1 },
	
    /*PB-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*PB-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-17*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-18*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-19*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*PB-20*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*PB-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
   
    /*PB-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PB-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    },
	{
	/**************PC组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*PC-00*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	   2,			0 },
	/*PC-01*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PC-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PC-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    
    /*PC-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PC-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-07*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    
    /*PC-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PC-09*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 }, 
    
    /*PC-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PC-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 }, 
    
    /*PC-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PC-17*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-18*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-19*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 }, 
    
    /*PC-20*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PC-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 }, 
    
    /*PC-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PC-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 }, 
    
    /*PC-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PC-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 },    
    /*PC-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			0 }, 
    },
	{
	/**************PD组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*PD-00*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PD-01*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PD-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PD-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	
	/*PD-04*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,			0 },
    /*PD-05*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	   HIDE,	ATTR_BIT_32,	USIGN,	65535,			0 },
    /*PD-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PD-07*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	  2,			0 },	    
    
    /*PD-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	  6,		    0 },
    /*PD-09*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PD-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PD-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },	    
    
    /*PD-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PD-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PD-14*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    /*PD-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
    },
	{
	/**************PE组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*PE-00*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PE-01*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PE-02*/	{ATTR_Dot_2, 	ATTR_Unit_kW,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			1 },
	/*PE-03*/	{ATTR_Dot_2, 	ATTR_Unit_A,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			1 },
	/*PE-04*/	{ATTR_Dot_2, 	ATTR_Unit_Nm,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			1 },

	/*PE-05*/	{ATTR_Dot_0, 	ATTR_Unit_V,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,			1 },
	/*PE-06*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,		   10 },
	/*PE-07*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXSPD,		   10 },
	/*PE-08*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	360,			1 },
	/*PE-09*/	{ATTR_Dot_3, 	ATTR_Unit_mH,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },

	/*PE-10*/	{ATTR_Dot_3, 	ATTR_Unit_mH,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0                                                                                                                                                                                                                },
	/*PE-11*/	{ATTR_Dot_3, 	ATTR_Unit_Ohm,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PE-12*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	4000,			0 },
	/*PE-13*/	{ATTR_Dot_2, 	ATTR_Unit_V,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PE-14*/	{ATTR_Dot_2, 	ATTR_Unit_KgSqucm,	ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			1 },

	/*PE-15*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	13,				0 },
	/*PE-16*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_32,	USIGN,	65535,		    0 },
	/*PE-17*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_32,	USIGN,	65535,			0 },
	/*PE-18*/	{ATTR_Dot_1, 	ATTR_Unit_EleAngle,	ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3599,			0 },
	/*PE-19*/	{ATTR_Dot_1, 	ATTR_Unit_EleAngle,	ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3599,			0 },

	/*PE-20*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	129,			0 },
	/*PE-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*PE-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*PE-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,		   10 },
	/*PE-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,		   10 },

	/*PE-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,		   10 },
	/*PE-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,		   10 },
	/*PE-27*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	2000,		   0  },
	/*PE-28*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	2000,		   0  },
	/*PE-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PE-30*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
	/*PE-31*/   {ATTR_Dot_3,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  3000,           0 },
	/*PE-32*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
	/*PE-33*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-34*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   DISPLAY,    ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-35*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
	
    /*PE-36*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-37*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-38*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-39*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-40*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-41*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-42*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-43*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },    
    
    /*PE-44*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-45*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-46*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },
    /*PE-47*/   {ATTR_Dot_0,    ATTR_Unit_Null,     ATTR_EFT_PowerOn,   ATTR_TYPE_Single,   HIDE,       ATTR_BIT_16,    USIGN,  65535,          0 },    
    },
	{
	/**************PF组功能码的出厂值************************/
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*PF-00*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-01*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-02*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	200,		   40 },
	/*PF-03*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	200,		   50 },
	/*PF-04*/	{ATTR_Dot_2, 	ATTR_Unit_A,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			1 },

	/*PF-05*/	{ATTR_Dot_0, 	ATTR_Unit_V,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	30000,			0 },
	/*PF-06*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			0 },
	/*PF-07*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-08*/	{ATTR_Dot_2, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1000,		  100 },
	/*PF-09*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	MAXTRQ,			0 },

	/*PF-10*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,	    DISPLAY,	ATTR_BIT_16,	USIGN,	0x0003,			0 },
	/*PF-11*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-12*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	120,			0 },
	/*PF-13*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	120,			0 },
	/*PF-14*/	{ATTR_Dot_2, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	300,		    5 },

	/*PF-15*/	{ATTR_Dot_1, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	10,			    0 },
	/*PF-16*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,  ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	10000,			0 },
	/*PF-17*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	200,		   50 },
	/*PF-18*/	{ATTR_Dot_2, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			1 },
	/*PF-19*/	{ATTR_Dot_2, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			1 },

	/*PF-20*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-21*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-22*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,	    DISPLAY,    ATTR_BIT_16,	USIGN,	0x8f,			0 },
	/*PF-23*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,	    DISPLAY,	ATTR_BIT_16,	USIGN,	0x03,			0 },
	/*PF-24*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,			    0 },

	/*PF-25*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	200, 			1 },
	/*PF-26*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-27*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x00FF,			0 },
	/*PF-28*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x00FF,			0 },
	/*PF-29*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x00FF,			0 },

	/*PF-30*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x00FF,			0 },
	/*PF-31*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-32*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x00FF,			0 },
	/*PF-33*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x00FF,			0 },
	/*PF-34*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x00FF,			0 },

	/*PF-35*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Bit,		DISPLAY,	ATTR_BIT_16,	USIGN,	0x00FF,			0 },
	/*PF-36*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-37*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-38*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	200,			0 },
	/*PF-39*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,			0 },

	/*PF-40*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-41*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-42*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*PF-43*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,    ATTR_BIT_16,	USIGN,	1,			0     },
	/*PF-44*/	{ATTR_Dot_0, 	ATTR_Unit_mS,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	500,		   50 },

	/*PF-45*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*PF-46*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-47*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 }, 
	/*PF-48*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,		    0 },
	/*PF-49*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,		    0 },

	/*PF-50*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },
	/*PF-51*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*PF-52*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	16,				1 },
	/*PF-53*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*PF-54*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	HIDE,		ATTR_BIT_16,	USIGN,	65535,			0 },

	/*PF-55*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },
	/*PF-56*/	{ATTR_Dot_0, 	ATTR_Unit_Percent,	ATTR_EFT_RunOff,    ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	110,			0 },
	/*PF-57*/	{ATTR_Dot_0, 	ATTR_Unit_Hz,		ATTR_EFT_RunOff,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,			0 },
	/*PF-58*/	{ATTR_Dot_0, 	ATTR_Unit_Sec,		ATTR_EFT_RunOff,    ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	100,		    1 },
	/*PF-59*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3,		   		0 },

	/*PF-60*/	{ATTR_Dot_1, 	ATTR_Unit_Percent,	ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	SIGN,	1000,		 (Uint16)-1000 },
	/*PF-61*/	{ATTR_Dot_0, 	ATTR_Unit_Rpm,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	2,		    	0 },
	/*PF-62*/	{ATTR_Dot_0,   	ATTR_Unit_Percent, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	200,			40},
	/*PF-63*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_PowerOn,	ATTR_TYPE_Hex,	    DISPLAY,	ATTR_BIT_16,	USIGN,	0x1A,		   	2 },
    
    /*PF-64*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-65*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-66*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-67*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-68*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-69*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-70*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-71*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-72*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-73*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-74*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-75*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-76*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-77*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-78*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-79*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-80*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-81*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-82*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-83*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-84*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-85*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-86*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-87*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-88*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},
    /*PF-89*/	{ATTR_Dot_0,   	ATTR_Unit_Null, 	ATTR_EFT_Now, 		ATTR_TYPE_Single,	HIDE,	    ATTR_BIT_16,	USIGN,	65535,			 0},   
    },	

	{\
	/**************7600组功能码的出厂值************************/\
	/************小数点*************单位******************操作属性***********数据类型*********参数显示******数据长度*****数据符号**最大值********最小值**/
	/*0x7600*/ 	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	4,				0 },\
	/*0x7601*/ 	{ATTR_Dot_3, 	ATTR_Unit_kHz,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	16000,			10 },\
	/*0x7602*/ 	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	6,				1 },\
	/*0x7603*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	6000,			500 },\
	/*0x7604*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	47,		0 },\
	\
	/*0x7605*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	47,		0 },\
	/*0x7606*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	47,		0 },\
	/*0x7607*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	47,		0 },\
	/*0x7608*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	47,		0 },\
	/*0x7609*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	47,		0 },\
	\
	/*0x760A*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	47,		0 },\
	/*0x760B*/	{ATTR_Dot_3, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3000,			0 },\
	/*0x760C*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },\
	/*0x760D*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_EFT_Now,		ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	3,				0 },\
	/*0x760E*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	1,				0 },\
	\
	/*0x760F*/	{ATTR_Dot_0, 	ATTR_Unit_Null,		ATTR_OPE_ReadOnly,	ATTR_TYPE_Single,	DISPLAY,	ATTR_BIT_16,	USIGN,	65535,			0 },\
	},
};
/******************************************************************************
**函 数 名：Uint16 GetFuncAddr(Uint16 Functioncode)
**描    述：根据功能代码，取得该功能码所在结构体的相对地址
** 调用  
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
Uint16 GetFuncAddr(Uint16 Functioncode)
{
	TYPE_WORD Func;
	Func.all= Functioncode;
	Uint16 size =0;
	switch(Func.Byte.High)//选择是哪个数组
	{
		case F76H_FUN_GROUP: size+=PF_FUN_GROUP_SIZE;

		case PF_FUN_GROUP:	size+=PE_FUN_GROUP_SIZE;
		case PE_FUN_GROUP:	size+=PD_FUN_GROUP_SIZE;
		case PD_FUN_GROUP:	size+=PC_FUN_GROUP_SIZE;
		case PC_FUN_GROUP:	size+=PB_FUN_GROUP_SIZE;
		case PB_FUN_GROUP:	size+=PA_FUN_GROUP_SIZE;

		case PA_FUN_GROUP:	size+=P9_FUN_GROUP_SIZE;
		case P9_FUN_GROUP:	size+=P8_FUN_GROUP_SIZE;
		case P8_FUN_GROUP:	size+=P7_FUN_GROUP_SIZE;
		case P7_FUN_GROUP:	size+=P6_FUN_GROUP_SIZE;
		case P6_FUN_GROUP:	size+=P5_FUN_GROUP_SIZE;
		
		case P5_FUN_GROUP:	size+=P4_FUN_GROUP_SIZE;
		case P4_FUN_GROUP:	size+=P3_FUN_GROUP_SIZE;
		case P3_FUN_GROUP:	size+=P2_FUN_GROUP_SIZE;
		case P2_FUN_GROUP:	size+=P1_FUN_GROUP_SIZE;
		case P1_FUN_GROUP:	size+=P0_FUN_GROUP_SIZE;
		
		case P0_FUN_GROUP:	size+=Func.Byte.Low;//计算当前数第多少个功能码
		break;
		default :size =sizeof(TYPE_FuncCodeVal);
		break;
	}
	return size;
}
/******************************************************************************
**函 数 名：Uint16 GetVarAddr(Uint16 *Var,Uint16 Area)
**描    述：根据变量查找数据
** 调用  
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
Uint16 GetVarAddr(Uint16 *Var)
{
	return (Uint16)(Var - RamFuncStartAddr);
}

