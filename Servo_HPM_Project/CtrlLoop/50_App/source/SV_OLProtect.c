/*
 * SV_OLProtect.c
 *
 *  Created on: 2015-11-27
 *      Author: zhangwei//
 */

//外部引用-------------------------------------------------------------//
#include "userdefine.h"
#include "SV_OLProtect.h"
#include "Common_Lib.h"
#include "SV_FuncVar.h"
#include "SV_FaultProtect.h"
#include "SV_UdcCtrl.h"
#include "SV_Servocode.h"
#include "Drive.h"

#define OL_DRVOLTAB_MAXA	32//

static Uint16 ServoLockOVLoadTable[36];
static Uint16 ServoOVLoadTable[36];
static Uint16 MotorOVLoadTable[36];

//过载能力定义-----------------------------------------------------------//
// 驱动器过载反时限曲线查询表:	注：每两个点之间相差6.4%
#define OL_DRVOLTAB_MAX	36

const Uint16 OL_DRVOLTAB[OL_DRVOLTAB_MAX+1][36] = {\
/*		      102.2%,  108.6%, 115.0%, 121.4%, 127.8%, 134.2%, 140.6%, 147.0%, 153.4%, 159.8%, 166.2%,  172.6%,  179.0%, 185.4%, 191.8%, 198.2%, 204.6%, 211.0%, 217.4%, 223.8%, 230.2%, 236.6%, 243.0%,  249.4%,  255.8%, 262.2%, 268.6%, 275.0%,  281.4%, 287.8%, 294.2%,  300.6%,  307.0%,  313.4%, 319.8%, 326.2%*/ \
/*0*/	      {60000,	3000,   1000,	 650,	 460,    360,    303,    269,    241,    216,    196,    178,     162,    148,    136,    125,    115,    107,     99,     91,     85,     80,     74,      69,      65,     61,     57,     53,      50,     48,     44,      42,      39,      37,     30,    25},\
/*1*/		  {0},
/*2*/		  {0},
/*3*/		  {0},
/*4*/		  {0},
/*5*/		  {0},
/*6*/		  {0},
/*7*/		  {0},
/*8*/		  {0},
/*9*/		  {0},
/*10*/		  {0},
/*11_TRM_11*/ {60000,  24000,  18000,  14000,  10000,   7600,   5000,   3000,   1500,   1200,   1020,    900,     780,    660,    580,    500,    420,    360,    300,    280,    260,    240,    220,     200,     180,    160,    120,     80,      60,     48,     30,      35,      35,      35,     35,    35},\
/*12_TRM_14*/ {60000,  24000,  18000,  14000,  10000,   7600,   5000,   3000,   1500,   1200,   1020,    900,     780,    660,    580,    500,    420,    360,    300,    280,    260,    240,    220,     200,     180,    160,    120,     80,      60,     48,     30,      35,      35,      35,     35,    35},\
/*13_TRM_17*/ {60000,  24000,  18000,  14000,  10000,   7600,   5000,   3000,   1500,   1200,   1020,    900,     780,    660,    580,    500,    420,    360,    300,    280,    260,    240,    220,     200,     180,    160,    120,     80,      60,     48,     30,      35,      35,      35,     35,    35},\
/*14_TRM_20*/ {60000,  24000,  18000,  14000,  10000,   7600,   5000,   3000,   1500,   1200,   1020,    900,     780,    660,    580,    500,    420,    360,    300,    280,    260,    240,    220,     200,     180,    160,    120,     80,      60,     48,     30,      35,      35,      35,     35,    35},\
/*15_TRM_25*/ {15000,  11000,   9000,   7000,   5500,   4000,   3000,   2200,   1400,   1100,    800,    640,     520,    440,    360,    320,    260,    200,    150,    120,     90,     80,     70,      60,      30,     30,     30,     30,      30,     30,     30,      30,      30,      30,     30,    30},\
/*16_TRM_32*/ {1200,     700,    560,    400,    320,    250,    190,    140,    100,     60,     32,     31,      31,     31,     31,     31,     31,     31,     31,     31,     31,     31,     31,      31,      31,     31,     31,     31,      31,     31,     31,      31,      31,      31,     31,    31},\
/*17_TRM_40*/ {1200,     700,    560,    400,    320,    250,    190,    140,    100,     60,     32,     31,      31,     31,     31,     31,     31,     31,     31,     31,     31,     31,     31,      31,      31,     31,     31,     31,      31,     31,     31,      31,      31,      31,     31,    31},\
/*18_TCM_11*/ {60000,  24000,  18000,  14000,  10000,   7600,   5000,   3000,   1500,   1200,   1020,    900,     780,    660,    580,    500,    420,    360,    300,    280,    260,    240,    220,     200,     180,    160,    120,     80,      60,     48,     30,      35,      35,      35,     35,    35},\
/*19_TCM_14*/ {60000,  24000,  18000,  14000,  10000,   7600,   5000,   3000,   1500,   1200,   1020,    900,     780,    660,    580,    500,    420,    360,    300,    280,    260,    240,    220,     200,     180,    160,    120,     80,      60,     48,     30,      35,      35,      35,     35,    35},\
/*20_TCM_17*/ {60000,  24000,  18000,  14000,  10000,   7600,   5000,   3000,   1500,   1200,   1020,    900,     780,    660,    580,    500,    420,    360,    300,    280,    260,    240,    220,     200,     180,    160,    120,     80,      60,     48,     30,      35,      35,      35,     35,    35},\
/*21_TCM_20*/ {60000,  24000,  18000,  14000,  10000,   7600,   5000,   3000,   1500,   1200,   1020,    900,     780,    660,    580,    500,    420,    360,    300,    280,    260,    240,    220,     200,     180,    160,    120,     80,      60,     48,     30,      35,      35,      35,     35,    35},\
/*22_TCM_25*/ {15000,  11000,   9000,   7000,   5500,   4000,   3000,   2200,   1400,   1100,    800,    640,     520,    440,    360,    320,    260,    200,    150,    120,     90,     80,     70,      60,      30,     30,     30,     30,      30,     30,     30,      30,      30,      30,     30,    30},\
/*23_TCM_32*/ {1200,     700,    560,    400,    320,    250,    190,    140,    100,     60,     32,     31,      31,     31,     31,     31,     31,     31,     31,     31,     31,     31,     31,      31,      31,     31,     31,     31,      31,     31,     31,      31,      31,      31,     31,    31},\
/*24_TCM_40*/ {1200,     700,    560,    400,    320,    250,    190,    140,    100,     60,     32,     31,      31,     31,     31,     31,     31,     31,     31,     31,     31,     31,     31,      31,      31,     31,     31,     31,      31,     31,     31,      31,      31,      31,     31,    31},\
/*25_TCHL_11*/{1200,     700,    560,    400,    320,    250,    190,    140,    100,     60,     32,     31,      31,     31,     31,     31,     31,     31,     31,     31,     31,     31,     31,      31,      31,     31,     31,     31,      31,     31,     31,      31,      31,      31,     31,    31},\
/*26_TCHL_14*/{60000,	3000,   1000,	 650,	 460,    360,    303,    269,    241,    216,    196,    178,     162,    148,    136,    125,    115,    107,     99,     91,     85,     80,     74,      69,      65,     61,     57,     53,      50,     48,     44,      42,      39,      37,     30,    25},\
/*27_TCHL_17*/{60000,	3000,   1000,	 650,	 460,    360,    303,    269,    241,    216,    196,    178,     162,    148,    136,    125,    115,    107,     99,     91,     85,     80,     74,      69,      65,     61,     57,     53,      50,     48,     44,      42,      39,      37,     30,    25},\
/*28_TCHL_20*/{60000,	3000,   1000,	 650,	 460,    360,    303,    269,    241,    216,    196,    178,     162,    148,    136,    125,    115,    107,     99,     91,     85,     80,     74,      69,      65,     61,     57,     53,      50,     48,     44,      42,      39,      37,     30,    25},\
/*29_TCHL_25*/{60000,	3000,   1000,	 650,	 460,    360,    303,    269,    241,    216,    196,    178,     162,    148,    136,    125,    115,    107,     99,     91,     85,     80,     74,      69,      65,     61,     57,     53,      50,     48,     44,      42,      39,      37,     30,    25},\
/*30_TCHL_32*/{60000,	3000,   1000,	 650,	 460,    360,    303,    269,    241,    216,    196,    178,     162,    148,    136,    125,    115,    107,     99,     91,     85,     80,     74,      69,      65,     61,     57,     53,      50,     48,     44,      42,      39,      37,     30,    25},\
};

//电机堵转过载反时限曲线查询表:	注：每两个点之间相差6.4%
const Uint16 OL_LOCKDRVOLTAB[OL_DRVOLTAB_MAX+1][36] ={ \
//          102.2%   108.6%    115%     121.4%     127.8%    134.2%    140.6%     147%   153.4%    159.8%    166.2%    172.6%   179%   185.4%   191.8%    198.4%    204.6%    211%     217.4%    223.8%   230.2%    236.6%    243%    249.4   255.8%  262.2%   268.6%    275%,    281.4%,   287.8%     294.2%    300.6%
//            0         1        2        3          4         5         6         7       8         9         10       11       12      13       14       15         16       17       18        19       20         21       22       23      24     25        26       27       28         29         30        31     32
/*TCM_XX*/   {1200,    700,      560,     400,      320,      250,      190,      140,    100,       60,       32,      31,      31,     31,      31,      31,        31,      31,      31,       31,      31,        31,      31,      31,     31,    31,       31,      31,      31,        31,        31,       31,    31}, 
/*1*/		 {0},
/*2*/		 {0},
/*3*/		 {0},
/*4*/		 {0},
/*5*/		 {0},
/*6*/		 {0},
/*7*/		 {0},
/*8*/		 {0},
/*9*/		 {0},
/*10*/		 {0},
/*11_TRM_11*/{30000, 12000,     9000,    7000,     5000,     3800,     2500,     1500,    750,      600,      510,     450,     390,    330,     290,     250,       210,     180,     150,      140,     130,       120,     110,     100,     90,    85,       80,      70,      60,        50,        40,       31,    31},
/*12_TRM_14*/{30000, 12000,     9000,    7000,     5000,     3800,     2500,     1500,    750,      600,      510,     450,     390,    330,     290,     250,       210,     180,     150,      140,     130,       120,     110,     100,     90,    85,       80,      70,      60,        50,        40,       31,    31}, 
/*13_TRM_17*/{30000, 12000,     9000,    7000,     5000,     3800,     2500,     1500,    750,      600,      510,     450,     390,    330,     290,     250,       210,     180,     150,      140,     130,       120,     110,     100,     90,    85,       80,      70,      60,        50,        40,       31,    31},
/*14_TRM_20*/{30000, 12000,     9000,    7000,     5000,     3800,     2500,     1500,    750,      600,      510,     450,     390,    330,     290,     250,       210,     180,     150,      140,     130,       120,     110,     100,     90,    85,       80,      70,      60,        50,        40,       31,    31},
/*15_TRM_25*/{7500,   5500,     4500,    3500,     2750,     2000,     1500,     1100,    700,      500,      400,     320,     260,    220,     180,     160,       130,     100,      60,       45,      35,        30,      25,      20,     15,    15,       15,      15,      15,        15,        15,       15,    15},
/*16_TRM_32*/{600,     350,      200,     140,      90,       60,       36,       26,      18,       12,       13,      15,      15,     15,      15,      15,        15,      15,      15,       15,      15,        15,      15,      15,     15,    15,       15,      15,      15,        15,        15,       15,    15}, 
/*17_TRM_40*/{600,     350,      200,     140,      90,       60,       36,       26,      18,       12,       13,      15,      15,     15,      15,      15,        15,      15,      15,       15,      15,        15,      15,      15,     15,    15,       15,      15,      15,        15,        15,       15,    15},
/*18_TCM_11*/{30000, 12000,     9000,    7000,     5000,     3800,     2500,     1500,    750,      600,      510,     450,     390,    330,     290,     250,       210,     180,     150,      140,     130,       120,     110,     100,     90,    85,       80,      70,      60,        50,        40,       31,    31},
/*19_TCM_14*/{30000, 12000,     9000,    7000,     5000,     3800,     2500,     1500,    750,      600,      510,     450,     390,    330,     290,     250,       210,     180,     150,      140,     130,       120,     110,     100,     90,    85,       80,      70,      60,        50,        40,       31,    31}, 
/*20_TCM_17*/{30000, 12000,     9000,    7000,     5000,     3800,     2500,     1500,    750,      600,      510,     450,     390,    330,     290,     250,       210,     180,     150,      140,     130,       120,     110,     100,     90,    85,       80,      70,      60,        50,        40,       31,    31},
/*21_TCM_20*/{30000, 12000,     9000,    7000,     5000,     3800,     2500,     1500,    750,      600,      510,     450,     390,    330,     290,     250,       210,     180,     150,      140,     130,       120,     110,     100,     90,    85,       80,      70,      60,        50,        40,       31,    31},
/*22_TCM_25*/{7500,   5500,     4500,    3500,     2750,     2000,     1500,     1100,    700,      500,      400,     320,     260,    220,     180,     160,       130,     100,      60,       45,      35,        30,      25,      20,     15,    15,       15,      15,      15,        15,        15,       15,    15},
/*23_TCM_32*/{600,     350,      200,     140,      90,       60,       36,       26,      18,       12,       13,      15,      15,     15,      15,      15,        15,      15,      15,       15,      15,        15,      15,      15,     15,    15,       15,      15,      15,        15,        15,       15,    15}, 
/*24_TCM_40*/{600,     350,      200,     140,      90,       60,       36,       26,      18,       12,       13,      15,      15,     15,      15,      15,        15,      15,      15,       15,      15,        15,      15,      15,     15,    15,       15,      15,      15,        15,        15,       15,    15},
/*25_TCHL_11*/{1200,    700,      560,     400,      320,      250,      190,      140,    100,       60,       32,      31,      31,     31,      31,      31,        31,      31,      31,       31,      31,        31,      31,      31,     31,    31,       31,      31,      31,        31,        31,       31,    31}, 
/*26_TCHL_14*/{1200,    700,      560,     400,      320,      250,      190,      140,    100,       60,       32,      31,      31,     31,      31,      31,        31,      31,      31,       31,      31,        31,      31,      31,     31,    31,       31,      31,      31,        31,        31,       31,    31}, 
/*27_TCHL_17*/{1200,    700,      560,     400,      320,      250,      190,      140,    100,       60,       32,      31,      31,     31,      31,      31,        31,      31,      31,       31,      31,        31,      31,      31,     31,    31,       31,      31,      31,        31,        31,       31,    31}, 
/*28_TCHL_20*/{1200,    700,      560,     400,      320,      250,      190,      140,    100,       60,       32,      31,      31,     31,      31,      31,        31,      31,      31,       31,      31,        31,      31,      31,     31,    31,       31,      31,      31,        31,        31,       31,    31}, 
/*29_TCHL_25*/{1200,    700,      560,     400,      320,      250,      190,      140,    100,       60,       32,      31,      31,     31,      31,      31,        31,      31,      31,       31,      31,        31,      31,      31,     31,    31,       31,      31,      31,        31,        31,       31,    31}, 
/*30_TCHL_32*/{1200,    700,      560,     400,      320,      250,      190,      140,    100,       60,       32,      31,      31,     31,      31,      31,        31,      31,      31,       31,      31,        31,      31,      31,     31,    31,       31,      31,      31,        31,        31,       31,    31}, 
};

//电机过载反时限曲线查询表:	注：每两个点之间相差25.6%
const Uint16 OL_MOTOLTAB[] = \
/*	  94.4%,	120.0%,	145.6%,	171.2%,	196.8%,	222.4%,	248.0%,	273.6%,	 299.2%       */ 
     {65535,	20000,	 5000,	 1200,	 900,	 600,	 360,	 240,     150,   100,  50};     // 0.1s

//制动电阻过载能力10倍点:
#define OL_BRAKERESABLE10  		(5000)			//10倍5s

//宏定义/结构体定义--------------------------------------------------------//
#define	OL_ERRPOINT				(0x7D000000)	//32000<<16
#define	OL_ALARMPOINT			(0x1400000)		//320<<16(因为还要乘以预报警系数 1%)
#define OL_REFSPD				(Spd_Ref.SpdRefDecm)		//当前参考转速
#define OL_GLOSPD				(RamMonitor->RotatingSpd)	//当前反馈转速
#define OL_STPROTSPDJUDUE1		(RamServo->N_Stall)     	//堵转转速1   10
#define OL_STPROTSPDJUDUE2		(RamServo->T_Stall)	        //堵转转速2   30
TYPE_OL Ol = {0,0,0,0,0,0,0,0,0,0};

//内部函数定义-----------------------------------------------------------//
//驱动器保护热积累
static void DrvOLHeatSum(Uint16 CurPerc, Uint16 Code)
{
	Uint32 DeltaHeat = 0;
    static Uint8  ServoFirstFlag = 0;
    
    if(ServoFirstFlag == 0)
    {
        if(Code > OL_DRVOLTAB_MAX)
        {
            Code = OL_DRVOLTAB_MAX;
        }
        
        memcpy(ServoLockOVLoadTable,(Uint8 *)&OL_LOCKDRVOLTAB[Code][0],sizeof(ServoLockOVLoadTable));   
        memcpy(ServoOVLoadTable,(Uint8 *)&OL_DRVOLTAB[Code][0],sizeof(ServoOVLoadTable)); 
        ServoFirstFlag = 1;
    }

	if(CurPerc >= 1022)		//>105.0%热累积
	{
        if(CurPerc >= 3262)
        {
            DeltaHeat = 0x1400000;    // 0.1s 
        }
        else if(OL_GLOSPD < OL_STPROTSPDJUDUE2)  //电机堵转
        {
			Uint16 Index = (CurPerc - 1022) >> 6;
			Uint16 Tau = ServoLockOVLoadTable[Index] - ((Uint32)(ServoLockOVLoadTable[Index] - ServoLockOVLoadTable[Index+1]) * (CurPerc - 1022 - (Index << 6)) >> 6);
			DeltaHeat = OL_ALARMPOINT / Tau;          
        }
        else
		{
			Uint16 Index = (CurPerc - 1022) >> 6;
			Uint16 Tau = ServoOVLoadTable[Index] - ((Uint32)(ServoOVLoadTable[Index] - ServoOVLoadTable[Index+1]) * (CurPerc - 1022 - (Index << 6)) >> 6);
			DeltaHeat = OL_ALARMPOINT / Tau;
		}
        
		Ol.DrvSum32B_var15 += DeltaHeat;
	}
	else if(CurPerc < 1000)	//<100.0%热消散
	{
        //;实际线电流有效值与驱动器电流额定比小于105%时 总值32000*2^16
        //;根据电流百分比分成段：1.(0，1000];2.(1000,1050)
        //;在1区间,散热值 y = 65536 - 61x  2^16=65536
        //;电流为0		65536		时间为32000*2^16 / 65536 = 32000(32s)
        //;电流为1000	4536		时间为32000*2^16 / 4576 = 462335(约7.7min)
        //;则电流为800	16736		时间为32000*2^16 / 16736 = 125307(约125s)
        //;在2区间,散热值 不散
		DeltaHeat = 65536 - 61 * CurPerc;

		if(Ol.DrvSum32B_var15 <= DeltaHeat){
			Ol.DrvSum32B_var15 = 0;
		}else{
			Ol.DrvSum32B_var15 -= DeltaHeat;
		}
	}
}

//电机保护热积累
static void MotOLHeatSum_Curr(Uint16 CurPerc)
{
	Uint32 DeltaHeat = 0;
    static Uint8 FirstFlag = 0;
    
    if(FirstFlag == 0)
    {
        memcpy(MotorOVLoadTable,OL_MOTOLTAB,sizeof(OL_MOTOLTAB));
        FirstFlag = 1;
    }

	if(CurPerc >= 1022)		//>105.0%热累积
	{
		if(CurPerc >= 2992)//电机电流大于2.992
		{
            DeltaHeat = 699050;	//320(00)<<16/15(00);	    //3s
		}
		else				//(105.0%,Max)
		{
			Uint16 Index = (CurPerc - 944) >> 8;
			Uint16 Tau = MotorOVLoadTable[Index] - ((Uint32)(MotorOVLoadTable[Index] - MotorOVLoadTable[Index+1]) * (CurPerc - 944 - (Index << 8)) >> 8);
			DeltaHeat = 0x1400000 / Tau;
		}     
        
		Ol.MotSum32B_var15 += DeltaHeat;
	}
	else if(CurPerc < 1000)	//<100.0%热消散
	{
        //;实际线电流有效值与驱动器电流额定比小于105%时 总值32000*2^16
        //;根据电流百分比分成段：1.(0，1000];2.(1000,1050)
        //;在1区间,散热值 y = 65536 - 61x  2^16=65536
        //;电流为0		65536		时间为32000*2^16 / 65536 = 32000(32s)
        //;电流为1000	4536		时间为32000*2^16 / 4576 = 462335(约7.7min)
        //;则电流为800	16736		时间为32000*2^16 / 16736 = 125307(约125s)
        //;在2区间,散热值 不散
		DeltaHeat = 65536 - 61 * CurPerc;

		if(Ol.DrvSum32B_var15 <= DeltaHeat){
			Ol.DrvSum32B_var15 = 0;
		}else{
			Ol.DrvSum32B_var15 -= DeltaHeat;
		}
	}
}

#define BrakeAvalid	(UdcCtrl.State == UDC_BRAKE)
//制动电阻保护热积累
static void BrakeResOLHeatSum_Power(void)
{
	if(BrakeAvalid)	//热积累
	{
		Uint16 tmp16;
		tmp16 = (Uint32)UdcCtrl.Udc * RamAid->DutyBrk / 100;	//制动有效电压1V
		tmp16 = (10UL * tmp16 * tmp16) / ((Uint32)RamAid->ResistorBrk * RamAid-> CapacityResistorBrk);	//制动电阻功率倍数0.1
		if(tmp16 > 2000)tmp16 = 2000;//最大200.0倍
		RamMonitor->BrkFactor = tmp16;//P0-14制动负载率
		Ol.ResSum32B += tmp16;
	}
	else if(++Ol.BrakeDecCnt >=1000)	//热消散
	{
		Ol.BrakeDecCnt =0;
		if(Ol.ResSum32B >= Ol.BrakeDec)
		{
			Ol.ResSum32B -= Ol.BrakeDec;
		}
		else
		{
			Ol.ResSum32B = 0;
		}
	}
}

//功率保护：实际功率>额定功率，维持时间600s
static Uint16 MotOLProtect_Power(Uint32 RealPower, Uint32 SysPower)
{
	static Uint32 Cnt = 0;

	if(RealPower > SysPower){
		if(Cnt >= 600000){
			return 2;
		}else{
			Cnt ++;
			return 0;
		}
	}else{
		Cnt = 0;
		return 0;
	}
}

//接口函数定义-----------------------------------------------------------//
//初始化
void Fault_OLInit(void)
{
	Ol.StopRota = 0.414f / (OL_STPROTSPDJUDUE2 - OL_STPROTSPDJUDUE1); // 8.28
	Ol.StopRotb = 1.414f + Ol.StopRota * OL_STPROTSPDJUDUE1;
	Ol.BrakeAblePoint = ((Uint32)OL_BRAKERESABLE10 * RamAid->DerateBrk);//扩大10倍——与制动电阻功率倍数0.1一致
	Ol.BrakeDec = Ol.BrakeAblePoint / 300;	//制动电阻热消散 能力300s(125℃→60℃) P8-21=100%报警时56℃→120℃
	if(Ol.BrakeDec == 0)Ol.BrakeDec = 1;
	Ol.SysPower = (Uint32)RamMotor->Ie * RamMotor->Ne;//仅仅用于判断基准
}
//驱动器过载保护
void Fault_DrvOLProtect(void)
{
	Uint16 RealCurPerc = 0;
    
    RealCurPerc = (1000UL * CurMonitor.IeeValuDisp) / RamServo->ServoIe;	//驱动器电流百分比
    
	DrvOLHeatSum(RealCurPerc, RamServo->ServoCode);
    
	if(Ol.DrvSum32B_var15 > OL_ERRPOINT)
	{	//大于故障点
		Ol.DrvSum32B_var15 = OL_ERRPOINT;	//限幅
		FaultPrtt_FaultInterface(DrvOvLoad);
	}
//	else if(Ol.DrvSum32B_var15 > OL_ALARMPOINT * RamAid->ServoOLAlarmPer)
//	{	//大于预报警点
//		FaultPrtt_FaultInterface(DrvOvLoadWarn);
//	}
	else
	{
		FaultPrtt_WarnInterfaceClr(DrvOvLoadWarn);
	}
}

//电机过载保护
void Fault_MotOLProtect(void)
{
	Uint16 ErrFlag = 0;//bit2|1|0 <-->堵转|功率|热
    Uint16 OLMotCurPerc = 0;

    OLMotCurPerc = (1000UL * CurMonitor.IeeValuDisp) / RamMotor->Ie;	    //电机电流百分比
    
	MotOLHeatSum_Curr(OLMotCurPerc);	//电机热积累
    
	if(Ol.MotSum32B_var15 > OL_ERRPOINT)
	{	//大于故障点
		Ol.MotSum32B_var15 = OL_ERRPOINT;	//限幅
		FaultPrtt_FaultInterface(MotOvLoad);//电机过载
		ErrFlag = 1;
	}
//	else if(Ol.MotSum32B_var15 > OL_ALARMPOINT * RamAid->MotorOLAlarmPer)
//	{	//大于预报警点
//		FaultPrtt_FaultInterface(MotOvLoadWarn);
//	}
	else
	{	//无
		FaultPrtt_WarnInterfaceClr(MotOvLoadWarn);
	}

	ErrFlag += MotOLProtect_Power(((Uint32)RamMonitor->Irms * abs(SpdMonitor.SpdAvrg)), Ol.SysPower);	//电机功率保护

	if(ErrFlag)
	{
		FaultPrtt_FaultInterface(MotOvLoad);//电机过载
	}
}

//电机堵转保护
void Fault_MotorLockedProtect(void)
{
    int16  IqRefTemp = 0; 
    Uint8  LockSpeedFlag  = 0;
    Uint8  LockTorqueFlag = 0;
    static Uint16 LockCnt = 0; //1ms计数一次
      
    if(IqRef.IqRefGet == 0)
    {
        IqRefTemp = _IQabs(CurMonitor.TorqRatsRef);
    }
    else
    {
        IqRefTemp = 0;
    }

    //判断标志位
    LockSpeedFlag  = (_IQabs(SpdFb.SpdFbDecm) < OL_STPROTSPDJUDUE1) ? 1 : 0;                //小于10rpm则认为堵转
    LockTorqueFlag = (IqRefTemp >= RamAid->LockIqThreshold) ? 1 : 0;
      
    if((1 == RamAid->LockSel) && (1 == LockSpeedFlag) && (1 == LockTorqueFlag))
    {
        LockCnt++;
        if(LockCnt >= RamAid->LockTime)
        {
            LockCnt = 0;
            FaultPrtt_FaultInterface(MotorLockErr);
        }
    }
    else
    {
        LockCnt = 0;  
    }
}

//制动电阻保护
void Fault_BrakeResProtect(void)
{
	BrakeResOLHeatSum_Power();

	if(Ol.ResSum32B >= Ol.BrakeAblePoint)
	{
		Ol.ResSum32B = Ol.BrakeAblePoint;
		FaultPrtt_FaultInterface(RstOvLoad);
	}
}

#define RunAwayDELAYTIME  100
void RunAwayMointor(void)
{
    static Uint8  RunAwayFlag = 0;
    static Uint8  RunAwayCnt_T = 0;      //转矩模式下飞车时延时计数值
    static Uint16 RunAwayCnt_PV = 0;     //位置或速度模式下飞车时延时计数值
    static Uint16 RunAwayCnt_PT = 0;     //转矩摸下伪速度调节器模式下飞车时延时计数值
    static int32  RunAwayToqFdb = 0;     //飞车时Q轴电流累加和
    static int32  RunAwaySpdFdb = 0;     //飞车时速度反馈累加和
    static int16  RunAwaySpdFdbLatch = 0;    //飞车速度判断时锁存上次的速度反馈信息
    static int32  RunAwayIqFdbLatch = 0;
    static Uint16 SpdMaxCnt = 0;

    int32  RunAwayAccSpdValue = 0;          //飞车速度判断时速度增量
    int32  RunAwayAccIqValue = 0;          //飞车速度判断时速度增量

    if((RamAid->RunAwaySel == 0) || (RunAwayFlag == 1))
    {
        RunAwayCnt_T = 0;
        RunAwayCnt_PV = 0;
        RunAwayCnt_PT = 0;
        RunAwayToqFdb = 0;
        RunAwaySpdFdb = 0;

        RunAwaySpdFdbLatch = SpdFb.SpdFbDecm;
        RunAwayIqFdbLatch  = IqRef.IqRef;
        return ;
    }
    
    if(CtrlMode.ActualMode == ACTUALMODE_TOR)
    {
        //RunAwayCnt_PV = 0;
        //RunAwayCnt_PT = 0;

        //if( (StateMachine.RegulFlg > 0)
        // && (_IQabs(SpdMonitor.SpdAvrg) > 50) ) //50rpm
        //{
        //    RunAwayCnt_T++;

        //    RunAwayToqFdb += (int32)CurMonitor.TorqRatsDisp;
        //    RunAwaySpdFdb += (int32)SpdMonitor.SpdAvrg;

        //    if(RunAwayCnt_T > RunAwayDELAYTIME) 
        //    {
        //        if(   ((RunAwayToqFdb > (int32)(1000 >> 3))&&((RunAwaySpdFdb +(int32)100) < 0))
        //            ||(((RunAwayToqFdb + (int32)(1000 >> 3))<0)&&(RunAwaySpdFdb >(int32)100))
        //          )
        //        {
        //            RunAwayFlag = 1;
        //            //PostErrMsg(UVW_FEICHE);
        //        }
        //        else
        //        {
        //            RunAwayToqFdb = 0;   //累加和要清零
        //            RunAwaySpdFdb = 0;
        //            RunAwayCnt_T = 0;    //计数值也要清零
        //        }
        //    }
        //}
        //else
        //{
        //    RunAwayToqFdb = 0;   //累加和要清零
        //    RunAwaySpdFdb = 0;
        //    RunAwayCnt_T = 0;    //计数值也要清零
        //}
    }
    //else if((CtrlMode.ActualMode == ACTUALMODE_TOR) && (0))
    //{
    //    RunAwayCnt_T = 0;
    //    RunAwayToqFdb = 0;   //累加和要清零
    //    RunAwaySpdFdb = 0;
    //    RunAwayCnt_PV = 0;

    //    RunAwayAccSpdValue = SpdFb.Fb - RunAwaySpdFdbLatch;

    //    if(StateMachine.RegulFlg)
    //    {
    //        if((CurMonitor.TorqRatsRef >= 0) 
    //           && (SpdMonitor.SpdAvrg < (-500))) 
    //        {
    //            if(RunAwayAccSpdValue > 0L)
    //            {
    //                RunAwayCnt_PT = 0;
    //            }
    //            else if(RunAwayAccSpdValue < 0L)
    //            {
    //                RunAwayCnt_PT++;
    //            }
    //        }
    //        else if( (CurMonitor.TorqRatsRef <= 0) 
    //              && (SpdMonitor.SpdAvrg > 500) )
    //        {
    //            if(RunAwayAccSpdValue < 0L)
    //            {
    //                RunAwayCnt_PT = 0;
    //            }
    //            else if(RunAwayAccSpdValue > 0L)
    //            {
    //                RunAwayCnt_PT++;
    //            }
    //        }
    //        else
    //        {
    //             RunAwayCnt_PT = 0;
    //        }
    //        if(RunAwayCnt_PT > 15)
    //        {
    //            //PostErrMsg(UVW_FEICHE);
    //            RunAwayCnt_PT = 17;
    //        }
    //    }
    //}
    else
    {
        RunAwayCnt_T = 0;
        RunAwayToqFdb = 0;   //累加和要清零
        RunAwaySpdFdb = 0;
        RunAwayCnt_PT = 0;

        RunAwayAccSpdValue = SpdMonitor.SpdAvrg - RunAwaySpdFdbLatch;
        RunAwayAccIqValue  = IqRef.IqRef - RunAwayIqFdbLatch;

        if(StateMachine.RegulFlg)
        {
            if((IqRef.IqRef >= 0) && (SpdFb.SpdFbDecm < (-500)))               
            {
                if(RunAwayAccSpdValue < 0 && (RunAwayAccIqValue >= 0))// && (RunAwayAccIqValue > 0 || Glo_DO_FUNC.Flag_Func0.bit.C_LT == 1)
                {
                    RunAwayCnt_PV++;
                }
                else
                {
                    RunAwayCnt_PV = 0;
                }
            }
            else if((IqRef.IqRef < 0) && (SpdFb.SpdFbDecm > 500))                 
            {
                if(RunAwayAccSpdValue > 0 && (RunAwayAccIqValue <= 0))//&& (RunAwayAccIqValue < 0 || Glo_DO_FUNC.Flag_Func0.bit.C_LT == 1)
                {
                    RunAwayCnt_PV++;
                }
                else
                {
                    RunAwayCnt_PV = 0;
                }
            }
            else
            {
                 RunAwayCnt_PV = 0;
            }

            if(RunAwayCnt_PV > 10)
            {
                RunAwayFlag = 1;
                FaultPrtt_FaultInterface(rs2);
            }
        }
        else
        {
            RunAwayCnt_PV = 0;
        }
    }

    if(StateMachine.RegulFlg && (SpdFb.SpdFbDecm > 100 || SpdFb.SpdFbDecm < -100))
    {
        SpdMaxCnt++;
        if(SpdMaxCnt > 1000)
        {
            RunAwayFlag = 1;
            SpdMaxCnt = 1001;
        }
    }

    RunAwaySpdFdbLatch = SpdMonitor.SpdAvrg;
    RunAwayIqFdbLatch  = IqRef.IqRef;
}


//===========================================================================
// End of file.
//===========================================================================

