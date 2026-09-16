/*
 * SV_Temperctl.c
 *
 *  Created on: 2016-1-26
 *      Author: rd0217////
 */

#include "SV_FuncVar.h"
#include "SV_Temperctl.h"
#include "Drive.h"
#include "SV_FaultProtect.h"

#define TEMPVOL1 0
#define TEMPVOL2 ((long)106*1365)
#define TEMPVOL3 ((long)230*1365)
#define TEMPVOL4 ((long)300*1365)
#define SLOPE1 -625
#define SLOPE2 -444
#define SLOPE3 -357	//-570
#define ENDPOINT1 ((long)146300*1365)
#define ENDPOINT2 ((long)127100*1365)
#define ENDPOINT3 ((long)107100*1365)	//((long)156100*1365)
#define IQCOFF 12		                //Q24(1/(1365*100*10))

#define IPMTemperaMIN      0   //监测最小温度
#define IPMTemperaMAX      125 //监测最大温度
#define IPMCurseIndexNum   25   //IPM温度曲线检测点0~25

//const Uint16 ServoTemper[] = 
//{
//3734,
//3604,
//3511,
//3373,
//3277,
//3150,
//2979,
//2867,
//2700,
//2560,
//2341,
//2262,
//2176,
//1902,
//1820,
//1619,
//1482,
//1328,
//1211,
//1099,
//1024,
//961,
//845,
//775,
//683,
//625,
//625
//};

const Uint16 ServoTemper[] = 
{
/*   0℃    */    3753,
/*   5℃    */    3666,
/*   10℃   */    3559,
/*   15℃   */    3441,
/*   20℃   */    3339,
/*   25℃   */    3150,
/*   30℃   */    2982,
/*   35℃   */    2802,
/*   40℃   */    2615,
/*   45℃   */    2421,
/*   50℃   */    2228,
/*   55℃   */    2034,
/*   60℃   */    1853,
/*   65℃   */    1679,
/*   70℃   */    1514,
/*   75℃   */    1362,
/*   80℃   */    1223,
/*   85℃   */    1098,
/*   90℃   */    985,
/*   95℃   */    884,
/*   100℃  */    774,
/*   105℃  */    714,
/*   110℃  */    644,
/*   115℃  */    583,
/*   120℃  */    528,
/*   125℃  */    481,
/*   125℃  */    481,
};

const Uint16 MotorTemper[] = 
{
/*   0℃    */    3916,
/*   5℃    */    3868,
/*   10℃   */    3810,
/*   15℃   */    3740,
/*   20℃   */    3658,
/*   25℃   */    3562,
/*   30℃   */    3456,
/*   35℃   */    3334,
/*   40℃   */    3199,
/*   45℃   */    3050,
/*   50℃   */    2888,
/*   55℃   */    2730,
/*   60℃   */    2560,
/*   65℃   */    2384,
/*   70℃   */    2208,
/*   75℃   */    2031,
/*   80℃   */    1865,
/*   85℃   */    1701,
/*   90℃   */    1547,
/*   95℃   */    1404,
/*   100℃  */    1265,
/*   105℃  */    1149,
/*   110℃  */    1034,
/*   115℃  */    931,
/*   120℃  */    836,
/*   125℃  */    745,
/*   125℃  */    745,
};

//const Uint16 MotorTemper[] = 
//{
//3734,
//3604,
//3511,
//3373,
//3277,
//3150,
//2979,
//2867,
//2700,
//2560,
//2341,
//2262,
//2176,
//1902,
//1820,
//1619,
//1482,
//1328,
//1211,
//1099,
//1024,
//961,
//845,
//775,
//683,
//625,
//625
//};

//const Uint16 MotorTemper[] = 
//{
///*   0℃    */    3917,
///*   5℃    */    3868,
///*   10℃   */    3809,
///*   15℃   */    3738,
///*   20℃   */    3656,
///*   25℃   */    3562,
///*   30℃   */    3452,
///*   35℃   */    3328,
///*   40℃   */    3192,
///*   45℃   */    3045,
///*   50℃   */    2888,
///*   55℃   */    2721,
///*   60℃   */    2548,
///*   65℃   */    2374,
///*   70℃   */    2199,
///*   75℃   */    2027,
///*   80℃   */    1860,
///*   85℃   */    1701,
///*   90℃   */    1547,
///*   95℃   */    1404,
///*   100℃  */    1270,
///*   105℃  */    1148,
///*   110℃  */    1038,
///*   115℃  */    936,
///*   120℃  */    843,
///*   125℃  */    760,
///*   125℃  */    760,
//};

void IGBTTemperCtl(void)
{
    Uint16 i;
    Uint16 temp0;
    Uint16 temp1;
    Uint16 ADCValue = 0;
    Uint16 temperature;
    
    static Uint16 Stempbuff[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static Uint32 StempSum = 0;
    static Uint8  STempcnt = 0;
    static Uint8  OverCnt1 = 0;
    static Uint8  OverCnt2 = 0;
    static Uint8  FirFlag  = 0;
    
    IGBTemperRead_Calc();
    
    ADCValue = _IQtoIQ12(AdRead.IGBTemper);
    
    if(STempcnt > 7)
    {
        STempcnt = 0;
        FirFlag = 1;
    }
    StempSum += ADCValue; 
    StempSum -= Stempbuff[STempcnt];
    Stempbuff[STempcnt] = ADCValue;
    STempcnt++;
    temperature = StempSum >> 3;
    
    //温度曲线为反时限曲线即温度越高，电阻越小，AD采样值越小。
    if (ADCValue >= ServoTemper[0])
    {//下限幅点，则置0度
        temperature = IPMTemperaMIN;
    }
    else if (ADCValue <= ServoTemper[IPMCurseIndexNum])
    {//上限幅点，则置125度
        temperature = IPMTemperaMAX;
    }
	else
	{
        for (i = 0; i < IPMCurseIndexNum; i++)
        {//查询表单，反推温度值
            if ((temperature >  ServoTemper[i+1]) 
               && (temperature <= ServoTemper[i]))
            {
                break;
            }
        }
        //进行插补运算，期间进行进行四舍五入运算
        temp0 = (ServoTemper[i] - temperature) << 2;
        temp1 = ServoTemper[i] - ServoTemper[i+1];
        temperature  = IPMTemperaMIN + (i * 5) + ((temp0 + (temp1 >> 1)) / temp1);
    }  
 
	RamMonitor->TIPM = temperature;

    if(FirFlag == 1)
    {
    	//达到故障温度，显示故障
    	if(temperature >= RamAid->ServoFaultTem)
    	{
            OverCnt1++;
            if(OverCnt1 > 5)
            {
    		    FaultPrtt_FaultInterface(DrvOvHeat); 
                OverCnt1 = 6;
            }
    	} 
    	//达到警告温度，显示警告
    	else if(temperature >= RamAid->ServoWarnTem)
    	{
            OverCnt2++;
            if(OverCnt2 > 5)
            {
    		    FaultPrtt_FaultInterface(DrvOvHeatWarn);
                OverCnt2 = 6;
            }
    	}
    	//未到警告温度，清除警告
    	else
    	{
            OverCnt1 = 0;
            OverCnt2 = 0;
    		FaultPrtt_WarnInterfaceClr(DrvOvHeatWarn);
    	}//2019.6.1屏蔽，把报警阈值降低为100度，就是以前的警告值
    }
}

void MortorTemperCtl(void) 
{
    Uint16 i;
    Uint16 temp0;
    Uint16 temp1;
    Uint16 ADCValue = 0;
    Uint16 temperature;
    
    static Uint16 tempbuff[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static Uint32 tempSum = 0;
    static Uint8  Tempcnt = 0;
    static Uint8  OverCnt1 = 0;
    static Uint8  OverCnt2 = 0;
    static Uint8  FirFlag  = 0;

    if(RamAid->MotorFaultTem == 0)
    {
        return;
    }   
    
    MorTorTemperRead_Calc();
    ADCValue = _IQtoIQ12(AdRead.MortorTemper);

    if(Tempcnt > 7)
    {
        Tempcnt = 0;
        FirFlag = 1;
    }
    tempSum += ADCValue; 
    tempSum -= tempbuff[Tempcnt];
    tempbuff[Tempcnt] = ADCValue;
    Tempcnt++;
    temperature = tempSum >> 3;
    
    //温度曲线为反时限曲线即温度越高，电阻越小，AD采样值越小。
    if (ADCValue >= MotorTemper[0])
    {//下限幅点，则置0度
        temperature = IPMTemperaMIN;
    }
    else if (ADCValue <= MotorTemper[IPMCurseIndexNum])
    {//上限幅点，则置125度
        temperature = IPMTemperaMAX;
    }
	else
	{
        for (i = 0; i < IPMCurseIndexNum; i++)
        {//查询表单，反推温度值
            if ((temperature >  MotorTemper[i+1]) 
               && (temperature <= MotorTemper[i]))
            {
                break;
            }
        }
        //进行插补运算，期间进行进行四舍五入运算
        temp0 = (MotorTemper[i] - temperature) << 2;
        temp1 = MotorTemper[i] - MotorTemper[i+1];
        temperature  = IPMTemperaMIN + (i * 5) + ((temp0 + (temp1 >> 1)) / temp1);
    }
    
    RamMonitor->MotorTemper = temperature;
    
    if(FirFlag == 1)
    {
        //达到故障温度，显示故障
    	if(temperature >= RamAid->MotorFaultTem)
    	{
            OverCnt1++;
            if(OverCnt1 > 5)
            {
    		    FaultPrtt_FaultInterface(MotOvHeat);
                OverCnt1 = 6;
            }
    	}
    	//达到警告温度，显示警告
    	else if(temperature >= RamAid->MotorWarnTem)
    	{
            OverCnt2++;
            if(OverCnt2 > 5)
            {
    		    FaultPrtt_FaultInterface(MotOvHeatWarn);
                OverCnt2 = 6;
            }
    	}
    	//未到警告温度，清除警告
    	else
    	{
            OverCnt1 = 0;
            OverCnt2 = 0;
    		FaultPrtt_WarnInterfaceClr(MotOvHeatWarn);
    	}//把报警阈值降低为100度，就是以前的警告值
    }
}

void MCUTemperCtl(void)
{
    #if SERVO_MCU == TI_C2000
    RamMonitor->McuTemper = 25;
    #elif SERVO_MCU == ST_STM32
    RamMonitor->McuTemper = 25;
    #elif SERVO_MCU == RZ_RZT1
	Uint16 McuTemp = 0;
    int16  McuTempOut = 0; 
    static Uint16 Cnt1s = 0;
	
    McuTemp = (uint16_t)(S12ADC0.ADTSDR);

    Cnt1s++;
    if(Cnt1s > 1000)
    {
        //1.21        --> 25℃    1210mv   
        //4.1mv/℃ 
        McuTemp = (Uint16)(((Uint32)McuTemp * 33000) >> 12); // 0.1mv
        McuTempOut = (McuTemp - 12100) / 41 + 25;
        RamMonitor->McuTemper = McuTempOut;
        Cnt1s = 0;
    }
    #endif
}



//	long long temIQ = 0;
//	long tem = 0;
//	Uint16 temperature = 0;
//	IGBTemperRead_Calc();
//	tem = _IQtoIQ12(AdRead.IGBTemper);
//	tem *= 100;
//
//	if(tem >= TEMPVOL1 && tem < TEMPVOL2)
//	{
//		temIQ = SLOPE1*tem;
//		temIQ += ENDPOINT1;
//	}
//	if(tem >= TEMPVOL2 && tem < TEMPVOL3)
//	{
//		temIQ = SLOPE2*tem;
//		temIQ += ENDPOINT2;
//	}
//	if(tem >= TEMPVOL3 && tem <= TEMPVOL4)
//	{
//		temIQ = SLOPE3*tem;
//		temIQ += ENDPOINT3;
//	}
//	temIQ *= IQCOFF;
//	temperature = temIQ >> 24;
