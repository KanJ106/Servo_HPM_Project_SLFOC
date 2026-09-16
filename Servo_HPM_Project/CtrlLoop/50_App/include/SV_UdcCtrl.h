/*
 * SV_UdcCtrl.h
 *
 *  Created on: 2015-12-29
 *      Author:
 */

#ifndef SV_UDCCTRL_H_
#define SV_UDCCTRL_H_

#include "Datatype.h"

#define UDC_VOLTBASE_48			(88)	//母线电压基准值(220V -10%~+15%)
#define UDC_POWERUPLEVEL_48		(20)	//临时测试 TODO
#define UDC_OVERVOLTLEVEL_48	(60)	//母线电压过压基准
#define UDC_BRKVOLTUPLEVEL_48	(56)	//母线电压泄放(能耗制动)点(119.0%)
#define UDC_BRKVOLTDNLEVEL_48	(54)	//母线电压泄放(能耗制动)滞环
#define UDC_UNDERVOLTLEVEL_48	(36)	//临时测试    母线电压欠压基准(31.25%)
#define UDC_SOFTOFFLEVEL_48		(5)     //临时测试    继电器OFF基准(46.6%)
/*
OVUDCSET311V    .set  26214 ;直流母线过压点400V, 32767对应500V
UDUDCSET311V    .set  13107  ;直流母线欠压点200V
UDCOKSET311V    .set  14417  ;220V,建立母线电压
BLKUDCSET311V   .set  24248 ;直流母线泻放点370
UDCRATE311V     .set  20381 ;母线电压额定值:311
DELTUDCRLYON311V    .set  114   ;电压变化率在3V以内，且母线电压在200V~400V之间
UDCDISPCOEFQ16311V  .set  1000  ;母线电压显示系数,Q16(500*1/32768)
UDCBLKHYS311V   .set	23592   ;母线泄放滞环360V
*/
#define UDC_VOLTBASE_220		(500)	//母线电压基准值(220V -10%~+15%)
#define UDC_POWERUPLEVEL_220	(217)	//母线电/压上电基准(70.0%)//原来80%
#define UDC_OVERVOLTLEVEL_220	(400)	//母线电压过压基准
#define UDC_BRKVOLTUPLEVEL_220	(370)	//母线电压泄放(能耗制动)点(119.0%)
#define UDC_BRKVOLTDNLEVEL_220	(360)	//母线电压泄放(能耗制动)滞环
#define UDC_UNDERVOLTLEVEL_220	(210)	//母线电压欠压基准(67.8%)
#define UDC_SOFTOFFLEVEL_220	(145)	//继电器OFF基准(46.6%)

/*
OVUDCSET540V    .set  26214 ;直流母线过压点800V, 32767对应1000V, 则400V对应26214
UDUDCSET540V    .set  11468  ;直流母线欠压点350V, 对应11468
UDCOKSET540V    .set  14085  ;430,建立母线电压
BLKUDCSET540V   .set  22609 ;直流母线泻放点690, 对应22609
UDCRATE540V     .set  17596 ;母线电压额定值:537V, 对应17596
DELTUDCRLYON540V    .set  114   ;电压变化率在3V以内，且母线电压在200V~400V之间
UDCDISPCOEFQ16540V  .set  2000  ;母线电压显示系数,Q16(1000*1/32768)
UDCBLKHYS540V   .set	22281   ;母线泄放滞环680V
*/
#define UDC_VOLTBASE_380		(1000)	//母线电压基准值(380V -10%~+15%)
#define UDC_POWERUPLEVEL_380	(430)	//母线电压上电基准(80.0%)
#define UDC_OVERVOLTLEVEL_380	(800)	//母线电压过压基准
#define UDC_BRKVOLTUPLEVEL_380	(690)	//母线电压泄放(能耗制动)点(128.5%)
#define UDC_BRKVOLTDNLEVEL_380	(680)	//母线电压泄放(能耗制动)滞环
#define UDC_UNDERVOLTLEVEL_380	(363)	//母线电压欠压基准(67.8%)
#define UDC_SOFTOFFLEVEL_380	(250)	//继电器OFF基准(46.6%)

#define UDC_DRVVOLT				(RamServo->ServoVolLv)	//驱动器电压等级
#define UDC_VOLTBASE			(UdcCtrl.UdcVoltBase)	//母线电压基准值
#define UDC_FLTRTIME			(10)	//母线电压滤波时间
#define UDC_POWERUPLEVEL		(UdcCtrl.PowerUpLevel)	//母线电压上电基准
#define UDC_OVERVOLTLEVEL		(UdcCtrl.OverVoltLevel)	//母线电压过压基准
#define UDC_OVERVOLTTHRE		(5)	//母线电压过压滞环
#define UDC_BRKVOLTUPLEVEL		(UdcCtrl.BrakeVoltUpLevle)	//母线电压泄放(能耗制动)点
#define UDC_BRKVOLTDNLEVEL		(UdcCtrl.BrakeVoltDnLevle)	//母线电压泄放(能耗制动)滞环
#define UDC_UNDERVOLTLEVEL		(UdcCtrl.UnderVoltLevel)//母线电压欠压基准
#define UDC_SOFTOFFLEVEL		(UdcCtrl.SoftOffLevel)	//soft-off基准

#define UDC_PLRT_TORQLIMIT		(13421773L)//_IQ(0.8)	//瞬停不停力矩限定值80%
#define UDC_SOFTONDELAY			(2000)		//软件吸合延时  --> 2000 2022.05.10

typedef enum{
	UDC_INIT = 0,				//初始化
	UDC_NORMAL = 1,				//正常
	UDC_UNDERVOLT = 2,			//欠压
	UDC_BRAKE = 3,				//制动
	UDC_OVERVOLT = 4			//过压
}ENUM_UDCSTATE;	//母线状态

typedef struct{
	ENUM_UDCSTATE 	State;				//状态
	Uint16 			Udc;				//母线电压值（1V）
	Uint16			UdcFltr;			//母线电压滤波值（1V）
	Uint16			UdcFltrRem;			//滤波余数
	Uint16			SoftOnCnt;			//软起吸合计数
	Uint16			PLRTCnt;			//瞬停不停计数
	Uint16			PLRTTime;			//瞬停不停时间
	Uint16			PLRTFlag;			//瞬停不停有效

	Uint16			UdcVoltBase;		//母线采样基值
	Uint16			PowerUpLevel;		//上电基准
	Uint16			OverVoltLevel;		//过压基准
	Uint16			BrakeVoltUpLevle;	//能耗制动点
	Uint16			BrakeVoltDnLevle;	//能耗制动滞环
	Uint16			UnderVoltLevel;		//欠压基准
	Uint16			SoftOffLevel;		//继电器OFF基准

}TYPE_UDCCTRL;	//母线结构体

extern TYPE_UDCCTRL UdcCtrl;		//母线结构体
extern void UdcCtrl_Schedule(void);	//电压控制调度程序
extern void UdcCtrl_Init(void);		//电压控制初始化程序

#endif /* SV_UDCCTRL_H_ */
/*----------------------------------------------------------------------------------*/
//No more
/*----------------------------------------------------------------------------------*/
