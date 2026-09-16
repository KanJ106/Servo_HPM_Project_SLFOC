/*
 * DrvCoeff.h
 *
 *  Created on: 2015-12-29
 *      Author: w
 *///

#ifndef DRVCOEFF_H_
#define DRVCOEFF_H_
#include "IQmathLib.h"//

typedef enum
{
	IncMode =0,
	AbsMode =1
}EncType;
struct DRVFLG_BITS {
    Uint16 EncType:1;
    Uint16 DOUBLEPWMEN:1;
    Uint16 RotPolar:1;//旋转极性
    Uint16 EncMode:1;//双采双控第二次采集编码器模式
    Uint16 SvpwmMode:1;
    Uint16 EncDir:1;
    Uint16 Rsvd1:10;
};
union DRVFLG {
    Uint16                 all;
   struct DRVFLG_BITS    bit;
};
typedef struct
{
	union DRVFLG DrvFlg;
    Uint16 Int1msMaxCnt;                    //1ms中断最大计数次数
	_iq    DrvIe;							//驱动器额定电流标幺值
	Uint16 DrvOriginalIe;					//驱动器额定电流，单位0.01A
	Uint16 MotOriginalIe;					//电机额定电流，单位0.01A
	Uint16 DrvGain;							//驱动器放大倍数，单位0.01times
	Uint16 DrvIbase;						//驱动器电流基值，3v对应的电流
	_iq MotIe;								//电机额定电流标幺值
	_iq MotIeInv;							//电机额定电流标幺值
	Uint16 MotTorqe;						//电机额定转矩，单位0.01Nm
	Uint16 Kt;								//转矩系数，单位0.01N.m/A
	Uint16 Uintype;							//驱动器输入电压类型，220v和380v
	Uint16 UdcBase;							//母线电压基值
	_iq    UdcEpu;							//额定母线电压标幺值
	Uint16 SpdBase;							//转速基值
	_iq SpdBasePu;							//_IQ(1/SpdBase)
	Uint16 SpdOrgNe;						//电机额定转速
	Uint16 SpdMax;							//电机最高转速
	_iq SpdNePu;							//_IQ(SpdOrgNe/SpdBasePu)
	_iq SpdMaxPu;							//_IQ(SpdMax/SpdBasePu)
	_iq Rs;									//电机定子电阻标幺值
	_iq Ld;									//电机D轴电感标幺值
	_iq Lq;									//电机Q轴电感标幺值
	_iq Jmot;								//电机转子惯量标幺值
	Uint16 MotPoles;						//电机极对数
	Uint32 MotEncLns;						//编码器四倍前线数，如果为串行编码器，则为分辨率/4
	Uint32 MotEncSglRevLns;					//编码器分辨率，一圈对应的脉冲总数
    Uint32 OutEncSglRevLns;                 //输出端编码器分辨率
	Uint32 PerAngleLines;					//每个电机极对数对应的编码器脉冲数，MotEncSglRevLns/MotPoles
	_iq We;									//rsvd
	_iq Wn;									//rsvd
	_iq Wne;								//rsvd
	Uint16 PosLoopDivCnt;					//位置环调度周期对应的次数
	Uint16 SpdLoopDivCnt;					//速度环调度周期对应的次数
	Uint16 Tpwm;							//电流环调度周期，单位0.01us
	Uint32 PosTsamp;						//位置环调度周期，单位0.01us
	Uint16 SpdTsamp;						//速度环调度周期，单位0.01us
	Uint16 SpdFrq;							//速度环调度频率，单位hz
	Uint16 TpwmFrq;							//电流环调度频率，单位hz
	Uint16 Cnt1ms;							//1ms对应的PWM数
	Uint16 CntPos1ms;						//1ms对应的位置环数
	_iq TpwmPu;								//电流环调度周期标幺值 单位1s/Q24
	_iq PosTsampPu;							//位置环调度周期标幺值 单位1s/Q24
	_iq SpdTsampPu;							//位置环调度周期标幺值 单位1s/Q24
	Uint32 EpwmPrd;							//PWM周期寄存器值
	_iq SoftOcPonit;						//软件过流点标幺值
	Uint16 pwmAdjustCnt;					//PWM校正值
	_iq UmaxLmt;							//输出电压最大百分比
	void (*init)();
}DRVCOEFF;
typedef DRVCOEFF *DRVCOEFF_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the DRVCOEFF_TYPE object.
-----------------------------------------------------------------------------*/
#define DRVCOEFF_DEFAULTS {    \
/*DrvFlg                  */0, \
/*Int1msMaxCnt            */0, \
/*DrvIe                   */0, \
/*DrvOriginalIe           */0, \
/*MotOriginalIe           */0, \
/*DrvGain                 */0, \
/*DrvIbase                */0, \
/*MotIe                   */0, \
/*MotIeInv                */0, \
/*MotTorqe                */0, \
/*Kt                      */0, \
/*Uintype                 */0, \
/*UdcBase                 */0, \
/*UdcEpu                  */0, \
/*SpdBase                 */0, \
/*SpdBasePu               */0, \
/*SpdOrgNe                */0, \
/*SpdMax                  */0, \
/*SpdNePu                 */0, \
/*SpdMaxPu                */0, \
/*Rs                      */0, \
/*Ld                      */0, \
/*Lq                      */0, \
/*Jmot                    */0, \
/*MotPoles                */0, \
/*MotEncLns               */0, \
/*MotEncSglRevLns         */0, \
/*MotEncSglRevLns         */0, \
/*PerAngleLines           */0, \
/*We                      */0, \
/*Wn                      */0, \
/*Wne                     */0, \
/*PosLoopDivCnt           */0, \
/*SpdLoopDivCnt           */0, \
/*Tpwm                    */0, \
/*PosTsamp                */0, \
/*SpdTsamp                */0, \
/*SpdFrq                  */0, \
/*TpwmFrq                 */0, \
/*Cnt1ms                  */0, \
/*CntPos1ms               */0, \
/*TpwmPu                  */0, \
/*PosTsampPu              */0, \
/*SpdTsampPu              */0, \
/*EpwmPrd                 */0, \
/*SoftOcPonit             */0, \
/*pwmAdjustCnt            */1800, \
/*UmaxLmt                 */0, \
						(void (*)(long)) DrvCoeff_init }
						void DrvCoeff_init(DRVCOEFF_handle);
extern DRVCOEFF DrvCoeff;
          
#endif /* DRVCOEFF_H_ */
